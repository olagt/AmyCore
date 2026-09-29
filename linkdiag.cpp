// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
// linkdiag: why a joint board sends no frames (see linkdiag.h)
#include "linkdiag.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <arpa/inet.h>
#include <dirent.h>
#include <ifaddrs.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <netinet/in.h>

using std::string;

static long long nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

string macStr(const uint8_t *m)
{
    char b[20]; snprintf(b, sizeof b, "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
    return b;
}

static string agoStr(long long ms)
{
    char b[32];
    if (ms < 10000) snprintf(b, sizeof b, "%.1f s", ms/1000.0); else snprintf(b, sizeof b, "%lld s", ms/1000);
    return b;
}

// ---- host side ----
struct HostState
{
    bool   carrier=false;        // /sys/class/net/<if>/carrier
    string oper;                 // operstate
    int    speed=0;              // Mb/s
    string ip;                   // IPv4 address, "" = none
    bool   dhcp=false;           // udhcpd is running
    unsigned long long rxPackets=0, rxRate=0;   // interface counter, packets/s
    bool   valid=false;
};
static std::mutex diagMutex;     // guards host, recs
static HostState host;
static string ifName;
static std::atomic<int> ifIndex{0};

static string readLine(const string &path)
{
    std::ifstream f(path); string s; if (f) std::getline(f, s); return s;
}

// the robot network's DHCP server is udhcpd (/etc/udhcpd.conf, on the robot interface). A socket on UDP 67 alone proves
// nothing: libvirt's dnsmasq holds 0.0.0.0:67 for its VM bridge. udhcpd exits (status 0, so systemd doesn't restart
// it) when the robot interface has no address at PC boot, i.e. when the robot was off then.
static bool dhcpServerRunning()
{
    DIR *d=opendir("/proc"); if (!d) return false;
    bool found=false;
    while (struct dirent *e=readdir(d))
    {
        if (e->d_name[0]<'0' || e->d_name[0]>'9') continue;
        if (readLine(string("/proc/")+e->d_name+"/comm")=="udhcpd") { found=true; break; }
    }
    closedir(d);
    return found;
}

static HostState checkHost(unsigned long long lastRx, long long dtMs)
{
    HostState h; h.valid=true;
    string sys="/sys/class/net/"+ifName+"/";
    h.carrier = readLine(sys+"carrier")=="1";          // unreadable while the interface is down
    h.oper    = readLine(sys+"operstate");
    h.speed   = atoi(readLine(sys+"speed").c_str());
    h.rxPackets = strtoull(readLine(sys+"statistics/rx_packets").c_str(), nullptr, 10);
    if (dtMs>0 && lastRx && h.rxPackets>=lastRx) h.rxRate=(h.rxPackets-lastRx)*1000/dtMs;
    struct ifaddrs *ifa=nullptr;
    if (getifaddrs(&ifa)==0)
    {
        for (auto *i=ifa; i; i=i->ifa_next)
            if (i->ifa_addr && i->ifa_addr->sa_family==AF_INET && ifName==i->ifa_name)
            { char b[INET_ADDRSTRLEN]; inet_ntop(AF_INET, &((sockaddr_in*)i->ifa_addr)->sin_addr, b, sizeof b); h.ip=b; break; }
        freeifaddrs(ifa);
    }
    h.dhcp=dhcpServerRunning();
    return h;
}

static string hostLine(const HostState &h)
{
    if (!h.valid) return ifName+": not checked yet";
    string s=ifName+": ";
    s += h.carrier ? "link up "+std::to_string(h.speed)+" Mb/s" : "! NO LINK (operstate "+h.oper+")";
    s += h.ip.empty() ? ", ! no IPv4 address" : ", "+h.ip;
    s += h.dhcp ? ", udhcpd running" : ", ! udhcpd (DHCP server) NOT running";
    s += ", rx "+std::to_string(h.rxRate)+" pkt/s";
    return s;
}

// ---- robot side: what arrived from each sender MAC ----
struct MacRec
{
    uint8_t   mac[6];
    long long firstMs=0, lastMs=0;
    uint64_t  accepted=0, ignored=0, dhcp=0, udp51200=0, tftp=0;
    long long lastAcceptedMs=0, lastIgnoredMs=0, lastDhcpMs=0, lastUdpMs=0, lastTftpMs=0;
    string    lastIgnored;       // what the last ignored packet was, and why
    string    dhcpType;          // DISCOVER / REQUEST ...
    string    udpText;           // start of the last UDP 51200 payload (IDENT / boot replies)
    string    tftpFile;
    string    ip;                // last IPv4 source address
};
static const size_t MAX_RECS=64;
static std::vector<MacRec> recs;

static MacRec *findRec(const uint8_t *mac)
{
    for (auto &r:recs) if (memcmp(r.mac, mac, 6)==0) return &r;
    return nullptr;
}

static string printable(const uint8_t *p, int n, int maxLen)
{
    string s;
    for (int i=0; i<n && (int)s.size()<maxLen; i++) s += (p[i]>=32 && p[i]<127) ? (char)p[i] : '.';
    while (!s.empty() && s.back()=='.') s.pop_back();
    return s;
}

static const char *dhcpName(int t)
{
    static const char *n[]={"?","DISCOVER","OFFER","REQUEST","DECLINE","ACK","NAK","RELEASE","INFORM"};
    return (t>=0 && t<=8) ? n[t] : "?";
}

// describe an IPv4 packet; fills the DHCP / TFTP / UDP 51200 fields of r
static string describeIp(const uint8_t *buf, int len, MacRec &r, long long now)
{
    if (len < 34) return "short IPv4 packet";
    const uint8_t *ip=buf+14;
    int ihl=(ip[0]&0x0f)*4;
    char src[INET_ADDRSTRLEN]; inet_ntop(AF_INET, ip+12, src, sizeof src);
    if (strcmp(src,"0.0.0.0")) r.ip=src;              // DHCP requests come from 0.0.0.0
    if (ip[9]==1) return string("ICMP from ")+src;
    if (ip[9]!=17 || len < 14+ihl+8) return "IPv4 proto "+std::to_string(ip[9])+" from "+src;
    const uint8_t *udp=ip+ihl;
    int sport=udp[0]<<8|udp[1], dport=udp[2]<<8|udp[3];
    const uint8_t *pl=udp+8; int plen=len-(int)(pl-buf);
    if (dport==67)                                    // BOOTP/DHCP request: the board is in its bootloader without an IP
    {
        int type=0;
        if (plen > 240)
            for (int i=240; i+1<plen; )               // options after the magic cookie
            {
                int code=pl[i];
                if (code==255) break;
                if (code==0) { i++; continue; }
                int olen=pl[i+1];
                if (code==53 && olen>=1 && i+2<plen) { type=pl[i+2]; break; }
                i+=2+olen;
            }
        r.dhcp++; r.lastDhcpMs=now; r.dhcpType=dhcpName(type);
        return string("DHCP ")+dhcpName(type)+" (asking for an IP)";
    }
    if (dport==69)
    {
        r.tftp++; r.lastTftpMs=now;
        if (plen>2) r.tftpFile=printable(pl+2, plen-2, 40);
        return "TFTP request "+r.tftpFile;
    }
    if (sport==51200 || dport==51200)
    {
        r.udp51200++; r.lastUdpMs=now; r.udpText=printable(pl, plen, 90);
        return "UDP 51200 \""+r.udpText+"\"";
    }
    return string("UDP ")+src+":"+std::to_string(sport)+" -> "+std::to_string(dport);
}

void linkDiagPacket(const uint8_t *buf, int len, int ifindex, int pkttype, bool accepted)
{
    if (len < 14 || pkttype==PACKET_OUTGOING) return;      // our own frames
    if (ifIndex && ifindex!=ifIndex) return;               // other interfaces (the raw socket sees all of them)
    long long now=nowMs();
    std::lock_guard<std::mutex> lock(diagMutex);
    MacRec *r=findRec(buf+6);
    if (!r)
    {
        if (recs.size()>=MAX_RECS) return;
        recs.emplace_back(); r=&recs.back(); memcpy(r->mac, buf+6, 6); r->firstMs=now;
    }
    r->lastMs=now;
    if (accepted) { r->accepted++; r->lastAcceptedMs=now; return; }

    r->ignored++; r->lastIgnoredMs=now;
    int etype=buf[12]<<8|buf[13];
    bool bcast=buf[0]&1;                                  // group bit of the destination MAC
    char b[160];
    if (etype==0x4a46)                                     // JRCP joint frame that the receive thread didn't take
    {
        int type = len>18 ? buf[18] : -1;
        const char *why = len!=80 ? "wrong size, AmyCore takes 80 bytes"
                        : bcast ? "sent to broadcast/multicast, not to this PC"
                        : "not a known board MAC, or not to this PC's MAC";
        snprintf(b, sizeof b, "JRCP frame type 0x%02x, %d bytes, to %s: %s", type, len, macStr(buf).c_str(), why);
        r->lastIgnored=b;
    }
    else if (etype==0x0800) r->lastIgnored=describeIp(buf, len, *r, now);
    else if (etype==0x0806) r->lastIgnored="ARP";
    else if (etype==0x86dd) r->lastIgnored="IPv6";
    else { snprintf(b, sizeof b, "ethertype 0x%04x, %d bytes", etype, len); r->lastIgnored=b; }
}

void linkDiagInit(const char *ifname)
{
    ifName=ifname;
    ifIndex=if_nametoindex(ifname);
    {
        std::lock_guard<std::mutex> lock(diagMutex);
        host=checkHost(0, 0);
    }
    std::cout<<"link check: "<<linkDiagHost()<<"\r\n"<<std::flush;
    if (!host.carrier) std::cout<<"link check: no link on "<<ifName<<": robot logic power (AUX 7.5V) off, or the cable/switch\r\n";
    if (!host.dhcp)    std::cout<<"link check: udhcpd (DHCP server) is not running: boards in their bootloader get no IP and "
                                  "don't answer IDENTIFY. Fix: sudo systemctl restart udhcpd\r\n"<<std::flush;
    std::thread([]{
        unsigned long long lastRx=0; long long lastMs=nowMs();
        while (true)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            long long t=nowMs();
            HostState h=checkHost(lastRx, t-lastMs);
            lastRx=h.rxPackets; lastMs=t;
            if (!ifIndex) ifIndex=if_nametoindex(ifName.c_str());
            std::lock_guard<std::mutex> lock(diagMutex);
            host=h;
        }
    }).detach();
}

string linkDiagHost()
{
    std::lock_guard<std::mutex> lock(diagMutex);
    return hostLine(host);
}

bool linkDiagHostOk()
{
    std::lock_guard<std::mutex> lock(diagMutex);
    return host.valid && host.carrier && !host.ip.empty() && host.dhcp;
}

string linkDiagBoard(const LinkDiagBoard &b, long long aliveAgoMs)
{
    long long now=nowMs();
    std::lock_guard<std::mutex> lock(diagMutex);
    string s = aliveAgoMs<0 ? "never sent a joint frame" : "no joint frame for "+agoStr(aliveAgoMs);
    if (host.valid && !host.carrier)
        return s+": no link on "+ifName+" (robot logic power / AUX 7.5V off, or cable/switch)";
    if (b.mac.size()!=6) return s+": board MAC unknown (not discovered)";
    const MacRec *r=findRec(b.mac.data());
    if (!r || !r->lastMs)
    {
        s += ": nothing at all received from "+macStr(b.mac.data())+" since AmyCore started";
        if (b.ident.empty()) s += ", no IDENTIFY answer";
        string others;
        for (auto &o:recs)
            if (now-o.lastMs < 5000)
                others += (others.empty() ? "" : ", ")+macStr(o.mac)+(o.ip.empty() ? "" : " "+o.ip)+
                          (o.accepted ? " (joint frames)" : o.lastIgnored.empty() ? "" : " ("+o.lastIgnored.substr(0,40)+")");
        s += others.empty() ? "; nothing arrives on "+ifName+" from any device" : "; "+ifName+" hears only: "+others;
    }
    else if (r->lastDhcpMs && now-r->lastDhcpMs < 15000 && r->lastDhcpMs >= r->lastAcceptedMs)
    {
        s += ": it is in its bootloader, asking for an IP (DHCP "+r->dhcpType+" x"+std::to_string(r->dhcp)+
             ", last "+agoStr(now-r->lastDhcpMs)+" ago)";
        s += host.dhcp ? "; udhcpd runs but gives it no lease: check journalctl -u udhcpd and its range (192.168.88.10-20)"
                       : "; udhcpd (DHCP server) is NOT running: sudo systemctl restart udhcpd";
    }
    else if (r->lastIgnoredMs && now-r->lastIgnoredMs < 2000 && r->lastIgnoredMs > r->lastAcceptedMs)
        s += ": it sends, but AmyCore ignores it: "+r->lastIgnored+" ("+std::to_string(r->ignored)+" ignored)";
    else
    {
        s += ": last packet of any kind "+agoStr(now-r->lastMs)+" ago (";
        s += r->lastMs==r->lastAcceptedMs ? string("a joint frame") : r->lastIgnored;
        s += ")";
        if (r->lastUdpMs && now-r->lastUdpMs < 60000) s += "; last UDP 51200 reply: \""+r->udpText.substr(0,60)+"\"";
    }
    if (host.valid && !host.dhcp && s.find("DHCP")==string::npos) s += "; udhcpd not running (a rebooting board can't get an IP)";
    if (host.valid && host.ip.empty()) s += "; "+ifName+" has no IPv4 address";
    return s;
}

string linkDiagReport(const std::vector<LinkDiagBoard> &boards)
{
    long long now=nowMs();
    string out="linkdiag: "+linkDiagHost()+"\n";
    std::lock_guard<std::mutex> lock(diagMutex);
    for (auto &b:boards)
        if (b.mac.size()==6 && !findRec(b.mac.data()))
            out += "linkdiag: "+macStr(b.mac.data())+" "+b.name+": nothing received"+(b.ident.empty() ? ", no IDENTIFY answer" : "")+"\n";
    for (auto &r:recs)
    {
        string who="unknown device";
        for (auto &b:boards) if (b.mac.size()==6 && memcmp(b.mac.data(), r.mac, 6)==0) who=b.name;
        string s="linkdiag: "+macStr(r.mac)+" "+who+(r.ip.empty() ? "" : " ip "+r.ip)+", last packet "+agoStr(now-r.lastMs)+" ago";
        if (r.accepted) s += ", joint frames "+std::to_string(r.accepted)+" (last "+agoStr(now-r.lastAcceptedMs)+" ago)";
        if (r.dhcp)     s += ", DHCP "+r.dhcpType+" x"+std::to_string(r.dhcp)+" (last "+agoStr(now-r.lastDhcpMs)+" ago)";
        if (r.tftp)     s += ", TFTP x"+std::to_string(r.tftp)+" "+r.tftpFile;
        if (r.udp51200) s += ", UDP 51200 x"+std::to_string(r.udp51200)+" \""+r.udpText.substr(0,60)+"\"";
        if (r.ignored)  s += ", ignored "+std::to_string(r.ignored)+" (last: "+r.lastIgnored+")";
        out += s+"\n";
    }
    if (recs.empty()) out += "linkdiag: nothing received on "+ifName+" since AmyCore started\n";
    return out;
}
