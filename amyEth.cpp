// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#include <fstream>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/ether.h>


#include <cstdlib>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>
#include <filesystem>
#include <unistd.h>

#include "jointJCB.h"
#include "udp_sender.h"
#include "amyEth.h"
#include "linkdiag.h"



using namespace std;




// The four joint boards by jointList index. No MACs are built in: a cold start learns them from the IDENTIFY answers
// and sorts the boards by the "type" in their DATABLOCK (DoTFTP); a hot start takes them from the DATABLOCKs saved at
// the last cold start (AmyConfig/datablock/joint<N>_<mac>.gz).
static const char   *boardType[4] ={"scapula","humerus","ulna","carpus"};   // head_pan + j0, j1 + j2, j3 + j4, j5 + j6
static const uint8_t boardHwNum[4]={0x00,0x10,0x18,0x20};                   // node byte of each index
static const string  datablockDir =AMY_ROOT "/AmyConfig/datablock/";

std::string robotIf=DEFAULT_IF;

bool parseForMac(string str_,MacAddress & mac_)
{
    //IDENT,137,Little_Eft,PRI_BOOT,192.168.88.20,02:12:34:56:78:9A,DATABLOCK_OK,-,-,-,-,-,-,- 
    std::stringstream ss(str_);
    vector<string> result;
    while( ss.good() )
    {
        string substr;
        getline( ss, substr, ',' );
        result.push_back( substr );
    }
    if (result.size()>=6) 
    {//02:12:34:56:78:9A
        auto hexstring=result[5];
        for(size_t i = 0; i < hexstring.length(); i += 3) 
        {
            std::istringstream strm(hexstring.substr(i, 2));
            auto substr = hexstring.substr(i, 2);
            uint8_t x= std::stoi(substr, nullptr, 16);
            mac_.push_back(x);
        }
    }
    return true;
}

AmyEth::AmyEth(vector <JointJCB> & jointList_)
{
    jointList=&jointList_;
}

AmyEth::~AmyEth()
{
}
int AmyEth::OpenUDP_IP()
{
    udpSender.openIpUDP("192.168.88.8",51200);
    return 0;
}

int AmyEth::SendLOADAPP2JointsUDP()
{
    // Loadning APP 
    char buf[1000];
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    {
        vector <unsigned char> vecBuf;
        vecBuf.insert(vecBuf.end(), { 'L','O','A','D',' ','A','P','P',' ',' ',' ',0x00});
        memcpy(&buf[0],&vecBuf[0],12);
    }
    
    cout<<"broadcasting LOAD APP \r\n"<<flush;
    udpSender.sendIPUDP("255.255.255.255",51200,buf,12);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    udpSender.sendIPUDP("255.255.255.255",51200,buf,12);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    udpSender.sendIPUDP("255.255.255.255",51200,buf,12);
    return 0;
}

int AmyEth::waitForJointsUDP()
{
    int retval=0;
    bool answered[4]={};
    char buf[12]={'I','D','E','N','T','I','F','Y',' ',' ',' ',0x00};
    int fd=udpSender.send_IpUdp_fd;
    rawUDP.openRawEth(robotIf);
    rawUDPListener.openRawEthListener(robotIf);

    char recBuf[1000];
    sockaddr_in srcaddr;
    int timeout=0;
    // IDENTIFY is a single UDP broadcast: send it again every 500 ms, so one lost packet or a board that is still busy
    // (e.g. right after its TFTP transfer) doesn't make the whole start fail
    while (retval<4 && timeout<300)
    {
        if (timeout%50==0) udpSender.sendIPUDP("255.255.255.255",51200,buf,12);
        socklen_t addrlen = sizeof(struct sockaddr_in);
        int rercBytes=recvfrom(fd, recBuf, sizeof(recBuf), MSG_DONTWAIT, (struct sockaddr *)&srcaddr, &addrlen);
        if (rercBytes>0)
        {
            //IDENT,137,Little_Eft,PRI_BOOT,192.168.88.20,02:12:34:56:78:9A,DATABLOCK_OK,-,-,-,-,-,-,- 
            string resStr(recBuf, rercBytes);
            MacAddress mac;
            parseForMac(resStr,mac);
            // a board keeps its index once it has one (matched by MAC); a new board takes the first free index,
            // and DoTFTP sorts the boards by their DATABLOCK type
            int jIdx=-1;
            for (int a=0; a<4; a++) if (jointList->at(a).mac==mac) jIdx=a;
            if (jIdx<0) for (int a=0; a<4; a++) if (jointList->at(a).mac.empty()) { jIdx=a; break; }
            if (mac.size()!=6)
                cout<<"IDENTIFY: answer without a MAC, ignored: "<<resStr<<"\r\n"<<flush;
            else if (jIdx<0)
                cout<<"IDENTIFY: answer from a fifth board, ignored: "<<resStr<<"\r\n"<<flush;
            else if (!answered[jIdx])       // (a board answers every resent IDENTIFY: count it once)
            {
                answered[jIdx]=true;
                auto joint = &(jointList->at(jIdx));
                joint->mac=mac;
                joint->privNo=jIdx;
                joint->jointHwNum=boardHwNum[jIdx];
                joint->state=JCB_STATE_INIT_02;
                joint->ipAdd=string ( inet_ntoa(srcaddr.sin_addr) );
                joint->ident1=resStr;
                joint->isGatewayed = resStr.find("GATEWAYED") != std::string::npos;
                cout<<"IDENTIFY: board "<<jIdx<<" "<<macStr(mac.data())<<" "<<joint->ipAdd<<": "<<resStr<<"\r\n"<<flush;
                retval++;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        timeout++;
    }
    // which boards did not answer, and why that may be (link, DHCP server: see linkdiag)
    cout<<"IDENTIFY: "<<retval<<" of 4 boards answered within "<<timeout*10<<" ms\r\n";
    if (retval<4)
    {
        for (int a=0; a<4; a++)
            if (!answered[a])
            {
                auto &m=jointList->at(a).mac;
                cout<<"IDENTIFY: board "<<a<<(m.size()==6 ? " ("+macStr(m.data())+")" : string())<<" did not answer\r\n";
            }
        cout<<"IDENTIFY: "<<linkDiagHost()<<"\r\n";
        cout<<"IDENTIFY: boards answer IDENTIFY only in their bootloader with an IP from DHCP; boards already running the app "
              "(hot start) don't answer and need the broadcast\r\n"<<flush;
    }
    return retval;
}

// the "type" line of a gzipped DATABLOCK (scapula, humerus, ulna, carpus; hand = the Axolotl)
static string datablockType(const vector<uint8_t> &gz)
{
    if (gz.empty()) return "";
    char fn[]="/tmp/amycore_datablock_XXXXXX";
    int fd=mkstemp(fn);
    if (fd<0) return "";
    bool ok = write(fd, gz.data(), gz.size())==(ssize_t)gz.size();
    close(fd);
    string type;
    if (ok)
        if (FILE *p=popen(("zcat "+string(fn)+" 2>/dev/null").c_str(),"r"))
        {
            char line[512];
            while (fgets(line,sizeof line,p))
                if (strncmp(line,"type: ",6)==0) { type=line+6; type.erase(type.find_last_not_of(" \r\n")+1); }
            pclose(p);
        }
    unlink(fn);
    return type;
}

// everything that belongs to the physical board; the register tables stay with the jointList index
static void swapBoardIdentity(JointJCB &a, JointJCB &b)
{
    std::swap(a.mac,b.mac);                 std::swap(a.ipAdd,b.ipAdd);
    std::swap(a.ident1,b.ident1);           std::swap(a.ident2,b.ident2);
    std::swap(a.isGatewayed,b.isGatewayed); std::swap(a.state,b.state);
    std::swap(a.fileBLI,b.fileBLI);         std::swap(a.fileGW_BLI,b.fileGW_BLI);
    std::swap(a.fileDATABLOCK,b.fileDATABLOCK); std::swap(a.fileGW_DATABLOCK,b.fileGW_DATABLOCK);
}

// put the boards into the jointList order by their DATABLOCK type; if a type is missing or doubled, keep the order
static void sortBoardsByType(vector <JointJCB> &jointList)
{
    string type[4];
    for (int a=0; a<4; a++) type[a]=datablockType(jointList[a].fileDATABLOCK);
    for (int want=0; want<4; want++)
        if (std::count(type, type+4, string(boardType[want]))!=1)
        {
            cout<<"boards: DATABLOCK types are "<<type[0]<<", "<<type[1]<<", "<<type[2]<<", "<<type[3]
                <<" - expected one each of scapula, humerus, ulna, carpus; keeping the IDENTIFY order\r\n"<<flush;
            return;
        }
    for (int want=0; want<4; want++)
        for (int a=want+1; a<4; a++)
            if (type[want]!=boardType[want] && type[a]==boardType[want])
            {
                swapBoardIdentity(jointList[want], jointList[a]);
                std::swap(type[want], type[a]);
            }
    for (int a=0; a<4; a++)
        cout<<"boards: "<<a<<" = "<<boardType[a]<<" "<<macStr(jointList[a].mac.data())<<" "<<jointList[a].ipAdd<<"\r\n";
}

int AmyEth::DoTFTP()
{
    char buf[1000];
    char recBuf[1000];
    UDPSender TFTP=UDPSender();
    TFTP.openIpUDP("192.168.88.8",0);
    //BLI
    for (int a = 0; a< jointList->size();a++)
    {
       
        auto joint=&jointList->at(a);
        auto ipAdd=joint->ipAdd;
        TFTP.receiveFileTFTP(ipAdd,"BLI",joint->fileBLI);
        if (joint->isGatewayed)
        {
            TFTP.receiveFileTFTP(ipAdd,"axolotl/BLI",joint->fileGW_BLI);
        }
    } //auto joint=(*jointList)[abb];
    for (int a = 0; a< jointList->size();a++)
    {
        auto joint=&jointList->at(a);
        auto ipAdd=joint->ipAdd;
        TFTP.receiveFileTFTP(ipAdd,"DATABLOCK",joint->fileDATABLOCK);
        if (joint->isGatewayed)
        {
            TFTP.receiveFileTFTP(ipAdd,"axolotl/DATABLOCK",joint->fileGW_DATABLOCK);
        }
    }
    sortBoardsByType(*jointList);
    // keep a copy of every board's DATABLOCK (gzipped YAML: board type, serial, HES offsets, calibration); a hot start
    // takes the boards' MACs from these file names, so older files of the same index (another board) are removed
    for (int a = 0; a< jointList->size();a++)
    {
        auto joint=&jointList->at(a);
        string macHex=vectorToHexStr(joint->mac);
        std::error_code ec;
        for (auto &e: std::filesystem::directory_iterator(datablockDir, ec))
        {
            string n=e.path().filename().string();
            if (n.rfind("joint"+to_string(a)+"_",0)==0 && n.find(macHex)==string::npos) std::filesystem::remove(e.path(), ec);
        }
        auto save=[&](const vector<uint8_t> &data, const string &suffix)
        {
            if (data.empty()) return;
            string fn=datablockDir+"joint"+to_string(a)+"_"+macHex+suffix+".gz";
            ofstream f(fn, ios::binary);
            f.write((const char*)data.data(), data.size());
            cout<<"saved "<<fn<<" ("<<data.size()<<" bytes)\r\n";
        };
        save(joint->fileDATABLOCK,"");
        save(joint->fileGW_DATABLOCK,"_axolotl");
    }
    cout<<"\r\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
   return 0;
}

// main keepalive and data polling frame 0x01
int AmyEth::sendCmd01(JointJCB &joint, int subType_)
{

    bool readyToSend=false;  
    bool zerosMostly=false;
    JointStatus_t js;
    memset(&js.data[0],0x00,66); // fill 
    //memcpy(&frameJcbStatus.data[0],&pktKeepAlive[47+14],66); 
    
    js.kaCmd01.jointHwNum[0]=joint.jointHwNum;
    js.kaCmd01.msgtyp40[0]=0x040;
    
    if (subType_==0) // pre working - zeros
    {
        zerosMostly=true;
        js.kaCmd01.msgtyp40_2[0]=0x01; //keepalive / poling
        js.kaCmd01.you64[0]=0x64;
        js.kaCmd01.you64_bis[0]=0x64;
        js.kaCmd01.bond07[3]=0x07; // 00 00 00 07 
        js.kaCmd01.bond07_bis[3]=0x07; // 00 00 00 07 
    }
    
 //00 00 00 40 31 42 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 7c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    
    if (subType_==1) // to transision from 22 to 32
    {
        js.kaCmd01.msgtyp40_2[0]=0x11; //keepalive / poling
        zerosMostly=true;

        js.kaCmd01.you64[0]=0x64;
        js.kaCmd01.you64_bis[0]=0x64;
        js.kaCmd01.bond07[3]=0x07; // 00 00 00 07 
        js.kaCmd01.bond07_bis[3]=0x07; // 00 00 00 07 
    }

    if (subType_==2) // normal wrokin / acking 
    {
        zerosMostly=false;
        js.kaCmd01.msgtyp40_2[0]=0x01; //keepalive / poling
    }
    if (subType_==4) // Intera sends one 0x21 frame per joint right before the CONFIG register writes
    {
        zerosMostly=false;
        js.kaCmd01.msgtyp40_2[0]=0x21;
    }
    if (subType_==3) // Super MODE ->0x32 ?  
    {
        zerosMostly=true;
        js.kaCmd01.msgtyp40_2[0]=0x31; //supermode
        js.kaCmd01.you64[0]=0x64;
        js.kaCmd01.you64_bis[0]=0x64;
        js.kaCmd01.bond07[3]=0x07; // 00 00 00 07 
        js.kaCmd01.bond07_bis[3]=0x07; // 00 00 00 07 
    }


    js.kaCmd01.ackFor05[0]=joint.cmd01Ack05Cnt;
    //frameJcbStatus.kaCmd01.someZeros
    //vector<uint8_t> pollJ0_0;
    //pollJ0_0.insert(pollJ0_0.end(), { 0x2b,0x2f,0xde,0xff });
    if (zerosMostly==false)
    {
        js.kaCmd01.brake1[0]=joint.CmdBrake2Joint1;
        js.kaCmd01.jCmdFlag1=joint.Cmd2Joint1;
        js.kaCmd01.brake2[0]=joint.CmdBrake2Joint2;
        js.kaCmd01.jCmdFlag2=joint.Cmd2Joint2;
        // stiffness % (payload 14/32) and control mode (payload 18/36), set by teachTick in main.cpp
        js.kaCmd01.you64[0]=joint.stiff1;
        js.kaCmd01.you64_bis[0]=joint.stiff2;
        js.kaCmd01.bond07[3]=joint.ctrlMode1;
        js.kaCmd01.bond07_bis[3]=joint.ctrlMode2;

       /* if (joint.brakeEngaged1==true)
    {
        js.kaCmd01.brake1[0]=0x01;
        joint.jogMoveSP1=0;
        joint.encoderSP1=joint.encoderPos1;
    }
    else
    {
        js.kaCmd01.brake1[0]=0x00;
    }
    //FIXME
    if (joint.moveEnabled1==true)
    {
        js.kaCmd01.jCmdFlag1=256; //cmd 
        
        //js.kaCmd01.jCmdFlag2[1]=0x01; 
        js.kaCmd01.you64[0]=0x64;
    }else
    {
        js.kaCmd01.jCmdFlag1=0x00; // disabled ? 
    }

    if (joint.brakeEngaged2==true)
    {
        js.kaCmd01.brake2[0]=0x01;
        joint.jogMoveSP2=0;
        joint.encoderSP2=joint.encoderPos2;
    }
    else
    {
        js.kaCmd01.brake2[0]=0x00;
    }

    if (joint.moveEnabled2==true)
    {
        js.kaCmd01.jCmdFlag2=256; // emergency ? //02 or 0
        
    }else
    {
        js.kaCmd01.jCmdFlag2=0x00; // disabled
    }*/


    joint.encoderSP1 = holdSetpoint(joint, 0);
    joint.encoderSP2 = holdSetpoint(joint, 1);


    /*if (abs(joint.encoderSP1 - joint.encoderPos1)>500)
    {
        cout << "ERROR CMD POS TO BIG \r\n " << flush ;
        joint.encoderSP1=joint.encoderPos1;
    }

    if (abs(joint.encoderSP2 - joint.encoderPos2)>500)
    {
        cout << "ERROR CMD POS TO BIG \r\n " << flush ;
        joint.encoderSP2=joint.encoderPos2;
    }*/

    // torque feed-forward (1/250 Nm, docs/JRCP_PROTOCOL.md): manual value + gravity compensation, clamped to int16
    js.kaCmd01.speedSP1=(int16_t)std::clamp(joint.effortSP1 + joint.gravityFF1, -32000, 32000);
    js.kaCmd01.speedSP2=(int16_t)std::clamp(joint.effortSP2 + joint.gravityFF2, -32000, 32000);
    
    
    js.kaCmd01.encoderSP1=joint.encoderSP1;
    js.kaCmd01.encoderSP2=joint.encoderSP2;
    
    

    
    //js.kaCmd01.bond07_bis[3]=debugValue3;
    //js.kaCmd01.someZeros5_bis[0]=debugValue4;
//    js.kaCmd01.lightsBtnBit[0]=0x7c; // 7c or 00 or 7e 
    js.kaCmd01.lightsBtnBit[0]=joint.debugIntValue2; // lights on keys , bitwise 
    
    
    js.kaCmd01.lightsHead[0]=20;//00 or 20
    js.kaCmd01.lightsHead[0]=joint.debugIntValue1;  // light all , or read head 
    // global command flags = payload 44-47 (LE32): byte 45 and 47 as set above, plus the LED window's display bits
    {
        uint32_t g = ((uint32_t)js.kaCmd01.lightsBtnBit[0]<<8) | ((uint32_t)js.kaCmd01.lightsHead[0]<<24) | (joint.ledFlags & LED_GLOBAL_MASK);
        js.kaCmd01.someZeros5_bis[4]=g&0xff; js.kaCmd01.lightsBtnBit[0]=(g>>8)&0xff;
        js.kaCmd01.someZero3[0]=(g>>16)&0xff; js.kaCmd01.lightsHead[0]=(g>>24)&0xff;
    }
    // cuff (ITB) lights: low byte of the real joint flags = high byte of jCmdFlag (see setJointFlags in main.cpp)
    js.kaCmd01.jCmdFlag1 |= (uint16_t)(joint.itbLights1 & 0xf0) << 8;
    js.kaCmd01.jCmdFlag2 |= (uint16_t)(joint.itbLights2 & 0xf0) << 8;
    js.kaCmd01.unknonwRest[0]=0; // zeros
    }
//acking type :01 [0] 00 00 00 40 01 -> 05 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 7c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//type :01 [0] 00 00 10 40 01 01 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//type :01 [0] 00 00 18 40 01 01 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 7c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
    
    //.726 from PC , frame type :01 [0] 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 7c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
    //00e04c3842ec000c42ced015080045000071000
    //      0400040110922c0a85801c0a85808d9369090005dd25101000001015e49931d14ae000af79e9fc64a460000004001040000000000000000640000000700000200000000000000000000640000000700000200000000007c0000000000000000000000000000000000000000
    // packetcounter is 0 - this is intentional 
    //if ( readyToSend==true)
    {
        // if (joint.jointHwNum==00)
        // {
        // for (int i=0 ;i<66;i++)  printf(" %.2x",js.data[i]);
        // printf("\r\n");  
        // }
        rawUDP.sendRawEth(joint.mac,0x464a,(char*)js.data,66);
    }
    return 1;
}


// joint limits (rad in the URDF): head_pan -5.0952..0.9064, j0 +-3.0503, j1 -3.8095..2.2736, j2 +-3.0426,
// j3 +-3.0439, j4/j5 +-2.9761, j6 +-4.7124
const double jointLimitDeg[4][2][2]={ {{-291.93, 51.93}, {-174.77, 174.77}},     // head_pan, j0
                                      {{-218.27, 130.27}, {-174.33, 174.33}},    // j1, j2
                                      {{-174.40, 174.40}, {-170.52, 170.52}},    // j3, j4
                                      {{-170.52, 170.52}, {-270.00, 270.00}} };  // j5, j6

// Position setpoint of one side, called once per command frame (1 kHz). Intera sets the target once after homing and
// keeps it; the joint closes the position loop itself. Before 2026-09-26 AmyCore sent "current position + jog" every
// frame, so the target followed the joint and nothing held it (pushed or sagging joints just moved).
// - side ENABLED + HOMED: on the first frame the target is latched at the current position; jog moves it by
//   jog/HOLD_JOG_DIV counts per frame (1500 -> ~94 counts/ms, ~5 deg/s); it never runs more than HOLD_MAX_LAG
//   counts (~3 deg) ahead of the joint, so a blocked or pushed joint doesn't wind up a big jump
// - jog never moves the target further out than LIMIT_MARGIN_DEG inside the joint limits (a joint already outside
//   keeps its hold there, and jog can only bring it back in)
// - a position jump of more than 90 deg in one frame is a sensor wrap-around (seen 2026-09-26: j5 jogged past -180 deg
//   read +171 deg, and the lag clamp then asked for almost a full turn against the end stop): the hold is dropped,
//   jog stops and the next frame latches a new target at the new reading
// - manual teaching (control mode 10): the target follows the joint every frame
// - otherwise: no hold, the setpoint is the current position (as before)
int32_t holdSetpoint(JointJCB &j, int side)
{
    bool    &hold   = side==0 ? j.hold1 : j.hold2;
    int32_t &target = side==0 ? j.holdSP1 : j.holdSP2;
    int32_t &last   = side==0 ? j.lastPos1 : j.lastPos2;
    int32_t &jog    = side==0 ? j.jogMoveSP1 : j.jogMoveSP2;
    bool    &wrapLock = side==0 ? j.wrapLock1 : j.wrapLock2;
    uint8_t status  = side==0 ? j.jointVF.vf.stFlags1 : j.jointVF.vf.stFlags2;
    int32_t pos     = side==0 ? j.encoderPos1 : j.encoderPos2;
    const double countsPerDeg = 1024.0*1000.0*M_PI/180.0;

    bool jumped = std::abs((int64_t)pos-last) > HOLD_WRAP_JUMP;
    last=pos;
    const bool enabledHomed = (status & 0x02) && (status & 0x10);     // CRSP ENABLED, IS_HOMED_AND_UNLOCKED
    if (!enabledHomed) { hold=false; wrapLock=false; return pos; }
    if (hold && jumped)
    {
        // the reading is now on the wrong side (e.g. -189 deg reads +171), so "inward" by the reading can push the
        // joint further into its end stop: hold where it is and ignore jog until the side is disabled and re-enabled
        printf("hold: board %d side %d position jumped by more than 90 deg (sensor wrap-around): holding in place, "
               "jog locked until the joint is disabled; turn it back by hand\n", j.privNo, side);
        hold=false; jog=0; wrapLock=true;
        return pos;
    }
    // manual teaching (control mode 10, stiffness 0): like Intera the setpoint follows the joint, so when the mode goes
    // back to position the target is latched exactly where the arm was let go (no jog, go-to or move acts meanwhile)
    if ((side==0 ? j.ctrlMode1 : j.ctrlMode2)==CTRL_MODE_TEACH) { hold=true; target=pos; jog=0; return pos; }
    if (!hold) { hold=true; target=pos; }

    int32_t next = target + (wrapLock ? 0 : jog / HOLD_JOG_DIV);
    if (j.privNo>=0 && j.privNo<4)
    {
        int32_t lo = (int32_t)((jointLimitDeg[j.privNo][side][0]+LIMIT_MARGIN_DEG)*countsPerDeg);
        int32_t hi = (int32_t)((jointLimitDeg[j.privNo][side][1]-LIMIT_MARGIN_DEG)*countsPerDeg);
        if (next > hi && next > target) next = std::max(target, hi);     // never further out than the limit
        if (next < lo && next < target) next = std::min(target, lo);
    }
    target = next;
    if (target > pos + HOLD_MAX_LAG) target = pos + HOLD_MAX_LAG;
    if (target < pos - HOLD_MAX_LAG) target = pos - HOLD_MAX_LAG;
    return target;
}

int AmyEth::ProcessRegistersComm05(JointJCB &joint)
{
    //cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!";
    
    auto frRec=&joint.jointRawFrame80Received[joint.idxRecRead];
    if ( frRec->rg.pcktCounter8== joint.frame04Cnt)
    {
      //  cout<<" just perfect we go ACK \r\n";
        // notyfy joint via cmd01 we got theri ack ! 
        joint.cmd01Ack05Cnt=joint.frame04Cnt;
        joint.delayCounter=0;
        joint.regCommStage=REGCOMM_SOMEDELAY;
        
        vector <JointRegisters_t> regFake;
            string reply= parsePayloadComl((char*)&frRec->rg.regbuff[0],58,regFake,0x05,frRec->jointHwNum[0]);
            printf("05: %s",reply.c_str());
            printf("\r\n");
            // raw reply (register read-backs carry their value after the "41 <size> <reg> <module>" entry)
            printf("05raw hw %02x:", frRec->jointHwNum[0]);
            for (int i=0;i<58;i++) printf(" %02x", frRec->rg.regbuff[i]);
            printf("\n");
            // a refused write comes back as 53 <code> <reg> <module> (fb: config locked, the board is running)
            for (int i=0; i+4<=58; )
            {
                uint8_t op=frRec->rg.regbuff[i], sz=frRec->rg.regbuff[i+1];
                if (op==0x53) { printf("REFUSED write hw %02x module 0x%02x reg 0x%02x code 0x%02x\n", frRec->jointHwNum[0], frRec->rg.regbuff[i+3], frRec->rg.regbuff[i+2], sz); joint.regRefused++; i+=4; }
                else if (op==0x43 || op==0x41) i+=4+sz;
                else if (op==0x51) i+=4;
                else break;
            }
            if (frRec->rg.regbuff[0]==0x41 || frRec->rg.regbuff[0]==0x51)     // read replies also go to a file for tools/regcompare.py
                if (FILE *rf=fopen("/tmp/amycore.regs","a"))
                {
                    fprintf(rf, "hw %02x:", frRec->jointHwNum[0]);
                    for (int i=0;i<40;i++) fprintf(rf, " %02x", frRec->rg.regbuff[i]);
                    fprintf(rf, "\n"); fclose(rf);
                }
            
            //for (int i=0 ;i<58;i++)  printf(" %.2x",frRec->rg.regbuff[i]);
              // printf("\r\n");  

    }
    else
    {
        // the joint keeps repeating its last 0x05 (e.g. from a previous AmyCore run that stopped mid-upload) and
        // ignores our 0x04 until that one is acknowledged: ack it and continue the joint's numbering
        uint8_t rec=frRec->rg.pcktCounter8;
        if (joint.regCommStage==REGCOMM_WAIT4ACK && joint.frame04Cnt!=(uint8_t)(rec+1))
        {
            cout<<" 0x05 sequence resync: we sent "<< +joint.frame04Cnt << ", joint repeats "<<+rec<<", continuing with "<<+(uint8_t)(rec+1)<<" \r\n";
            joint.cmd01Ack05Cnt=rec;
            joint.frame04Cnt=rec+1;
            joint.retry04Counter=0;
        }
    }
    //   search05Ack=false;
    //             //joint->ackFramePcktCounterUploadReg=joint->framePcktCounterUploadReg;
    //             //cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!";
    //             //joint->ackCounterThe=joint->jointRawFrame80Received.rg.pcktCounter;
    //             // TODO :more parsing 
    //             joint->ack05Cnt=frRec->rg.pcktCounter8; // FORCE 
    //             joint->isWaitingFor05Ack=false;
    //             //if (joint->privNo==0)
    //             {
    //                 //if ( (frRec->msgtyp40_2[0]==0x04) || (frRec->msgtyp40_2[0]==0x05) ) // registers 
    //                 if ( (frRec->msgtyp40_2[0]==0x05) ) // registers 
    //                 {
    //                     sprintf(strbuff," framerec type:%.2x ",frRec->msgtyp40_2[0]);
    //                     sprintf(strbuff,"[%d] SS:%d",  frRec->rg.pcktCounter8,frRec->rg.someStatus[0]);

    //                     vector <JointRegisters_t> regFake;
    //                     string reply =parsePayloadComl((char*)&frRec->rg.regbuff[0],66,regFake,frRec->msgtyp40_2[0],frRec->jointHwNum[0]);
    //                     sprintf(strbuff," %s",reply.c_str());
    //                     printf("reply05: %s",reply.c_str());
    //                 }
    //             }
    return 0;
}


int AmyEth::ProcessRegistersComm(JointJCB &joint)
{
    if (joint.regCommStage==REGCOMM_DONE) 
    {
        joint.regCommStage=REGCOMM_NULL;
        //cout<<" REGCOMM_DONE-> REGCOMM_NULL \r\n";
    }
    if (joint.regCommStage==REGCOMM_NULL)  
    {  //prepare message 
        if (joint.regNoToUploadList.size()>0) 
        {
          //  cout<<" REGCOMM_NULL \r\n";
            joint.savedSubListregNoFromList.clear();
            joint.retry04Counter=0;
            JointStatus_t frameJcbStatus;
            memset(&frameJcbStatus.data[0],0x00,66); // fill 
            joint.frame04Cnt++;
            frameJcbStatus.rg.pcktCounter8=joint.frame04Cnt;
            //frameJcbStatus.rg.pcktCounter8++; // inc by one 
            frameJcbStatus.rg.dummy3a[0]=0x3a;
            frameJcbStatus.rg.subAddress[0]=0x00;
            frameJcbStatus.rg.jointHwNum[0]=joint.jointHwNum;
            frameJcbStatus.rg.msgtyp40[0]=0x40;
            frameJcbStatus.rg.msgtyp40_2[0]=0x04; // reg upload
            {
                int cnt=0;
                int maxLen=56; // just in case bcose ack may have add some data
                bool finishMessage=false;
                int freeBytes=maxLen-cnt;
                //while ( (freeBytes>0) && (finishMessage==false) )
                //for (int idx=0;idx<regNoToUploadList.size();idx++)
                int idx=0;
                while ( (freeBytes>0) && (finishMessage==false) )
                {
                    freeBytes=maxLen-cnt;
                    //auto reg=joint.registerList[joint.loadRegCounter];
                    auto reg=joint.registerList[joint.regNoToUploadList[idx]];
                    vector<uint8_t> regbufVal;
                    vector<uint8_t> regbufAck;
                    
                    int exclusive=prepareReg2Send(reg,regbufVal,regbufAck);
                    if ( (exclusive==1) && (idx==0)) // only one message , and we are first
                    {
                        finishMessage=true ;
                    }
                    if ( (exclusive==1) && (idx!=0)) // only one message , and we are NOT first 
                    {
                        finishMessage=true ; // just finnish message , do not add anything
                    }
                    else  // 
                    {
                    
                        if (freeBytes>= (regbufVal.size()+regbufAck.size() ) )
                        {
                            if (!regbufVal.empty()) memcpy(&frameJcbStatus.rg.regbuff[cnt],&regbufVal[0],regbufVal.size());
                            cnt+=regbufVal.size();
                            memcpy(&frameJcbStatus.rg.regbuff[cnt],&regbufAck[0],regbufAck.size());
                            cnt+=regbufAck.size();
                            freeBytes=maxLen-cnt;
                            int regIdx=joint.regNoToUploadList[idx];
                            joint.savedSubListregNoFromList.push_back(regIdx);
                            
                        }
                        else
                        {
                                finishMessage=true ;
                        }
                    }
                    idx++;
                    if (idx>=joint.regNoToUploadList.size())
                    {
                        finishMessage=true ;                        
                    }

                }
                if (cnt>0)
                {
                    memcpy(&joint.jointUploadRegCopy.data[0],&frameJcbStatus.data[0],66);
                    // remove only the registers that went into this frame: idx also counts the one that didn't fit
                    // (it used to be removed too, so one register per full frame was never sent - e.g. j3's pole count)
                    for (size_t a=0;a<joint.savedSubListregNoFromList.size();a++)
                    {
                        joint.regNoToUploadList.erase(joint.regNoToUploadList.begin());
                    }
                    rawUDP.sendRawEth(joint.mac,0x464a,(char*)frameJcbStatus.data,66);

            vector <JointRegisters_t> regFake;
            string reply= parsePayloadComl((char*)&frameJcbStatus.rg.regbuff[0],58,regFake,0x04,frameJcbStatus.jointHwNum[0]);
            printf("04: %s",reply.c_str());
            printf("\r\n");
                    joint.regCommStage=REGCOMM_WAIT4ACK;
                }else
                {
                    cout << "ERROR Can't prepare register for a message ! \r\n"<<flush;
                }

            }

        }
    }else 
    if (joint.regCommStage==REGCOMM_WAIT4ACK)  // this flag will be deleted by complemetary 05 handler 
    {  
        // retry cnt++ 
        //cout<<" REGCOMM_WAIT4ACK \r\n";
        // the joints answer every 2-3 ms: resend every 20 loop passes (~20 ms), give up after ~0.4 s
        joint.retry04Counter++;
        if (joint.retry04Counter%20!=0 && joint.retry04Counter<=400) return 0;
        if (joint.retry04Counter>400)
        {
            cout << "ERROR joint "<<joint.privNo<<": 04 frame "<<+joint.frame04Cnt<<" not acknowledged, register upload aborted\r\n"<<flush;
            joint.regFailed++;
            joint.regNoToUploadList.clear();
            joint.regCommStage=REGCOMM_NULL;
        }
        else
        {
            JointStatus_t frameJcbStatus;
            memset(&frameJcbStatus.data[0],0x00,66); // fill 
            // restore message 
            memcpy(&frameJcbStatus.data[0],&joint.jointUploadRegCopy.data[0],66);
            frameJcbStatus.rg.pcktCounter8=joint.frame04Cnt; // just if we need to force resend
            rawUDP.sendRawEth(joint.mac,0x464a,(char*)frameJcbStatus.data,66);
        }

    }else
    if (joint.regCommStage==REGCOMM_SOMEDELAY) // wait until propgates 
    {
           //cout<<" REGCOMM_SOMEDELAY \r\n";
        joint.delayCounter++;
        if (joint.delayCounter>3)
        {
            joint.delayCounter=0;
            joint.regCommStage=REGCOMM_DONE;
             //cout<<" REGCOMM_DONE \r\n";
        }
    }

    return 0;
}


int AmyEth::sendRegistersToJoint(JointJCB &joint, vector<int> regListFromListToSend_) // only 0x04 i 0x05
{
    /*bool readyToSend=false;
    JointStatus_t frameJcbStatus;
    memset(&frameJcbStatus.data[0],0x00,66); // fill 
    static int mycnt=0;

    //if (joint.isGatewayed==true) return 0; // no GW done yet

    // if (regFromListToSend_>=joint.registerList.size()-1)
    //     regFromListToSend_=(joint.registerList.size()-1);
    if (regListFromListToSend_.size()>4) // max 4 regs fit into one frame , later it should be dynamic
    {
        cout << " regListFromListToSend_ to big (4)" <<flush;
        return 0;

    }
    //joint.loadRegCounter=regFromListToSend_; // what reg we want to send 
    
// prepare and send
        //if (joint.waitingForAck==false)
     {
        //frameJcbStatus.rg.pcktCounter8=joint.framePcktCounterUploadReg;
       // frameJcbStatus.rg.pcktCounter8=joint.ack05Cnt;
        //frameJcbStatus.rg.pcktCounter8++; // bigger by one 
        frameJcbStatus.rg.dummy3a[0]=0x3a;
        frameJcbStatus.rg.subAddress[0]=0x00;
        frameJcbStatus.rg.jointHwNum[0]=joint.jointHwNum;
        frameJcbStatus.rg.msgtyp40[0]=0x40;
        frameJcbStatus.rg.msgtyp40_2[0]=0x04; // reg upload
        {
            int cnt=0;
            int maxLen=56; // just in case bcose ack may have add some data
            bool finishMessage=false;
            int freeBytes=maxLen-cnt;
            //while ( (freeBytes>0) && (finishMessage==false) )
            for (int idx=0;idx<regListFromListToSend_.size();idx++)
            {
                freeBytes=maxLen-cnt;
                //auto reg=joint.registerList[joint.loadRegCounter];
                auto reg=joint.registerList[regListFromListToSend_[idx]];
                vector<uint8_t> regbufVal;
                vector<uint8_t> regbufAck;
                
                prepareReg2Send(reg,regbufVal,regbufAck);
                if (freeBytes>= (regbufVal.size()+regbufAck.size() ) )
                {
                    memcpy(&frameJcbStatus.rg.regbuff[cnt],&regbufVal[0],regbufVal.size());
                    cnt+=regbufVal.size();
                    memcpy(&frameJcbStatus.rg.regbuff[cnt],&regbufAck[0],regbufAck.size());
                    cnt+=regbufAck.size();
                    freeBytes=maxLen-cnt;
                }
                else
                {
                        finishMessage=true ;
                }
                finishMessage=true ; //FIXME
                // joint.loadRegCounter++;
                // if (joint.loadRegCounter>=joint.registerList.size())
                // {
                //     joint.registersUploaded=true;
                //     finishMessage=true ;
                // }
            }
        }
        //save in case not acked 
        //joint.isWaitingFor05Ack=true;

        readyToSend=true;
    }
    if (readyToSend==true)
    {
        auto js=frameJcbStatus;
        printf("%.2x SEND reg %.2x %.2x:[%d] ",js.rg.jointHwNum[0],js.rg.someStatus[0],js.rg.msgtyp40_2[0],js.rg.pcktCounter8);
        vector <JointRegisters_t> regFake;
        string reply= parsePayloadComl((char*)&js.rg.regbuff[0],58,regFake,js.msgtyp40_2[0],js.jointHwNum[0]);
        printf(" %s",reply.c_str());
        printf("\r\n");
    }
   
    if ( readyToSend==true)
    {
        rawUDP.sendRawEth(joint.mac,0x464a,(char*)frameJcbStatus.data,66);
    }
    */
    return 1;
}

int AmyEth::sendPollingBroadcastEn(int type_)
{
    static uint16_t pcktCounter=0;
    
    MacAddress dstMac;
    dstMac.insert(dstMac.end(), { 0xff,0xff,0xff,0xff,0xff,0xff });
    
    
    FrameBroadcast84_t frameBroadcast84;
    memset(&frameBroadcast84.data[0],0x00,70); // fill 

    //memcpy(&frameBroadcast84.data[66],&buff[0],4);
    if (type_==0) // all day 
    {
        frameBroadcast84.allday.subAddress[0]=1;
        frameBroadcast84.allday.jointHwNum[0]=0xf8; // hmm ? 
        frameBroadcast84.allday.msgtyp40[0]=0x40;
        frameBroadcast84.allday.msgtyp40_2[0]=0xd0;
        //frameBroadcast84.allday.msgtyp40_2[0]=debugValue4;
        frameBroadcast84.allday.pcktCounter=pcktCounter;
    }
    

    if (type_==1) // enabling Mac
    {
        frameBroadcast84.en.subAddress[0]=1;
        frameBroadcast84.en.jointHwNum[0]=0xf8; // hmm ? 
        frameBroadcast84.en.msgtyp40[0]=0x40;
        //frameJcbWakeup.msgtyp40_2[0]=0x20;
        frameBroadcast84.en.msgtyp40_2[0]=0xd0;
        frameBroadcast84.en.pcktCounter=pcktCounter;
       
        static const uint8_t noMac[6]={};
        auto macOf=[&](int i) { auto &m=jointList->at(i).mac; return m.size()==6 ? m.data() : noMac; };
        frameBroadcast84.en.mac1Id[0]=0x80;
        memcpy(&frameBroadcast84.en.mac1[0],macOf(0),6);
        frameBroadcast84.en.mac2Id[0]=0x81;
        memcpy(&frameBroadcast84.en.mac2[0],macOf(1),6);
        frameBroadcast84.en.mac3Id[0]=0x82;
        memcpy(&frameBroadcast84.en.mac3[0],macOf(2),6);
        frameBroadcast84.en.mac4Id[0]=0x83;
        memcpy(&frameBroadcast84.en.mac4[0],macOf(3),6);
        frameBroadcast84.en.some80[0]=0x80;
    }
  
    
    // count byte 1 = one message + CRC (data[2..65], stored at data[66..69]) like Intera; AMY_NO_BCAST_CRC=1 leaves it 0
    static const bool noCrc = getenv("AMY_NO_BCAST_CRC") != nullptr;
    if (!noCrc) { uint32_t crc=jrcpCrc(&frameBroadcast84.data[2]); memcpy(&frameBroadcast84.data[66], &crc, 4); }
    rawUDP.sendRawEth(dstMac,0x464a,(char*)frameBroadcast84.data,70);
    pcktCounter++;
    return 0;    
}


// Boards already running the app (hot start) don't answer IDENTIFY: take their MACs from the DATABLOCKs saved at the
// last cold start (AmyConfig/datablock/joint<N>_<mac>.gz, already in the jointList order; the newest one per index)
bool AmyEth::InitJointsManual()
{
    bool all=true;
    cout<<"not all boards answered IDENTIFY: hot start with the boards saved at the last cold start\r\n"<<flush;
    for (int a=0; a<4; a++)
    {
        auto joint = &jointList->at(a);
        string pre="joint"+to_string(a)+"_";
        MacAddress mac;
        std::filesystem::file_time_type newest{};
        std::error_code ec;
        for (auto &e: std::filesystem::directory_iterator(datablockDir, ec))
        {
            string n=e.path().filename().string();                      // joint<N>_<12 hex digits>.gz
            if (n.size()!=pre.size()+12+3 || n.rfind(pre,0)!=0 || n.compare(n.size()-3,3,".gz")!=0) continue;
            string hex=n.substr(pre.size(),12);
            if (hex.find_first_not_of("0123456789abcdef")!=string::npos) continue;
            if (!mac.empty() && e.last_write_time()<=newest) continue;
            mac.clear();
            for (int i=0; i<6; i++) mac.push_back((uint8_t)stoi(hex.substr(2*i,2),nullptr,16));
            newest=e.last_write_time();
        }
        joint->mac=mac;
        joint->state=JCB_STATE_HOT_START;
        joint->isGatewayed=(a==3);          // the carpus board relays the Axolotl
        joint->jointHwNum=boardHwNum[a];
        joint->privNo=a;
        if (mac.empty())
        {
            all=false;
            cout<<"hot start: board "<<a<<" ("<<boardType[a]<<"): no saved DATABLOCK - start once with the boards in their "
                  "bootloader (logic power cycled, e.g. --poweron) so AmyCore can identify them\r\n"<<flush;
        }
        else
            cout<<"hot start: board "<<a<<" ("<<boardType[a]<<") "<<macStr(mac.data())<<"\r\n"<<flush;
    }
    return all;
}
