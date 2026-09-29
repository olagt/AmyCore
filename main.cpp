// AmyCore: controller for the Rethink Robotics Sawyer arm, talking directly to the joint controller boards.
// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <csignal>

#include <fstream>


#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include "udp_sender.h"
#include "sawyerFrames.h"
#include "sawyerHelper.h" // Rethink defs
#include "amyEth.h"
#include "jointJCB.h"
#include "build_info.h"

#include "gravity.h"
#include "collision.h"
#include "linkdiag.h"
#include <linux/if_packet.h>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>
#include <algorithm>
#include <deque>


//IMGUI 
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include "console_log.h"
#include <stdio.h>
#include <SDL.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif

//open serial for esp32
#include <unistd.h>
#include <sys/socket.h>      // startup capability check (raw Ethernet socket)
#include <linux/if_ether.h>
#include <arpa/inet.h>
#include <sys/select.h>

//serial port
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


using namespace std;

PacketList registersPacketList;


void initRegistersManualFIX(vector <JointJCB> &jointList)
{
    { // UGLY ! Manual override , should be in all reg aparam csv file   
    JointRegisters_t r;
    r.valSubType=25;
    r.type=JCB_REG_TYPE4304;
    r.regNo=0;//HesOffset0 , 0 i 2 
    r.value=2043; // def is 2048
    jointList[0].registerList.push_back(r);
    r.value=2027;
    jointList[1].registerList.push_back(r);
    r.value=2166;
    jointList[2].registerList.push_back(r);
    r.value=2048;
    jointList[3].registerList.push_back(r);
    
    r.regNo=2;//HesOffset0 , 0 i 2 
    r.value=2043; // def is 2048
    jointList[0].registerList.push_back(r);
    r.value=2027;
    jointList[1].registerList.push_back(r);
    r.value=2166;
    jointList[2].registerList.push_back(r);
    r.value=2048;
    jointList[3].registerList.push_back(r);

    r.value=5;
    r.regNo=8;  //HesLookupTable  8 i 20?
    //r.type=JCB_REG_TYPE4104; // for 8 it has 2 lenght ! checked !
    r.type=JCB_REG_TYPE4302;
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.value=5;
    r.regNo=20;  //HesLookupTable  8 i 20?
    r.type=JCB_REG_TYPE4304;
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.type=JCB_REG_TYPE4304;
//# head ,j0 [0]
// j1 ,j2  [1]
// j3 , j4  [2]
// j5 ,j6  [3]
    r.regNo=11;  //SDSCalibrationSlope , module :6
    r.value=2044; // guses ! nie bylo danych dla glowy
    jointList[0].registerList.push_back(r);
    r.value=5082; //1
    jointList[1].registerList.push_back(r);
    r.value=-2091; //j3
    jointList[2].registerList.push_back(r);
    r.value=651;
    jointList[3].registerList.push_back(r);
    
    r.regNo=23;  //SDSCalibrationSlope , module :6
    r.value=5317; //j0
    jointList[0].registerList.push_back(r);
    r.value=2078;  // j2
    jointList[1].registerList.push_back(r);
    r.value=651; //j4
    jointList[2].registerList.push_back(r);
    r.value=651; //j6 
    jointList[3].registerList.push_back(r);

}
{
    JointRegisters_t r;
    r.valSubType=11;
    r.type=JCB_REG_TYPE0302;
    r.regNo=0;//
    r.value=22; // 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);
    
    r.type=JCB_REG_TYPE032a;
    r.regNo=1;
    r.rawData.clear();
                     //0500110e0402130e04020c0e04020d0e0402004e0402 73
    HexStringToVector("0500110e0402130e04020c0e04020d0e0402004e04020074", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);
    r.regNo=2;
    r.rawData.clear();
    HexStringToVector("0600034e0202024e0402014e040200220202030e04020b0e040200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);
    r.regNo=3;
    r.rawData.clear();
    HexStringToVector("0600050e04020a0e04020e0e04020f0e0402010602020006020200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=4;
    r.rawData.clear();
    HexStringToVector("0700160e0402180e0402140e0402161a02020b1a02020d1a02020a1a020200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=5;
    r.rawData.clear();
    HexStringToVector("07000c1a0202041a0202012a0202070e040210060202094a0402004a020200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=6;
    r.rawData.clear();
    HexStringToVector("0500030604020153040202530402025204020352040200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=8;
    r.rawData.clear();
    HexStringToVector("07000e260202112602020d26020210260202162202020a2604020b26040200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=11;
    r.rawData.clear();
    HexStringToVector("09001b0e02021a0e02021d0e02021c0e02021f0e02021e0e02021206020211060202110e040200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=12;
    r.rawData.clear();
    HexStringToVector("0500130e04020c0e04020d0e0402004e0402034e020200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=13;
    r.rawData.clear();
    HexStringToVector("0500024e0402014e040200220202030e04020b0e040200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=14;
    r.rawData.clear();
    HexStringToVector("0600050e04020a0e04020e0e04020f0e0402010602023206020200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=15;
    r.rawData.clear();
    HexStringToVector("0700160e0402180e0402140e0402171a02020f1a0202111a02020e1a020200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=16;
    r.rawData.clear();
    HexStringToVector("0700101a0202091a0202012a0202070e0402100602023b4a0402324a020200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);

    r.regNo=17;
    r.rawData.clear();
    HexStringToVector("0500030604020353040204530402165204021752040200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);
    
    r.regNo=20;
    r.rawData.clear();
    HexStringToVector("07001d26040223260202262602022b220202052a020222220202010e040200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);
    r.regNo=22;
    r.rawData.clear();
    HexStringToVector("08001b0e02021a0e02021d0e02021c0e02021f0e02021e0e0202120602021106020200", r.rawData); 
    for (int a=0;a<4;a++)jointList[a].registerList.push_back(r);


}

}

int setupAndOpenESP32(string portname)
{    
    // non-blocking: if the ESP32's USB stops taking data, a full buffer must drop commands, not freeze AmyCore
    int serial_port = open(portname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port < 0) 
    {
        cout << "Error while opening device... " << portname<<" errno = " << errno << endl;
        perror("Something went wrong with open()");
        return -1;
    }  
    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;
      // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) 
  {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      close(serial_port);
      return -1;
  }
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 0;    // non-blocking read: return at once with whatever is buffered (never stall the GUI)
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
  {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      close(serial_port);
      return -1;
  }
  else
  {
    cout<<"serial port for ESP 32 opened OK \r\n"<<flush;
  }

    return serial_port;
}

int serial_port=0;

// ---- body board emulation: ESP32 enable wave (see esp32_enable/src/main.cpp for the protocol) ----
// The ESP32 runs the wave only while it gets a line at least every 50 ms, so the keep-alive is sent
// from the main loop: a stalled loop (or a crash) stops the wave.
#define ESP_KEEPALIVE_MS     20    // ESP32 watchdog is 50 ms; fewer USB transfers for the S2's CDC stack
#define ESP_STATUS_MS        500
#define MEAS_WINDOW_MS       100   // ESP32 read-back window (MEAS_PERIOD_MS in the firmware)
#define JOINT_SILENT_MS      100   // any joint quiet this long -> wave off
#define HB_FREQ_HZ           550   // must match SIG_FREQ_HZ in the ESP32 firmware
#define HB_FREQ_TOL_HZ       55    // read-back tolerance
#define HB_CHECK_MS          1500  // read-back must confirm the wave within this time (status comes every 500 ms)
#define HB_MIN_LOW_MS        2000  // joints boot with the wave off: keep it off at least this long before starting it
// torso board power-on timing, from a video of the original robot (wave on 0.966 s after power-on, off at 3.400 s)
#define HB_TEST_ON_MS        966
#define HB_TEST_OFF_MS       3400
#define JOINT_BOOT_WAIT_MS   20000 // after the test pulse, before joint discovery

std::atomic<bool> waveWanted{false}; // AmyCore wants the enable wave running
string waveStopReason="";            // why it was last stopped (shown in the GUI)
string espLastStatus="";             // last "ST ..." line from the ESP32
string espLastMsg="";                // last other reply (OK/ERR/EV)
int    espLastHz=-1;                 // measured wave frequency from the ESP32 read-back, -1 = unknown
bool   hbVerified=false;             // read-back confirmed the running wave
std::chrono::steady_clock::time_point espLastKeepalive, espLastStatusReq, espStatusTime, waveStartTime;
std::chrono::steady_clock::time_point hbOffSince=std::chrono::steady_clock::now();

bool rel24V=false;
bool relAux=false;


std::atomic<int> espWriteFails{0};   // writes the ESP32 port didn't take (buffer full / USB stalled)
std::mutex espWriteMutex;             // espWrite is called from the main loop and the keep-alive thread
void espWrite(const string &cmd)
{
    string line=cmd+"\r";
    std::lock_guard<std::mutex> lock(espWriteMutex);
    if (write(serial_port, line.c_str(), line.size()) != (ssize_t)line.size()) espWriteFails++;
}

// Keep-alive for the ESP32 enable wave, from its own thread so a short GUI stall (window drawing,
// screenshots) doesn't trip the 50 ms firmware watchdog. It only sends while the main loop is alive:
// the main loop stamps mainLoopAliveMs every iteration, and a stamp older than MAIN_LOOP_STALL_MS
// stops the keep-alive, so the ESP32 drops the wave 50 ms later.
#define MAIN_LOOP_STALL_MS   250
#define ESP_DEAD_MS          1500
bool espDead=false;

// ---- automatic recovery when the ESP32's USB serial stalls ----
// The S2 keeps running (its watchdog stops the wave) but stops taking USB data; a USB port reset
// (USBDEVFS_RESET, like re-plugging but without resetting the chip, so the relays keep their state)
// brings it back. Runs in its own thread: it waits for the port to come back (up to seconds) and the
// 1 kHz comm thread must not wait for that. The udev rule gives AmyCore access to the USB device node.
std::atomic<bool> espReconnecting{false};
std::atomic<int>  espReconnects{0};
string espReconnectMsg="";

bool espUsbReset()
{
    char link[64]={0};
    if (readlink("/dev/amy-esp32", link, sizeof(link)-1)<=0) return false;       // e.g. "ttyACM0"
    string dev=string("/sys/class/tty/")+link+"/device/../";                      // tty -> interface -> usb device
    int bus=0, num=0;
    { ifstream f(dev+"busnum"); f>>bus; }
    { ifstream f(dev+"devnum"); f>>num; }
    if (!bus || !num) return false;
    char node[64];
    snprintf(node, sizeof node, "/dev/bus/usb/%03d/%03d", bus, num);
    int fd=open(node, O_WRONLY);
    if (fd<0) { cout<<"ESP32 USB reset: cannot open "<<node<<" ("<<strerror(errno)<<")\r\n"<<flush; return false; }
    int rc=ioctl(fd, USBDEVFS_RESET, 0);
    close(fd);
    cout<<"ESP32 USB reset of "<<node<<(rc==0?" done":" FAILED")<<"\r\n"<<flush;
    return rc==0;
}

void espReconnectStart()
{
    if (espReconnecting.exchange(true)) return;
    std::thread([]
    {
        espReconnects++;
        cout<<"ESP32 not responding: resetting its USB port (attempt "<<espReconnects<<")\r\n"<<flush;
        espUsbReset();
        int fd=-1;
        for (int i=0; i<50 && fd<0; i++)                   // up to 5 s for the port to come back
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (access("/dev/amy-esp32", F_OK)==0) fd=setupAndOpenESP32("/dev/amy-esp32");
        }
        if (fd>=0)
        {
            std::lock_guard<std::mutex> lock(espWriteMutex);
            close(serial_port);
            serial_port=fd;
            espReconnectMsg="ESP32 port reopened after USB reset (#"+to_string(espReconnects.load())+")";
        }
        else espReconnectMsg="ESP32 port did not come back after USB reset - replug its USB";
        cout<<espReconnectMsg<<"\r\n"<<flush;
        std::this_thread::sleep_for(std::chrono::seconds(3));   // let status lines arrive before the next attempt
        espReconnecting=false;
    }).detach();
}
std::atomic<long long> mainLoopAliveMs{0};
long long steadyMs() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
void mainLoopAlive() { mainLoopAliveMs=steadyMs(); }

void espKeepaliveThread()
{
    while (true)
    {
        long long now=steadyMs();
        if (waveWanted && now-mainLoopAliveMs < MAIN_LOOP_STALL_MS) espWrite("K");
        std::this_thread::sleep_for(std::chrono::milliseconds(ESP_KEEPALIVE_MS));
    }
}

// the torso board switched the 24V together with the enable wave, so starting the wave also turns the 24V relay on
void waveEnable(bool on, const string &reason="")
{
    auto now=std::chrono::steady_clock::now();
    if (on)
    {
        waveStopReason="";
        if (!rel24V) { rel24V=true; espWrite("P1"); }
        waveStartTime=now;
        hbVerified=false;
    }
    else
    {
        if (waveWanted || waveStopReason.empty()) waveStopReason=reason;
        if (waveWanted) hbOffSince=now;
    }
    waveWanted=on;
    espWrite(on ? "E1" : "E0");
    espLastKeepalive=now;
}

// ---- AUX relay = robot 7.5V logic supply (joint boards) ----
// Rules, to not damage the robot with frequent power cycles: off at least AUX_MIN_OFF_S before switching
// on again, on at least AUX_MIN_ON_S before switching off. The time of the last switch is kept in a file, so the
// limits also hold across AmyCore restarts. Note: flashing/resetting the ESP32 drops the relay too (too short an off time).
#define AUX_MIN_OFF_S   10
#define AUX_MIN_ON_S    120
#define AUX_STATE_FILE  AMY_ROOT "/AmyConfig/aux_relay_state.txt"
string auxMsg="";

// returns seconds since the last AUX switch (wall clock) and its state from the file; -1 if unknown
long auxSecondsSinceSwitch(bool &lastOn)
{
    ifstream f(AUX_STATE_FILE);
    long long t=0; int on=0;
    if (!(f>>t>>on)) return -1;
    lastOn=on;
    return (long)(time(nullptr)-t);
}

// switch the AUX relay if the timing rules allow it; returns false (and sets auxMsg) if refused
bool auxSet(bool on)
{
    bool lastOn=false;
    long since=auxSecondsSinceSwitch(lastOn);
    if (since>=0)
    {
        if (on && !lastOn && since<AUX_MIN_OFF_S)
        { auxMsg="AUX on refused: logic power off only "+to_string(since)+" s, wait "+to_string(AUX_MIN_OFF_S-since)+" s"; return false; }
        if (!on && lastOn && since<AUX_MIN_ON_S)
        { auxMsg="AUX off refused: logic power on only "+to_string(since)+" s, wait "+to_string(AUX_MIN_ON_S-since)+" s"; return false; }
    }
    espWrite(on ? "A1" : "A0");
    if (since<0 || on!=lastOn)
    {
        ofstream f(AUX_STATE_FILE);
        f<<(long long)time(nullptr)<<" "<<(on?1:0)<<"\n";
    }
    auxMsg=string("AUX relay (robot logic 7.5V) ")+(on?"ON":"OFF");
    cout<<auxMsg<<"\r\n"<<flush;
    return true;
}

bool hbLowLongEnough()
{
    return !waveWanted && std::chrono::steady_clock::now()-hbOffSince >= std::chrono::milliseconds(HB_MIN_LOW_MS);
}

// read everything the ESP32 sent, one line at a time (non-blocking port)
void espPoll()
{
    static string lineBuf;
    char buf[256];
    int n;
    while ((n=read(serial_port, buf, sizeof(buf))) > 0)
    {
        for (int i=0; i<n; i++)
        {
            char c=buf[i];
            if (c!='\r' && c!='\n') { if (lineBuf.size()<200) lineBuf+=c; continue; }
            if (lineBuf.empty()) continue;

            if (lineBuf.rfind("ST ",0)==0)
            {
                espLastStatus=lineBuf;
                espStatusTime=std::chrono::steady_clock::now();
                auto p=lineBuf.find("hz=");
                espLastHz = (p==string::npos) ? -1 : atoi(lineBuf.c_str()+p+3);
                // the relay checkboxes show what the ESP32 reports, not what AmyCore last asked for
                auto q=lineBuf.find("p24="); if (q!=string::npos) rel24V = lineBuf[q+4]=='1';
                q=lineBuf.find("aux=");      if (q!=string::npos) relAux = lineBuf[q+4]=='1';
            }
            else
            {
                espLastMsg=lineBuf;
                if (lineBuf=="EV TRIP" && waveWanted)
                {
                    waveWanted=false;
                    hbOffSince=std::chrono::steady_clock::now();
                    waveStopReason="ESP32 keep-alive timeout (EV TRIP)";
                    cout<<"ESP32: "<<waveStopReason<<"\r\n"<<flush;
                }
                if (lineBuf.rfind("ERR",0)==0 && waveWanted)
                {
                    waveWanted=false;
                    hbOffSince=std::chrono::steady_clock::now();
                    waveStopReason="ESP32: "+lineBuf;
                    cout<<waveStopReason<<"\r\n"<<flush;
                }
            }
            lineBuf.clear();
        }
    }
}

const char *sideName(int board, int side);
// ---- "no frames" diagnosis (linkdiag.cpp): why a board is silent, refreshed by the comm thread every 500 ms ----
string boardDiag[4];                      // "" while the board sends joint frames, else the reason (status file, GUI)
int    boardFrameRate[4]={};              // accepted joint frames per second
long long boardLastFrameMs[4]={};         // steadyMs of its last accepted frame, 0 = never

LinkDiagBoard linkDiagBoardOf(const vector <JointJCB> &jointList, int a)
{
    char n[40]; snprintf(n, sizeof n, "board %d (%s/%s)", a, sideName(a,0), sideName(a,1));
    return {n, jointList[a].mac, jointList[a].ident1};
}
string boardSilentReason(const vector <JointJCB> &jointList, int a)
{
    long long now=steadyMs();
    string r=linkDiagBoard(linkDiagBoardOf(jointList,a), boardLastFrameMs[a] ? now-boardLastFrameMs[a] : -1);
    if (!relAux) r+="; robot logic power (AUX relay) is off";
    return r;
}
string linkDiagFullReport(const vector <JointJCB> &jointList)
{
    vector<LinkDiagBoard> b;
    for (int a=0; a<(int)jointList.size(); a++) b.push_back(linkDiagBoardOf(jointList,a));
    return linkDiagReport(b);
}

void linkDiagTick(const vector <JointJCB> &jointList)
{
    static long long lastTick=0, lastPrint[4]={};
    static int lastCnt[4]={}; static bool wasSilent[4]={};
    long long now=steadyMs();
    if (now-lastTick<500) return;
    long long dt=lastTick ? now-lastTick : 500; lastTick=now;
    for (int a=0; a<4 && a<(int)jointList.size(); a++)
    {
        int cnt=jointList[a].statRecCounter;
        boardFrameRate[a]=(int)((cnt-lastCnt[a])*1000LL/dt);
        if (cnt!=lastCnt[a]) boardLastFrameMs[a]=now;
        lastCnt[a]=cnt;
        bool silent = !boardLastFrameMs[a] || now-boardLastFrameMs[a]>300;
        boardDiag[a] = silent ? boardSilentReason(jointList,a) : "";
        // console: when a board goes silent (after 1 s), every 15 s while it stays silent, and when it is back
        long long silentFor = boardLastFrameMs[a] ? now-boardLastFrameMs[a] : now-lastPrint[a];
        if (silent && (wasSilent[a] ? now-lastPrint[a]>=15000 : silentFor>=1000))
        {
            cout<<"linkdiag: "<<sideName(a,0)<<"/"<<sideName(a,1)<<" board "<<a<<" "<<boardDiag[a]<<"\r\n"<<flush;
            lastPrint[a]=now; wasSilent[a]=true;
        }
        else if (!silent && wasSilent[a])
        {
            cout<<"linkdiag: board "<<a<<" sends joint frames again ("<<boardFrameRate[a]<<"/s)\r\n"<<flush;
            wasSilent[a]=false;
        }
        if (!lastPrint[a]) lastPrint[a]=now;   // start of the "never sent" clock
    }
}

// called every main loop iteration
void bodyBoardTick(vector <JointJCB> &jointList)
{
    auto now=std::chrono::steady_clock::now();
    espPoll();

    // every joint must keep sending while the wave runs
    static vector<int> lastRecCnt;
    static vector<std::chrono::steady_clock::time_point> lastRecTime;
    lastRecCnt.resize(jointList.size(), -1);
    lastRecTime.resize(jointList.size(), now);
    for (size_t a=0; a<jointList.size(); a++)
    {
        if (jointList[a].statRecCounter!=lastRecCnt[a])
        {
            lastRecCnt[a]=jointList[a].statRecCounter;
            lastRecTime[a]=now;
        }
        else if (waveWanted && now-lastRecTime[a] > std::chrono::milliseconds(JOINT_SILENT_MS))
        {
            waveEnable(false, "joint "+to_string(a)+" silent");
            cout<<"enable wave off: "<<waveStopReason<<" - "<<boardSilentReason(jointList,a)<<"\r\n"<<flush;
        }
    }

    // the wave must be seen by the ESP32 read-back (status is requested every ESP_STATUS_MS)
    if (waveWanted && !hbVerified)
    {
        bool fresh = espStatusTime > waveStartTime+std::chrono::milliseconds(2*MEAS_WINDOW_MS);
        if (fresh && abs(espLastHz-HB_FREQ_HZ) <= HB_FREQ_TOL_HZ)
            hbVerified=true;
        else if (now-waveStartTime > std::chrono::milliseconds(HB_CHECK_MS))
        {
            waveEnable(false, "HB read-back failed (hz="+to_string(espLastHz)+")");
            cout<<"enable wave off: "<<waveStopReason<<"\r\n"<<flush;
        }
    }
    else if (waveWanted && abs(espLastHz-HB_FREQ_HZ) > HB_FREQ_TOL_HZ) // e.g. 24V switched off from the GUI
    {
        waveEnable(false, "HB lost (hz="+to_string(espLastHz)+")");
        cout<<"enable wave off: "<<waveStopReason<<"\r\n"<<flush;
    }

    if (now-espLastStatusReq >= std::chrono::milliseconds(ESP_STATUS_MS))
    {
        espWrite("S");
        espLastStatusReq=now;
    }
    // no status line for ESP_DEAD_MS: the ESP32 (or its USB serial) is not responding
    static auto started=now;
    auto lastSeen = espStatusTime > started ? espStatusTime : started;
    espDead = now-lastSeen > std::chrono::milliseconds(ESP_DEAD_MS);
    if (espDead) espReconnectStart();
    if (espDead && waveWanted)
    {
        waveEnable(false, "ESP32 not responding (no status for 1 s, "+to_string(espWriteFails.load())+" writes dropped) - replug its USB");
        cout<<"enable wave off: "<<waveStopReason<<"\r\n"<<flush;
    }
}


// ---- arm enable sequence, copied from Intera (rethink.log 2023-09-06 09:38 + a 2023 boot capture) ----
// resume pulse 1.5 s -> wait for motor power (undervoltage clears) -> reset latched errors 40 ms ->
// ENABLE | MANUAL_HOME_AND_UNLOCK_REQUEST one joint side every 100 ms, j6 first, head pan last -> joints home.
// Joint command flags are bytes 20-21 of the command frame (LE16): low byte = high byte of Cmd2Joint,
// high byte = CmdBrake2Joint. Global flags are bytes 44-47 (LE32): byte 45 = debugIntValue2 (nav LEDs),
// byte 47 = debugIntValue1 (0x20 = SAFETY_CTRLR_RESUME).
#define JF_ENABLE              0x0001
#define JF_RESET_LATCHED       0x0002
#define JF_MANUAL_HOME_UNLOCK  0x0200
#define JF_HEAD_PAN            0x0400
#define JS_ENABLED             0x02   // CRSP_JOINT_FLAG_ENABLED
#define JS_HOMED               0x10   // CRSP_JOINT_FLAG_IS_HOMED_AND_UNLOCKED
#define GLOB_UNDERVOLTAGE      0x02
#define ARM_RESUME_MS          1500
#define ARM_POWER_TIMEOUT_MS   5000
#define ARM_RESET_MS           40
#define ARM_STAGGER_MS         100
#define ARM_ENABLE_CHECK_MS    500    // a side must report ENABLED this long after its ENABLE

enum ArmStep { ARM_OFF, ARM_RESUME, ARM_WAIT_POWER, ARM_RESET, ARM_ENABLING, ARM_ENABLED, ARM_RETRY_RESET, ARM_RETRY_ENABLE };
const char *armStepName[]={"off","resume pulse","waiting for motor power","reset latched errors","enabling joints","enabled","retry: reset latched errors","retry: enabling"};
ArmStep armStep=ARM_OFF;
string  armMsg="";
std::chrono::steady_clock::time_point armStepStart;
int     armSidesEnabled=0;                  // how many entries of armOrder were handled so far (enabled or skipped)
int     armLastEnabled=-1;                  // armOrder index of the last side that really got ENABLE
bool    armSkip[4][2]={};                   // sides left disabled (HES_CONFIGURATION_ERROR with the ignore box ticked)
int     sideIgnoreErr[4][2]={};             // error bits masked on the board (JointErrorOverrideMask, "mask" command): not counted
// robot joint name of a board side (board 0 side 0 is the head pan)
const char *sideName(int board, int side)
{
    static const char *n[4][2]={{"head_pan","j0"},{"j1","j2"},{"j3","j4"},{"j5","j6"}};
    return (board>=0 && board<4 && side>=0 && side<2) ? n[board][side] : "?";
}
bool    armIgnoreHesCfg=false;              // GUI: let "Enable arm" run although sides report HES_CONFIGURATION_ERROR
bool    armContinuePastFail=false;          // GUI: a side that refuses ENABLE is left disabled and the sequence goes on
string  armRefused;                         // sides that refused ENABLE in this run
#define JE_HES_CONFIG          0x0100       // CRSP_JOINT_ERROR_HES_CONFIGURATION_ERROR
#define JE_LATCHED             0x0001       // CRSP_JOINT_ERROR_LATCHED
int armErrMask() { return armIgnoreHesCfg ? ~JE_HES_CONFIG : ~0; }

// jointList index 3 = carpus (j5/j6) ... 0 = scapula (head pan side 0, j0 side 1); side 1 = index 1 = second joint
static const int armOrder[8][2]={{3,1},{3,0},{2,1},{2,0},{1,1},{1,0},{0,1},{0,0}};

void setJointFlags(JointJCB &j, int side, uint16_t f)
{
    if (side==0) { j.Cmd2Joint1=(f&0xff)<<8; j.CmdBrake2Joint1=f>>8; }
    else         { j.Cmd2Joint2=(f&0xff)<<8; j.CmdBrake2Joint2=f>>8; }
}
uint16_t sideBaseFlags(int joint, int side) { return (joint==0 && side==0) ? JF_HEAD_PAN : 0; }

void armGoto(ArmStep s, const string &msg="")
{
    armStep=s; armStepStart=std::chrono::steady_clock::now();
    if (!msg.empty()) armMsg=msg;
    cout<<"arm: "<<armStepName[s]<<(msg.empty()?"":" - "+msg)<<"\r\n"<<flush;
}

void armDisable(vector <JointJCB> &jointList, const string &why)
{
    for (auto &j:jointList) { setJointFlags(j,0,0); setJointFlags(j,1,0); j.debugIntValue1=0; }
    armSidesEnabled=0;
    armGoto(ARM_OFF, why);
}

void armEnableStart(vector <JointJCB> &jointList)
{
    for (auto &j:jointList)
        if (j.state!=JCB_STATE_WORKING_22) { armMsg="not started: joint "+to_string(j.privNo)+" not running"; return; }
    if (!waveWanted || !hbVerified) { armMsg="not started: HB (square wave) not running/verified"; return; }
    string skipped;
    for (int a=0; a<(int)jointList.size() && a<4; a++)
        for (int s=0; s<2; s++)
        {
            int err=(uint16_t)(s==0 ? jointList[a].jointVF.vf.jErFlags1 : jointList[a].jointVF.vf.jErFlags2) & ~sideIgnoreErr[a][s];
            // a LATCHED bit (0x1) is cleared by the reset step, so only the other bits count here
            // with "continue" ticked (test setup) any side with an error is skipped instead of refusing the start;
            // "Retry refused/faulted" can try it later
            armSkip[a][s] = (armIgnoreHesCfg && (err & JE_HES_CONFIG)) || (armContinuePastFail && (err & ~JE_LATCHED));
            if (armSkip[a][s]) { char b[16]; snprintf(b,sizeof b,"(0x%x)",err); skipped+=string(" ")+sideName(a,s)+b; continue; }
            if (err & ~JE_LATCHED)
            {
                char b[140];
                snprintf(b,sizeof b,"not started: %s has errors 0x%x (256 = HES_CONFIGURATION_ERROR)", sideName(a,s), err);
                armMsg=b; return;
            }
        }
    for (int a=0; a<(int)jointList.size(); a++)
        for (int s=0; s<2; s++) setJointFlags(jointList[a], s, JF_MANUAL_HOME_UNLOCK|sideBaseFlags(a,s));
    for (auto &j:jointList) j.debugIntValue1=0x20;          // SAFETY_CTRLR_RESUME
    armSidesEnabled=0; armLastEnabled=-1; armRefused="";
    armGoto(ARM_RESUME, skipped.empty() ? "" : "skipping (error, stay disabled):"+skipped);
}

// "Retry refused/faulted sides": reset latched errors on them, then ENABLE | home/unlock one by one, 100 ms apart
std::vector<std::pair<int,int>> armRetryQueue;
void armRetryStart(vector <JointJCB> &jointList)
{
    armRetryQueue.clear();
    for (int i=0; i<8; i++)
    {
        int a=armOrder[i][0], s=armOrder[i][1];
        if (armSkip[a][s]) { armRetryQueue.push_back({a,s}); setJointFlags(jointList[a], s, JF_MANUAL_HOME_UNLOCK|JF_RESET_LATCHED|sideBaseFlags(a,s)); }
    }
    if (armRetryQueue.empty()) { armMsg="nothing to retry"; return; }
    armRefused="";
    armGoto(ARM_RETRY_RESET, "retrying "+to_string(armRetryQueue.size())+" sides");
}

// ---- per-side commands (GUI "Joint control" window) ----
#define JF_AUTO_HOME_UNLOCK    0x0100
#define JF_TARE                0x0004   // CCMD_JOINT_FLAG_TARE: zero the torque (deflection) sensor
#define JF_HEAD_NOD            0x0008   // CCMD_JOINT_FLAG_DO_HEAD_NOD (head pan board)
bool      sideManualUsed=false;                 // a per-side command was sent: always send the full command frame
long long sideResetUntilMs[4][2]={};            // flag pulse (reset/tare/nod) running until this time (0 = none)
uint16_t  sidePulseFlag[4][2]={};               // the flag that pulse sets
// "go to position": a P controller on top of the jog offset (the joint follows position + offset), see gotoTick
double    gotoTarget[4][2]={};                  // counts
bool      gotoActive[4][2]={};
long long gotoStartMs[4][2]={};                 // for the go-to acceleration ramp
string    gotoMsg[4][2];                  // result of the last "go to", shown in "Joint detail"
long long gotoProgressMs[4][2]={};
int       gotoLastPos[4][2]={};

uint16_t getJointFlags(JointJCB &j, int side)
{
    return side==0 ? (uint16_t)(((j.Cmd2Joint1>>8)&0xff) | (j.CmdBrake2Joint1<<8))
                   : (uint16_t)(((j.Cmd2Joint2>>8)&0xff) | (j.CmdBrake2Joint2<<8));
}

long long headEnableAtMs=0;                     // head pan: when to add ENABLE after the 0x0600 step (sideTick)
const int HEAD_ENABLE_DELAY_MS=700;
void sideCommand(vector <JointJCB> &jointList, int a, int s, const string &what)
{
    auto &j=jointList[a];
    uint16_t f=getJointFlags(j,s);
    sideManualUsed=true;
    if (what=="reset" || what=="tare" || what=="nod")
    {
        uint16_t pf = what=="reset" ? JF_RESET_LATCHED : what=="tare" ? JF_TARE : JF_HEAD_NOD;
        setJointFlags(j, s, f|pf);
        sidePulseFlag[a][s]=pf;
        sideResetUntilMs[a][s]=steadyMs()+(what=="nod" ? 200 : ARM_RESET_MS);
    }
    else if ((what=="home" || what=="enable") && a==0 && s==0)
    {   // head pan like Intera (2023 capture): head-pan request WITHOUT enable first (0x0600 for ~0.7 s), then 0x0601;
        // jumping straight to 0x0401 leaves IS_HEAD_PAN_ENABLED (0x40) off after the head switched itself off
        setJointFlags(j, s, JF_MANUAL_HOME_UNLOCK|JF_HEAD_PAN);
        headEnableAtMs = steadyMs()+HEAD_ENABLE_DELAY_MS; armSkip[a][s]=false;
    }
    else if (what=="home")      { setJointFlags(j, s, JF_ENABLE|JF_MANUAL_HOME_UNLOCK|sideBaseFlags(a,s)); armSkip[a][s]=false; }
    else if (what=="autohome")  { setJointFlags(j, s, JF_ENABLE|JF_AUTO_HOME_UNLOCK|sideBaseFlags(a,s)); armSkip[a][s]=false; }
    else if (what=="disable")   { setJointFlags(j, s, JF_MANUAL_HOME_UNLOCK); armSkip[a][s]=true; }
    else if (what=="enable")    { setJointFlags(j, s, JF_ENABLE|sideBaseFlags(a,s)); armSkip[a][s]=false; }   // no home request
    cout<<"side command: "<<sideName(a,s)<<" "<<what<<" (flags now 0x"<<hex<<getJointFlags(j,s)<<dec<<")\r\n"<<flush;
}

void sideTick(vector <JointJCB> &jointList)
{
    long long now=steadyMs();
    if (headEnableAtMs && now>=headEnableAtMs && !jointList.empty())
    {
        headEnableAtMs=0;
        setJointFlags(jointList[0], 0, JF_ENABLE|JF_MANUAL_HOME_UNLOCK|JF_HEAD_PAN);   // 0x0601; dropped to 0x0401 once HOMED
        printf("side head_pan: ENABLE after the head-pan request (flags 0x%04x)\n", getJointFlags(jointList[0],0));
    }
    for (int a=0; a<4 && a<(int)jointList.size(); a++)
        for (int s=0; s<2; s++)
        {
            // like Intera: once a side reports HOMED, drop the home/unlock request and keep ENABLE (0x0001, head pan
            // 0x0401). While the request stays set the joint ignores position/jog commands.
            auto &j=jointList[a];
            uint16_t f=getJointFlags(j,s);
            int st = s==0 ? j.jointVF.vf.stFlags1 : j.jointVF.vf.stFlags2;
            if ((f&JF_ENABLE) && (f&(JF_MANUAL_HOME_UNLOCK|JF_AUTO_HOME_UNLOCK)) && (st&JS_HOMED) && !sideResetUntilMs[a][s])
            {
                setJointFlags(j, s, JF_ENABLE|sideBaseFlags(a,s));
                printf("side %s homed: flags 0x%04x\n", sideName(a,s), getJointFlags(j,s));
            }
            if (sideResetUntilMs[a][s] && now>=sideResetUntilMs[a][s])
            {
                setJointFlags(jointList[a], s, getJointFlags(jointList[a],s) & ~(sidePulseFlag[a][s] ? sidePulseFlag[a][s] : JF_RESET_LATCHED));
                sideResetUntilMs[a][s]=0; sidePulseFlag[a][s]=0;
            }
        }
}

// ---- command API: pipe /tmp/amycore.cmd, status file /tmp/amycore.status ----
// Everything needed to run the robot without the GUI (AmyCore --headless) goes through here; tools/amyctl wraps it.
// one command per line:  start (startup sequence: joints -> 0x22, then HB)   config (CONFIG phase)   autohome all
//                        arm | arm off | arm skip <side...>   wave on|off   p24 on|off   aux on|off   quit
//                        setreg <side> <mod> <reg> <size> <val> | mask <side> <hex> | readreg <side> <mod> <reg> <size>
//                        regdump <board 0-3|all> <mod> <first reg> <last reg> <size>  (replies in /tmp/amycore.regs)
//                        reset|home|autohome|enable|disable|tare|nod <side>   jog <side> <value> <ms>   goto <side> <deg>
//                        status [side]   stop (all jogs and gotos)   linkdiag (why boards send no frames)
//                        teach on|off (manual teaching)   stiffness <side>|all <0-100>
// <side> = head_pan, j0 ... j6. A jog value is re-applied every frame as an offset to the current position (the
// joint keeps moving while it is nonzero), so jog always has a duration and returns to 0 by itself.
// Replies go to stdout (lines starting with "cmd:" / "status"); /tmp/amycore.status is rewritten every 200 ms
// with the overall state and every side, so a script can simply read it.
#define CMD_PIPE    "/tmp/amycore.cmd"
#define STATUS_FILE "/tmp/amycore.status"
int       cmdPipeFd=-1;
long long jogUntilMs[4][2]={};
bool      jogFromGui[4][2]={};         // jog held in the "Joints" window: re-armed every GUI frame, auto-zeroed quietly
bool      startSequence=false;         // startup state machine request (GUI button "startSequence", "start")
int       startStateNow=0;             // its current state, for the status file
std::atomic<bool> appQuit{false};      // "quit" command / Ctrl-C in headless mode
bool      autoConfig=false;            // --auto: start sequence, then CONFIG, without the GUI
bool      autoArm=false;               // --arm:  after CONFIG, run "Enable arm" (no homing: joints are homed one by one by hand)
// "autohome all": AUTO_HOME (the joint searches its absolute position by itself, ~2 s) one side at a time.
// MANUAL_HOME (what Intera and "Enable arm" request) has so far never started by itself on this robot (in 2023 it did,
// 27-43 s after the enable; the missing condition is unknown). It completed once when the joint was moved by hand.
vector<pair<int,int>> homeQueue;
long long homeSideStartMs=0;
long long homeNextMs=0;                // "home all" waits HOME_PAUSE_MS after each homed joint
const int HOME_PAUSE_MS=3000;
void homeAllStart(vector <JointJCB> &jointList);
void cfgStart(vector <JointJCB> &jointList);
const char *cfgStateName();
extern string cfgMsg;

bool sideByName(const string &n, int &a, int &s)
{
    for (a=0; a<4; a++) for (s=0; s<2; s++) if (n==sideName(a,s)) return true;
    return false;
}

string sideStatusLine(vector <JointJCB> &jointList, int a, int s)
{
    auto &j=jointList[a];
    int st  = s==0 ? j.jointVF.vf.stFlags1 : j.jointVF.vf.stFlags2;
    int err = (uint16_t)(s==0 ? j.jointVF.vf.jErFlags1 : j.jointVF.vf.jErFlags2);
    int pos = s==0 ? j.jointVF.vf.encoder1 : j.jointVF.vf.encoder2;
    int eff = s==0 ? j.jointVF.vf.effort1 : j.jointVF.vf.effort2;
    int jog = s==0 ? j.jogMoveSP1 : j.jogMoveSP2;
    bool hold = s==0 ? j.hold1 : j.hold2;
    int target = s==0 ? j.holdSP1 : j.holdSP2;
    char b[240];
    snprintf(b, sizeof b, "%-8s st %3d%s%s%s err 0x%04x glob 0x%02x pos %10d effort %6d flags 0x%04x jog %d", sideName(a,s), st,
             (st&JS_ENABLED)?" ENABLED":"", (st&JS_HOMED)?" HOMED":"", (st&32)?" homing":"", err,
             j.jointVF.vf.globalErrFlag, pos, eff, getJointFlags(j,s), jog);
    string line=b;
    line += hold ? " hold "+to_string(target) : " hold -";     // held target (holdSetpoint), "-" = not holding
    line += " ff "+to_string(s==0 ? j.gravityFF1 : j.gravityFF2);  // gravity feed-forward sent (1/250 Nm)
    line += " stiff "+to_string(s==0 ? j.stiff1 : j.stiff2)+" mode "+to_string(s==0 ? j.ctrlMode1 : j.ctrlMode2);  // sent (payload 14/18)
    return line;
}

void printSideStatus(vector <JointJCB> &jointList, int a, int s)
{
    printf("status %s\n", sideStatusLine(jointList,a,s).c_str());
}

void homeAllStart(vector <JointJCB> &jointList)
{
    homeQueue.clear(); homeSideStartMs=0;
    for (int i=0; i<8; i++)       // same order as "Enable arm": j6 first, head_pan last
    {
        int a=armOrder[i][0], s=armOrder[i][1];
        auto &j=jointList[a];
        int st = s==0 ? j.jointVF.vf.stFlags1 : j.jointVF.vf.stFlags2;
        if ((st & JS_ENABLED) && !(st & JS_HOMED)) homeQueue.push_back({a,s});
    }
    printf("cmd: autohome all: %d sides to home\n", (int)homeQueue.size());
}

void homeAllTick(vector <JointJCB> &jointList, long long now)
{
    if (homeQueue.empty()) return;
    auto [a,s]=homeQueue.front();
    auto &j=jointList[a];
    int st = s==0 ? j.jointVF.vf.stFlags1 : j.jointVF.vf.stFlags2;
    if (homeSideStartMs==0)
    {
        if (now<homeNextMs) return;                      // pause between joints
        sideCommand(jointList,a,s,"autohome"); homeSideStartMs=now; printf("autohome: %s\n", sideName(a,s)); return;
    }
    bool done=(st & JS_HOMED), timeout=now-homeSideStartMs>10000, lost=!(st & JS_ENABLED);
    if (!done && !timeout && !lost) return;
    if (!done)                                           // stop the whole sequence, don't go on to the next joint
    {
        printf("autohome: %s %s, home all STOPPED\n", sideName(a,s), lost ? "no longer enabled" : "not homed after 10 s");
        homeQueue.clear(); homeSideStartMs=0; return;
    }
    printf("autohome: %s HOMED, next joint in %d s\n", sideName(a,s), HOME_PAUSE_MS/1000);
    homeQueue.erase(homeQueue.begin()); homeSideStartMs=0; homeNextMs=now+HOME_PAUSE_MS;
}

const double COUNTS_PER_DEG = 1024.0*1000.0*M_PI/180.0;   // position unit 1/1024 mrad
int gotoMaxJog=1500;                                         // speed limit of "go to" (jog offset)
// home posture: AmyConfig/home_pose.txt (<joint> <degrees> per line, default Intera's neutral pose), read at every use;
// "homepose all" goes through the joints in the file's order, one at a time
#define HOME_POSE_FILE AMY_ROOT "/AmyConfig/home_pose.txt"
struct HomePoseEntry { int a, s; double deg; };
vector<HomePoseEntry> loadHomePose()
{
    vector<HomePoseEntry> out;
    ifstream f(HOME_POSE_FILE);
    string line;
    while (getline(f, line))
    {
        auto h=line.find('#'); if (h!=string::npos) line=line.substr(0,h);
        istringstream in(line); string name; double deg;
        int a, s;
        if (in>>name>>deg && sideByName(name,a,s)) out.push_back({a,s,deg});
    }
    return out;
}
bool homePoseDeg(int a, int s, double &deg)
{
    for (auto &e:loadHomePose()) if (e.a==a && e.s==s) { deg=e.deg; return true; }
    return false;
}
// ---- coordinated, collision-checked moves: "Go to home posture", "Go to", "Go to home" ----
// Like Intera's move_to_neutral() (intera_sdk limb.py): all moving joints together on a straight joint-space path, all
// arriving at the same time, slow (speed_deg_s for the joint with the longest way), with acceleration and braking.
// Before starting, a worker thread checks the whole path in step_deg steps against the link meshes and the table plane
// (collision.cpp); any collision -> refused. During the move the held targets (holdSetpoint) follow the path; a joint
// falling more than max_lag_deg behind, losing ENABLED/HOMED or reporting an error stops everything where it is.
// Every joint must be ENABLED + HOMED + holding (otherwise the pose, and so the collision check, is unknown).
#define COLLISION_FILE   AMY_ROOT "/AmyConfig/collision.txt"
#define MESH_PACKAGE_DIR AMY_ROOT "/thirdparty/sawyer_robot"
struct PairMargin { string a, b; double m; };
struct MoveCfg { double margin=0.02, tableZ=-0.10, speed=5, accel=1.5, maxLag=2, step=0.5; bool table=true; vector<PairMargin> pairs;
                 bool ignore[GRAVITY_JOINTS]={};                 // ignore_joints: not moved, not required, not collision-checked
                 struct Obstacle { string name; int type; double c[3], d[3]; }; vector<Obstacle> obstacles;
                 double contactNm[GRAVITY_JOINTS]={0, 4, 5, 4, 4, 2.5, 2, 2}; } moveCfg;   // contact thresholds (head: off)
bool   collisionReady=false;
string collisionMsg="";
std::mutex collisionMutex;                       // collisionFree is not thread-safe (worker thread + pipe command)
enum MoveState { MV_IDLE, MV_CHECKING, MV_RUNNING, MV_SETTLING };
const char *moveStateName[]={"idle","checking path","moving","settling"};
struct Move
{
    MoveState st=MV_IDLE;
    double q0[GRAVITY_JOINTS]={}, q1[GRAVITY_JOINTS]={};   // start and goal [rad], gravity.h order = board*2+side
    bool   use[GRAVITY_JOINTS]={};
    double D=0, T=0, Ta=0, acc=0, vmax=0;                  // longest way [deg], total time, ramp time [s], accel, speed
    long long t0=0, settleMs=0;
    string label, msg;
    std::thread worker; std::atomic<int> check{0};          // 0 running, 1 path free, 2 collision
    string checkMsg; bool cancel=false;
} mv;

void loadMoveCfg()
{
    ifstream f(COLLISION_FILE); string line;
    while (getline(f, line))
    {
        auto h=line.find('#'); if (h!=string::npos) line=line.substr(0,h);
        istringstream in(line); string k, v; if (!(in>>k>>v)) continue;
        if (k=="pair_margin_m") { string b; double mm; if (in>>b>>mm) moveCfg.pairs.push_back({v,b,mm}); continue; }
        if (k=="obstacle")          // obstacle <name> box <x> <y> <z> <sx> <sy> <sz> | cylinder <x> <y> <z> <r> <len> | sphere <x> <y> <z> <r>
        {
            MoveCfg::Obstacle o; o.name=v; string t; in>>t;
            o.type = t=="box" ? 0 : t=="cylinder" ? 1 : t=="sphere" ? 2 : -1;
            int nd = o.type==0 ? 3 : o.type==1 ? 2 : 1;
            bool ok = o.type>=0 && (bool)(in>>o.c[0]>>o.c[1]>>o.c[2]);
            for (int i=0; i<3; i++) o.d[i]=0;
            for (int i=0; ok && i<nd; i++) ok = (bool)(in>>o.d[i]);
            if (ok) moveCfg.obstacles.push_back(o); else printf("collision.txt: bad obstacle line for '%s'\n", v.c_str());
            continue;
        }
        if (k=="contact_nm")        // contact_nm <j0> <j1> ... <j6>: torque change [Nm] that counts as contact (0 = off)
        {
            moveCfg.contactNm[1]=atof(v.c_str());
            for (int i=2; i<GRAVITY_JOINTS; i++) { double x; if (in>>x) moveCfg.contactNm[i]=x; }
            continue;
        }
        if (k=="ignore_joints")
        {
            for (string n=v; !n.empty(); n.clear(), in>>n) { int a, sd; if (sideByName(n,a,sd)) moveCfg.ignore[a*2+sd]=true; else printf("collision.txt: unknown joint '%s'\n", n.c_str()); }
            continue;
        }
        if (k=="table_z_m") { if (v=="none") moveCfg.table=false; else { moveCfg.table=true; moveCfg.tableZ=atof(v.c_str()); } }
        else if (k=="margin_m")    moveCfg.margin=atof(v.c_str());
        else if (k=="speed_deg_s") moveCfg.speed=std::clamp(atof(v.c_str()), 0.5, 20.0);
        else if (k=="accel_s")     moveCfg.accel=std::clamp(atof(v.c_str()), 0.2, 5.0);
        else if (k=="max_lag_deg") moveCfg.maxLag=std::clamp(atof(v.c_str()), 0.5, 5.0);
        else if (k=="step_deg")    moveCfg.step=std::clamp(atof(v.c_str()), 0.1, 2.0);
    }
}

// check the current pose (pipe "collision"), result as text
string collisionCheckNow(vector <JointJCB> &jointList)
{
    if (!collisionReady) return "collision model not loaded: "+collisionMsg;
    double q[GRAVITY_JOINTS];
    for (int i=0; i<GRAVITY_JOINTS; i++) { auto &v=jointList[i/2].jointVF.vf; q[i]=(i%2==0 ? v.encoder1 : v.encoder2)/(1024.0*1000.0); }
    char info[512]=""; bool free;
    { std::lock_guard<std::mutex> l(collisionMutex); free=collisionFree(q, info, sizeof info); }
    return free ? "current pose: no collision" : string("current pose COLLIDES: ")+info;
}

void moveSetMsg(const string &m)
{
    mv.msg=m; printf("move: %s\n", m.c_str());
    for (int i=0; i<GRAVITY_JOINTS; i++) if (mv.use[i]) gotoMsg[i/2][i%2]="last go to: "+m;
}

// ---- contact detection (like Intera's collision detection): measured joint torque vs the gravity model ----
// effort (CRsp) is in the same unit as the feed-forward, 1/250 Nm (it matches 250 x Pinocchio gravity within ~1.2 Nm at
// rest). When a move/jog starts, each joint's difference is stored as baseline (static sensor offsets, a contact that
// already exists); the motion stops when a joint's difference changes by more than contact_nm for CONTACT_MS.
const int CONTACT_MS=20;
extern double gravityTau[4][2];
extern bool   gravityPoseOk;
extern bool   gravityReady;
double contactBase[GRAVITY_JOINTS]={}; int contactOverMs[GRAVITY_JOINTS]={}; bool contactArmed=false;
double contactResidual(vector <JointJCB> &jointList, int i)
{
    auto &v=jointList[i/2].jointVF.vf;
    return (i%2==0 ? v.effort1 : v.effort2)/250.0 - gravityTau[i/2][i%2];
}
void contactReset(vector <JointJCB> &jointList)
{
    contactArmed = gravityReady && gravityPoseOk;
    for (int i=0; i<GRAVITY_JOINTS; i++) { contactBase[i] = contactArmed ? contactResidual(jointList,i) : 0; contactOverMs[i]=0; }
}
// "" or a description of the contact
string contactCheck(vector <JointJCB> &jointList)
{
    if (!contactArmed || !gravityPoseOk) return "";
    for (int i=1; i<GRAVITY_JOINTS; i++)
    {
        if (moveCfg.contactNm[i]<=0 || moveCfg.ignore[i]) continue;
        double dev = contactResidual(jointList,i)-contactBase[i];
        if (fabs(dev)>moveCfg.contactNm[i]) { if (++contactOverMs[i]>=CONTACT_MS) { char b[140]; snprintf(b, sizeof b, "CONTACT on %s: torque changed by %.1f Nm (limit %.1f)", sideName(i/2,i%2), dev, moveCfg.contactNm[i]); return b; } }
        else contactOverMs[i]=0;
    }
    return "";
}

// freeze every joint where it is (the held target = current position)
void moveStop(vector <JointJCB> &jointList, const string &why)
{
    if (mv.st==MV_IDLE) return;
    if (mv.st==MV_CHECKING) { mv.cancel=true; moveSetMsg("cancelled ("+why+")"); return; }   // worker finishes, then dropped
    for (auto &j:jointList) { j.holdSP1=j.encoderPos1; j.holdSP2=j.encoderPos2; j.jogMoveSP1=0; j.jogMoveSP2=0; }
    mv.st=MV_IDLE;
    moveSetMsg("STOPPED, all joints hold where they are: "+why);
}

bool teachBusy();          // manual teaching running (teach section below)
bool teachFloating();      // the arm floats (called with commMutex held)
void teachHalt(const string &why);   // end manual teaching at the next 1 ms step (commMutex held)
extern bool teachBiasOn; extern double teachBias[4][2];

// start a coordinated move to target[] [rad] for the joints with use[i]; false + why if refused
bool moveStart(vector <JointJCB> &jointList, const double target[GRAVITY_JOINTS], const bool use[GRAVITY_JOINTS], const string &label, string &why)
{
    if (teachBusy()) { why="manual teaching is on"; return false; }
    if (!collisionReady) { why="collision model not loaded ("+collisionMsg+")"; return false; }
    if (mv.st!=MV_IDLE) { why=string("another move is ")+moveStateName[mv.st]; return false; }
    if (jointList.size()<4) { why="not all boards connected"; return false; }
    for (int i=0; i<GRAVITY_JOINTS; i++)
    {
        int a=i/2, s=i%2; auto &j=jointList[a]; auto &v=j.jointVF.vf;
        bool crsp=(j.jointVF.msgtyp40_2[0]&0x0f)==2;
        uint8_t st = s==0 ? v.stFlags1 : v.stFlags2;
        double deg = (s==0 ? v.encoder1 : v.encoder2)/COUNTS_PER_DEG;
        if (moveCfg.ignore[i])          // not moved, not required; its reading if it is plausible, else 0
        {
            bool ok = crsp && (st&JS_HOMED) && deg>=jointLimitDeg[a][s][0] && deg<=jointLimitDeg[a][s][1];
            mv.q0[i] = mv.q1[i] = ok ? deg*M_PI/180 : 0.0; mv.use[i]=false;
            if (use[i]) { why=string(sideName(a,s))+" is ignored (ignore_joints in collision.txt)"; return false; }
            continue;
        }
        if (!crsp || !(st&JS_ENABLED) || !(st&JS_HOMED) || !(s==0 ? j.hold1 : j.hold2))
        { why=string(sideName(a,s))+" is not ENABLED + HOMED + holding (every joint must be, so the pose is known)"; return false; }
        if (s==0 ? j.wrapLock1 : j.wrapLock2) { why=string(sideName(a,s))+" has a wrap lock: disable, turn back by hand, enable"; return false; }
        if (deg<jointLimitDeg[a][s][0] || deg>jointLimitDeg[a][s][1])
        { char b[120]; snprintf(b, sizeof b, "%s reads %.1f deg, outside its range (maybe wrapped)", sideName(a,s), deg); why=b; return false; }
        mv.q0[i] = (s==0 ? j.holdSP1 : j.holdSP2)/(1024.0*1000.0);
        mv.use[i] = use[i];
        mv.q1[i] = use[i] ? target[i] : mv.q0[i];
        if (use[i])
        {
            double t=target[i]*180/M_PI, lo=jointLimitDeg[a][s][0]+LIMIT_MARGIN_DEG, hi=jointLimitDeg[a][s][1]-LIMIT_MARGIN_DEG;
            if (t<lo || t>hi) { char b[140]; snprintf(b, sizeof b, "%s goal %.1f deg is outside %.1f .. %.1f deg", sideName(a,s), t, lo, hi); why=b; return false; }
        }
    }
    mv.D=0; for (int i=0; i<GRAVITY_JOINTS; i++) mv.D=std::max(mv.D, fabs(mv.q1[i]-mv.q0[i])*180/M_PI);
    if (mv.D<0.05) { why="already there"; return false; }
    // trapezoidal profile of the longest way: accelerate in accel_s to speed_deg_s, brake the same; triangular if short
    mv.vmax=moveCfg.speed; mv.acc=mv.vmax/moveCfg.accel;
    if (mv.D < mv.vmax*moveCfg.accel) { mv.Ta=sqrt(mv.D/mv.acc); mv.T=2*mv.Ta; }
    else                              { mv.Ta=moveCfg.accel;     mv.T=mv.D/mv.vmax+mv.Ta; }
    mv.label=label; mv.cancel=false; mv.check=0; mv.checkMsg="";
    mv.st=MV_CHECKING;
    if (mv.worker.joinable()) mv.worker.join();
    double q0[GRAVITY_JOINTS], q1[GRAVITY_JOINTS]; memcpy(q0,mv.q0,sizeof q0); memcpy(q1,mv.q1,sizeof q1);
    int n=std::max(1,(int)ceil(mv.D/moveCfg.step));
    mv.worker=std::thread([q0,q1,n]()
    {
        for (int k=0; k<=n; k++)
        {
            double f=(double)k/n, q[GRAVITY_JOINTS];
            for (int i=0; i<GRAVITY_JOINTS; i++) q[i]=q0[i]+f*(q1[i]-q0[i]);
            char info[512]=""; bool free;
            { std::lock_guard<std::mutex> l(collisionMutex); free=collisionFree(q, info, sizeof info); }
            if (!free) { char b[64]; snprintf(b, sizeof b, "collision at %.0f%% of the path: ", 100.0*f); mv.checkMsg=string(b)+info; mv.check=2; return; }
        }
        mv.check=1;
    });
    char b[200]; snprintf(b, sizeof b, "%s: checking the path (%d steps), then %.1f s at max %.1f deg/s", label.c_str(), n+1, mv.T, mv.vmax);
    moveSetMsg(b);
    return true;
}

// position [deg] along the longest way at time t [s]
double moveProfile(double t)
{
    double vpk=mv.acc*mv.Ta;
    if (t<=0) return 0;
    if (t<mv.Ta) return 0.5*mv.acc*t*t;
    if (t<mv.T-mv.Ta) return 0.5*mv.acc*mv.Ta*mv.Ta + vpk*(t-mv.Ta);
    if (t<mv.T) return mv.D - 0.5*mv.acc*(mv.T-t)*(mv.T-t);
    return mv.D;
}

void moveTick(vector <JointJCB> &jointList, long long now)
{
    if (mv.st==MV_IDLE) return;
    if (mv.st==MV_CHECKING)
    {
        if (mv.check==0) return;
        if (mv.worker.joinable()) mv.worker.join();
        if (mv.cancel) { mv.st=MV_IDLE; return; }
        if (mv.check==2) { mv.st=MV_IDLE; moveSetMsg("REFUSED, "+mv.checkMsg); return; }
        mv.st=MV_RUNNING; mv.t0=now;
        contactReset(jointList);
        moveSetMsg(mv.label+": path is free, moving");
        return;
    }
    double t=(now-mv.t0)/1000.0, f = mv.D>0 ? moveProfile(t)/mv.D : 1.0;
    for (int i=0; i<GRAVITY_JOINTS; i++)
    {
        int a=i/2, s=i%2; auto &j=jointList[a]; auto &v=j.jointVF.vf;
        if (moveCfg.ignore[i]) continue;
        uint16_t err=(uint16_t)(s==0 ? v.jErFlags1 : v.jErFlags2);
        if (!(s==0 ? j.hold1 : j.hold2)) { moveStop(jointList, string(sideName(a,s))+" lost ENABLED/HOMED"); return; }
        if (err) { char b[80]; snprintf(b, sizeof b, "%s reports error 0x%04x", sideName(a,s), err); moveStop(jointList, b); return; }
        double target = mv.q0[i]+f*(mv.q1[i]-mv.q0[i]);
        int32_t tc=(int32_t)std::lround(target*1024.0*1000.0);
        (s==0 ? j.holdSP1 : j.holdSP2)=tc; (s==0 ? j.jogMoveSP1 : j.jogMoveSP2)=0;
        double lag=fabs((s==0 ? j.encoderPos1 : j.encoderPos2)-tc)/COUNTS_PER_DEG;
        if (lag>moveCfg.maxLag)
        { char b[140]; snprintf(b, sizeof b, "%s fell %.1f deg behind its target (blocked or collision?)", sideName(a,s), lag); moveStop(jointList, b); return; }
    }
    { string c=contactCheck(jointList); if (!c.empty()) { moveStop(jointList, c); return; } }
    if (mv.st==MV_RUNNING && t>=mv.T) { mv.st=MV_SETTLING; mv.settleMs=now; }
    if (mv.st==MV_SETTLING)
    {
        double worst=0;
        for (int i=0; i<GRAVITY_JOINTS; i++)
        { if (moveCfg.ignore[i]) continue; auto &j=jointList[i/2]; worst=std::max(worst, fabs((i%2==0 ? j.encoderPos1 : j.encoderPos2)/(1024.0*1000.0)-mv.q1[i])*180/M_PI); }
        if (worst<0.3 || now-mv.settleMs>3000)
        {
            mv.st=MV_IDLE;
            char b[140]; snprintf(b, sizeof b, "%s: reached (largest remaining difference %.2f deg)", mv.label.c_str(), worst);
            moveSetMsg(b);
        }
    }
}

void gotoStart(vector <JointJCB> &jointList, int a, int s, double deg)
{
    double target[GRAVITY_JOINTS]={}; bool use[GRAVITY_JOINTS]={};
    target[a*2+s]=deg*M_PI/180; use[a*2+s]=true;
    char label[64]; snprintf(label, sizeof label, "%s to %.1f deg", sideName(a,s), deg);
    string why;
    if (!moveStart(jointList, target, use, label, why)) { gotoMsg[a][s]="last go to: refused, "+why; printf("goto: %s refused: %s\n", sideName(a,s), why.c_str()); }
}

// ---- Cartesian jog of the hand ("Cartesian jog" window, pipe "cart"): the hand moves along the arm base axes
// (x forward, y left, z up) while a direction is held (dead-man). A 50 Hz thread turns the hand velocity into joint
// velocities (Pinocchio Jacobian, damped least squares, handVelocityToJoints), caps them per joint, stops at the joint
// limits and checks the next pose and a short look-ahead with the collision model; the 1 kHz loop (cartTick) then moves
// the held targets at those joint velocities and stops if a joint falls behind (like coordinated moves).
std::mutex commMutex;                      // comm thread <-> GUI <-> cartesian jog thread
extern bool gravityReady;
const double CART_DT=0.02;                 // thread period [s]
const double CART_LAMBDA=0.03;             // damping of the least squares (larger = smoother near singular poses)
const double CART_JOINT_MAX_DEG_S=15.0;    // per-joint speed cap
const int    CART_LOOKAHEAD=10;            // collision check also CART_LOOKAHEAD periods ahead (0.2 s)
struct CartJog
{
    double dir[3]={};                      // requested direction per axis (-1, 0, 1)
    long long deadmanUntilMs=0;            // the direction is valid until then (GUI re-arms it every frame)
    double speedMm=20; float speedMm_f=20; bool keepOri=true;
    bool   haveTarget=false; double target[GRAVITY_JOINTS]={}, rate[GRAVITY_JOINTS]={};   // [rad], [rad/s]
    double tcp[3]={}; bool tcpValid=false; double sigma=0;
    string msg;
} cart;
std::atomic<bool> cartRun{false};

void cartStop(vector <JointJCB> &jointList, const string &why)
{
    for (auto &d:cart.dir) d=0;
    cart.deadmanUntilMs=0;
    if (cart.haveTarget) { cart.haveTarget=false; for (auto &j:jointList) { j.holdSP1=j.encoderPos1; j.holdSP2=j.encoderPos2; } }
    if (!why.empty()) { cart.msg="STOPPED: "+why; printf("cart: %s\n", cart.msg.c_str()); }
}

// 50 Hz thread
void cartThreadFn(vector <JointJCB> *jl)
{
    auto &jointList=*jl;
    auto next=std::chrono::steady_clock::now();
    while (cartRun)
    {
        next+=std::chrono::milliseconds((int)(CART_DT*1000)); std::this_thread::sleep_until(next);
        double q[GRAVITY_JOINTS]={}, dir[3], speed; bool keep, active, ok=true; string why;
        // manual teaching: check the current pose and where the joints' speeds lead in 0.15 s; a collision ends it
        {
            bool floating=false; double qt[GRAVITY_JOINTS]={}, qta[GRAVITY_JOINTS]={};
            {
                std::lock_guard<std::mutex> lock(commMutex);
                floating = teachFloating() && jointList.size()>=4;
                for (int i=0; floating && i<GRAVITY_JOINTS; i++)
                {
                    auto &v=jointList[i/2].jointVF.vf; int sd=i%2;
                    qt[i]=(sd==0 ? v.encoder1 : v.encoder2)/(1024.0*1000.0);
                    qta[i]=qt[i] + (i==0 ? 0.0 : (sd==0 ? v.speed1 : v.speed2)*0.001*0.15);
                }
            }
            if (floating && collisionReady)
            {
                char info[512]=""; bool free;
                { std::lock_guard<std::mutex> l(collisionMutex); free = collisionFree(qt, info, sizeof info) && collisionFree(qta, info, sizeof info); }
                if (!free) { std::lock_guard<std::mutex> lock(commMutex); teachHalt(string("collision ahead: ")+info); }
            }
        }
        {
            std::lock_guard<std::mutex> lock(commMutex);
            long long now=steadyMs();
            active = now<cart.deadmanUntilMs && (cart.dir[0]||cart.dir[1]||cart.dir[2]);
            if (jointList.size()<4) { ok=false; why="not all boards connected"; }
            for (int i=0; ok && i<GRAVITY_JOINTS; i++)
            {
                int a=i/2, s=i%2; auto &j=jointList[a]; auto &v=j.jointVF.vf;
                bool crsp=(j.jointVF.msgtyp40_2[0]&0x0f)==2; uint8_t st = s==0 ? v.stFlags1 : v.stFlags2;
                bool hold = s==0 ? j.hold1 : j.hold2;
                double enc=(s==0 ? v.encoder1 : v.encoder2)/(1024.0*1000.0);
                if (i==0) { q[0] = (crsp && (st&JS_HOMED)) ? enc : 0.0; continue; }   // head: not in the hand's chain
                if (!crsp || !(st&JS_ENABLED) || !(st&JS_HOMED) || !hold) { ok=false; why=string(sideName(a,s))+" is not ENABLED + HOMED + holding"; break; }
                if (s==0 ? j.wrapLock1 : j.wrapLock2) { ok=false; why=string(sideName(a,s))+" has a wrap lock"; break; }
                q[i] = (s==0 ? j.holdSP1 : j.holdSP2)/(1024.0*1000.0);
            }
            if (mv.st!=MV_IDLE) { ok=false; why="a coordinated move is running"; }
            for (int k=0; k<3; k++) dir[k]=cart.dir[k];
            speed=cart.speedMm; keep=cart.keepOri;
            if (!active) cart.haveTarget=false;
            if (active && !ok) { cart.haveTarget=false; cart.msg="not moving: "+why; }
        }
        double pos[3], rot[9]; bool tcpOk = gravityReady && handPose(q, pos, rot);
        { std::lock_guard<std::mutex> lock(commMutex); if (tcpOk) { memcpy(cart.tcp,pos,sizeof pos); cart.tcpValid=ok; } }
        if (!active || !ok || !gravityReady) continue;

        double v[3]={dir[0]*speed/1000, dir[1]*speed/1000, dir[2]*speed/1000}, dq[GRAVITY_JOINTS], sigma=0;
        handVelocityToJoints(q, v, keep, CART_LAMBDA, dq, &sigma);
        double fastest=0; for (int i=1; i<GRAVITY_JOINTS; i++) fastest=std::max(fastest, fabs(dq[i])*180/M_PI);
        if (fastest>CART_JOINT_MAX_DEG_S) for (auto &x:dq) x*=CART_JOINT_MAX_DEG_S/fastest;   // slower, same direction
        double qn[GRAVITY_JOINTS], qa[GRAVITY_JOINTS];
        string stop;
        for (int i=0; i<GRAVITY_JOINTS; i++) { qn[i]=q[i]+dq[i]*CART_DT; qa[i]=q[i]+dq[i]*CART_DT*CART_LOOKAHEAD; }
        for (int i=1; i<GRAVITY_JOINTS && stop.empty(); i++)
        {
            int a=i/2, s=i%2; double d=qa[i]*180/M_PI;
            double lo=jointLimitDeg[a][s][0]+LIMIT_MARGIN_DEG, hi=jointLimitDeg[a][s][1]-LIMIT_MARGIN_DEG;
            if ((d<lo && dq[i]<0) || (d>hi && dq[i]>0)) stop=string(sideName(a,s))+" reached its limit";
        }
        if (stop.empty() && collisionReady)
        {
            char info[512]="";
            std::lock_guard<std::mutex> l(collisionMutex);
            if (!collisionFree(qa, info, sizeof info) || !collisionFree(qn, info, sizeof info)) stop=string("collision ahead: ")+info;
        }
        std::lock_guard<std::mutex> lock(commMutex);
        cart.sigma=sigma;
        if (!stop.empty()) { cartStop(jointList, stop); continue; }
        if (steadyMs()>=cart.deadmanUntilMs) { cart.haveTarget=false; continue; }    // released meanwhile
        memcpy(cart.target, qn, sizeof qn); memcpy(cart.rate, dq, sizeof dq); cart.haveTarget=true;
        char b[160]; snprintf(b, sizeof b, "moving %s%s%s at %.0f mm/s%s", dir[0]?(dir[0]>0?"+x ":"-x "):"", dir[1]?(dir[1]>0?"+y ":"-y "):"",
                              dir[2]?(dir[2]>0?"+z ":"-z "):"", speed, fastest>CART_JOINT_MAX_DEG_S ? " (slowed: joint speed cap)" : "");
        cart.msg=b;
    }
}

// 1 kHz (comm thread): move the held targets at the joint rates towards the thread's latest targets
void cartTick(vector <JointJCB> &jointList, long long now)
{
    static bool wasActive=false;
    if (!cart.haveTarget) { wasActive=false; return; }
    if (teachBusy()) { cartStop(jointList, "manual teaching is on"); wasActive=false; return; }
    if (!wasActive) { contactReset(jointList); wasActive=true; }
    { string c=contactCheck(jointList); if (!c.empty()) { cartStop(jointList, c); wasActive=false; return; } }
    if (now>=cart.deadmanUntilMs) { cart.haveTarget=false; cart.msg="released"; return; }   // targets stay: joints stop
    for (int i=1; i<GRAVITY_JOINTS; i++)
    {
        int a=i/2, s=i%2; auto &j=jointList[a];
        if (!(s==0 ? j.hold1 : j.hold2)) { cartStop(jointList, string(sideName(a,s))+" lost ENABLED/HOMED"); return; }
        int32_t &hsp = s==0 ? j.holdSP1 : j.holdSP2;
        double step=fabs(cart.rate[i])*0.001*1024*1000, want=cart.target[i]*1024*1000;
        double d=want-hsp;
        hsp += (int32_t)std::lround(std::clamp(d, -step-1, step+1));
        (s==0 ? j.jogMoveSP1 : j.jogMoveSP2)=0;
        double lag=fabs((s==0 ? j.encoderPos1 : j.encoderPos2)-hsp)/COUNTS_PER_DEG;
        if (lag>moveCfg.maxLag) { char b[120]; snprintf(b, sizeof b, "%s fell %.1f deg behind (blocked or collision?)", sideName(a,s), lag); cartStop(jointList, b); return; }
    }
}

void homePoseAllStart(vector <JointJCB> &jointList)
{
    double target[GRAVITY_JOINTS]={}; bool use[GRAVITY_JOINTS]={};
    auto pose=loadHomePose();
    for (auto &e:pose) if (!moveCfg.ignore[e.a*2+e.s]) { target[e.a*2+e.s]=e.deg*M_PI/180; use[e.a*2+e.s]=true; }
    string why;
    if (!moveStart(jointList, target, use, "home posture", why)) moveSetMsg("home posture refused: "+why);
}

// every 1 ms: moves the side's held target (holdSetpoint in amyEth.cpp) towards the goal at no more than gotoMaxJog
// (jog units: gotoMaxJog/HOLD_JOG_DIV counts per ms); done when the target is at the goal and the joint within ~0.1 deg
// (or has settled next to it); aborted if the joint stops moving for 2 s while the target still has to move,
// or it loses ENABLED/HOMED
void gotoTick(vector <JointJCB> &jointList, long long now)
{
    for (int a=0; a<4 && a<(int)jointList.size(); a++)
        for (int s=0; s<2; s++)
        {
            if (!gotoActive[a][s]) continue;
            auto &j=jointList[a];
            int pos = s==0 ? j.jointVF.vf.encoder1 : j.jointVF.vf.encoder2;
            int &jog = s==0 ? j.jogMoveSP1 : j.jogMoveSP2;
            bool hold = s==0 ? j.hold1 : j.hold2;
            double target = s==0 ? j.holdSP1 : j.holdSP2;
            double remaining = gotoTarget[a][s]-target;           // how far the held target still has to go
            double err = gotoTarget[a][s]-pos;
            bool moving = abs(pos-gotoLastPos[a][s])>300;
            if (moving) { gotoLastPos[a][s]=pos; gotoProgressMs[a][s]=now; }
            long long still = now-gotoProgressMs[a][s];
            const char *end=nullptr;
            if (!hold) end="aborted: joint not enabled/homed";
            else if (fabs(remaining)<HOLD_JOG_DIV && fabs(err)<2000) end="reached";
            else if (fabs(remaining)<HOLD_JOG_DIV && still>1000) end="reached target, joint settled next to it";
            else if (still>2000) end="aborted: joint not moving";
            if (end)
            {
                jog=0; gotoActive[a][s]=false; jogUntilMs[a][s]=0;
                printf("goto: %s %s at %.2f deg\n", sideName(a,s), end, pos/COUNTS_PER_DEG);
                char b[120]; snprintf(b, sizeof b, "last go to: %s at %.2f deg", end, pos/COUNTS_PER_DEG); gotoMsg[a][s]=b;
                continue;
            }
            // trapezoidal speed of the held target (counts per ms): accelerate to the go-to speed in GOTO_RAMP_MS, brake so
            // it arrives with zero speed, exactly the rest of the way in the last frame
            const double GOTO_RAMP_MS=1000.0;
            double vmax = gotoMaxJog/(double)HOLD_JOG_DIV;                    // counts per ms
            double acc  = vmax/GOTO_RAMP_MS;                                  // counts per ms^2
            double v = std::min({vmax, acc*std::max(1.0,(double)(now-gotoStartMs[a][s])), std::sqrt(2.0*acc*fabs(remaining)), fabs(remaining)});
            v = std::max(v, std::min(1.0, fabs(remaining)));                  // never stall just before the goal
            jog=(int)std::lround((remaining<0 ? -v : v)*HOLD_JOG_DIV);
            jogUntilMs[a][s]=now+200; jogFromGui[a][s]=true;     // a stalled loop still stops the jog
        }
}

// ---- fault record: what every board reported around the moment an enabled side latched a new error ----
// The global error bits (UNDERVOLTAGE, NO_EXTERNAL_ENABLE, ...) are not latched: they clear as soon as the cause is gone,
// and the status file / GUI only show current values. So the comm thread keeps the last FAULT_WIN_MS of every board's
// global byte and bus voltage, and on a new error records FAULT_WIN_MS more, then writes all of it to the console and
// FAULT_FILE (2026-09-26: all 8 sides latched 0x0001 after j5 hit its end stop; nothing showed what caused it).
#define FAULT_FILE "/tmp/amycore.faults"
const int FAULT_WIN_MS=100;
string dbgDecode(const uint8_t raw[][20]);
struct FaultState
{
    uint8_t  globRing[4][FAULT_WIN_MS]={}; uint16_t busRing[4][FAULT_WIN_MS]={}; int idx=0;
    uint8_t  dbg[4][2][20]={};            // latest debug-stream pages 0/1 per board
    uint16_t prevErr[4][2]={}; uint8_t prevSt[4][2]={};
    long long untilMs=0; string trigger, sidesAtTrigger;
    uint8_t  globOr[4]={}; uint16_t busMin[4]={}; uint16_t errOr[4][2]={};
} fault;

void faultTick(vector <JointJCB> &jointList, long long now)
{
    int nb = std::min(4, (int)jointList.size());
    // jointVF holds the board's last frame of any kind; only cyclic responses (kind 2, e.g. 0x22/0x32) carry the
    // error/status fields. While a board sends anything else (IDENT 0x06 after a power cut, register replies) its
    // bytes are ignored and its history cleared, so no false record is made (seen 10:43: IDENT bytes read as errors)
    bool crsp[4]={};
    for (int a=0; a<nb; a++)
    {
        crsp[a] = (jointList[a].jointVF.msgtyp40_2[0] & 0x0f) == 2;
        if (!crsp[a]) { fault.prevErr[a][0]=fault.prevErr[a][1]=0; fault.prevSt[a][0]=fault.prevSt[a][1]=0; }
    }
    for (int a=0; a<nb; a++)
    {
        if (!crsp[a]) continue;
        auto &v=jointList[a].jointVF.vf;
        if (v.dbgPage<2) memcpy(fault.dbg[a][v.dbgPage], v.dbgVarBuff20, 20);
        uint16_t bus; memcpy(&bus, &fault.dbg[a][0][8], 2);          // page 0 entry 4 = MOTOR bus voltage (mV)
        fault.globRing[a][fault.idx]=v.globalErrFlag; fault.busRing[a][fault.idx]=bus;
        if (fault.untilMs) { fault.globOr[a]|=v.globalErrFlag; if (bus && bus<fault.busMin[a]) fault.busMin[a]=bus; }
    }
    fault.idx=(fault.idx+1)%FAULT_WIN_MS;

    for (int a=0; a<nb; a++)
        for (int s=0; s<2; s++)
        {
            if (!crsp[a]) continue;
            auto &v=jointList[a].jointVF.vf;
            uint16_t err = (uint16_t)(s==0 ? v.jErFlags1 : v.jErFlags2);
            uint8_t  st  = s==0 ? v.stFlags1 : v.stFlags2;
            uint16_t newBits = err & ~fault.prevErr[a][s];
            bool wasEnabled = fault.prevSt[a][s] & JS_ENABLED;
            fault.prevErr[a][s]=err; fault.prevSt[a][s]=st;
            if (fault.untilMs) fault.errOr[a][s]|=err;
            if (!newBits || !wasEnabled || fault.untilMs) continue;
            // start a record: the last FAULT_WIN_MS from the rings, the next FAULT_WIN_MS live
            char b[160]; snprintf(b, sizeof b, "%s latched 0x%04x (%s)", sideName(a,s), err, erFlagToString(err).c_str());
            fault.trigger=b; fault.sidesAtTrigger="";
            for (int a2=0; a2<nb; a2++)
            {
                fault.globOr[a2]=0; fault.busMin[a2]=0xffff;
                for (int k=0; k<FAULT_WIN_MS; k++)
                {
                    fault.globOr[a2]|=fault.globRing[a2][k];
                    if (fault.busRing[a2][k] && fault.busRing[a2][k]<fault.busMin[a2]) fault.busMin[a2]=fault.busRing[a2][k];
                }
                for (int s2=0; s2<2; s2++)
                {
                    auto &v2=jointList[a2].jointVF.vf;
                    fault.errOr[a2][s2]=(uint16_t)(s2==0 ? v2.jErFlags1 : v2.jErFlags2);
                    snprintf(b, sizeof b, "  %-8s st 0x%02x err 0x%04x pos %.2f deg\n", sideName(a2,s2), s2==0 ? v2.stFlags1 : v2.stFlags2,
                             fault.errOr[a2][s2], (s2==0 ? v2.encoder1 : v2.encoder2)/COUNTS_PER_DEG);
                    fault.sidesAtTrigger+=b;
                }
            }
            fault.untilMs=now+FAULT_WIN_MS;
        }

    if (fault.untilMs && now>=fault.untilMs)
    {
        fault.untilMs=0;
        time_t t=time(nullptr); char ts[32]; strftime(ts, sizeof ts, "%F %T", localtime(&t));
        string r="=== FAULT "+string(ts)+": "+fault.trigger+"\n"+"sides at the moment:\n"+fault.sidesAtTrigger
                +"seen from "+to_string(FAULT_WIN_MS)+" ms before to "+to_string(FAULT_WIN_MS)+" ms after:\n";
        char b[200];
        for (int a=0; a<nb; a++)
        {
            snprintf(b, sizeof b, "  board %d: global 0x%02x %s, bus min %u mV, errors %s 0x%04x / %s 0x%04x\n", a, fault.globOr[a],
                     fault.globOr[a] ? globalErFlagToString(fault.globOr[a]).c_str() : "(none)", fault.busMin[a]==0xffff ? 0 : fault.busMin[a],
                     sideName(a,0), fault.errOr[a][0], sideName(a,1), fault.errOr[a][1]);
            r+=b;
            uint8_t raw[2][20]; memcpy(raw[0], fault.dbg[a][0], 20); memcpy(raw[1], fault.dbg[a][1], 20);
            r+="    motor: "+dbgDecode(raw)+"\n";
        }
        r+="  ESP32: "+espLastStatus+"\n";
        printf("%s", r.c_str());
        if (FILE *f=fopen(FAULT_FILE,"a")) { fputs(r.c_str(), f); fclose(f); }
    }
}

// ---- gravity compensation (Layer A): Pinocchio gravity torque from the current pose -> torque feed-forward ----
// The joints close their own position loop; like Intera, the PC only adds the torque that holds the arm against gravity.
// Units: the feed-forward field is 1/250 Nm (fit of Intera's values against Pinocchio for 10,262 poses of the 2023
// capture: 249-255 per Nm for j1-j5, R^2 0.967-1.000). On by default, per side;
// ramps in/out over GRAVITY_RAMP_MS; computed only while all arm joints (j0-j6) are homed. It ramps from gravityBase
// (the effort measured when the side became enabled + homed, 0 when off) and back to it when the pose is unknown.
#define GRAVITY_URDF AMY_ROOT "/AmyConfig/model/sawyer_intera2023.urdf"
// this robot's gravity calibration (tools/gravcal.py fit --apply): link masses/COMs and per-joint torque offsets; used
// for the gravity feed-forward (and the hand kinematics, which it doesn't change) when present; collision keeps the original
#define GRAVITY_URDF_CAL    AMY_ROOT "/AmyConfig/model/sawyer_calibrated.urdf"
#define GRAVITY_OFFSETS_CAL AMY_ROOT "/AmyConfig/model/gravity_offsets.txt"
double gravityOffsetNm[8]={};                   // added to the model torque (torque-sensor offsets), from GRAVITY_OFFSETS_CAL
                                                // (loaded with or without the calibrated URDF)
string gravityModelName="Intera URDF (uncalibrated)";
const double GRAVITY_FF_PER_NM = 250.0;
const int    GRAVITY_RAMP_MS   = 1000;
const int    GRAVITY_FF_MAX    = 15000;         // Intera's largest value in the 2023 capture: j1 -9155
bool   gravityReady=false;                      // model loaded
// on by default (2026-09-26: without it the joints lag their targets under load and motion is jerky); it only acts
// once all arm joints are homed and ramps in over GRAVITY_RAMP_MS
bool   gravityOn[4][2]={{true,true},{true,true},{true,true},{true,true}};   // switched on per side
double gravityScale[4][2]={};                   // ramp 0..1: feed-forward = scale*gravity + (1-scale)*gravityBase
double gravityTau[4][2]={};                     // last computed torque [Nm]
// bumpless start, like Intera (2023 captures): when a side becomes ENABLED + HOMED its feed-forward is the effort it
// measures at that moment (Intera latched it at HOMED and switched to the model once the last joint was homed), and
// after manual teaching it is the effort at the release; from there it ramps to the gravity model
double gravityBase[4][2]={};                    // raw feed-forward units (1/250 Nm)
int    gravityRampMs[4][2]={{GRAVITY_RAMP_MS,GRAVITY_RAMP_MS},{GRAVITY_RAMP_MS,GRAVITY_RAMP_MS},{GRAVITY_RAMP_MS,GRAVITY_RAMP_MS},{GRAVITY_RAMP_MS,GRAVITY_RAMP_MS}};
bool   gravityWasReady[4][2]={};                // ENABLED + HOMED in the previous tick
bool   gravityPoseOk=false;                     // all arm joints homed (pose known)
string gravityMsg="";

void gravityTick(vector <JointJCB> &jointList)
{
    if (!gravityReady || jointList.size()<4) return;
    // pose in gravity.h order (head_pan, j0..j6) = AmyCore board/side order; head_pan unhomed counts as 0 (no arm load)
    double q[GRAVITY_JOINTS], tau[GRAVITY_JOINTS];
    bool ok=true;
    for (int i=0; i<GRAVITY_JOINTS; i++)
    {
        auto &j=jointList[i/2]; int s=i%2;
        auto &v=j.jointVF.vf;
        bool crsp=(j.jointVF.msgtyp40_2[0]&0x0f)==2, homed=crsp && ((s==0 ? v.stFlags1 : v.stFlags2) & JS_HOMED);
        q[i] = homed ? (s==0 ? v.encoder1 : v.encoder2)/(1024.0*1000.0) : 0.0;
        if (i>=1 && !homed) ok=false;
    }
    if (ok != gravityPoseOk) { gravityPoseOk=ok; printf("gravity: %s\n", ok ? "all arm joints homed, pose known" : "an arm joint is not homed: compensation ramps out"); }
    bool computed = ok && gravityCompute(q, tau);
    if (computed) for (int i=1; i<GRAVITY_JOINTS; i++) tau[i]+=gravityOffsetNm[i];   // calibration offsets (0 without)
    for (int i=0; i<GRAVITY_JOINTS; i++)
    {
        int a=i/2, s=i%2;
        auto &j=jointList[a];
        uint8_t st = s==0 ? j.jointVF.vf.stFlags1 : j.jointVF.vf.stFlags2;
        bool ready = (j.jointVF.msgtyp40_2[0]&0x0f)==2 && (st&JS_ENABLED) && (st&JS_HOMED);
        if (!ready || !gravityOn[a][s]) gravityBase[a][s]=0;
        else if (!gravityWasReady[a][s])        // just became ENABLED + HOMED: start from what the joint carries now
        {
            gravityBase[a][s]=std::clamp((double)(s==0 ? j.jointVF.vf.effort1 : j.jointVF.vf.effort2), (double)-GRAVITY_FF_MAX, (double)GRAVITY_FF_MAX);
            gravityScale[a][s]=0; gravityRampMs[a][s]=GRAVITY_RAMP_MS;
            printf("gravity: %s enabled + homed, feed-forward starts at its measured effort %.0f (%.2f Nm)\n", sideName(a,s), gravityBase[a][s], gravityBase[a][s]/GRAVITY_FF_PER_NM);
        }
        gravityWasReady[a][s]=ready;
        double want = (gravityOn[a][s] && computed) ? 1.0 : 0.0, step = 1.0/std::max(1, gravityRampMs[a][s]);
        gravityScale[a][s] += std::clamp(want-gravityScale[a][s], -step, step);
        if (gravityScale[a][s]>=1.0) gravityRampMs[a][s]=GRAVITY_RAMP_MS;     // a faster release ramp only once
        if (computed) gravityTau[a][s]=tau[i];
        double ffv = gravityScale[a][s]*gravityTau[a][s]*GRAVITY_FF_PER_NM + (1.0-gravityScale[a][s])*gravityBase[a][s];
        if (teachBiasOn) ffv += teachBias[a][s];       // manual teaching: gravity + bias (teachTick)
        int ff = (int)std::lround(std::clamp(ffv, (double)-GRAVITY_FF_MAX, (double)GRAVITY_FF_MAX));
        (s==0 ? j.gravityFF1 : j.gravityFF2) = ff;
    }
}

// start the feed-forward of a side at 'raw' (1/250 Nm) and ramp it to the gravity model over ms (teach release)
void gravityRampFrom(int a, int s, double raw, int ms)
{
    gravityBase[a][s]=std::clamp(raw, (double)-GRAVITY_FF_MAX, (double)GRAVITY_FF_MAX);
    gravityScale[a][s]=0; gravityRampMs[a][s]=ms;
}

void gravitySet(int a, int s, bool on)
{
    gravityOn[a][s]=on;
    printf("gravity: %s %s\n", sideName(a,s), on ? "on (ramping in)" : "off (ramping out)");
}

// ---- stiffness and manual teaching (free drive), copied from Intera (2023 captures,
// docs/JRCP_PROTOCOL.md) ----
// Command payload 14/18 of each side (32/36 for side 2) = stiffness % and control mode (jointJCB.h CTRL_MODE_*).
// Intera, cuff gripped: every arm joint (j0-j6, not the head) gets stiffness 0 and in the next frame mode 10; the
// setpoint follows the joint (holdSetpoint) and the feed-forward stays the gravity torque, so the arm floats and can be
// guided by hand. Cuff released: mode 7 again with the setpoint latched where the joint is, the stiffness ramps back
// by 5 % per ms (20 ms), and the feed-forward jumps to the measured effort and ramps to gravity over ~0.6 s. Its
// releases were overdamped (< 1 deg, no ringing).
// Here: "teach on|off" (pipe), the checkbox in the main panel, or teachCuff (the Axolotl's cuff sensor, CRsp payload
// 11 bit 0x08, once AmyCore brings the Axolotl up; not done yet). Teaching only starts when every arm joint is
// ENABLED + HOMED without errors and the gravity compensation is fully on (otherwise the arm would drop), and ends by
// itself as soon as that no longer holds. "stiffness <side>|all <0-100>" sets the position-mode stiffness.
const int TEACH_STIFF_STEP=5;                  // % per ms when the stiffness goes up (Intera)
const int TEACH_RELEASE_FF_MS=600;             // feed-forward ramp after the release (Intera: ~0.6 s)
const int TEACH_RELEASE_FF_MAX_DEV=1250;       // 5 Nm: the release feed-forward stays this close to gravity
// Bias: with stiffness 0 the joint is a pure torque controller (it drives its torque-sensor reading to the
// feed-forward), so any gravity-model error or sensor offset becomes a real, unopposed torque. 2026-09-26 the first
// try moved the arm up fast and into itself within 0.9 s: in position mode j1 needed 1 Nm less than the model, j2
// 1.5 Nm less, and j6's sensor read +4 Nm with no load. So before floating, AmyCore measures for TEACH_SAMPLE_MS, with
// the arm still in position mode, bias = measured effort - gravity model per joint, and sends gravity + bias while
// teaching: at the start pose that is exactly what the joint carries (zero net torque); away from it the model
// follows the change of gravity.
const int    TEACH_SAMPLE_MS=300;
const int    TEACH_STILL_MRAD_S=20;            // "still" during the measurement: every arm joint below ~1.1 deg/s
const int    TEACH_BIAS_MAX=1250;              // 5 Nm: a larger bias means a bad model or a sensor that needs a tare
// guards while floating; each ends teaching (position mode, target where the joint is, stiffness back)
const int    TEACH_SETTLE_MS=500;              // in the first 0.5 s nobody moves the arm yet: it must stay put...
const double TEACH_DRIFT_DEG=2.0;              // ...within 2 deg
const double TEACH_MAX_DEG_S[8]={0, 45, 45, 45, 45, 90, 90, 90};   // per side (head_pan, j0..j6); faster = runaway
int    stiffWant[4][2]={{100,100},{100,100},{100,100},{100,100}};   // position-mode stiffness per side [%]
bool   teachWanted=false;                      // pipe / GUI
bool   teachCuff=false;                        // cuff sensor (not wired yet)
enum TeachState { TEACH_OFF, TEACH_SAMPLING, TEACH_ENTERING, TEACH_ON };
const char *teachStateName[]={"off","measuring","starting","on"};
TeachState teachState=TEACH_OFF;
string teachMsg="";
long long teachSinceMs=0, teachSampleStartMs=0;
double teachBias[4][2]={}, teachBiasSum[4][2]={}; int teachSampleN=0;
bool   teachBiasOn=false;                      // gravityTick adds teachBias to the feed-forward
int32_t teachEntryPos[4][2]={};
string teachHaltWhy="";                        // set by the collision check in the cartesian-jog thread
bool teachBusy() { return teachState!=TEACH_OFF || teachWanted; }
bool teachFloating() { return teachState==TEACH_ON; }
void teachHalt(const string &why) { if (teachHaltWhy.empty()) teachHaltWhy=why; }

static bool isArmSide(int a, int s) { return !(a==0 && s==0); }     // everything but head_pan

// can the arm float? every arm side ENABLED + HOMED, no error, no wrap lock, gravity compensation fully in
bool teachOk(vector <JointJCB> &jointList, string &why, bool starting)
{
    if (jointList.size()<4) { why="not all boards connected"; return false; }
    if (!gravityReady) { why="gravity model not loaded"; return false; }
    if (!gravityPoseOk) { why="an arm joint is not homed (the gravity torque is unknown)"; return false; }
    for (int a=0; a<4; a++) for (int s=0; s<2; s++)
    {
        if (!isArmSide(a,s)) continue;
        auto &j=jointList[a]; auto &v=j.jointVF.vf;
        uint8_t st = s==0 ? v.stFlags1 : v.stFlags2;
        int err = (uint16_t)(s==0 ? v.jErFlags1 : v.jErFlags2) & ~sideIgnoreErr[a][s];
        if ((j.jointVF.msgtyp40_2[0]&0x0f)!=2 || !(st&JS_ENABLED) || !(st&JS_HOMED)) { why=string(sideName(a,s))+" is not ENABLED + HOMED"; return false; }
        if (err) { char b[80]; snprintf(b, sizeof b, "%s reports error 0x%x", sideName(a,s), err); why=b; return false; }
        if (!gravityOn[a][s]) { why=string("gravity compensation is off for ")+sideName(a,s); return false; }
        if (starting && gravityScale[a][s]<0.999) { why=string("gravity compensation of ")+sideName(a,s)+" is still ramping in"; return false; }
        if (starting && (s==0 ? j.wrapLock1 : j.wrapLock2)) { why=string(sideName(a,s))+" has a wrap lock"; return false; }
    }
    return true;
}

void teachSet(bool on, const string &who)
{
    if (on==teachWanted) return;
    teachWanted=on;
    printf("teach: %s requested (%s)\n", on ? "manual teaching" : "end of manual teaching", who.c_str());
}

void stiffnessSet(int a, int s, int pct)
{
    stiffWant[a][s]=std::clamp(pct, 0, 100);
    printf("stiffness: %s %d %%\n", sideName(a,s), stiffWant[a][s]);
}

// back to position mode: target where the joint is (holdSetpoint followed it), stiffness ramps up (below), the
// feed-forward starts at the measured effort and ramps to plain gravity
static void teachRelease(vector <JointJCB> &jointList, long long now, const string &why)
{
    for (int a=0; a<4; a++) for (int s=0; s<2; s++)
    {
        if (!isArmSide(a,s)) continue;
        auto &j=jointList[a];
        (s==0 ? j.ctrlMode1 : j.ctrlMode2)=CTRL_MODE_POSITION;
        double grav=gravityTau[a][s]*GRAVITY_FF_PER_NM, eff=s==0 ? j.jointVF.vf.effort1 : j.jointVF.vf.effort2;
        if (gravityOn[a][s]) gravityRampFrom(a, s, std::clamp(eff, grav-TEACH_RELEASE_FF_MAX_DEV, grav+TEACH_RELEASE_FF_MAX_DEV), TEACH_RELEASE_FF_MS);
    }
    teachBiasOn=false;
    char b[260]; snprintf(b, sizeof b, "%s after %.1f s%s%s", why.empty() ? "ended" : "ENDED", (now-teachSinceMs)/1000.0, why.empty() ? "" : ": ", why.c_str());
    teachMsg=b; printf("teach: %s; arm holds where it is\n", teachMsg.c_str());
    teachState=TEACH_OFF;
    if (!why.empty()) teachWanted=false;
}
// every 1 ms (comm thread), before gravityTick
void teachTick(vector <JointJCB> &jointList, long long now)
{
    if (jointList.size()<4) return;
    bool want = teachWanted || teachCuff;
    string why;
    auto refuse=[&](const string &w) { teachMsg="REFUSED: "+w; printf("teach: %s\n", teachMsg.c_str()); teachWanted=false; teachState=TEACH_OFF; };
    if (teachState==TEACH_OFF && want)
    {
        if (!teachOk(jointList, why, true)) refuse(why);        // a held cuff has to be let go and gripped again
        else
        {
            // stop everything that moves the held targets, then measure the bias with the arm still (position mode)
            for (int a=0; a<4; a++) for (int s=0; s<2; s++) { (s==0?jointList[a].jogMoveSP1:jointList[a].jogMoveSP2)=0; jogUntilMs[a][s]=0; gotoActive[a][s]=false; }
            if (!homeQueue.empty()) { homeQueue.clear(); homeSideStartMs=0; }
            moveStop(jointList, "manual teaching started");
            if (cart.haveTarget) cartStop(jointList, "manual teaching started");
            for (auto &r:teachBiasSum) r[0]=r[1]=0;
            teachSampleN=0; teachSampleStartMs=now; teachHaltWhy.clear();
            teachState=TEACH_SAMPLING; teachMsg="measuring: keep the arm still";
            printf("teach: %s (%d ms)\n", teachMsg.c_str(), TEACH_SAMPLE_MS);
        }
    }
    else if (teachState==TEACH_SAMPLING)
    {
        if (!want) { teachState=TEACH_OFF; teachMsg="cancelled"; }
        else if (!teachOk(jointList, why, false)) refuse(why);
        else
        {
            string moving;
            for (int a=0; a<4; a++) for (int s=0; s<2; s++)
            {
                if (!isArmSide(a,s)) continue;
                auto &v=jointList[a].jointVF.vf;
                if (abs(s==0 ? v.speed1 : v.speed2)>TEACH_STILL_MRAD_S) moving=sideName(a,s);
                teachBiasSum[a][s] += (s==0 ? v.effort1 : v.effort2) - gravityTau[a][s]*GRAVITY_FF_PER_NM;
            }
            teachSampleN++;
            if (!moving.empty()) refuse(moving+" is moving: hold the arm still (or let go) while it measures");
            else if (now-teachSampleStartMs>=TEACH_SAMPLE_MS)
            {
                string big, list;
                for (int a=0; a<4; a++) for (int s=0; s<2; s++)
                {
                    if (!isArmSide(a,s)) { teachBias[a][s]=0; continue; }
                    teachBias[a][s]=teachBiasSum[a][s]/std::max(1,teachSampleN);
                    char b[48]; snprintf(b, sizeof b, " %s %+.2f", sideName(a,s), teachBias[a][s]/GRAVITY_FF_PER_NM); list+=b;
                    if (fabs(teachBias[a][s])>TEACH_BIAS_MAX) big+=string(" ")+sideName(a,s);
                    teachEntryPos[a][s]=s==0 ? jointList[a].encoderPos1 : jointList[a].encoderPos2;
                }
                printf("teach: bias (measured effort - gravity model) [Nm]:%s\n", list.c_str());
                if (!big.empty()) refuse("bias above 5 Nm on"+big+" (gravity model or torque-sensor offset; tare?)");
                else
                {
                    // stiffness 0 now, mode 10 in the next frame (Intera); feed-forward = gravity + bias from now on
                    teachBiasOn=true;
                    for (int a=0; a<4; a++) for (int s=0; s<2; s++) if (isArmSide(a,s)) (s==0 ? jointList[a].stiff1 : jointList[a].stiff2)=0;
                    teachState=TEACH_ENTERING; teachSinceMs=now;
                }
            }
        }
    }
    else if (teachState==TEACH_ENTERING)
    {
        for (int a=0; a<4; a++) for (int s=0; s<2; s++) if (isArmSide(a,s)) (s==0 ? jointList[a].ctrlMode1 : jointList[a].ctrlMode2)=CTRL_MODE_TEACH;
        teachState=TEACH_ON;
        teachMsg="ON: the arm floats (gravity compensated), guide it by hand";
        printf("teach: %s\n", teachMsg.c_str());
    }
    else if (teachState==TEACH_ON)
    {
        string halt;
        if (!teachOk(jointList, why, false)) halt=why;
        else if (!teachHaltWhy.empty()) halt=teachHaltWhy;
        for (int a=0; a<4 && halt.empty(); a++) for (int s=0; s<2 && halt.empty(); s++)
        {
            if (!isArmSide(a,s)) continue;
            auto &v=jointList[a].jointVF.vf;
            double degS=(s==0 ? v.speed1 : v.speed2)*0.001*180/M_PI, deg=(s==0 ? v.encoder1 : v.encoder2)/COUNTS_PER_DEG;
            double drift=fabs((double)((s==0 ? v.encoder1 : v.encoder2)-teachEntryPos[a][s]))/COUNTS_PER_DEG;
            char b[140]; b[0]=0;
            if (now-teachSinceMs<TEACH_SETTLE_MS && drift>TEACH_DRIFT_DEG)
                snprintf(b, sizeof b, "%s moved %.1f deg by itself right after the start (feed-forward doesn't match)", sideName(a,s), drift);
            else if (fabs(degS)>TEACH_MAX_DEG_S[a*2+s])
                snprintf(b, sizeof b, "%s too fast (%.0f deg/s, limit %.0f)", sideName(a,s), fabs(degS), TEACH_MAX_DEG_S[a*2+s]);
            else if ((deg<jointLimitDeg[a][s][0]+LIMIT_MARGIN_DEG && degS<0) || (deg>jointLimitDeg[a][s][1]-LIMIT_MARGIN_DEG && degS>0))
                snprintf(b, sizeof b, "%s at its joint limit (%.1f deg)", sideName(a,s), deg);
            halt=b;
        }
        if (!halt.empty() || !want) teachRelease(jointList, now, halt);
    }
    // stiffness: 0 on the arm while floating, otherwise the set value; it goes down at once and up by 5 % per ms
    for (int a=0; a<4; a++) for (int s=0; s<2; s++)
    {
        auto &j=jointList[a];
        uint8_t &st = s==0 ? j.stiff1 : j.stiff2;
        int target = ((teachState==TEACH_ENTERING || teachState==TEACH_ON) && isArmSide(a,s)) ? 0 : stiffWant[a][s];
        st = (uint8_t)(target<st ? target : std::min(target, st+TEACH_STIFF_STEP));
    }
}

void cmdPipeTick(vector <JointJCB> &jointList)
{
    long long now=steadyMs();
    homeAllTick(jointList, now);
    gotoTick(jointList, now);
    moveTick(jointList, now);
    cartTick(jointList, now);
    faultTick(jointList, now);
    teachTick(jointList, now);
    gravityTick(jointList);
    for (int a=0; a<4 && a<(int)jointList.size(); a++)
        for (int s=0; s<2; s++)
            if (jogUntilMs[a][s] && now>=jogUntilMs[a][s])
            {
                (s==0 ? jointList[a].jogMoveSP1 : jointList[a].jogMoveSP2)=0;
                jogUntilMs[a][s]=0;
                if (jogFromGui[a][s]) { jogFromGui[a][s]=false; continue; }
                printf("cmd: jog %s done\n", sideName(a,s)); printSideStatus(jointList,a,s);
            }
    // pose for the 3D viewer (tools/amyview) at ~30 Hz: angle [rad] ("nan" if unknown), held target [rad] or "nan",
    // status flags, joint error, and the goal of a running coordinated move
    static long long lastPoseMs=0;
    if (now-lastPoseMs>=33)
    {
        lastPoseMs=now;
        string pf="time "+to_string(now)+"\n";
        for (int i=0; i<GRAVITY_JOINTS && i/2<(int)jointList.size(); i++)
        {
            int a=i/2, s2=i%2; auto &j=jointList[a]; auto &v=j.jointVF.vf;
            bool crsp=(j.jointVF.msgtyp40_2[0]&0x0f)==2;
            uint8_t st = crsp ? (s2==0 ? v.stFlags1 : v.stFlags2) : 0;
            char b[160];
            snprintf(b, sizeof b, "%s %s %s %d %d\n", sideName(a,s2),
                     (crsp && (st&JS_HOMED)) ? to_string((s2==0 ? v.encoder1 : v.encoder2)/(1024.0*1000.0)).c_str() : "nan",
                     (s2==0 ? j.hold1 : j.hold2) ? to_string((s2==0 ? j.holdSP1 : j.holdSP2)/(1024.0*1000.0)).c_str() : "nan",
                     st, crsp ? (uint16_t)(s2==0 ? v.jErFlags1 : v.jErFlags2) : 0);
            pf+=b;
        }
        if (mv.st!=MV_IDLE) { pf+="goal"; for (int i=0; i<GRAVITY_JOINTS; i++) pf+=" "+to_string(mv.q1[i]); pf+="\n"; }
        pf+=string("move ")+moveStateName[mv.st]+"\n";
        if (FILE *f=fopen("/tmp/amycore.pose.tmp","w")) { fputs(pf.c_str(), f); fclose(f); rename("/tmp/amycore.pose.tmp","/tmp/amycore.pose"); }
    }
    static long long lastStatusMs=0;
    if (now-lastStatusMs>=200)
    {
        lastStatusMs=now;
        string sf=string("time ")+to_string(now)+"\nstart "+(startSequence?"running":"idle")+" state "+to_string(startStateNow)+
                  "\nwave "+(waveWanted?(hbVerified?"on":"on-unverified"):"off")+" p24 "+(rel24V?"1":"0")+" aux "+(relAux?"1":"0")+
                  (espDead?" ESP32-DEAD":"")+"\nconfig "+cfgStateName()+"\narm "+armStepName[armStep]+(armMsg.empty()?"":" - "+armMsg)+"\n"+
                  "move "+moveStateName[mv.st]+(mv.msg.empty()?"":" - "+mv.msg)+"\n"+
                  "teach "+teachStateName[teachState]+(teachMsg.empty()?"":" - "+teachMsg)+"\n"+
                  "link "+linkDiagHost()+"\n";
        for (int a=0; a<4 && a<(int)jointList.size(); a++)
            {
                auto &jv=jointList[a].jointVF;
                bool cr=(jv.msgtyp40_2[0]&0x0f)==2;      // buttons/knob only valid in cyclic responses
                char bb[64]; snprintf(bb, sizeof bb, cr ? " buttons 0x%02x knob %d" : " buttons - knob -", jv.data[43], (int8_t)jv.data[21]);
                sf+="board "+to_string(a)+" state "+to_string(jointList[a].state)+bb+" frames "+to_string(boardFrameRate[a])+"/s"+
                    (boardDiag[a].empty() ? "" : " diag "+boardDiag[a])+"\n";
            }
        for (int a=0; a<4 && a<(int)jointList.size(); a++) for (int s=0; s<2; s++) sf+="side "+sideStatusLine(jointList,a,s)+"\n";
        FILE *f=fopen(STATUS_FILE ".tmp","w");
        if (f) { fputs(sf.c_str(),f); fclose(f); rename(STATUS_FILE ".tmp", STATUS_FILE); }
    }
    if (cmdPipeFd<0)
    {
        mkfifo(CMD_PIPE, 0666); chmod(CMD_PIPE, 0666);
        cmdPipeFd=open(CMD_PIPE, O_RDONLY|O_NONBLOCK);
        if (cmdPipeFd>=0) open(CMD_PIPE, O_WRONLY|O_NONBLOCK);      // keep a writer open: no EOF when a writer exits
        if (cmdPipeFd<0) return;
    }
    static string buf;
    char tmp[256]; ssize_t n;
    while ((n=read(cmdPipeFd,tmp,sizeof tmp))>0) buf.append(tmp,n);
    size_t nl;
    while ((nl=buf.find('\n'))!=string::npos)
    {
        string line=buf.substr(0,nl); buf.erase(0,nl+1);
        istringstream in(line); string c, name; in>>c>>name;
        int a=0, s=0;
        printf("cmd: %s\n", line.c_str());
        if (c=="status")
        {
            if (!name.empty() && sideByName(name,a,s)) printSideStatus(jointList,a,s);
            else for (a=0; a<4; a++) for (s=0; s<2; s++) printSideStatus(jointList,a,s);
        }
        else if (c=="linkdiag")       // why boards send no frames: host checks + everything received per sender MAC
        {
            fputs(linkDiagFullReport(jointList).c_str(), stdout);
            for (int b=0; b<(int)jointList.size(); b++)
                if (!boardDiag[b].empty()) printf("linkdiag: board %d %s\n", b, boardDiag[b].c_str());
        }
        else if (c=="arm")            // full "Enable arm" sequence; "arm skip j6 ..." never enables the listed sides
        {
            if (name=="off") { armDisable(jointList, "disabled from command pipe"); continue; }
            armContinuePastFail=true;
            armEnableStart(jointList);
            if (name=="skip")
            {
                string sn; int sa, ss;
                while (in>>sn) if (sideByName(sn,sa,ss)) { armSkip[sa][ss]=true; setJointFlags(jointList[sa],ss,JF_MANUAL_HOME_UNLOCK); printf("cmd: arm will not enable %s\n", sn.c_str()); }
            }
            printf("cmd: arm start: %s\n", armMsg.c_str());
        }
        else if (c=="regdump")            // regdump <board 0-3|all> <mod hex> <first reg hex> <last reg hex> <size>: read a range
        {                                 // replies are appended to /tmp/amycore.regs (tools/regcompare.py compares boards)
            unsigned mod=0, r0=0, r1=0; int size=4; in>>hex>>mod>>r0>>r1>>dec>>size;
            for (int b=0; b<(int)jointList.size(); b++)
            {
                if (name!="all" && name!=to_string(b)) continue;
                for (unsigned reg=r0; reg<=r1 && reg<256; reg++)
                {
                    JointRegisters_t rr{};
                    rr.type = size==2 ? JCB_REG_TYPE4102 : JCB_REG_TYPE4104; rr.regNo=reg; rr.valSubType=mod;
                    jointList[b].registerList.push_back(rr);
                    jointList[b].regNoToUploadList.push_back(jointList[b].registerList.size()-1);
                }
            }
            printf("cmd: regdump boards %s module 0x%02x regs 0x%02x-0x%02x size %d queued\n", name.c_str(), mod, r0, r1, size);
        }
        else if (c=="readreg")            // readreg <side> <mod hex> <reg hex> <size>: value appears in the "05raw" log line
        {
            if (!sideByName(name,a,s)) { printf("cmd: unknown side '%s'\n", name.c_str()); continue; }
            unsigned mod=0, reg=0; int size=2; in>>hex>>mod>>reg>>dec>>size;
            JointRegisters_t rr{};
            rr.type = size==2 ? JCB_REG_TYPE4102 : JCB_REG_TYPE4104; rr.regNo=reg; rr.valSubType=mod;
            jointList[a].registerList.push_back(rr);
            jointList[a].regNoToUploadList.push_back(jointList[a].registerList.size()-1);
            printf("cmd: board of %s: read module 0x%02x reg 0x%02x size %d\n", name.c_str(), mod, reg, size);
        }
        else if (c=="setreg" || c=="mask")   // write one config register live: setreg <side> <mod hex> <reg hex> <size> <value>
        {                                    //                              mask <side> <hex>  (JointErrorOverrideMask)
            if (!sideByName(name,a,s)) { printf("cmd: unknown side '%s'\n", name.c_str()); continue; }
            unsigned mod=0x05, reg=(s==0?0x00:0x32); int size=2; long val=0;
            if (c=="setreg") in>>hex>>mod>>reg>>dec>>size>>val;
            else             { string v; in>>v; val=strtol(v.c_str(),nullptr,0); sideIgnoreErr[a][s]=(int)val; }
            JointRegisters_t rr{};
            rr.type = size==2 ? JCB_REG_TYPE4302 : JCB_REG_TYPE4304; rr.regNo=reg; rr.valSubType=mod; rr.value=(int32_t)val;
            jointList[a].registerList.push_back(rr);
            jointList[a].regNoToUploadList.push_back(jointList[a].registerList.size()-1);
            printf("cmd: board of %s: write module 0x%02x reg 0x%02x size %d value %ld (0x%lx)\n", name.c_str(), mod, reg, size, val, val);
        }
        else if (c=="dbgpage")          // dbgpage <board 0-3|all> <page> <hex>: define a debug-variable stream page (page 0: number of pages)
        {                               // page entries: <count> 00 {<var> <module*4+2> <size> 02}... 00, at most 20 data bytes per page
            unsigned page=0; string hx; in>>dec>>page>>hx;
            vector<uint8_t> raw;
            for (size_t i=0; i+1<hx.size(); i+=2) raw.push_back((uint8_t)strtol(hx.substr(i,2).c_str(),nullptr,16));
            for (int b=0; b<4 && b<(int)jointList.size(); b++)
            {
                if (name!="all" && name!=to_string(b)) continue;
                JointRegisters_t rr{};
                rr.regNo=page; rr.valSubType=0x0b;                  // module DEBUG_VAR_STREAMING, group 3
                if (page==0) { rr.type=JCB_REG_TYPE0302; rr.value = raw.empty() ? 0 : raw[0]; }
                else         { rr.type=JCB_REG_TYPE032a; rr.rawData=raw; rr.rawData.resize(0x2a, 0); }
                jointList[b].registerList.push_back(rr);
                jointList[b].regNoToUploadList.push_back(jointList[b].registerList.size()-1);
            }
            printf("cmd: dbgpage boards %s page %u: %s\n", name.c_str(), page, hx.c_str());
        }
        else if (c=="collision") printf("cmd: %s\n", collisionCheckNow(jointList).c_str());   // check the current pose
        else if (c=="cart")               // cart <x|y|z><+|-> <ms>: move the hand along a base axis for ms (max 3000)
        {
            int ms=0; in>>ms;
            if (name.size()!=2 || string("xyz").find(name[0])==string::npos || (name[1]!='+' && name[1]!='-') || ms<=0)
            { printf("cmd: cart <x|y|z><+|-> <ms>\n"); continue; }
            for (auto &d:cart.dir) d=0;
            cart.dir[name[0]-'x'] = name[1]=='+' ? 1 : -1;
            cart.deadmanUntilMs=steadyMs()+std::min(ms,3000);
            printf("cmd: cart %s for %d ms at %.0f mm/s\n", name.c_str(), std::min(ms,3000), cart.speedMm);
        }
        else if (c=="cartspeed") { cart.speedMm=std::clamp(atof(name.c_str()), 2.0, 50.0); cart.speedMm_f=(float)cart.speedMm; printf("cmd: cart speed %.0f mm/s\n", cart.speedMm); }
        else if (c=="flags")              // flags <side> <hex>: set the joint command flags exactly (experiments; bypasses the arm logic)
        {
            if (!sideByName(name,a,s)) { printf("cmd: unknown side '%s'\n", name.c_str()); continue; }
            string v; in>>v; uint16_t f=(uint16_t)strtoul(v.c_str(), nullptr, 16);
            setJointFlags(jointList[a], s, f); printf("cmd: %s flags 0x%04x\n", name.c_str(), f);
        }
        else if (c=="gravity")            // gravity <side>|all on|off: gravity compensation feed-forward
        {
            string v; in>>v; bool on = v=="on";
            if (!gravityReady) { printf("cmd: gravity model not loaded: %s\n", gravityMsg.c_str()); continue; }
            if (name=="all") { for (a=0; a<4; a++) for (s=0; s<2; s++) gravitySet(a,s,on); continue; }
            if (!sideByName(name,a,s)) { printf("cmd: unknown side '%s'\n", name.c_str()); continue; }
            gravitySet(a,s,on);
        }
        else if (c=="teach")              // teach on|off: manual teaching (free drive: arm floats, gravity compensated)
        {
            if (name!="on" && name!="off") { printf("cmd: teach on|off (now %s: %s)\n", teachState==TEACH_ON?"on":"off", teachMsg.c_str()); continue; }
            teachSet(name=="on", "command pipe");
        }
        else if (c=="stiffness")          // stiffness <side>|all <0-100>: position-mode stiffness in % (Intera: 100)
        {
            int v=-1; in>>v;
            if (v<0 || v>100) { printf("cmd: stiffness <side>|all <0-100>\n"); continue; }
            if (name=="all") { for (a=0; a<4; a++) for (s=0; s<2; s++) stiffnessSet(a,s,v); continue; }
            if (!sideByName(name,a,s)) { printf("cmd: unknown side '%s'\n", name.c_str()); continue; }
            stiffnessSet(a,s,v);
        }
        else if (c=="leds")               // leds <hex>: display bits of the global command flags (masked with LED_GLOBAL_MASK)
        {
            uint32_t v=(uint32_t)strtoul(name.c_str(), nullptr, 16) & LED_GLOBAL_MASK;
            for (auto &j:jointList) j.ledFlags=v;
            printf("cmd: leds 0x%08x\n", v);
        }
        else if (c=="itb")                // itb <side> <hex>: cuff lights 0x10/0x20 ITB inner/outer, 0x40/0x80 ITB1
        {
            if (!sideByName(name,a,s)) { printf("cmd: unknown side '%s'\n", name.c_str()); continue; }
            string v; in>>v; uint8_t bits=(uint8_t)strtoul(v.c_str(), nullptr, 16) & 0xf0;
            (s==0 ? jointList[a].itbLights1 : jointList[a].itbLights2)=bits;
            printf("cmd: itb %s 0x%02x\n", name.c_str(), bits);
        }
        else if (c=="homepose" || c=="neutral")   // homepose <side>|all: go to the home posture (AmyConfig/home_pose.txt)
        {
            if (name=="all") { homePoseAllStart(jointList); continue; }
            if (!sideByName(name,a,s)) { printf("cmd: unknown side '%s'\n", name.c_str()); continue; }
            double deg;
            if (!homePoseDeg(a,s,deg)) { printf("cmd: %s not in %s\n", name.c_str(), HOME_POSE_FILE); continue; }
            gotoStart(jointList, a, s, deg);
        }
        else if (c=="autohome" && name=="all") homeAllStart(jointList);
        else if (c=="start")  { startSequence=true; printf("cmd: startup sequence started\n"); }
        else if (c=="config") { cfgStart(jointList); printf("cmd: config: %s %s\n", cfgStateName(), cfgMsg.c_str()); }
        else if (c=="wave")   { waveEnable(name=="on", "stopped from command pipe"); printf("cmd: wave %s\n", waveWanted?"on":"off"); }
        else if (c=="p24")
        {
            bool on=name=="on";
            if (!on && waveWanted) waveEnable(false, "24V switched off from command pipe");
            rel24V=on; espWrite(on ? "P1" : "P0"); printf("cmd: 24V %s\n", on?"on":"off");
        }
        else if (c=="aux")    { bool ok=auxSet(name=="on"); printf("cmd: aux %s %s\n", ok?"done":"REFUSED", auxMsg.c_str()); }
        else if (c=="quit")   { appQuit=true; printf("cmd: quitting\n"); }
        else if (c=="stop")
        {
            for (a=0; a<4; a++) for (s=0; s<2; s++) { (s==0?jointList[a].jogMoveSP1:jointList[a].jogMoveSP2)=0; jogUntilMs[a][s]=0; gotoActive[a][s]=false; }
            if (!homeQueue.empty()) { homeQueue.clear(); homeSideStartMs=0; printf("cmd: home all cancelled\n"); }
            moveStop(jointList, "stop command");
            cartStop(jointList, "");
            teachSet(false, "stop command");
            printf("cmd: all jogs 0\n");
        }
        else if (!sideByName(name,a,s)) printf("cmd: unknown side '%s'\n", name.c_str());
        else if (c=="reset" || c=="home" || c=="autohome" || c=="disable" || c=="enable" || c=="tare" || c=="nod") sideCommand(jointList,a,s,c);
        else if (c=="goto") { double deg=0; if (in>>deg) gotoStart(jointList,a,s,deg); else printf("cmd: goto <side> <degrees>\n"); }
        else if (c=="jog")
        {
            int v=0, ms=0; in>>v>>ms;
            if (ms<=0 || ms>3000 || abs(v)>5000) { printf("cmd: jog needs value (|v|<=5000) and 1..3000 ms\n"); continue; }
            (s==0 ? jointList[a].jogMoveSP1 : jointList[a].jogMoveSP2)=v;
            jogUntilMs[a][s]=now+ms;
            printSideStatus(jointList,a,s);
        }
        else printf("cmd: unknown command '%s'\n", c.c_str());
    }
}

void armTick(vector <JointJCB> &jointList)
{
    if (armStep==ARM_OFF) return;
    auto now=std::chrono::steady_clock::now();
    long ms=(long)std::chrono::duration_cast<std::chrono::milliseconds>(now-armStepStart).count();

    // the external enable must stay on the whole time
    if (!waveWanted) { armDisable(jointList, "aborted: HB stopped ("+waveStopReason+")"); return; }

    // any joint error on a side that was enabled -> disable everything (Intera: "robot automatically disabling")
    if (armStep>=ARM_ENABLING)
        for (int i=0; i<armSidesEnabled; i++)
        {
            int a=armOrder[i][0], s=armOrder[i][1];
            if (armSkip[a][s]) continue;
            auto &j=jointList[a];
            int err = (uint16_t)(s==0 ? j.jointVF.vf.jErFlags1 : j.jointVF.vf.jErFlags2) & armErrMask() & ~sideIgnoreErr[a][s];
            if (err==JE_LATCHED && sideIgnoreErr[a][s]) err=0;   // latched only because of a masked error
            if (err)
            {
                char b[16]; snprintf(b,sizeof b,"0x%x",err);
                if (armContinuePastFail)
                {   // test setup: a side that refuses ENABLE or faults later is left disabled, the others go on
                    bool atEnable = (i==armLastEnabled && armStep==ARM_ENABLING);
                    setJointFlags(j, s, JF_MANUAL_HOME_UNLOCK|sideBaseFlags(a,s));
                    armSkip[a][s]=true;
                    armRefused+=string(" ")+sideName(a,s)+"("+b+(atEnable?"":" while running")+")";
                    cout<<"arm: "<<sideName(a,s)<<(atEnable?" refused ENABLE":" FAULTED")<<", error "<<b<<", left disabled, others continue\r\n"<<flush;
                    if (i==armLastEnabled) armLastEnabled=-1;
                    armMsg="refused/faulted:"+armRefused+"; others enabled";
                    continue;
                }
                armDisable(jointList, string("aborted: ")+sideName(a,s)+" error "+b);
                return;
            }
        }

    switch (armStep)
    {
    case ARM_RESUME:
        if (ms>=ARM_RESUME_MS)
        {
            for (auto &j:jointList) j.debugIntValue1=0;
            armGoto(ARM_WAIT_POWER);
        }
        break;
    case ARM_WAIT_POWER:
    {
        bool power=true;
        for (auto &j:jointList) if (j.jointVF.vf.globalErrFlag & GLOB_UNDERVOLTAGE) power=false;
        if (power)
        {
            for (int a=0; a<(int)jointList.size(); a++)
                for (int s=0; s<2; s++) setJointFlags(jointList[a], s, JF_MANUAL_HOME_UNLOCK|JF_RESET_LATCHED|sideBaseFlags(a,s));
            armGoto(ARM_RESET);
        }
        else if (ms>ARM_POWER_TIMEOUT_MS) armDisable(jointList, "aborted: no motor power (undervoltage did not clear)");
        break;
    }
    case ARM_RESET:
        if (ms>=ARM_RESET_MS)
        {
            for (int a=0; a<(int)jointList.size(); a++)
                for (int s=0; s<2; s++) setJointFlags(jointList[a], s, JF_MANUAL_HOME_UNLOCK|sideBaseFlags(a,s));
            armGoto(ARM_ENABLING);
            armStepStart-=std::chrono::milliseconds(ARM_STAGGER_MS);   // first side right away
        }
        break;
    case ARM_ENABLING:
        // the previous side must have come up before the next one is enabled
        if (armLastEnabled>=0)
        {
            int a=armOrder[armLastEnabled][0], s=armOrder[armLastEnabled][1];
            auto &j=jointList[a];
            int st = s==0 ? j.jointVF.vf.stFlags1 : j.jointVF.vf.stFlags2;
            if (!(st & JS_ENABLED))
            {
                if (ms>ARM_ENABLE_CHECK_MS)
                {
                    if (!armContinuePastFail) { armDisable(jointList, string("aborted: ")+sideName(a,s)+" did not enable"); break; }
                    setJointFlags(j, s, JF_MANUAL_HOME_UNLOCK|sideBaseFlags(a,s));
                    armSkip[a][s]=true;
                    armRefused+=string(" ")+sideName(a,s)+"(not enabled)";
                    cout<<"arm: "<<sideName(a,s)<<" did not enable, left disabled\r\n"<<flush;
                    armLastEnabled=-1;
                }
                else break;
            }
        }
        while (armSidesEnabled<8 && armSkip[armOrder[armSidesEnabled][0]][armOrder[armSidesEnabled][1]]) armSidesEnabled++;
        if (ms>=ARM_STAGGER_MS)
        {
            if (armSidesEnabled==8)
            {
                string m = armRefused.empty() ? "" : "refused:"+armRefused+"; ";
                if (armMsg.find("skipping")==0) m+=armMsg+"; ";
                armGoto(ARM_ENABLED, m.empty() ? "all sides enabled, homing" : m+"others enabled, homing");
                break;
            }
            int a=armOrder[armSidesEnabled][0], s=armOrder[armSidesEnabled][1];
            setJointFlags(jointList[a], s, JF_ENABLE|JF_MANUAL_HOME_UNLOCK|sideBaseFlags(a,s));
            cout<<"arm: enabling "<<sideName(a,s)<<"\r\n"<<flush;
            armLastEnabled=armSidesEnabled;
            armSidesEnabled++;
            armStepStart=now;
        }
        break;
    case ARM_RETRY_RESET:
        if (ms>=ARM_RESET_MS)
        {
            for (auto &q:armRetryQueue) setJointFlags(jointList[q.first], q.second, JF_MANUAL_HOME_UNLOCK|sideBaseFlags(q.first,q.second));
            armGoto(ARM_RETRY_ENABLE);
            armStepStart-=std::chrono::milliseconds(ARM_STAGGER_MS);
        }
        break;
    case ARM_RETRY_ENABLE:
        if (ms>=ARM_STAGGER_MS)
        {
            if (armRetryQueue.empty()) { armGoto(ARM_ENABLED, armRefused.empty() ? "retry done, all retried sides enabled" : "retry done, refused/faulted:"+armRefused); break; }
            auto q=armRetryQueue.front(); armRetryQueue.erase(armRetryQueue.begin());
            armSkip[q.first][q.second]=false;             // monitored again by the error check
            setJointFlags(jointList[q.first], q.second, JF_ENABLE|JF_MANUAL_HOME_UNLOCK|sideBaseFlags(q.first,q.second));
            cout<<"arm: retry enabling "<<sideName(q.first,q.second)<<"\r\n"<<flush;
            armStepStart=std::chrono::steady_clock::now();
        }
        break;
    case ARM_ENABLED:
        // once a side reports homed, drop the home/unlock request (Intera then sends 0x0001 / head pan 0x0401)
        for (int a=0; a<(int)jointList.size(); a++)
            for (int s=0; s<2; s++)
            {
                if (armSkip[a][s]) continue;
                auto &j=jointList[a];
                int st = s==0 ? j.jointVF.vf.stFlags1 : j.jointVF.vf.stFlags2;
                uint16_t f = getJointFlags(j,s);
                // only when a home request is still set (like sideTick). It used to rewrite every homed side every ms,
                // which wiped out any other flags at once: the head-pan enable sequence (0x0600 step), the Expert
                // window's flag checkboxes and the "flags" command (2026-09-26)
                if ((st & JS_HOMED) && (f & JF_ENABLE) && (f & (JF_MANUAL_HOME_UNLOCK|JF_AUTO_HOME_UNLOCK)))
                    setJointFlags(j, s, JF_ENABLE|sideBaseFlags(a,s));
            }
        break;
    default: break;
    }
}

// ---- joint CONFIG phase, copied from Intera (2023 boot capture) ----
// Intera's register writes per node were extracted from the captures into
// AmyConfig/intera/intera_config_node<N>.txt; jointList index 0..3 = node 0 (scapula), 2 (humerus), 3 (ulna), 4 (carpus).
// Sequence: one 0x21 frame per joint -> all register writes through the acked 0x04/0x05 queue -> 0x31 frames.
#define CFG_TIMEOUT_MS   30000
#define CFG_31_FRAMES    4
static const int cfgNodeOfJoint[4]={0,2,3,4};
enum CfgStep { CFG_OFF, CFG_SEND21, CFG_WRITING, CFG_SEND31, CFG_RESET, CFG_CHECK, CFG_DONE };
const char *cfgStepName[]={"not configured","sending 0x21","writing registers","sending 0x31","reset latched errors","checking","done"};
CfgStep cfgStep=CFG_OFF;
string  cfgMsg="";
int     cfgSubTypeOverride=-1;          // >=0: command frame subtype to send instead of pullAckMode
int     cfgFramesLeft=0;
std::chrono::steady_clock::time_point cfgStepStart;
bool    cfgTablesLoaded=false;

// ---- register metadata for the "Config registers" window ----
// name/joint and model value come from the comments of the generated tables (intera_full), defConfig is what
// "Configure joints" writes (model value + this robot's DATABLOCK calibration), the range is over all joints' model values.
struct RegInfo { string name, joint; int32_t model=0, defConfig=0; bool hasDef=false; };
std::map<int,RegInfo> regInfo[4];                    // per board, key = module<<8 | register
std::map<string,pair<int32_t,int32_t>> regRange;     // variable name -> min/max of the model values over all joints
std::map<string,string> regHelp;                     // AmyConfig/register_help.txt

void loadRegisterHelp(const string &fn)
{
    ifstream f(fn); string line;
    while (getline(f,line))
    {
        if (line.empty() || line[0]=='#') continue;
        auto bar=line.find(" | "); if (bar==string::npos) continue;
        regHelp[line.substr(0,bar)]=line.substr(bar+3);
    }
}

string regHelpText(const string &name)
{
    auto it=regHelp.find(name); if (it!=regHelp.end()) return it->second;
    // generic text from the name
    string s;
    if      (name.find("Sawyer_PositionController_DOB")==0) s="Disturbance observer (DOB) of the position controller";
    else if (name.find("Sawyer_JointPositionController")==0) s="Joint position controller";
    else if (name.find("Sawyer_JointTorqueController")==0)   s="Joint torque controller";
    else if (name.find("Sawyer_PWMTorqueController_TR")==0)  s="PWM torque controller, torque-ripple compensation/estimator";
    else if (name.find("Sawyer_PWMTorqueController")==0)     s="PWM (motor) torque controller";
    else if (name.find("Sawyer_SnsConsChecker")==0)          s="Sensor consistency checker";
    else s="No description";
    if (name.find("ZeroG")!=string::npos) s+=", zero-gravity (hand-guiding) mode";
    if (name.find("GainP")!=string::npos) s+=": proportional gain";
    else if (name.find("GainI")!=string::npos) s+=": integral gain";
    else if (name.find("GainD")!=string::npos) s+=": derivative gain";
    else if (name.find("GainN")!=string::npos) s+=": derivative filter coefficient";
    else if (name.find("Coeff")!=string::npos) s+=": filter coefficient";
    else if (name.find("Schedule")!=string::npos) s+=": gain/velocity schedule point";
    else if (name.find("Limit")!=string::npos) s+=": limit";
    s+=". (generic text from the name)";
    return s;
}

// after the DATABLOCK calibration: the value "Configure joints" writes is the last occurrence in the list
void regInfoTakeDefaults(vector <JointJCB> &jointList)
{
    for (int a=0; a<(int)jointList.size() && a<4; a++)
        for (auto &r:jointList[a].registerList)
        {
            if (r.type!=JCB_REG_TYPE4302 && r.type!=JCB_REG_TYPE4304) continue;
            auto &ri=regInfo[a][r.valSubType<<8|r.regNo];
            ri.defConfig=r.value; ri.hasDef=true;
        }
}

// replaces each joint's registerList with Intera's CONFIG writes; returns false if a file is missing
bool loadInteraConfig(vector <JointJCB> &jointList, const string &folder, bool append=false)
{
    for (int a=0; a<(int)jointList.size() && a<4; a++)
    {
        string fn=folder+"intera_config_node"+to_string(cfgNodeOfJoint[a])+".txt";
        ifstream f(fn);
        if (!f) { cout<<"CONFIG table missing: "<<fn<<"\r\n"<<flush; return false; }
        vector <JointRegisters_t> list;
        string line;
        while (getline(f,line))
        {
            if (line.empty() || line[0]=='#') continue;
            unsigned mod, reg; int size; long val;
            if (sscanf(line.c_str(),"%x %x %d %ld",&mod,&reg,&size,&val)!=4) continue;
            JointRegisters_t r{};
            r.type = size==2 ? JCB_REG_TYPE4302 : JCB_REG_TYPE4304;
            r.regNo=reg; r.valSubType=mod; r.value=(int32_t)val;
            list.push_back(r);
            auto h=line.find('#');                   // "# right_j3 MotorDirection"
            if (h!=string::npos)
            {
                istringstream c(line.substr(h+1)); string joint, name; c>>joint>>name;
                if (!name.empty())
                {
                    auto &ri=regInfo[a][mod<<8|reg]; ri.joint=joint; ri.name=name; ri.model=(int32_t)val;
                    auto rg=regRange.find(name);
                    if (rg==regRange.end()) regRange[name]={(int32_t)val,(int32_t)val};
                    else { rg->second.first=min(rg->second.first,(int32_t)val); rg->second.second=max(rg->second.second,(int32_t)val); }
                }
            }
        }
        // the captures lost frames, so registers only the CSV list (Intera parameter dump + manual fixes) has are added
        int same=0, differ=0, added=0;
        bool complete = folder.find("intera_full")!=string::npos;   // generated from Intera's full variable list: no CSV merge
        if (complete && getenv("AMY_DUMP_CSV_DIFF"))                  // diagnostic: CSV registers the complete table lacks
            for (auto &c:jointList[a].registerList)
            {
                bool found=false;
                for (auto &r:list) if (r.regNo==c.regNo && r.valSubType==c.valSubType) found=true;
                if (!found && (c.type==JCB_REG_TYPE4302 || c.type==JCB_REG_TYPE4304))
                    cout<<"CSVONLY joint "<<a<<" mod 0x"<<hex<<+c.valSubType<<" reg 0x"<<+c.regNo<<dec<<" size "<<(c.type==JCB_REG_TYPE4302?2:4)<<" value "<<c.value<<"\r\n";
            }
        if (!complete)
        for (auto &c:jointList[a].registerList)
        {
            bool found=false;
            for (auto &r:list)
                if (r.regNo==c.regNo && r.valSubType==c.valSubType)
                {
                    found=true;
                    if (r.value==c.value) same++;
                    else { differ++; cout<<"  joint "<<a<<" mod "<<+c.valSubType<<" reg "<<+c.regNo<<": capture "<<r.value<<" CSV "<<c.value<<"\r\n"; }
                }
            if (!found && (c.type==JCB_REG_TYPE4302 || c.type==JCB_REG_TYPE4304)) { list.push_back(c); added++; }
        }
        if (append) jointList[a].registerList.insert(jointList[a].registerList.end(), list.begin(), list.end());
        else        jointList[a].registerList=list;
        cout<<"joint "<<a<<": "<<(append?"+":"")<<list.size()<<" CONFIG registers from "<<fn<<" (CSV: "<<same<<" same, "<<differ
            <<" different - capture kept, "<<added<<" added)\r\n"<<flush;
    }
    return true;
}

void setConfigReg(vector <JointRegisters_t> &list, int mod, int reg, int32_t value)
{
    bool found=false;
    for (auto &r:list)                                    // every occurrence: the list can hold a register twice
        if (r.valSubType==mod && r.regNo==reg) { r.value=value; found=true; }
    if (found) return;
    JointRegisters_t r{};
    r.type=JCB_REG_TYPE4304; r.regNo=reg; r.valSubType=mod; r.value=value;
    list.push_back(r);
}

// Per-board calibration from each board's own DATABLOCK (saved by DoTFTP in AmyConfig/datablock/, gzipped YAML).
// The Intera tables come from a different set of boards (2023 captures), so these registers must be replaced:
//   HesOffset0 -> mod 0x19 reg 0x00/0x02, SDSCalibrationSlope -> 0x19 0x0b/0x17, JointAssy2RobotOffset_mRads_q10 -> 0x1d 0x0e/0x40
// (side 0/side 1). Keys missing in a DATABLOCK keep the table value (Intera used 651 for a missing SDS slope).
void applyDatablockCalibration(vector <JointJCB> &jointList, const string &folder)
{
    struct { const char *key; int mod; int reg[2]; } fields[]={
        {"HesOffset0",0x19,{0x00,0x02}}, {"SDSCalibrationSlope",0x19,{0x0b,0x17}}, {"JointAssy2RobotOffset_mRads_q10",0x1d,{0x0e,0x40}} };
    // joint name -> jointList index, side (index 0 side 0 is the head pan)
    struct { const char *name; int joint; int side; } joints[]={
        {"j0",0,1},{"j1",1,0},{"j2",1,1},{"j3",2,0},{"j4",2,1},{"j5",3,0},{"j6",3,1} };
    for (int a=0; a<(int)jointList.size() && a<4; a++)
    {
        // the file of the board now at this index (by MAC); without a MAC, any saved one for the index
        string mac=vectorToHexStr(jointList[a].mac);
        string cmd="zcat "+folder+"joint"+to_string(a)+"_"+(mac.empty() ? string("*[0-9a-f]") : mac)+".gz 2>/dev/null";
        FILE *p=popen(cmd.c_str(),"r");
        if (!p) continue;
        string yaml; char buf[512];
        while (fgets(buf,sizeof buf,p)) yaml+=buf;
        pclose(p);
        if (yaml.empty()) { cout<<"joint "<<a<<": no saved DATABLOCK, calibration from the Intera table\r\n"<<flush; continue; }
        auto sp=yaml.find("serial: ");
        string serial = sp==string::npos ? "?" : yaml.substr(sp+8, yaml.find('\n',sp)-sp-8);
        for (auto &jn:joints)
        {
            if (jn.joint!=a) continue;
            auto lp=yaml.find(string("\n  ")+jn.name+": {");
            if (lp==string::npos) continue;
            string line=yaml.substr(lp, yaml.find('\n',lp+1)-lp);
            for (auto &f:fields)
            {
                auto kp=line.find(string(f.key)+": ");
                if (kp==string::npos) continue;
                int32_t v=atoi(line.c_str()+kp+strlen(f.key)+2);
                setConfigReg(jointList[a].registerList, f.mod, f.reg[jn.side], v);
                cout<<"joint "<<a<<" ("<<serial<<") "<<jn.name<<" "<<f.key<<" = "<<v<<"\r\n";
            }
        }
    }
    // local corrections on top of the factory DATABLOCK values: AmyConfig/joint_calibration.txt, lines
    // "<joint> <key> <value>" with the same keys (HesOffset0 = zero of the torque sensor, raw Hall counts:
    // torque [mNm] = (RawHes0 - HesOffset0) * SDSCalibrationSlope / 100). 2026-09-26: j6 read +3.3 Nm unloaded
    // (RawHes0 2597 at rest vs the DATABLOCK's 2088, probably since the lamp impact).
    if (FILE *cf=fopen((folder+"../joint_calibration.txt").c_str(),"r"))
    {
        char line[256];
        while (fgets(line, sizeof line, cf))
        {
            char jn[16], key[64]; int v;
            if (line[0]=='#' || sscanf(line, "%15s %63s %d", jn, key, &v)!=3) continue;
            bool done=false;
            for (auto &j:joints) for (auto &f:fields)
                if (!done && string(jn)==j.name && string(key)==f.key && j.joint<(int)jointList.size())
                {
                    setConfigReg(jointList[j.joint].registerList, f.mod, f.reg[j.side], v);
                    cout<<"joint "<<j.joint<<" "<<jn<<" "<<key<<" = "<<v<<" (AmyConfig/joint_calibration.txt)\r\n";
                    done=true;
                }
            if (!done) cout<<"joint_calibration.txt: ignored line: "<<line<<flush;
        }
        fclose(cf);
    }
    cout<<flush;
}

void cfgGoto(CfgStep s, const string &msg="")
{
    cfgStep=s; cfgStepStart=std::chrono::steady_clock::now();
    if (!msg.empty()) cfgMsg=msg;
    cout<<"config: "<<cfgStepName[s]<<(msg.empty()?"":" - "+msg)<<"\r\n"<<flush;
}

const char *cfgStateName() { return cfgStepName[cfgStep]; }

// ---- debug-variable stream (docs/JRCP_PROTOCOL.md section 8) ----
// Intera configures the stream whenever a node enters CONFIGURATION; AmyCore does it in every CONFIG with its own
// two pages of motor diagnostics. Entries: variable, module byte (module*4+2 = debug group), size 2.
struct DbgVar { const char *name; uint8_t var, modByte; bool hex; };
const DbgVar dbgPages[2][10]={
    {   // page 1 (streamed as page index 0): MOTOR module (8) debug variables
        {"m0_fault_status",15,0x22,true}, {"m1_fault_status",35,0x22,true}, {"m0_diag_flags",22,0x22,true},
        {"m1_diag_flags",42,0x22,true},   {"bus_mV",0,0x22,false},          {"bus5V_raw",44,0x22,false},
        {"m0_enabled",21,0x22,false},     {"m1_enabled",41,0x22,false},     {"m0_temp",16,0x22,false},
        {"m1_temp",36,0x22,false} },
    {   // page 2 (streamed as page index 1): BLDC module (9) hall sensor counters
        {"m0_hall_state",0,0x26,false},   {"m1_hall_state",20,0x26,false},  {"m0_hall_bad_transitions",13,0x26,false},
        {"m1_hall_bad_transitions",33,0x26,false}, {"m0_hall_transitions",19,0x26,false}, {"m1_hall_transitions",39,0x26,false},
        {nullptr,0,0,false}, {nullptr,0,0,false}, {nullptr,0,0,false}, {nullptr,0,0,false} } };

int dbgPageCount(int p) { int n=0; while (n<10 && dbgPages[p][n].name) n++; return n; }

// replace any earlier debug-stream writes in the board's list with the page count and the two pages;
// only call while the upload queue is empty (it erases entries, so queued indices would shift)
void dbgStreamQueue(JointJCB &j)
{
    auto &rl=j.registerList;
    rl.erase(std::remove_if(rl.begin(), rl.end(), [](const JointRegisters_t &r){
        return r.valSubType==0x0b && (r.type==JCB_REG_TYPE0302 || r.type==JCB_REG_TYPE032a); }), rl.end());
    JointRegisters_t cnt{}; cnt.type=JCB_REG_TYPE0302; cnt.regNo=0; cnt.valSubType=0x0b; cnt.value=2;
    rl.push_back(cnt);
    for (int p=0; p<2; p++)
    {
        JointRegisters_t pg{}; pg.type=JCB_REG_TYPE032a; pg.regNo=p+1; pg.valSubType=0x0b;
        int n=dbgPageCount(p);
        pg.rawData={(uint8_t)n, 0};
        for (int e=0; e<n; e++) pg.rawData.insert(pg.rawData.end(), {dbgPages[p][e].var, dbgPages[p][e].modByte, 2, 2});
        pg.rawData.push_back(0);
        pg.rawData.resize(0x2a, 0);
        rl.push_back(pg);
    }
}

// named values of the two pages from a board's raw stream buffers (index 0 and 1), e.g. "bus_mV=24186 ..."
string dbgDecode(const uint8_t raw[][20])
{
    string out;
    for (int p=0; p<2; p++)
        for (int e=0; e<dbgPageCount(p); e++)
        {
            uint16_t v; memcpy(&v, &raw[p][2*e], 2);
            char b[48]; snprintf(b, sizeof b, dbgPages[p][e].hex ? "%s=0x%04x " : "%s=%u ", dbgPages[p][e].name, v);
            out+=b;
        }
    return out;
}

void cfgStart(vector <JointJCB> &jointList)
{
    if (!cfgTablesLoaded) { cfgMsg="not started: CONFIG tables not loaded"; return; }
    for (auto &j:jointList)
        if (j.state!=JCB_STATE_WORKING_22) { cfgMsg="not started: joint "+to_string(j.privNo)+" not running"; return; }
    for (auto &j:jointList) { j.regFailed=0; j.regRefused=0; }
    cfgSubTypeOverride=4; cfgFramesLeft=2;       // +1: cfgTick runs before the frames of a loop pass go out
    cfgGoto(CFG_SEND21, "");
}

int cfgQueued(vector <JointJCB> &jointList)
{
    int n=0;
    for (auto &j:jointList) n+=j.regNoToUploadList.size() + (j.regCommStage==REGCOMM_WAIT4ACK ? 1 : 0);
    return n;
}

// called once per main loop iteration, after the command frames were sent
void cfgTick(vector <JointJCB> &jointList)
{
    long ms=(long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-cfgStepStart).count();
    switch (cfgStep)
    {
    case CFG_SEND21:
        if (--cfgFramesLeft<=0)
        {
            cfgSubTypeOverride=-1;
            for (auto &j:jointList)
            {
                j.regNoToUploadList.clear();
                dbgStreamQueue(j);                   // debug-stream pages go last, after the config registers
                for (int a=0; a<(int)j.registerList.size(); a++) j.regNoToUploadList.push_back(a);
            }
            cfgGoto(CFG_WRITING);
        }
        break;
    case CFG_WRITING:
    {
        int left=cfgQueued(jointList);
        cfgMsg=to_string(left)+" register frames/entries left";
        for (auto &j:jointList)
            if (j.regFailed) { cfgGoto(CFG_OFF, "FAILED: joint "+to_string(j.privNo)+" did not acknowledge a register frame"); return; }
        if (left==0) { cfgSubTypeOverride=3; cfgFramesLeft=CFG_31_FRAMES+1; cfgGoto(CFG_SEND31); }
        else if (ms>CFG_TIMEOUT_MS)
        {
            for (auto &j:jointList) j.regNoToUploadList.clear();
            cfgGoto(CFG_OFF, "FAILED: register writes not acknowledged ("+to_string(left)+" left)");
        }
        break;
    }
    case CFG_SEND31:
        if (--cfgFramesLeft<=0)
        {
            cfgSubTypeOverride=-1;
            // errors latched before the registers were set (e.g. HES_CONFIGURATION_ERROR) stay until reset
            // Intera sends MANUAL_HOME_AND_UNLOCK_REQUEST on every side from the start (0x0200, without ENABLE),
            // with a RESET_LATCHED_ERRORS pulse (0x0202); errors latched before the registers were set stay until reset
            for (int a=0; a<(int)jointList.size(); a++)
                for (int s=0; s<2; s++) setJointFlags(jointList[a], s, JF_MANUAL_HOME_UNLOCK|JF_RESET_LATCHED);   // head pan request only at enable
            cfgGoto(CFG_RESET);
        }
        break;
    case CFG_RESET:
        if (ms>=ARM_RESET_MS)
        {
            for (int a=0; a<(int)jointList.size(); a++)
                for (int s=0; s<2; s++) setJointFlags(jointList[a], s, JF_MANUAL_HOME_UNLOCK);
            cfgGoto(CFG_CHECK);
        }
        break;
    case CFG_CHECK:
        if (ms>1000)
        {
            string res;
            bool ok=true;
            for (auto &j:jointList)
            {
                char b[80];
                snprintf(b,sizeof b,"J%d type 0x%02x err 0x%x/0x%x  ", j.privNo, j.jointVF.msgtyp40_2[0],
                         (unsigned)(uint16_t)j.jointVF.vf.jErFlags1, (unsigned)(uint16_t)j.jointVF.vf.jErFlags2);
                res+=b;
                if (j.jointVF.vf.jErFlags1 || j.jointVF.vf.jErFlags2) ok=false;
            }
            cfgGoto(CFG_DONE, (ok ? "OK: " : "errors remain: ")+res);
        }
        break;
    default: break;
    }
}

// torso board emulation at power-on: 24V on, a short test pulse of the wave that must be seen by the
// read-back (on and off again), then the wave stays off while the joints boot. Returns false (24V off) on failure.
// The 24V must be on while the joints boot: tested 2026-09-29 (two cold starts each way), with only the logic power
// on, the carpus board (j5/j6, relays the Axolotl) stays in its bootloader after LOAD APP, starts its app only once
// the 24V comes on later, then resets and stays silent. With this sequence all four boards start.
bool torsoPowerOn()
{
    using namespace std::chrono;
    auto t0=steady_clock::now();
    auto ms=[&]{ return (long)duration_cast<milliseconds>(steady_clock::now()-t0).count(); };
    auto service=[&]
    {
        espPoll();
        auto now=steady_clock::now();
        mainLoopAlive();
        if (now-espLastStatusReq >= milliseconds(ESP_STATUS_MS)) { espWrite("S"); espLastStatusReq=now; }
        std::this_thread::sleep_for(milliseconds(5));
    };
    auto fail=[&](const string &why)
    {
        waveEnable(false, why);
        espWrite("P0"); rel24V=false;
        cout<<"power-on FAILED: "<<why<<", 24V off\r\n"<<flush;
        return false;
    };

    // logic power (7.5V) first; auxSet() enforces the power-cycle rules (min 10 s off)
    if (!auxSet(true)) { cout<<"power-on FAILED: "<<auxMsg<<"\r\n"<<flush; return false; }
    relAux=true;

    cout<<"power-on: 24V on\r\n"<<flush;
    rel24V=true; espWrite("P1");
    while (ms() < HB_TEST_ON_MS) service();

    cout<<"power-on: HB test pulse\r\n"<<flush;
    waveEnable(true);
    bool sawOn=false;
    while (ms() < HB_TEST_OFF_MS)
    {
        service();
        if (!waveWanted) return fail("HB stopped during the test pulse: "+waveStopReason);
        if (espStatusTime > waveStartTime+milliseconds(2*MEAS_WINDOW_MS) && abs(espLastHz-HB_FREQ_HZ) <= HB_FREQ_TOL_HZ) sawOn=true;
    }
    waveEnable(false, "power-on test pulse done");
    auto tOff=steady_clock::now();
    if (!sawOn) return fail("HB read-back never saw the wave (hz="+to_string(espLastHz)+")");

    while (!(espStatusTime > tOff+milliseconds(2*MEAS_WINDOW_MS)))
    {
        service();
        if (steady_clock::now()-tOff > milliseconds(HB_CHECK_MS)) return fail("no ESP32 status after the test pulse");
    }
    if (espLastHz!=0) return fail("HB did not stop (hz="+to_string(espLastHz)+")");

    cout<<"power-on: HB test OK, waiting "<<JOINT_BOOT_WAIT_MS/1000<<" s for the joints to boot\r\n"<<flush;
    while (ms() < HB_TEST_OFF_MS+JOINT_BOOT_WAIT_MS) service();
    return true;
}

ConsoleLog consoleLog;   // everything printed, for the ImGui "Console" window

// ---- service GUI ----
static const ImVec4 colOk(0.3f,1,0.3f,1), colWarn(1,0.6f,0.1f,1), colErr(1,0.35f,0.35f,1);
int guiJogSpeed=1500;
int detailA=0, detailS=1;                             // side shown in "Joint detail" (default j0)
bool showDetail=false;

// is a board sending? (its frame counter moved within the last 300 ms); without frames the joint data is stale
bool boardAlive(vector <JointJCB> &jointList, int a)
{
    static int lastCnt[4]={}; static long long lastMs[4]={};
    long long now=steadyMs();
    if (jointList[a].statRecCounter!=lastCnt[a]) { lastCnt[a]=jointList[a].statRecCounter; lastMs[a]=now; }
    return lastMs[a] && now-lastMs[a]<300;
}

void textWrappedColored(const ImVec4 &c, const string &s)
{
    ImGui::PushStyleColor(ImGuiCol_Text, c); ImGui::TextWrapped("%s", s.c_str()); ImGui::PopStyleColor();
}

const char *linkStateName(int st)
{
    switch (st)
    {
    case JCB_STATE_INIT_02:           return "booting";
    case JCB_STATE_SENDING_SERIAL_06: return "loaded, waiting";
    case JCB_STATE_WORKING_22:        return "running";
    case JCB_STATE_WORKING:           return "running (config)";
    case JCB_STATE_HOT_START:         return "hot start";
    default:                          return "not found";
    }
}

// "Joints" window: the everyday service view, one row per joint side
void drawJointsWindow(vector <JointJCB> &jointList, bool *open)
{
    ImGui::SetNextWindowPos(ImVec2(370,0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(1100,440), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(1.0f);
    if (!ImGui::Begin("Joints", open)) { ImGui::End(); return; }
    if (!waveWanted || !hbVerified) ImGui::TextColored(colWarn, "Enable wave (HB) is off: joints cannot be enabled.");
    if (ImGui::Button("STOP all jogs")) for (auto &j:jointList) { j.jogMoveSP1=0; j.jogMoveSP2=0; }
    ImGui::SameLine(); ImGui::SetNextItemWidth(200); ImGui::SliderInt("jog speed", &guiJogSpeed, 500, 5000);
    ImGui::SameLine(); ImGui::TextDisabled("hold < or > to jog (needs ENABLED + HOMED; speed 1500 = about 5 deg/s)");

    long long now=steadyMs();
    const ImGuiTableFlags tf=ImGuiTableFlags_Borders|ImGuiTableFlags_RowBg|ImGuiTableFlags_SizingFixedFit;
    if (ImGui::BeginTable("joints", 10, tf|ImGuiTableFlags_NoSavedSettings))   // widths fixed below, not from imgui.ini
    {
        // fixed widths sized for the longest content, so the table doesn't grow while jogging ("moving", jog value);
        // the error column takes the rest and wraps
        const ImGuiStyle &sty=ImGui::GetStyle();
        auto tw=[](const char *t){ return ImGui::CalcTextSize(t).x; };
        float btn = tw("ResetEnableHome (auto)Home (Intera)Disable") + 5*2*sty.FramePadding.x + 4*sty.ItemSpacing.x;
        float jogw = 2*ImGui::GetFrameHeight() + 2*sty.ItemSpacing.x + tw("-5000");
        struct Col { const char *h; float w; } cols[]={
            {"joint", tw("head_pan")}, {"status", tw("disabled, homing, moving")}, {"error", 0},
            {"position [deg]", tw("position [deg]")}, {"speed", tw("-000000")}, {"effort", tw("-000000")}, {"force", tw("-000000")},
            {"cmd flags", tw("cmd flags")}, {"commands", btn}, {"jog", jogw} };
        for (auto &c:cols)
            ImGui::TableSetupColumn(c.h, c.w>0 ? ImGuiTableColumnFlags_WidthFixed : ImGuiTableColumnFlags_WidthStretch, c.w);
        ImGui::TableHeadersRow();
        for (int a=0; a<4 && a<(int)jointList.size(); a++)
            for (int s=0; s<2; s++)
            {
                auto &j=jointList[a];
                int st  = s==0 ? j.jointVF.vf.stFlags1 : j.jointVF.vf.stFlags2;
                int err = (uint16_t)(s==0 ? j.jointVF.vf.jErFlags1 : j.jointVF.vf.jErFlags2);
                int pos = s==0 ? j.jointVF.vf.encoder1 : j.jointVF.vf.encoder2;
                int eff = s==0 ? j.jointVF.vf.effort1 : j.jointVF.vf.effort2;
                bool alive=boardAlive(jointList,a);
                if (!alive) st=err=pos=eff=0;
                bool en=st&JS_ENABLED, homed=st&JS_HOMED;
                ImGui::PushID(a*2+s);
                ImGui::TableNextRow();

                ImGui::TableNextColumn();
                if (ImGui::Selectable(sideName(a,s), showDetail && detailA==a && detailS==s)) { detailA=a; detailS=s; showDetail=true; }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("click: open in \"Joint detail\" (plots, go to position, tare)");
                if (!alive)
                {
                    ImGui::TableNextColumn(); ImGui::TextDisabled("no frames");
                    if (ImGui::IsItemHovered() && !boardDiag[a].empty()) ImGui::SetTooltip("%s", boardDiag[a].c_str());
                    for (int c=0; c<8; c++) ImGui::TableNextColumn();
                    ImGui::PopID();
                    continue;
                }

                ImGui::TableNextColumn();
                string stat = en ? "enabled" : "disabled";
                if (homed) stat+=", homed"; else if (st&0x20) stat+=", homing";
                if (st&4) stat+=", moving";
                ImGui::TextColored(en ? colOk : ImVec4(0.7f,0.7f,0.7f,1), "%s", stat.c_str());
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("status %d: %s", st, stFlagToString(st).c_str());

                ImGui::TableNextColumn();
                if (err) { char h[12]; snprintf(h, sizeof h, "0x%04x ", err); textWrappedColored(colErr, h+erFlagToString(err)); }
                else     ImGui::TextDisabled("-");

                ImGui::TableNextColumn();                     // position unit: 1/1024 mrad
                ImGui::Text("%9.2f", pos/1024.0/1000.0*180.0/M_PI);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%d counts%s", pos, homed ? "" : " (not homed: not the absolute position)");

                ImGui::TableNextColumn(); ImGui::Text("%6d", s==0 ? j.jointVF.vf.speed1 : j.jointVF.vf.speed2);
                ImGui::TableNextColumn(); ImGui::Text("%6d", eff);
                ImGui::TableNextColumn(); ImGui::Text("%6d", s==0 ? j.jointVF.vf.force1 : j.jointVF.vf.force2);
                ImGui::TableNextColumn(); ImGui::Text("0x%04x", getJointFlags(j,s));

                ImGui::TableNextColumn();
                if (ImGui::SmallButton("Reset")) sideCommand(jointList,a,s,"reset");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("clear latched errors (40 ms RESET_LATCHED_ERRORS pulse)");
                ImGui::SameLine(); if (ImGui::SmallButton("Enable")) sideCommand(jointList,a,s,"enable");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("enable without a homing request.\nBoth joints of a board must be enabled, or neither gives torque.");
                ImGui::SameLine(); if (ImGui::SmallButton("Home (auto)")) sideCommand(jointList,a,s,"autohome");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("ENABLE + AUTO_HOME_AND_UNLOCK (0x0101): the joint homes itself at once (~2 s, it moves a little)");
                ImGui::SameLine(); if (ImGui::SmallButton("Home (Intera)")) sideCommand(jointList,a,s,"home");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("ENABLE + MANUAL_HOME_AND_UNLOCK (0x0201), exactly what Intera sent.\nIn the 2023 recording each joint then started homing by itself 27-43 s later (no PC command changed).\nOn this robot it has stayed in 'homing' for minutes so far: use Home (auto) instead.");
                ImGui::SameLine(); if (ImGui::SmallButton("Disable")) sideCommand(jointList,a,s,"disable");

                ImGui::TableNextColumn();
                ImGui::BeginDisabled(!(en && homed));
                int dir=0;
                ImGui::ArrowButton("jl", ImGuiDir_Left);  if (ImGui::IsItemActive()) dir=-1;
                ImGui::SameLine(); ImGui::ArrowButton("jr", ImGuiDir_Right); if (ImGui::IsItemActive()) dir=1;
                ImGui::EndDisabled();
                if (dir)
                {   // re-armed every GUI frame; cmdPipeTick sets it back to 0 150 ms after the button is released
                    (s==0 ? j.jogMoveSP1 : j.jogMoveSP2)=dir*guiJogSpeed;
                    jogUntilMs[a][s]=now+150; jogFromGui[a][s]=true;
                }
                int jog = s==0 ? j.jogMoveSP1 : j.jogMoveSP2;
                if (jog) { ImGui::SameLine(); ImGui::Text("%d", jog); }
                ImGui::PopID();
            }
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Boards");
    if (ImGui::BeginTable("boards", 5, tf))
    {
        for (const char *h : {"board","joints","link","global error","buttons / knob"})
            ImGui::TableSetupColumn(h, string(h)=="global error" ? ImGuiTableColumnFlags_WidthStretch : ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();
        for (int a=0; a<4 && a<(int)jointList.size(); a++)
        {
            auto &j=jointList[a];
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%d  %s", a, vectorToHexStr(j.mac).c_str());
            ImGui::TableNextColumn(); ImGui::Text("%s + %s", sideName(a,0), sideName(a,1));
            ImGui::TableNextColumn();
            if (!boardAlive(jointList,a))
            {
                ImGui::TextColored(colWarn, "no frames");
                if (ImGui::IsItemHovered() && !boardDiag[a].empty()) ImGui::SetTooltip("%s", boardDiag[a].c_str());
                ImGui::TableNextColumn(); ImGui::TableNextColumn(); continue;
            }
            ImGui::Text("%s, frames 0x%02x", linkStateName(j.state), j.jointVF.msgtyp40_2[0]);
            ImGui::TableNextColumn();
            int g=j.jointVF.vf.globalErrFlag;
            if (g) textWrappedColored(colErr, to_string(g)+" "+globalErFlagToString(g)); else ImGui::TextDisabled("-");
            ImGui::TableNextColumn();
            auto &r=j.jointRawFrame80Received[j.idxRecRead];
            ImGui::Text("buttons 0x%02x  knob %d", r.vf.valbtn, r.vf.encKnob);
        }
        ImGui::EndTable();
    }
    ImGui::End();
}


// ---- joint history (for the plots) and the "Joint detail" window ----
const int HIST_N=1000;                                // at ~100 GUI frames/s = 10 s
struct SideHist { float pos[HIST_N]={}, speed[HIST_N]={}, effort[HIST_N]={}, force[HIST_N]={}; };
SideHist hist[4][2];
int histIdx=0;

void histSample(vector <JointJCB> &jointList)
{
    histIdx=(histIdx+1)%HIST_N;
    for (int a=0; a<4 && a<(int)jointList.size(); a++)
    {
        auto &v=jointList[a].jointVF.vf;
        hist[a][0].pos[histIdx]=v.encoder1/COUNTS_PER_DEG; hist[a][0].speed[histIdx]=v.speed1;
        hist[a][0].effort[histIdx]=v.effort1;              hist[a][0].force[histIdx]=v.force1;
        hist[a][1].pos[histIdx]=v.encoder2/COUNTS_PER_DEG; hist[a][1].speed[histIdx]=v.speed2;
        hist[a][1].effort[histIdx]=v.effort2;              hist[a][1].force[histIdx]=v.force2;
    }
}

void plotHist(const char *label, float *d, const char *fmt)
{
    float mn=d[0], mx=d[0];
    for (int i=0; i<HIST_N; i++) { mn=std::min(mn,d[i]); mx=std::max(mx,d[i]); }
    if (mx-mn<1e-3f) { mn-=1; mx+=1; }
    char ov[64]; snprintf(ov, sizeof ov, fmt, d[histIdx]);
    ImGui::PlotLines(label, d, HIST_N, (histIdx+1)%HIST_N, ov, mn, mx, ImVec2(-120, 55));
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("last 10 s, min %.2f, max %.2f", mn, mx);
}

void drawJointDetail(vector <JointJCB> &jointList, bool *open)
{
    ImGui::SetNextWindowPos(ImVec2(880,300), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(610,760), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(1.0f);
    if (!ImGui::Begin("Joint detail", open)) { ImGui::End(); return; }
    for (int a=0; a<4; a++) for (int s=0; s<2; s++)
    {
        if (a||s) ImGui::SameLine();
        bool sel = a==detailA && s==detailS;
        if (ImGui::RadioButton(sideName(a,s), sel)) { detailA=a; detailS=s; }
    }
    int a=detailA, s=detailS;
    auto &j=jointList[a];
    auto &v=j.jointVF.vf;
    bool alive=boardAlive(jointList,a);
    int st  = s==0 ? v.stFlags1 : v.stFlags2;
    int err = (uint16_t)(s==0 ? v.jErFlags1 : v.jErFlags2);
    int pos = s==0 ? v.encoder1 : v.encoder2;
    bool en=st&JS_ENABLED, homed=st&JS_HOMED;
    if (!alive)
    {
        ImGui::TextColored(colWarn, "board %d sends no frames", a);
        if (!boardDiag[a].empty()) textWrappedColored(colWarn, boardDiag[a]);
        ImGui::End(); return;
    }

    ImGui::SeparatorText("State");
    static const char *stBit[8]={"current limiting","ENABLED","moving","CCW limit","HOMED","homing","head pan enabled","bit 7"};
    for (int b=0; b<8; b++)
    {
        if (b%4) ImGui::SameLine(150.0f*(b%4)+8);
        bool on=st&(1<<b);
        ImGui::TextColored(on ? colOk : ImVec4(0.45f,0.45f,0.45f,1), "%s %s", on ? "[x]" : "[ ]", stBit[b]);
    }
    if (err) textWrappedColored(colErr, "error 0x"+[](int e){char b[8];snprintf(b,8,"%04x",e);return string(b);}(err)+" "+erFlagToString(err));
    else     ImGui::TextDisabled("no joint error");
    if (v.globalErrFlag) textWrappedColored(colErr, "board: "+globalErFlagToString(v.globalErrFlag));
    ImGui::Text("command flags 0x%04x   jog speed %d%s", getJointFlags(j,s), s==0 ? j.jogMoveSP1 : j.jogMoveSP2, gotoActive[a][s] ? "   (go to running)" : "");
    if (s==0 ? j.hold1 : j.hold2)
        ImGui::Text("holding target %.3f deg (joint %+.3f deg from it)", (s==0 ? j.holdSP1 : j.holdSP2)/COUNTS_PER_DEG,
                    ((s==0 ? j.encoderPos1 : j.encoderPos2)-(s==0 ? j.holdSP1 : j.holdSP2))/COUNTS_PER_DEG);
    else
        ImGui::TextDisabled("not holding (a target is kept only while ENABLED + HOMED)");
    if (s==0 ? j.wrapLock1 : j.wrapLock2)
        textWrappedColored(colErr, "position wrapped around (jumped > 90 deg): jog and go to are locked. Disable, turn the joint back by hand into its range, enable.");
    ImGui::TextDisabled("limits %.1f .. %.1f deg (jog and go to stop %.0f deg inside)", jointLimitDeg[a][s][0], jointLimitDeg[a][s][1], LIMIT_MARGIN_DEG);

    ImGui::SeparatorText("Measurements (raw units unless noted)");
    ImGui::Text("position %.3f deg  (%d counts, 1/1024 mrad)%s", pos/COUNTS_PER_DEG, pos, homed ? "" : "  - not homed");
    ImGui::Text("speed %d   effort %d   force %d", s==0 ? v.speed1 : v.speed2, s==0 ? v.effort1 : v.effort2, s==0 ? v.force1 : v.force2);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("effort: rises with the load the motor holds (j2 ~1000 against gravity);\nforce: second torque-like value; units not known");
    ImGui::TextDisabled("board extras: byte 24 %d, byte 40 %d, byte 43 %d, knob %d, buttons 0x%02x", v.speedSpecial1, v.unkAnalog, v.val7, v.encKnob, v.valbtn);
    ImGui::TextDisabled("not reported by the joints: motor current, voltage, power, temperature");
    auto &h=hist[a][s];
    plotHist("position [deg]", h.pos, "%.2f deg");
    plotHist("speed", h.speed, "%.0f");
    plotHist("effort", h.effort, "%.0f");
    plotHist("force", h.force, "%.0f");

    ImGui::SeparatorText("Control");
    if (ImGui::Button("Reset errors")) sideCommand(jointList,a,s,"reset");
    ImGui::SameLine(); if (ImGui::Button("Enable")) sideCommand(jointList,a,s,"enable");
    ImGui::SameLine(); if (ImGui::Button("Home (auto)")) sideCommand(jointList,a,s,"autohome");
    ImGui::SameLine(); if (ImGui::Button("Home (Intera)")) sideCommand(jointList,a,s,"home");
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("ENABLE + MANUAL_HOME_AND_UNLOCK (0x0201), exactly what Intera sent.\nIn the 2023 recording joints started homing by themselves 27-43 s later; on this robot not yet.");
    ImGui::SameLine(); if (ImGui::Button("Disable")) sideCommand(jointList,a,s,"disable");
    if (ImGui::Button("Tare")) sideCommand(jointList,a,s,"tare");
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("TARE flag pulse: zero the torque sensor (firmware flag; not tested yet)");
    if (a==0) { ImGui::SameLine(); if (ImGui::Button("Head nod")) sideCommand(jointList,0,0,"nod"); if (ImGui::IsItemHovered()) ImGui::SetTooltip("DO_HEAD_NOD flag pulse (not tested yet)"); }

    ImGui::BeginDisabled(!(en && homed && alive));
    ImGui::SetNextItemWidth(200); ImGui::SliderInt("jog / go-to speed", &guiJogSpeed, 500, 5000);
    gotoMaxJog=guiJogSpeed;
    int dir=0;
    ImGui::Button("<< jog"); if (ImGui::IsItemActive()) dir=-1;
    ImGui::SameLine(); ImGui::Button("jog >>"); if (ImGui::IsItemActive()) dir=1;
    if (dir) { (s==0 ? j.jogMoveSP1 : j.jogMoveSP2)=dir*guiJogSpeed; jogUntilMs[a][s]=steadyMs()+150; jogFromGui[a][s]=true; gotoActive[a][s]=false; }
    static float target=0;
    ImGui::SetNextItemWidth(120); ImGui::InputFloat("deg", &target, 1, 10, "%.2f");
    ImGui::SameLine(); if (ImGui::Button("Go to")) gotoStart(jointList,a,s,target);
    ImGui::SameLine(); if (ImGui::Button("Here")) target=pos/COUNTS_PER_DEG;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("copy the current position into the target");
    double homeDeg;
    if (homePoseDeg(a,s,homeDeg))
    {
        char nb[48]; snprintf(nb, sizeof nb, "Go to home (%.1f deg)", homeDeg);
        if (ImGui::Button(nb)) { target=(float)homeDeg; gotoStart(jointList,a,s,homeDeg); }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("this joint's home posture angle from AmyConfig/home_pose.txt (default: Intera's neutral pose); moves with go to");
    }
    else ImGui::TextDisabled("no home angle for this joint in AmyConfig/home_pose.txt");
    ImGui::EndDisabled();
    ImGui::SameLine(); if (ImGui::Button("STOP")) { gotoActive[a][s]=false; (s==0 ? j.jogMoveSP1 : j.jogMoveSP2)=0; moveStop(jointList, "STOP pressed"); }
    if (!(en && homed)) ImGui::TextDisabled("jog and go to need the joint ENABLED + HOMED");
    if (!gotoMsg[a][s].empty()) textWrappedColored(gotoMsg[a][s].find("aborted")!=string::npos ? colWarn : colOk, gotoMsg[a][s]);
    ImGui::TextDisabled("go to: moves with the jog offset (at most the speed above) until within ~0.1 deg;\nstops if the joint doesn't move for 2 s - then try a higher speed");

    // the raw setpoints as sliders (same as in "Expert / protocol"): they stay where they are left, no timeout
    ImGui::SeparatorText("Direct setpoints (stay set until changed)");
    ImGui::SetNextItemWidth(300);
    if (ImGui::SliderInt("jog speed", s==0 ? &j.jogMoveSP1 : &j.jogMoveSP2, -5000, 5000)) { jogUntilMs[a][s]=0; gotoActive[a][s]=false; }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("moves the held target by value/16 counts per ms (1500 = about 5 deg/s) while not 0;\n< 0 one way, > 0 the other. Ctrl+click to type a value.");
    ImGui::SetNextItemWidth(300);
    ImGui::SliderInt("torque feed-forward", s==0 ? &j.effortSP1 : &j.effortSP2, -8000, 8000);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("bytes 8-9 / 26-27 of the command: extra torque, sign = direction.\nIntera used it for gravity compensation (j1 about -6000).");
    if (ImGui::Button("Zero both")) { (s==0 ? j.jogMoveSP1 : j.jogMoveSP2)=0; (s==0 ? j.effortSP1 : j.effortSP2)=0; }

    ImGui::SeparatorText("Gravity compensation (Pinocchio, torque feed-forward)");
    if (!gravityReady) textWrappedColored(colErr, "model "+gravityMsg);
    else
    {
        bool on=gravityOn[a][s];
        if (ImGui::Checkbox("on for this joint", &on)) gravitySet(a,s,on);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("adds the torque that holds this joint against gravity (like Intera);\nramps in and out over 1 s; only while all arm joints are homed");
        ImGui::SameLine();
        ImGui::Text("gravity %.2f Nm -> feed-forward %d (%.0f%%)", gravityTau[a][s], s==0 ? j.gravityFF1 : j.gravityFF2, gravityScale[a][s]*100);
        if (!gravityPoseOk) ImGui::TextDisabled("waiting: all arm joints (j0-j6) must be homed so the pose is known");
    }

    ImGui::SeparatorText("Stiffness and control mode");
    ImGui::SetNextItemWidth(300);
    if (ImGui::SliderInt("stiffness [%]", &stiffWant[a][s], 0, 100)) {}
    if (ImGui::IsItemDeactivatedAfterEdit()) printf("stiffness: %s %d %%\n", sideName(a,s), stiffWant[a][s]);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("position-loop stiffness in %% (command payload 14 / 32); Intera: 100.\nManual teaching overrides it with 0 on the arm joints.");
    {
        int m = s==0 ? j.ctrlMode1 : j.ctrlMode2;
        ImGui::Text("sent: stiffness %d %%, mode %d (%s)", s==0 ? j.stiff1 : j.stiff2, m, m==CTRL_MODE_TEACH ? "manual teaching" : m==CTRL_MODE_POSITION ? "position" : "?");
        if (teachState!=TEACH_OFF && isArmSide(a,s)) { ImGui::SameLine(); ImGui::TextColored(colOk, "manual teaching"); }
    }
    ImGui::SameLine(); ImGui::TextDisabled("(STOP above only zeroes the jog offset)");
    ImGui::End();
}

// "Config registers" window: the CONFIG table of one board; a value is written when Enter is pressed
// "Cartesian jog" window: move the hand along the arm base axes while a button or key is held
bool showCart=false;
void drawCartWindow(vector <JointJCB> &jointList, bool *open)
{
    ImGui::SetNextWindowSize(ImVec2(430,430), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Cartesian jog", open)) { ImGui::End(); return; }
    if (cart.tcpValid) ImGui::Text("hand   x %7.1f   y %7.1f   z %7.1f  mm", cart.tcp[0]*1000, cart.tcp[1]*1000, cart.tcp[2]*1000);
    else               ImGui::TextDisabled("hand position unknown (all arm joints must be ENABLED + HOMED + holding)");
    ImGui::TextDisabled("arm base frame: x forward, y left, z up");
    ImGui::SetNextItemWidth(220); ImGui::SliderFloat("speed [mm/s]", &cart.speedMm_f, 2.0f, 50.0f, "%.0f");
    cart.speedMm=cart.speedMm_f;
    ImGui::Checkbox("keep the hand's orientation", &cart.keepOri);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("on: only the hand's position moves (6D task)\noff: position only, the orientation may change (3D task, more reach)");

    double dir[3]={0,0,0};
    ImGui::SeparatorText("hold to move");
    float b=ImGui::GetFrameHeight()*1.6f;
    ImVec2 sz(b,b);
    ImGui::Dummy(ImVec2(b,b)); ImGui::SameLine();
    ImGui::Button("+x##up", sz);   if (ImGui::IsItemActive()) dir[0]=1;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("forward (+x), key: arrow up");
    ImGui::SameLine(0, b*1.5f); ImGui::Button("+z##zup", sz); if (ImGui::IsItemActive()) dir[2]=1;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("up (+z), key: Page Up");
    ImGui::Button("+y##left", sz); if (ImGui::IsItemActive()) dir[1]=1;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("left (+y), key: arrow left");
    ImGui::SameLine(); ImGui::Dummy(ImVec2(b,b)); ImGui::SameLine();
    ImGui::Button("-y##right", sz); if (ImGui::IsItemActive()) dir[1]=-1;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("right (-y), key: arrow right");
    ImGui::Dummy(ImVec2(b,b)); ImGui::SameLine();
    ImGui::Button("-x##down", sz); if (ImGui::IsItemActive()) dir[0]=-1;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("back (-x), key: arrow down");
    ImGui::SameLine(0, b*1.5f); ImGui::Button("-z##zdown", sz); if (ImGui::IsItemActive()) dir[2]=-1;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("down (-z), key: Page Down");
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows))
    {
        if (ImGui::IsKeyDown(ImGuiKey_UpArrow))    dir[0]=1;
        if (ImGui::IsKeyDown(ImGuiKey_DownArrow))  dir[0]=-1;
        if (ImGui::IsKeyDown(ImGuiKey_LeftArrow))  dir[1]=1;
        if (ImGui::IsKeyDown(ImGuiKey_RightArrow)) dir[1]=-1;
        if (ImGui::IsKeyDown(ImGuiKey_PageUp))     dir[2]=1;
        if (ImGui::IsKeyDown(ImGuiKey_PageDown))   dir[2]=-1;
    }
    if (dir[0]||dir[1]||dir[2]) { for (int k=0; k<3; k++) cart.dir[k]=dir[k]; cart.deadmanUntilMs=steadyMs()+150; }   // dead-man
    if (ImGui::Button("STOP")) cartStop(jointList, "STOP pressed");
    ImGui::SameLine(); ImGui::TextDisabled("keys work while this window has focus");
    ImGui::Separator();
    if (!cart.msg.empty()) textWrappedColored(cart.msg.find("STOPPED")!=string::npos || cart.msg.find("not moving")!=string::npos ? colWarn : colOk, cart.msg);
    if (cart.sigma>0) (cart.sigma<0.05 ? ImGui::TextColored(colWarn, "near a singular pose (sigma %.3f): the arm moves less precisely", cart.sigma)
                                       : ImGui::TextDisabled("manipulability sigma %.3f", cart.sigma));
    ImGui::TextDisabled("stops at joint limits, before collisions (0.2 s look-ahead), and if a joint falls behind;\njoint speed capped at %.0f deg/s", CART_JOINT_MAX_DEG_S);
    ImGui::End();
}

// "LEDs & buttons" window: display bits of the global command flags (every board), cuff (ITB) lights per side,
// and the button/knob bytes of every board's cyclic response with change highlighting, to find which input is where
bool showLeds=false;
void drawLedsWindow(vector <JointJCB> &jointList, bool *open)
{
    ImGui::SetNextWindowSize(ImVec2(640,620), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("LEDs & buttons", open)) { ImGui::End(); return; }
    int nb=std::min(4,(int)jointList.size());

    ImGui::SeparatorText("LEDs (global command flags, sent to every board)");
    struct Bit { const char *name; unsigned bit; };
    static const Bit leds[]={ {"nav A",0x4000}, {"nav B",0x2000}, {"nav C",0x1000}, {"nav D",0x800}, {"nav E",0x400},
                              {"red",0x8000}, {"green",0x10000}, {"blue",0x20000}, {"LCD display power",0x40000}, {"lamp test",0x10000000} };
    static unsigned ledBase=0;
    static bool blink=false;
    for (int i=0; i<(int)(sizeof leds/sizeof leds[0]); i++)
    {
        if (i%5) ImGui::SameLine();
        ImGui::CheckboxFlags(leds[i].name, &ledBase, leds[i].bit);
    }
    if (ImGui::Button("all on")) ledBase=LED_GLOBAL_MASK;
    ImGui::SameLine(); if (ImGui::Button("all off")) ledBase=0;
    ImGui::SameLine(); ImGui::Checkbox("blink (0.5 s)", &blink);
    bool phaseOff = blink && (steadyMs()/500)%2;
    for (auto &j:jointList) j.ledFlags = phaseOff ? 0 : (ledBase & LED_GLOBAL_MASK);
    ImGui::TextDisabled("only display bits can be set here (mask 0x%08x); flags word 0x%08x", LED_GLOBAL_MASK, ledBase);

    ImGui::SeparatorText("Cuff (ITB) lights, per joint side");
    static const Bit itb[]={ {"ITB inner",0x10}, {"ITB outer",0x20}, {"ITB1 inner",0x40}, {"ITB1 outer",0x80} };
    for (int a=0; a<nb; a++)
        for (int s=0; s<2; s++)
        {
            ImGui::PushID(a*2+s);
            ImGui::Text("%-8s", sideName(a,s));
            unsigned v = s==0 ? jointList[a].itbLights1 : jointList[a].itbLights2;
            for (auto &b:itb) { ImGui::SameLine(); ImGui::CheckboxFlags(b.name, &v, b.bit); }
            (s==0 ? jointList[a].itbLights1 : jointList[a].itbLights2) = (uint8_t)v;
            ImGui::PopID();
        }

    ImGui::SeparatorText("Buttons and knob (cyclic responses; changed bits turn green for 1 s)");
    // byte 43 = JRCP_SEA_CRSP_GLOBAL_FLAG_*: OK 0x80, A 0x40, B 0x20, C 0x10, D 0x08, E 0x04, spare 0x02, camera strobe 0x01
    static const Bit btn[]={ {"OK",0x80}, {"A",0x40}, {"B",0x20}, {"C",0x10}, {"D",0x08}, {"E",0x04}, {"spare",0x02}, {"strobe",0x01} };
    static const int watchBytes[]={21, 37, 40, 41, 43, 44, 45};       // knob, side 2 extra, analog, button byte, unknown
    static uint8_t last[4][66]; static long long changedMs[4][66][8]; static bool init[4];
    struct Ev { long long ms; string text; };
    static std::deque<Ev> events;
    long long now=steadyMs();
    for (int a=0; a<nb; a++)
    {
        auto &jv=jointList[a].jointVF;
        bool cr=(jv.msgtyp40_2[0]&0x0f)==2;
        ImGui::Text("board %d (%s + %s)", a, sideName(a,0), sideName(a,1));
        if (!cr) { ImGui::SameLine(); ImGui::TextDisabled("  no cyclic responses"); init[a]=false; continue; }
        for (int b:watchBytes)
        {
            uint8_t v=jv.data[b];
            if (init[a] && v!=last[a][b])
                for (int k=0; k<8; k++)
                    if ((v^last[a][b])&(1<<k))
                    {
                        changedMs[a][b][k]=now;
                        char t[96]; snprintf(t, sizeof t, "board %d byte %d bit 0x%02x %s", a, b, 1<<k, (v&(1<<k)) ? "set" : "cleared");
                        events.push_front({now,t}); if (events.size()>12) events.pop_back();
                    }
            last[a][b]=v;
        }
        init[a]=true;
        uint8_t b43=jv.data[43];
        ImGui::Text("   buttons:"); 
        for (auto &b:btn)
        {
            int k=__builtin_ctz(b.bit);
            bool on=b43&b.bit, recent=now-changedMs[a][43][k]<1000;
            ImGui::SameLine();
            ImGui::TextColored(recent ? colOk : (on ? ImVec4(1,1,1,1) : ImVec4(0.45f,0.45f,0.45f,1)), "%s%s", on ? "[x]" : "[ ]", b.name);
        }
        ImGui::Text("   knob %4d   raw:", (int8_t)jv.data[21]);
        for (int b:watchBytes)
        {
            bool recent=false; for (int k=0; k<8; k++) recent|=now-changedMs[a][b][k]<1000;
            ImGui::SameLine(); ImGui::TextColored(recent ? colOk : ImVec4(0.7f,0.7f,0.7f,1), "%d:%02x", b, jv.data[b]);
        }
    }
    ImGui::TextDisabled("press a button or turn a knob and watch which board and bit change (GUI rate, very short presses can be missed)");
    ImGui::SeparatorText("Last changes");
    for (auto &e:events) ImGui::Text("%6.1f s ago  %s", (now-e.ms)/1000.0, e.text.c_str());
    ImGui::End();
}

void drawRegistersWindow(vector <JointJCB> &jointList, bool *open)
{
    ImGui::SetNextWindowPos(ImVec2(370,450), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(780,700), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(780,300), ImVec2(FLT_MAX,FLT_MAX));
    ImGui::SetNextWindowBgAlpha(1.0f);
    if (!ImGui::Begin("Config registers", open)) { ImGui::End(); return; }
    static int b=0;
    for (int a=0; a<4 && a<(int)jointList.size(); a++)
    {
        if (a) ImGui::SameLine();
        string l=to_string(a)+": "+sideName(a,0)+"/"+sideName(a,1);
        ImGui::RadioButton(l.c_str(), &b, a);
    }
    auto &j=jointList[b];
    ImGui::Text("MAC %s   write queue %d   failed frames %d   refused writes %d", vectorToHexStr(j.mac).c_str(), (int)j.regNoToUploadList.size(), j.regFailed, j.regRefused);
    if (ImGui::Button("Write all to board"))
        for (int a=0; a<(int)j.registerList.size(); a++) j.regNoToUploadList.push_back(a);
    ImGui::SameLine();
    if (ImGui::Button("Reset all to default"))
        for (int i=0; i<(int)j.registerList.size(); i++)
        {
            auto &r=j.registerList[i]; auto it=regInfo[b].find(r.valSubType<<8|r.regNo);
            if (it!=regInfo[b].end() && it->second.hasDef && r.value!=it->second.defConfig) { r.value=it->second.defConfig; j.regNoToUploadList.push_back(i); }
        }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("write the default config (what \"Configure joints\" writes) to every changed register");
    static bool onlyChanged=false;
    ImGui::SameLine(); ImGui::Checkbox("only changed", &onlyChanged);
    static ImGuiTextFilter filter;
    filter.Draw("filter (name, joint or \"mod 0x21\")", 220);
    ImGui::TextDisabled("edit a value and press Enter to write it; orange = differs from the default config");

    // one line per register: the last occurrence in the list is the one CONFIG writes
    std::map<int,int> last;
    for (int i=0; i<(int)j.registerList.size(); i++)
    {
        auto &r=j.registerList[i];
        if (r.type==JCB_REG_TYPE4302 || r.type==JCB_REG_TYPE4304) last[r.valSubType<<8|r.regNo]=i;
    }
    ImGui::BeginChild("regs");
    if (ImGui::BeginTable("regtab", 4, ImGuiTableFlags_RowBg|ImGuiTableFlags_SizingFixedFit))
    {
        ImGui::TableSetupColumn("joint"); ImGui::TableSetupColumn("register");
        ImGui::TableSetupColumn("value"); ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
        for (auto &[key,i] : last)
        {
            auto &r=j.registerList[i];
            auto it=regInfo[b].find(key);
            RegInfo ri = it!=regInfo[b].end() ? it->second : RegInfo{};
            string shortName = ri.name.empty() ? "unknown (from the CSV list, not in Intera's tables)" : ri.name;
            string jointShort = ri.joint.find("right_")==0 ? ri.joint.substr(6) : (ri.joint.empty() ? "board" : ri.joint);
            if (shortName.find("Sawyer_")==0) shortName=shortName.substr(7);
            char addr[48]; snprintf(addr, sizeof addr, "mod 0x%02x reg 0x%02x", r.valSubType, r.regNo);
            string all=ri.joint+" "+ri.name+" "+addr;
            if (filter.IsActive() && !filter.PassFilter(all.c_str())) continue;
            bool changed = ri.hasDef && r.value!=ri.defConfig;
            if (onlyChanged && !changed) continue;

            ImGui::PushID(i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(jointShort.c_str());
            ImGui::TableNextColumn(); ImGui::TextUnformatted(shortName.c_str());
            bool hover=ImGui::IsItemHovered();
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(130);
            if (changed) ImGui::PushStyleColor(ImGuiCol_Text, colWarn);
            if (ImGui::InputInt("##v", &r.value, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue)) j.regNoToUploadList.push_back(i);
            if (changed) ImGui::PopStyleColor();
            hover|=ImGui::IsItemHovered();
            ImGui::TableNextColumn();
            if (changed && ImGui::SmallButton("def")) { r.value=ri.defConfig; j.regNoToUploadList.push_back(i); }
            else if (!changed) ImGui::TextDisabled("%s", addr);
            if (changed && ImGui::IsItemHovered()) ImGui::SetTooltip("reset to the default config (%d) and write it", ri.defConfig);
            if (hover)
            {
                bool two = r.type==JCB_REG_TYPE4302;
                ImGui::BeginTooltip();
                ImGui::PushTextWrapPos(ImGui::GetFontSize()*38);
                ImGui::Text("%s  (%s)", ri.name.empty() ? "unnamed register" : ri.name.c_str(), ri.joint.c_str());
                ImGui::TextDisabled("%s, %s", addr, two ? "int16" : "int32");
                ImGui::Separator();
                ImGui::TextWrapped("%s", regHelpText(ri.name).c_str());
                ImGui::Separator();
                if (ri.hasDef) ImGui::Text("default config: %d", ri.defConfig); else ImGui::TextDisabled("default config: none (not in the tables)");
                if (ri.hasDef && !ri.name.empty() && ri.model!=ri.defConfig)
                    ImGui::Text("model value %d, this robot's calibration/test setting %d", ri.model, ri.defConfig);
                auto rg=regRange.find(ri.name);
                if (rg!=regRange.end()) ImGui::Text("model values over all joints: min %d, max %d", rg->second.first, rg->second.second);
                ImGui::TextDisabled("type range: %d .. %d (no documented limits)", two ? -32768 : INT32_MIN, two ? 32767 : INT32_MAX);
                ImGui::PopTextWrapPos();
                ImGui::EndTooltip();
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::EndChild();
    ImGui::End();
}

int main(int argc, char *argv[])
{
    // --headless: no window, control only through the command API (/tmp/amycore.cmd, /tmp/amycore.status)
    // --auto:     run the startup sequence and CONFIG by itself;  --arm: then "Enable arm" (implies --auto)
    bool headless=false;
    for (int a=1; a<argc; a++)
    {
        string s=argv[a];
        if (s=="--headless") headless=true;
        else if (s=="--auto") autoConfig=true;
        else if (s=="--arm") autoConfig=autoArm=true;
        else if (s=="--poweron") ;
        else if (s=="--if" && a+1<argc) robotIf=argv[++a];     // robot network interface (default DEFAULT_IF in amyEth.h)
        else if (s=="--version") { printf("%s\n", amyVersionLine); return 0; }
        else { printf("usage: AmyCore [--poweron] [--headless] [--auto] [--arm] [--if <interface>] [--version]\n"); return 1; }
    }
    // Raw Ethernet needs cap_net_raw (file capabilities, dropped by every relink). Without it every socket call fails
    // ("Raw Ethsocket: Operation not permitted", "SIOCGIFINDEX: Bad file descriptor") and nothing reaches the joints,
    // so stop here, before the ESP32 or the relays are touched. The build re-applies the capabilities by itself once
    // tools/install-setcap-sudoers.sh has been run on the machine.
    {
        int fd=socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
        if (fd<0)
        {
            char exe[512]={}; ssize_t n=readlink("/proc/self/exe", exe, sizeof exe-1); if (n<0) strcpy(exe, "build-robot/AmyCore");
            fprintf(stderr, "AmyCore: cannot open a raw Ethernet socket (%s): the binary has no capabilities (relinked?).\n"
                            "  fix now:  sudo setcap cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock+ep %s\n"
                            "  for good: tools/install-setcap-sudoers.sh (once), then every build sets them itself\n", strerror(errno), exe);
            return 1;
        }
        close(fd);
    }
    if (headless) { signal(SIGINT, [](int){ appQuit=true; }); signal(SIGTERM, [](int){ appQuit=true; }); }
    else consoleLog.startCapture();
    printf("%s\n", amyVersionLine);   // after startCapture, so it is in the GUI console too
    // Setup SDL
    if (!headless && SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
    {
        printf("Error: %s\n", SDL_GetError());
        return -1;
    }
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    // From 2.0.18: Enable native IME.
#ifdef SDL_HINT_IME_SHOW_UI
    SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif

    //int serial_port=setupAndOpenESP32("/dev/ttyUSB0");
   serial_port=setupAndOpenESP32("/dev/amy-esp32"); // udev symlink, see esp32_enable/99-amy-esp32.rules
    if (serial_port<0) { cout<<"ESP32 port not available, exiting\r\n"<<flush; return 1; }
    mainLoopAlive();
    std::thread(espKeepaliveThread).detach();

    // --poweron: switch the robot on like the torso board did, before looking for the joints
    for (int a=1; a<argc; a++)
        if (string(argv[a])=="--poweron" && !torsoPowerOn()) return 1;
  
  //int num_bytes;
  //char read_buf [256];
  //string gCmd="";
  
    
  // homimg 
  //gCmd="11\r\n";
  //write(serial_port, gCmd.c_str(), gCmd.length());
  
  //setRelays(0,0);

  //setRelays(1,1);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // setRelays(0,0);
//return 0;
    vector <JointJCB> jointList(4); 

    AmyEth	amyEth(jointList);
    string MainConfigFolder=AMY_ROOT "/AmyConfig/";

    //if (loadPacketsFromFile(MainConfigFolder+"jointRegPackets_66_AE.txt",registersPacketList)==false) return -1;

    cout<<"loading all paramters \r\n" <<flush ;
    loadRobotRegisters(AMY_ROOT "/AmyConfig/parameterALL.csv");
    
    amyRegisters2regList(0,jointList[0].registerList);
    amyRegisters2regList(1,jointList[1].registerList);
    amyRegisters2regList(2,jointList[2].registerList);
    amyRegisters2regList(3,jointList[3].registerList);
    initRegistersManualFIX(jointList);
    // complete tables generated from Intera's sawyer_mp4_data.yaml (model defaults) by tools/genconfig.py;
    // the per-board calibration from each board's DATABLOCK is applied on top (applyDatablockCalibration)
    // AMY_CONFIG_DIR (e.g. "intera" = the older capture tables) overrides the table folder, for tests
    // After a cold boot the complete tables alone leave HES_CONFIGURATION_ERROR on the first joint of several boards,
    // while the capture-based tables (+ CSV) followed by the complete tables give a clean configuration (tested
    // 2026-09-25, runs 31/32). So both are written, in that order; the DATABLOCK calibration is applied to both.
    if (getenv("AMY_CONFIG_DIR"))
        cfgTablesLoaded=loadInteraConfig(jointList, MainConfigFolder+getenv("AMY_CONFIG_DIR")+"/");
    else
        cfgTablesLoaded=loadInteraConfig(jointList, MainConfigFolder+"intera/") &&
                        loadInteraConfig(jointList, MainConfigFolder+"intera_full/", true);
 

    linkDiagInit(robotIf.c_str());   // link, IP and DHCP server check; records every packet from the robot from now on
    amyEth.OpenUDP_IP();
    
    int jointsFound=amyEth.waitForJointsUDP();
    if (jointsFound==4)
    {
        amyEth.DoTFTP();
    }else
    {
        if (!amyEth.InitJointsManual())
        {
            cout<<"AmyCore: the joint boards can't be identified: not all answered IDENTIFY, and there are no saved DATABLOCKs "
                  "for them. Switch the logic power off for 10 s and start with --poweron.\r\n"<<flush;
            return 1;
        }
    }
    
// PRINT found JCBs
    for (int a = 0; a< jointList.size();a++)
    {
            auto joint=jointList[a];
            cout<<"JOINT JCB: "<<+a<<" mac : "<< vectorToHexStr(joint.mac)<< \
                " "<<" prvNo : "<<to_string(joint.privNo)<<"loaded registerList size :"<<to_string(joint.registerList.size())<<"\r\n";
    }
    applyDatablockCalibration(jointList, MainConfigFolder+"datablock/");
    // j6 (board 3 side 1) without multi-turn tracking by default (module 7 = 0x1d, JointMultiturnEnable = var 0x42):
    // with the model value 1, homing across its sensor's 0/360 deg point latches MAE_SENSOR_ERROR (0x11).
    // AMY_J6_MULTITURN=1 keeps the model value.
    if (!getenv("AMY_J6_MULTITURN") && jointList.size()>3) { setConfigReg(jointList[3].registerList, 0x1d, 0x42, 0); cout<<"j6 JointMultiturnEnable = 0 (AMY_J6_MULTITURN=1 keeps 1)\r\n"<<flush; }
    regInfoTakeDefaults(jointList);
    loadRegisterHelp(MainConfigFolder+"register_help.txt");

    // tell joints to load the app 
    jointsFound=amyEth.waitForJointsUDP();
     if (jointsFound==4)
    {
        amyEth.SendLOADAPP2JointsUDP();
        
    }else
    {
        cout<<"ERROR cant load the app \r\n"<<flush;
    }
    

    //std::vector<double> plotData[10];

    
    // THREAD ! 
    //auto recRawUdpTH = ([&jointList,&rawUDPListener]() 
    auto recRawUdpTH = ([&jointList,&amyEth]() 
    {
        cout <<"Starting thread   \r\n";
        char recBuf[1000];
        while (1)
        {
            //int rercBytes=recvfrom(rawUDPListener.sockMacUdpfd, recBuf, sizeof(recBuf), MSG_DONTWAIT , NULL,NULL);
            sockaddr_ll from{}; socklen_t fromLen=sizeof from;
            int rercBytes=recvfrom(amyEth.rawUDPListener.sockMacUdpfd, recBuf, sizeof(recBuf), 0 , (sockaddr*)&from, &fromLen);
            bool accepted=false;
            if (rercBytes==80)
            {
                // is it for me ? 
                if (equalAr(&recBuf[0],6,(char*)&amyEth.rawUDPListener.RawEthListenerMac[0])==true)
                {
                    //cout <<"rec ==80 for me\r\n";
                    for (int a = 0; a< jointList.size();a++)
                    {
                        auto joint=&jointList[a];
                        if (equalAr(&recBuf[6],6,(char*)&joint->mac[0])==true)
                        {
                           // cout <<"parsing joint: "<<a<<" \r\n";
                           joint->idxRecWrite = (joint->idxRecWrite+1) % 5 ;
                           memcpy((char*)&joint->jointRawFrame80Received[joint->idxRecWrite],&recBuf[0],80);
                           joint->statRecCounter++;
                           accepted=true;
                           // update critical stuff here
                           //recBuf.
                           //(JointStatus_t*)
                           //joint->encoderPos1=joint->jointRawFrame80Received[joint->idxRecWrite].vf.encoder1;

                        }
                    }

                }

            }
            // everything else on the robot interface is recorded for the "no frames" diagnosis (linkdiag)
            if (rercBytes>0) linkDiagPacket((const uint8_t*)recBuf, rercBytes, from.sll_ifindex, from.sll_pkttype, accepted);
        }       
        
    });

    //start thread
    std::thread th_obj( recRawUdpTH  ) ;

    cout<<"Stat main polling loop \r\n"<<flush;
    
    bool brodcastAllDay=true; // like Intera: broadcast from LOAD APP on, the joints wait for it before answering
    bool brodcastMacEn=false;

    uint loopcnt=0;
    uint lastloopcnt=0;
    uint loopCnt1sec=0;
    uint pckcnt;
    
    
    int typePolling=0;
    int pollingCnt=0;

    int warmingCounter=0;

    int sendSTAGE=0;

    DebugVariables debugVariables; // class for debuging data parsing 

   // Create window with graphics context
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    
    SDL_Window* window = nullptr;
    SDL_GLContext gl_context = nullptr;
    if (!headless)
    {
    string windowTitle=string("Amy Robot Control - ")+amyVersionLine;
    window = SDL_CreateWindow(windowTitle.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1500, 1200, window_flags);
    gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
//    SDL_GL_SetSwapInterval(1); // Enable vsync
    SDL_GL_SetSwapInterval(0); // Enable vsync
    }


    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();
    // Setup Platform/Renderer backends
    if (!headless)
    {
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);
    }

    // Our state
    bool show_demo_window = false;
    bool show_another_window = false;
    bool showButtons = true;
    bool showConsole = false;
    bool showJointControl = true;     // "Joints" service window

    bool show_registers = false;
    bool show_moveControl = false;    // "Expert / protocol" window
    
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    bool done = false;

    static int values_offset = 0;
    static int pullAckMode = 2;
    static int initRegSend = 0;

    static int sendRegCmd=0;
    static int sendRegTries=0;
    static char strbuff[500]={0};

    static bool search05Ack=false;
    static int debugPrintJ=0;
    bool lastStartSequence=false;

    sprintf(strbuff," no 0x5 nor 0x04");

    
    #define STATE_DISABLED          0
    #define STATE_INIT              1
    #define STATE_RESET_BROADCOAST 2
    #define STATE_ENABLE_JOINTS 3
    #define STATE_POLLING_22     4
    #define STATE_POLLING_32     5

    int stateMachine=STATE_DISABLED;
    int stateMachineTimeout=0;
    
    int stateMachineERROR=0;
    
    
    std::chrono::high_resolution_clock::time_point curTime = chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point lastCurTime=curTime;
    int timePntCnt=0;
    double timeLoop=0.0;

    int statBroadCastCnt=0;
    
    uint32_t debugVars[4][23][7]={0};
    static uint8_t dbgRaw[4][23][20]={};


    // Robot communication runs in its own thread with a fixed 1 ms period (the joints expect ~1 kHz).
    // It used to share this loop with the GUI, so window drawing (~1.5 ms) limited it to ~650/s and a blocked
    // GPU swap (window hidden, screen blanked) stopped the robot traffic completely.
    // commMutex: held by the comm thread for each step and by the GUI while it builds its widgets (they read and
    // change the same state); not held during rendering / SDL_GL_SwapWindow, which may block.
    // commMutex is global (above): the cartesian jog thread needs it too
    std::atomic<bool> commRun{true};
    auto commStep = [&]()
    {


        loopcnt++;
        warmingCounter++;
        // if (loopcnt%100==0)
        // {
        //     for (int a=0;a<4;a++)
        //     {
        //         auto j=jointList[a];
        //         auto axes = CvPlot::makePlotAxes();
        //         axes.create<CvPlot::Series>(j.plotData[0], "-o");
        //         axes.create<CvPlot::Series>(j.plotData[1], "-b");
        //         axes.create<CvPlot::Series>(j.plotData[2], "-g");
        //         axes.create<CvPlot::Series>(j.plotData[3], "-r");
        //         axes.create<CvPlot::Series>(j.plotData[4], "-c");
        //         axes.create<CvPlot::Series>(j.plotData[5], "-m");
        //         axes.create<CvPlot::Series>(j.plotData[6], "-k");
            
        //         cv::Mat mat = axes.render(768, 1024);
        //         string winName="analog Joint "+to_string(a);
        //         cv::imshow(winName, mat);
             
        //     }
        //        auto k=cv::waitKey(1);
        // }

      

        // handling , processing last received frame data from joints
        for (int a = 0; a< jointList.size();a++)
        {
            auto joint=&jointList[a];
            if (joint->idxRecRead!=joint->idxRecWrite) 
            {
                joint->newFrameRecived=true; 
            }
            else 
            {
                joint->newFrameRecived=false; 
                continue;
            }

            auto frRec=&joint->jointRawFrame80Received[joint->idxRecRead];
            
            //if (joint->jointRawFrame80Received.msgtyp40_2[0]==0x02) // start
            if (frRec->msgtyp40_2[0]==0x02) // start
            {
                joint->state=JCB_STATE_INIT_02;
                
            }else

            if (frRec->msgtyp40_2[0]==0x05) // ACK from load register 
            {
                amyEth.ProcessRegistersComm05(*joint);
            }
            else
            if (frRec->msgtyp40_2[0]==0x06) // still in info frame ? 
            {
                joint->state=JCB_STATE_SENDING_SERIAL_06;
               // brodcastEn=true;
            } 
            else
            if (frRec->msgtyp40_2[0]==0x12) // semi working 
            {

            }
            else
            if ((frRec->msgtyp40_2[0]==0x22)|| (frRec->msgtyp40_2[0]==0x32)) // semi working 22 or 32
            {
                memcpy(&joint->jointVF,frRec,80);

                joint->state=JCB_STATE_WORKING_22;
                
                joint->encoderPos1=frRec->vf.encoder1;
                joint->encoderPos2=frRec->vf.encoder2;
                
            }

            //if (a==0)
            {
                JointStatus_t js;
                memcpy(&js,frRec,80);
                if (a==debugPrintJ) 
                {
                    // printf(" framerec type:%.2x ",js.msgtyp40_2[0]);
                    // if ( (js.msgtyp40_2[0]==0x04) || (js.msgtyp40_2[0]==0x05) ) // registers 
                    //     printf("[%d] SS:%d",  js.rg.pcktCounter8,js.rg.someStatus[0]);
                    // else
                    // if ( (js.msgtyp40_2[0]==0x01) ) // cmd/ack
                    //     printf("SS:%d",js.rg.someStatus[0]);
                    // else // value frame 
                    //     printf("[%d]",  js.vf.pcktCounter);
                    // for (int i=0 ;i<66;i++)  printf(" %.2x",js.data[i]);
                    // printf("\r\n");  
                }else
                {
                    /*if  ((js.msgtyp40_2[0]!=0x22) || (js.msgtyp40_2[0]!=0x32))
                    {
                        printf(" framerec type:%.2x ",js.msgtyp40_2[0]);
                        if ( (js.msgtyp40_2[0]==0x04) || (js.msgtyp40_2[0]==0x05) ) // registers 
                            printf("[%d] SS:%d",  js.rg.pcktCounter8,js.rg.someStatus[0]);
                        else
                        if ( (js.msgtyp40_2[0]==0x01) ) // cmd/ack
                            printf("SS:%d",js.rg.someStatus[0]);
                        else // value frame 
                            printf("[%d]",  js.vf.pcktCounter);
                        for (int i=0 ;i<66;i++)  printf(" %.2x",js.data[i]);
                        printf("\r\n");  
                    }*/
                }

                if ((joint->state==JCB_STATE_WORKING) || (joint->state==JCB_STATE_WORKING_22))
                {
                    //if (joint->idxRecRead!=joint->idxRecWrite)
                    if (joint->newFrameRecived)
                    {
                        pckcnt++;
                        if (pckcnt % 5==0)  // not all packets to add 
                        {
                            //printf("delta %d %d ",joint->idxRecWrite,joint->idxRecRead);

                            auto speed=frRec->vf.speed1;
                            auto encoder1=frRec->vf.encoder1;
                            auto val2=frRec->vf.encKnob;
                            auto val3=frRec->vf.effort1;
                            //auto val4=frRec->vf.speed2;
                            //auto val5=frRec->vf.val5;
                            //auto val6=frRec->vf.val6;
                            //auto val7=frRec->vf.val7;

                            joint->valTab1[joint->valTabOffset]=(float)encoder1;
                            //joint->valTab2[joint->valTabOffset]=(float)val2;
                            //joint->valTab3[joint->valTabOffset]=(float)val3;
                            //joint->valTab4[joint->valTabOffset]=(float)val5;

                            //values_offset = (values_offset + 1) % IM_ARRAYSIZE(joint->valTab1);
                            // joint->valTabOffset = (joint->valTabOffset + 1) % 90;

                            // joint->plotData[0].push_back(speed);
                            // joint->plotData[1].push_back(val2);
                            // joint->plotData[2].push_back(val3);

                            // joint->plotData[3].push_back(val5); 
                            // joint->plotData[4].push_back(val6); 
                            // joint->plotData[5].push_back(val7); 

                        
                            // for (int w=0;w<6;w++)
                            // {
                            //     if (joint->plotData[w].size()>500) 
                            //     {
                            //         joint->plotData[w].erase(joint->plotData[w].begin());
                            //     }
                                
                            // }
                            
                        }
                    }
                }
            }
            if (joint->idxRecRead!=joint->idxRecWrite) // FIXME: handling que
            {
                joint->idxRecRead=joint->idxRecWrite;
                //cout<<"XXXXX\r\n"<<flush;
            }

        }
        

        // state machine handling 
        {
            startStateNow=stateMachine;
            // --auto / --arm: the same steps as the GUI buttons, one after the other
            if (autoConfig)
            {
                static int autoStep=0;
                bool allRunning=true;
                for (auto &j:jointList) if (j.state!=JCB_STATE_WORKING_22) allRunning=false;
                if (autoStep==0) { startSequence=true; autoStep=1; cout<<"auto: startup sequence\r\n"<<flush; }
                else if (autoStep==1 && !startSequence && allRunning && waveWanted && hbVerified)
                { cfgStart(jointList); autoStep=2; cout<<"auto: CONFIG\r\n"<<flush; }
                else if (autoStep==1 && !startSequence && stateMachineERROR) { startSequence=true; cout<<"auto: startup sequence again\r\n"<<flush; }
                else if (autoStep==2 && cfgStep==CFG_DONE)
                {
                    autoStep=3;
                    if (autoArm) { armContinuePastFail=true; armEnableStart(jointList); cout<<"auto: Enable arm\r\n"<<flush; }
                }
                else if (autoStep==3 && autoArm && armStep==ARM_ENABLED)
                { autoStep=4; cout<<"auto: arm enabled - home the joints one by one (Joints window or amyctl autohome <joint>)\r\n"<<flush; }
            }
            if (startSequence)
            {
                if (lastStartSequence!=startSequence)
                {
                    lastStartSequence=startSequence; // start!
                    stateMachine=STATE_INIT;
                    stateMachineERROR=0;
                    if (waveWanted) waveEnable(false, "start sequence restarted");

                }
                switch (stateMachine)
                {
                case STATE_DISABLED:
                    stateMachine=STATE_INIT;
                    stateMachineTimeout=200;
                    break;
                 case STATE_INIT:
                    pullAckMode=4; // fixme: 4 means no sending 
                    initRegSend=0;
                    brodcastAllDay=1;
                    stateMachineERROR=0;
                    {
                        bool allJointsOK=true;
                        for (int a = 0; a< jointList.size();a++)
                        //for (int a = 0; a< 1;a++)
                        {
                            auto joint=&jointList[a];
                            if (joint->state!=JCB_STATE_SENDING_SERIAL_06) 
                            {
                                //stateMachine=STATE_ENABLE_JOINTS;
                                allJointsOK=false;
                            }
                        }
                        if (allJointsOK)
                            stateMachine=STATE_ENABLE_JOINTS;
                    }
                    if (stateMachineTimeout-- <= 0)
                    {
                        stateMachineERROR=1; // bad
                        stateMachine=STATE_DISABLED;
                        startSequence=false;
                        if (waveWanted) waveEnable(false, "start sequence timeout");
                    }
                    
                    break;
                case STATE_ENABLE_JOINTS:
                    pullAckMode=0;
                    initRegSend=0;
                    brodcastAllDay=1;
                    brodcastMacEn=1;
                    stateMachineERROR=0;
                    stateMachineTimeout=20;
                    {
                        bool allJointsOK=true;
                        for (int a = 0; a< jointList.size();a++)
                        {
                            auto joint=&jointList[a];
                            if (joint->state==JCB_STATE_WORKING_22) 
                            {
                                //stateMachine=STATE_POLLING_22;
                                allJointsOK=false;
                            }
                    
                        }
                        if (allJointsOK)
                            stateMachine=STATE_POLLING_22;
                    }
                    if (stateMachineTimeout-- <= 0)

                    if (stateMachineTimeout-- <= 0)
                    {
                        stateMachineERROR=1; // bad
                        stateMachine=STATE_DISABLED;
                        startSequence=false;
                        if (waveWanted) waveEnable(false, "start sequence timeout");
                    }
                    break;
                case STATE_POLLING_22:
                    //pullAckMode=2;
                    initRegSend=0;
                    brodcastAllDay=1;
                    brodcastMacEn=0;
                    stateMachineERROR=0;
                    stateMachineTimeout=0;
                    // all joints up: start the enable wave (and 24V) like the torso board did,
                    // but only after the wave has been off for HB_MIN_LOW_MS
                    if (waveWanted || hbLowLongEnough())
                    {
                        if (!waveWanted) waveEnable(true);
                        startSequence=false;
                    }
                    break;
                default:
                    break;
                }

            }
        }

        // unsync the I/O of C and C++.
        //ios_base::sync_with_stdio(false);
        
        
        timePntCnt++;
        
        curTime = chrono::high_resolution_clock::now();
        timeLoop =  chrono::duration_cast<chrono::seconds>(curTime - lastCurTime).count();

        if (timeLoop>=1.0)
        {
            using namespace std::chrono_literals;

            lastCurTime+= 1s ;
            loopCnt1sec=loopcnt-lastloopcnt;
            lastloopcnt=loopcnt;
        }
        // if (timePntCnt>=9) 
        // {
        //     curTime = chrono::high_resolution_clock::now();
        //     timeLoop =  chrono::duration_cast<chrono::nanoseconds>(curTime - lastCurTime).count();
        //     lastCurTime=curTime;
        //     timeLoop/=10.0;//avaragre
        //     timeLoop/=1000000.0; // from nano to [ms]
        //     cout<<timeLoop<<" [ms] ";
        //     timePntCnt=0;
        // }
        //if (timePntCnt%100==0) cout<<timeLoop<<" [ms] ";

        //timePoints[timePntCnt] = chrono::high_resolution_clock::now();
        


        mainLoopAlive();
        bodyBoardTick(jointList); // ESP32 status, read-back and joint silence checks
        linkDiagTick(jointList);  // why a silent board is silent (console, status file, GUI)
        armTick(jointList);       // arm enable sequence
        cfgTick(jointList);       // joint CONFIG phase
        sideTick(jointList);      // per-side reset pulses from the "Joint control" window
        cmdPipeTick(jointList);   // commands from /tmp/amycore.cmd

        // torso board emulation: HB (and 24V) on once all joints have booted into the app.
        // Only once per run: after a trip or a manual HB off it stays off until switched on again.
        {
            static bool hbAutoDone=false;
            bool allBooted=true;
            for (auto &j:jointList) if (j.state!=JCB_STATE_WORKING_22) allBooted=false;
            if (!hbAutoDone && allBooted && hbLowLongEnough())
            {
                cout<<"all joints booted: HB on\r\n"<<flush;
                waveEnable(true);
                hbAutoDone=true;
            }
            if (waveWanted) hbAutoDone=true; // switched on another way (GUI, start sequence)
        }

        {   // main sending main freq should be 1 kH - 1 ms 

        if  (brodcastAllDay==true)
        {
            statBroadCastCnt++;
            if (brodcastMacEn)
                amyEth.sendPollingBroadcastEn(1);
            else
                amyEth.sendPollingBroadcastEn(0);
            
            std::this_thread::sleep_for(std::chrono::microseconds(20));
        }
            for (int a = 0; a< jointList.size();a++)
            {
                auto joint=&jointList[a];
                // if (joint->registersUploaded==false)
                // {
                //     rawUDP.sendRegistersToJoint(*joint,initRegSend);
                //     std::this_thread::sleep_for(std::chrono::microseconds(50));
                // }
                amyEth.ProcessRegistersComm(*joint);

                // if (joint->isWaitingFor05Ack==false)
                // {
                //     if (joint->regNoToUploadList.size()>0)
                //     {
                //         joint->waitingFor05AckTimeOut=4;
                        
                //         int cnt=0;
                //         // from big (or small) list to sublist only for sending , may be 1..4
                //         joint->savedSubListregNoFromList.clear();
                //         while ( (joint->regNoToUploadList.size()>0) && (cnt < 4) )
                //         {
                //             joint->savedSubListregNoFromList.push_back(joint->regNoToUploadList[0]);
                //             joint->regNoToUploadList.erase(joint->regNoToUploadList.begin()); // FIXME : tricky ! can't be resend !
                //             cnt++;
                //         }

                //         amyEth.sendRegistersToJoint(*joint,joint->savedSubListregNoFromList);
                        
                        
                //         //rawUDP.sendRegistersToJoint(*joint,regNoFromList);
                //         //rawUDP.sendRegistersToJoint(*joint,joint->savedregNoFromList);
                //         joint->statSendCounter++;
                //     }

                // }else
                // {
                //     joint->waitingFor05AckTimeOut--;
                //     if (joint->waitingFor05AckTimeOut<=0)
                //     {
                //         joint->waitingFor05AckTimeOut=0;
                //         cout<<"ERROR UPLOADING REG !!!!!" <<flush;
                        
                //         auto frRec=&joint->jointRawFrame80Received[joint->idxRecRead];
                //         joint->ack05Cnt=frRec->rg.pcktCounter8; // FORCE  
                //         joint->isWaitingFor05Ack=false;
                //     }
                   
                //     //rawUDP.sendRegistersToJoint(*joint,joint->savedregNoFromList);
                //     // resend same list (the saved one)
                //     //else 
                //     {
                //       //  if (joint->waitingFor05AckTimeOut%3 ==0 )
                //         {
                //             amyEth.sendRegistersToJoint(*joint,joint->savedSubListregNoFromList);
                //             joint->statSendCounter++;
                //         }
                //     }

                    
                    
                //}
                /*if (sendRegTries>0)
                {
                    rawUDP.sendRegistersToJoint(*joint,sendRegCmd);
                    sendRegTries--;
                }else
                if (search05Ack)
                {
                    rawUDP.sendRegistersToJoint(*joint,0);
                    joint->ack05Cnt++;
                }*/




                if (pullAckMode<4)
                {
                    // subType 0 ("mostly zeros") drops the joint and global flags, so from the reset step of CONFIG on (0x0200,
                    // reset, ENABLE ...) the full frame (subType 2) is sent, like Intera always does
                    int st = cfgSubTypeOverride>=0 ? cfgSubTypeOverride : pullAckMode;
                    if (st==0 && (cfgStep>=CFG_RESET || armStep!=ARM_OFF || sideManualUsed)) st=2;
                    amyEth.sendCmd01(*joint, st); //polling and  ack for 05
                    joint->statSendCounter++;
                }

            }
        }

    };
    {   // gravity model for the torque feed-forward (gravityTick); self-test: arm stretched out, j1 needs about -39.25 Nm
        char err[256]="";
        bool cal = access(GRAVITY_URDF_CAL, R_OK)==0;
        gravityReady = gravityInit(cal ? GRAVITY_URDF_CAL : GRAVITY_URDF, err, sizeof err);
        if (gravityReady && cal) gravityModelName="calibrated (tools/gravcal.py)";
        if (gravityReady)          // torque-sensor offsets (gravcal.py fit --apply, or gravcal.py offsets for j0/j6 alone)
        {
            if (FILE *f=fopen(GRAVITY_OFFSETS_CAL,"r"))
            {
                char name[64]; double v;
                while (fscanf(f, " %63s", name)==1)
                {
                    if (name[0]=='#') { int c; while ((c=fgetc(f))!=EOF && c!='\n'); continue; }
                    if (fscanf(f, "%lf", &v)!=1) break;
                    for (int i=0; i<GRAVITY_JOINTS; i++) if (string(name)==(i==0 ? "head_pan" : "right_j"+to_string(i-1))) gravityOffsetNm[i]=v;
                }
                fclose(f);
            }
            printf("gravity: %s model, offsets [Nm] j0..j6: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", gravityModelName.c_str(),
                   gravityOffsetNm[1], gravityOffsetNm[2], gravityOffsetNm[3], gravityOffsetNm[4], gravityOffsetNm[5], gravityOffsetNm[6], gravityOffsetNm[7]);
        }
        if (gravityReady)
        {
            double q[GRAVITY_JOINTS]={}, tau[GRAVITY_JOINTS]={};
            gravityCompute(q, tau);
            char b[200]; snprintf(b, sizeof b, "%s model loaded (self-test, all joints 0: j1 %.2f Nm, j2 %.2f Nm, j3 %.2f Nm)", gravityModelName.c_str(), tau[2], tau[3], tau[4]);
            gravityMsg=b;
        }
        else gravityMsg=string("NOT available: ")+err;
        printf("gravity: %s\n", gravityMsg.c_str());
    }
    {   // collision model for coordinated moves (link meshes of the official sawyer_description package + table plane)
        loadMoveCfg();
        char err[512]="";
        collisionReady = collisionInit(GRAVITY_URDF, MESH_PACKAGE_DIR, moveCfg.margin, moveCfg.table, moveCfg.tableZ, err, sizeof err);
        char b[200];
        if (collisionReady) snprintf(b, sizeof b, "model loaded (margin %.3f m, table %s, speed %.1f deg/s)", moveCfg.margin, moveCfg.table ? (to_string(moveCfg.tableZ)+" m").c_str() : "none", moveCfg.speed);
        collisionMsg = collisionReady ? b : string("NOT available: ")+err;
        for (auto &o:moveCfg.obstacles)
        {
            char e2[256]="";
            if (collisionReady && collisionAddObstacle(o.name.c_str(), o.type, o.c, o.d, e2, sizeof e2))
                printf("collision: obstacle %s at %.3f %.3f %.3f\n", o.name.c_str(), o.c[0], o.c[1], o.c[2]);
            else if (collisionReady) printf("collision: obstacle %s NOT added: %s\n", o.name.c_str(), e2);
        }
        if (!moveCfg.obstacles.empty()) collisionMsg += ", "+to_string(moveCfg.obstacles.size())+" obstacle(s)";
        if (collisionReady)
            for (auto &p:moveCfg.pairs)
                if (!collisionSetPairMargin(p.a.c_str(), p.b.c_str(), p.m)) printf("collision: pair_margin_m %s %s: pair not checked\n", p.a.c_str(), p.b.c_str());
                else printf("collision: margin %s - %s = %.3f m\n", p.a.c_str(), p.b.c_str(), p.m);
        if (collisionReady && moveCfg.ignore[0])       // head_pan ignored: its angle may be unknown, so its links are not checked
        {
            collisionDisableLink("head"); collisionDisableLink("screen");
            printf("collision: head_pan IGNORED (ignore_joints): not moved, not required, head not collision-checked\n");
            collisionMsg += ", head_pan ignored";
        }
        for (int i=1; i<GRAVITY_JOINTS; i++)
            if (moveCfg.ignore[i]) { printf("collision: %s IGNORED (ignore_joints): not moved, not required - its links are still checked at its last reading\n", sideName(i/2,i%2)); collisionMsg += string(", ")+sideName(i/2,i%2)+" ignored"; }
        printf("collision: %s\n", collisionMsg.c_str());
    }
    // Realtime: lock memory and run the 1 kHz thread with SCHED_FIFO.
    // Needs cap_ipc_lock / cap_sys_nice (setcap line in README); without them AmyCore runs as before, with a warning.
    if (mlockall(MCL_CURRENT|MCL_FUTURE)!=0) printf("realtime: mlockall failed (%s) - add cap_ipc_lock\n", strerror(errno));
    std::thread commThread([&]()
    {
        {
            sched_param sp{}; sp.sched_priority=80;
            int e=pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
            if (e) printf("realtime: SCHED_FIFO not set (%s) - add cap_sys_nice; comm thread runs with normal priority\n", strerror(e));
            else   printf("realtime: comm thread SCHED_FIFO priority 80\n");
        }
        auto next=std::chrono::steady_clock::now();
        while (commRun)
        {
            {
                std::lock_guard<std::mutex> lock(commMutex);
                commStep();
            }
            next+=std::chrono::milliseconds(1);
            auto now=std::chrono::steady_clock::now();
            if (next<now-std::chrono::milliseconds(20)) next=now;   // fell far behind (e.g. GUI held the lock): don't burst
            std::this_thread::sleep_until(next);
        }
    });
    cartRun=true;
    std::thread cartJogThread(cartThreadFn, &jointList);     // Cartesian jog, 50 Hz (normal priority)

    while (!done && !appQuit)
    {
#define GUI        
     #ifdef GUI
     if (!headless)
     {
        std::unique_lock<std::mutex> guiLock(commMutex);   // released before rendering
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)                done = true;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window))                done = true;
        }
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        // ---- main panel: bring-up and power ----
        {
            ImGui::SetNextWindowPos(ImVec2(0,0), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(360,860), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowBgAlpha(1.0f);
            char panelTitle[160];   // "###AmyCore": fixed ID, so imgui.ini keeps the panel's position whatever the version
            snprintf(panelTitle, sizeof(panelTitle), "AmyCore v%s  %s  %s###AmyCore", amyVersion, amyGitRev, amyBuildDate);
            ImGui::Begin(panelTitle);

            ImGui::SeparatorText("Communication");
            if (loopCnt1sec>1100 || loopCnt1sec<850) ImGui::TextColored(colErr, "send loop: %d /s (should be 1000)", loopCnt1sec);
            else                                     ImGui::Text("send loop: %d /s", loopCnt1sec);
            for (int a=0; a<(int)jointList.size(); a++)
            {
                bool alive=boardAlive(jointList,a);
                ImGui::TextColored(alive ? ImVec4(1,1,1,1) : colWarn, "board %d %-8s %-3s %s", a, sideName(a,0), sideName(a,1),
                                   alive ? linkStateName(jointList[a].state) : "no frames");
                if (!alive && !boardDiag[a].empty())
                {
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", boardDiag[a].c_str());
                    ImGui::Indent(); textWrappedColored(colWarn, boardDiag[a]); ImGui::Unindent();
                }
            }
            if (!linkDiagHostOk()) textWrappedColored(colErr, linkDiagHost());

            ImGui::SeparatorText("Power (torso board emulation)");
            if (espDead) ImGui::TextColored(colErr, "ESP32 NOT RESPONDING%s (%d writes dropped)", espReconnecting ? " - resetting its USB" : " - replug its USB", espWriteFails.load());
            if (!espReconnectMsg.empty()) ImGui::TextColored(colWarn, "%s", espReconnectMsg.c_str());
            if (ImGui::Checkbox("Motor power 24V", &rel24V))
            {
                if (!rel24V && waveWanted) waveEnable(false, "24V switched off in GUI"); // P0 drops the wave anyway
                espWrite(rel24V ? "P1" : "P0");
            }
            if (ImGui::Checkbox("Logic power 7.5V (AUX relay)", &relAux))
                if (!auxSet(relAux)) relAux=!relAux;       // refused by the power-cycle timing rules
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("off at least 10 s, on at least 2 min between switches");
            if (!auxMsg.empty()) ImGui::TextColored(colWarn, "%s", auxMsg.c_str());
            bool hbBox=waveWanted;
            if (ImGui::Checkbox("Enable wave (HB, 550 Hz)", &hbBox)) waveEnable(hbBox, "switched off in GUI");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("the joints' external enable; switching it on also switches the 24V on");
            ImGui::SameLine();
            if (!waveWanted)    ImGui::TextDisabled("off");
            else if (hbVerified) ImGui::TextColored(colOk, "on, read-back OK");
            else                 ImGui::TextColored(colWarn, "on, checking");
            if (!waveWanted && !waveStopReason.empty()) ImGui::TextColored(colWarn, "  stopped: %s", waveStopReason.c_str());
            ImGui::TextDisabled("%s", espLastStatus.c_str());

            ImGui::SeparatorText("Bring-up");
            static const char *startStateName[]={"idle","waiting for joints","reset broadcast","joints starting","joints running","joints running"};
            if (ImGui::Button("1. Connect joints")) startSequence=true;
            ImGui::SameLine();
            if (startSequence)          ImGui::Text("%s", startStateName[stateMachine<6 ? stateMachine : 0]);
            else if (stateMachineERROR) ImGui::TextColored(colErr, "timed out, try again");
            else if (stateMachine==STATE_POLLING_22) ImGui::TextColored(colOk, "done");
            else                        ImGui::TextDisabled("idle");

            ImGui::BeginDisabled(!(cfgStep==CFG_OFF || cfgStep==CFG_DONE));
            if (ImGui::Button("2. Configure joints")) cfgStart(jointList);
            ImGui::EndDisabled();
            ImGui::SameLine(); ImGui::Text("%s", cfgStepName[cfgStep]);
            if (!cfgMsg.empty()) ImGui::TextWrapped("  %s", cfgMsg.c_str());

            if (armStep==ARM_OFF) { if (ImGui::Button("3. Enable arm")) armEnableStart(jointList); }
            else
            {
                if (ImGui::Button("Disable arm")) armDisable(jointList, "disabled from GUI");
                if (armStep==ARM_ENABLED) { ImGui::SameLine(); if (ImGui::Button("Retry refused")) armRetryStart(jointList); }
            }
            ImGui::SameLine(); ImGui::Text("%s", armStepName[armStep]);
            if (!armMsg.empty()) ImGui::TextWrapped("  %s", armMsg.c_str());
            if (armStep==ARM_OFF && ImGui::TreeNode("Enable arm options"))
            {
                ImGui::Checkbox("continue past joints that refuse (test setup)", &armContinuePastFail);
                ImGui::Checkbox("skip joints with HES config error (0x100)", &armIgnoreHesCfg);
                ImGui::TreePop();
            }

            if (ImGui::Button("4. Home all joints")) homeAllStart(jointList);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("AUTO_HOME each enabled joint that isn't homed, one at a time:\nthe next starts 3 s after the previous one is HOMED; stops if one fails.\nPreferred: home each joint yourself with its Home (auto) button in the Joints window.");
            ImGui::SameLine();
            if (!homeQueue.empty())
            {
                ImGui::Text("homing %s (%d left)", sideName(homeQueue[0].first, homeQueue[0].second), (int)homeQueue.size());
                ImGui::SameLine(); if (ImGui::Button("Stop home all")) { homeQueue.clear(); homeSideStartMs=0; printf("home all cancelled\n"); }
            }
            else                    ImGui::TextDisabled("idle");
            ImGui::Text("Gravity comp.:");
            ImGui::SameLine(); if (ImGui::Button("all on"))  for (int a=0; a<4; a++) for (int s=0; s<2; s++) gravitySet(a,s,true);
            ImGui::SameLine(); if (ImGui::Button("all off")) for (int a=0; a<4; a++) for (int s=0; s<2; s++) gravitySet(a,s,false);
            ImGui::SameLine();
            {
                int n=0; for (auto &r:gravityOn) n+=r[0]+r[1];
                if (!gravityReady) ImGui::TextColored(colErr, "model not loaded");
                else ImGui::TextDisabled("%d of 8 on%s", n, gravityPoseOk ? "" : ", waiting for homing");
            }
            {
                bool t=teachWanted;
                if (ImGui::Checkbox("Manual teaching (free drive)", &t)) teachSet(t, "GUI");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("like Intera with the cuff gripped: every arm joint gets stiffness 0 and control mode 10,\n"
                    "the setpoint follows the joint and only the gravity torque is sent, so the arm floats and can be guided by hand.\n"
                    "Untick to hold: the arm stays where it is, stiffness ramps back in 20 ms, feed-forward from the\n"
                    "measured effort to gravity in 0.6 s. Needs every arm joint ENABLED + HOMED and gravity compensation fully on;\n"
                    "ends by itself if that stops. If the gravity model is wrong the arm sags: keep a hand on it. Pipe: teach on|off");
                ImGui::SameLine();
                if (!teachMsg.empty()) textWrappedColored(teachState==TEACH_ON ? colOk : (teachMsg.find("REFUSED")!=string::npos || teachMsg.find("ENDED")!=string::npos ? colWarn : colOk), teachMsg);
                else ImGui::TextDisabled("off");
                static int stAll=100;
                ImGui::SetNextItemWidth(200);
                if (ImGui::SliderInt("stiffness all joints [%]", &stAll, 0, 100)) for (int a=0; a<4; a++) for (int s=0; s<2; s++) stiffWant[a][s]=stAll;
                if (ImGui::IsItemDeactivatedAfterEdit()) printf("stiffness: all %d %%\n", stAll);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("command payload 14/32: stiffness of the joints' position loop in %% (Intera: always 100 outside manual teaching).\n"
                    "Lower = softer, the joint gives way when pushed. Goes down at once, up by 5 %% per ms. Pipe: stiffness <side>|all <0-100>");
            }
            ImGui::BeginDisabled(!collisionReady || mv.st!=MV_IDLE);
            if (ImGui::Button("5. Go to home posture")) homePoseAllStart(jointList);
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("all joints together to AmyConfig/home_pose.txt on a straight joint-space path, slowly (AmyConfig/collision.txt);\nthe whole path is checked for collisions (link meshes, table plane) first, and the move stops if a joint falls behind.");
            if (mv.st!=MV_IDLE) { ImGui::SameLine(); if (ImGui::Button("STOP move")) moveStop(jointList, "STOP pressed"); }
            if (!collisionReady) textWrappedColored(colErr, "collision model: "+collisionMsg);
            else if (!mv.msg.empty()) textWrappedColored(mv.msg.find("REFUSED")!=string::npos || mv.msg.find("STOPPED")!=string::npos ? colWarn : colOk, "move: "+mv.msg);

            ImGui::SeparatorText("Windows");
            ImGui::Checkbox("Joints", &showJointControl);
            ImGui::SameLine(); ImGui::Checkbox("Joint detail", &showDetail);
            ImGui::SameLine(); ImGui::Checkbox("Console", &showConsole);
            ImGui::Checkbox("Config registers", &show_registers);
            ImGui::SameLine(); ImGui::Checkbox("Expert / protocol", &show_moveControl);
            ImGui::Checkbox("LEDs & buttons", &showLeds);             // own row: the panel is too narrow for four
            ImGui::SameLine(); ImGui::Checkbox("Cartesian jog", &showCart);
            ImGui::TextDisabled("command API: tools/amyctl   GUI %.0f fps", io.Framerate);
            ImGui::End();
        }

        if (showConsole) consoleLog.draw(&showConsole);
        histSample(jointList);
        if (showJointControl) drawJointsWindow(jointList, &showJointControl);
        if (showDetail) drawJointDetail(jointList, &showDetail);
        if (show_registers) drawRegistersWindow(jointList, &show_registers);
        if (showLeds) drawLedsWindow(jointList, &showLeds);
        if (showCart) drawCartWindow(jointList, &showCart);

        // ---- expert / protocol window: raw controls for development, they bypass the enable logic ----
        if (show_moveControl)
        {
            ImGui::SetNextWindowPos(ImVec2(900,450), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(600,700), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowBgAlpha(1.0f);
            ImGui::Begin("Expert / protocol", &show_moveControl);
            ImGui::TextColored(colWarn, "Raw protocol controls for development.\nThey bypass the arm enable logic and its safety checks.");

            ImGui::PushItemWidth(140);
            ImGui::SeparatorText("Global command flags (every board)");
            int g47=jointList[0].debugIntValue1, g45=jointList[0].debugIntValue2;
            if (ImGui::InputInt("byte 47 (0x20 = safety controller resume)", &g47, 1, 16, ImGuiInputTextFlags_CharsHexadecimal))
                for (auto &j:jointList) j.debugIntValue1=g47&0xff;
            if (ImGui::InputInt("byte 45 (0x7c = nav LEDs A-E)", &g45, 1, 16, ImGuiInputTextFlags_CharsHexadecimal))
                for (auto &j:jointList) j.debugIntValue2=g45&0xff;

            ImGui::SeparatorText("Frame sending");
            ImGui::TextUnformatted("command frame subtype:");
            for (int m=0; m<5; m++) { ImGui::SameLine(); ImGui::RadioButton(m<4 ? to_string(m).c_str() : "4 = none", &pullAckMode, m); }
            ImGui::Checkbox("broadcast always (the joints need it after LOAD APP)", &brodcastAllDay);
            ImGui::Checkbox("broadcast MAC table", &brodcastMacEn);
            ImGui::TextUnformatted("print frames of board:");
            for (int m=0; m<5; m++) { ImGui::SameLine(); ImGui::RadioButton(m<4 ? (to_string(m)+"##pr").c_str() : "none##pr", &debugPrintJ, m); }

            ImGui::SeparatorText("Raw command per joint");
            for (int a=0; a<(int)jointList.size(); a++)
                for (int s=0; s<2; s++)
                {
                    auto &j=jointList[a];
                    ImGui::PushID(100+a*2+s);
                    if (ImGui::TreeNode(sideName(a,s)))
                    {
                        unsigned f=getJointFlags(j,s), f0=f;
                        ImGui::CheckboxFlags("ENABLE", &f, JF_ENABLE);               ImGui::SameLine();
                        ImGui::CheckboxFlags("RESET_LATCHED", &f, JF_RESET_LATCHED); ImGui::SameLine();
                        ImGui::CheckboxFlags("AUTO_HOME", &f, JF_AUTO_HOME_UNLOCK);
                        ImGui::CheckboxFlags("MANUAL_HOME", &f, JF_MANUAL_HOME_UNLOCK); ImGui::SameLine();
                        ImGui::CheckboxFlags("HEAD_PAN", &f, JF_HEAD_PAN);           ImGui::SameLine();
                        ImGui::Text("= 0x%04x", f);
                        if (f!=f0) { setJointFlags(j,s,(uint16_t)f); sideManualUsed=true; }
                        ImGui::Text("payload 14/18 (side 2: 32/36): stiffness %d %%, control mode %d (set in Joint detail / main panel)",
                                    s==0 ? j.stiff1 : j.stiff2, s==0 ? j.ctrlMode1 : j.ctrlMode2);
                        ImGui::SliderInt("torque feed-forward (raw)", s==0 ? &j.effortSP1 : &j.effortSP2, -8000, 8000);
                        if (ImGui::IsItemHovered()) ImGui::SetTooltip("bytes 8-9 / 26-27 of the command (called speedSP in the code).\nIntera sends a per-joint value here once the joint is homed that changes with the arm's pose:\ngravity compensation (2023 capture: j1 about -5900..-6900, others a few hundred).");
                        ImGui::SliderInt("jog speed (raw)", s==0 ? &j.jogMoveSP1 : &j.jogMoveSP2, -5000, 5000);
                        ImGui::TreePop();
                    }
                    ImGui::PopID();
                }

            ImGui::PopItemWidth();
            ImGui::SeparatorText("Debug variables (motor diagnostics stream, set up by CONFIG)");
            for (int a=0; a<(int)jointList.size() && a<4; a++)
            {
                string l="board "+to_string(a);
                if (ImGui::TreeNode(l.c_str()))
                {
                    // pages set up by CONFIG (dbgStreamQueue): 0 = MOTOR diagnostics, 1 = hall sensors
                    for (int p=0; p<2; p++)
                        for (int e=0; e<dbgPageCount(p); e++)
                        {
                            uint16_t v; memcpy(&v, &dbgRaw[a][p][2*e], 2);
                            if (dbgPages[p][e].hex) ImGui::Text("%-24s 0x%04x", dbgPages[p][e].name, v);
                            else                    ImGui::Text("%-24s %u", dbgPages[p][e].name, v);
                        }
                    ImGui::TextDisabled("all values also in /tmp/amycore.dbgvars");
                    ImGui::TreePop();
                }
            }
            ImGui::End();
        }

        guiLock.unlock();
        // Rendering
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    } // gui subloop
    #endif
        {   // debug variables: collected in GUI and headless mode
            std::lock_guard<std::mutex> dbgLock(commMutex);
        // debug variables arrive page by page in the 0x32 frames; collected here for the expert window
        for (auto &j:jointList)
        {
            auto d=j.jointVF;
            int jn=j.privNo, no=d.vf.dbgPage;
            if (jn<0 || jn>=4 || no<0 || no>=23) continue;
            int n=debugVariables.ParseDebugFrame(no,&d.vf.dbgVarBuff20[0]);
            for (int x=0; x<n && x<7; x++) debugVars[jn][no][x]=debugVariables.agnoVal[x];
            memcpy(dbgRaw[jn][no], &d.vf.dbgVarBuff20[0], 20);
        }
        // the same values as a file for offline comparison (page layout: sawyerFrames.cpp DebugVariables)
        {
            static long long lastDbgFileMs=0;
            long long now=steadyMs();
            if (now-lastDbgFileMs>=500)
            {
                lastDbgFileMs=now;
                if (FILE *df=fopen("/tmp/amycore.dbgvars.tmp","w"))
                {
                    fprintf(df, "time %lld\n", now);
                    for (int a=0; a<4; a++) for (int l=0; l<23; l++)
                    {
                        fprintf(df, "board %d page %2d: %d %d %d %d %d %d %d  raw", a, l, debugVars[a][l][0], debugVars[a][l][1], debugVars[a][l][2],
                                debugVars[a][l][3], debugVars[a][l][4], debugVars[a][l][5], debugVars[a][l][6]);
                        for (int x=0; x<20; x++) fprintf(df, " %02x", dbgRaw[a][l][x]);
                        fprintf(df, "\n");
                    }
                    for (int a=0; a<4; a++) fprintf(df, "board %d decoded: %s\n", a, dbgDecode(dbgRaw[a]).c_str());
                    fclose(df);
                    rename("/tmp/amycore.dbgvars.tmp", "/tmp/amycore.dbgvars");
                }
            }
        }

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(8));   // GUI ~100 fps is plenty
    }

    cartRun=false; cartJogThread.join();
    commRun=false;
    commThread.join();
    if (mv.worker.joinable()) mv.worker.join();   // a path check still running (a joinable std::thread would terminate)
    waveEnable(false, "exit"); // the ESP32 watchdog would stop it anyway within 50 ms

    if (headless) return 0;
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;


}

