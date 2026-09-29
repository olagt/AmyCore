// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#pragma once
#include "udp_sender.h"
#include "amyEth.h"

// ---- robot network ----
// The PC's Ethernet interface wired to the robot's internal network (here the NUC's onboard port). Change the default
// here, or start AmyCore with --if <name>. The DHCP server (udhcpd.conf, "interface") must use the same interface.
#define DEFAULT_IF "eno1"
extern std::string robotIf;          // the interface in use: DEFAULT_IF, or --if <name>

// position hold (holdSetpoint in amyEth.cpp): jog speed divisor and the most the target may lead the joint
const int HOLD_JOG_DIV = 16;          // jog 1500 -> 94 counts per ms, about 5 deg/s
const int HOLD_MAX_LAG = 53000;       // counts, about 3 deg (1/1024 mrad units)
const int32_t HOLD_WRAP_JUMP = 1608495;  // counts, 90 deg: a bigger position jump in one frame is a sensor wrap-around
const double LIMIT_MARGIN_DEG = 2.0;   // jog/go-to stop this far inside the joint limits
// joint limits in degrees per [AmyCore board][side] (Intera's URDF in param_dump.yaml, right_j* / head_pan <limit>)
extern const double jointLimitDeg[4][2][2];
// global command flag bits the LED window may set (JRCP_SEA_CCMD_GLOBAL_FLAG_*): nav LEDs A-E 0x7c00, red 0x8000,
// green 0x10000, blue 0x20000, LCD display power 0x40000, lamp test 0x10000000
const uint32_t LED_GLOBAL_MASK = 0x7c00 | 0x8000 | 0x10000 | 0x20000 | 0x40000 | 0x10000000;
class JointJCB;
int32_t holdSetpoint(JointJCB &joint, int side);

class AmyEth
{
private:
    /* data */
public:
    AmyEth(    vector <JointJCB> &jointList_ );
    ~AmyEth();
    int OpenUDP_IP();
    int SendLOADAPP2JointsUDP();
    bool InitJointsManual();
    int sendCmd01(JointJCB &joint, int subType_);
    int ProcessRegistersComm(JointJCB &joint);
    int ProcessRegistersComm05(JointJCB &joint);
    int sendRegistersToJoint(JointJCB &joint, vector<int> regListFromListToSend_); // only 0x04 i 0x05
    int sendPollingBroadcastEn(int type_);
    int waitForJointsUDP();
    int DoTFTP();

    vector <JointJCB> *jointList;

    
    //struct ether_header *eh = (struct ether_header *) recBuf;

    UDPSender udpSender=UDPSender();
    UDPSender rawUDP=UDPSender();
    UDPSender rawUDPListener=UDPSender();
};