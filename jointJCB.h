// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#pragma once

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>
#include <limits>

using namespace std;


#include "sawyerFrames.h"

#define REGCOMM_NULL 0
#define REGCOMM_INITANDSEND 1
#define REGCOMM_WAIT4ACK 2
#define REGCOMM_SOMEDELAY 3
#define REGCOMM_DONE 4
#define CTRL_MODE_POSITION 7     // command payload 18/36: position (impedance around the setpoint)
#define CTRL_MODE_TEACH    10    // manual teaching / free drive (Intera sends it only with stiffness 0)


typedef  vector <uint8_t> MacAddress ;

class JointJCB
{
    public:
    JointJCB()
    {ipAdd="";    ident1="";    ident2="";    state=0;    enabled=0;    isGatewayed=false;
        privNo=0; 
        regCommStage=REGCOMM_NULL;
        loadRegCounter=0xFFFF; 
        registersUploaded=true;
        
        framePcktCounterUploadReg=0x01;
        ackFramePcktCounterUploadReg=0;
        //counterThe=0;ackCounterThe=0;
        framePcktCounter=0;
        registerAcksent=true; // we start optimistic 
        //isWaitingFor05Ack=false;
       // waitingFor05AckTimeOut=0;
        delayCounter=0;
        savedSubListregNoFromList.clear();
        
        valTabOffset=0;
        // valTab1={0.0f};
        // valTab2={0.0f};
        // valTab3={0};
        // valTab4={0};
        frame04Cnt=0;
        //ack05Cnt=0;
        cmd01Ack05Cnt=0;
        //sendAckForRegNo=-1;
        retry04Counter=0;
        idxRecWrite=0;
        idxRecRead=0;
////////////////////////////////////

        brakeEngaged1=true;
        brakeEngaged2=true;

        moveEnabled1=false;
        moveEnabled2=false;
 	    
        encoderPos1=std::numeric_limits<int32_t>::max();
        encoderPos2=std::numeric_limits<int32_t>::max();
        
        encoderSP1=encoderPos1;
        encoderSP2=encoderPos2;

       
        jogMoveSP1=0;
        jogMoveSP2=0;
        effortSP1=0;
        effortSP2=0;

        statRecCounter=0;
        statSendCounter=0;
        Cmd2Joint1=0;
        Cmd2Joint2=0;
        CmdBrake2Joint1=0;
        CmdBrake2Joint2=0;

    // debugIntValue1 -> command frame byte 47 = global flags bits 24-31 (0x20 SAFETY_CTRLR_RESUME, 0x08 ASSERT_ESTOP, ...): must be 0
    // debugIntValue2 -> byte 45 = global flags bits 8-15: 0x7c = nav LEDs A-E, what Intera sends
    debugIntValue1=0x00;
    debugIntValue2=0x7c;

    debugIntValue3=0x7c;
     debugIntValue4=0xd0;

    };
    
    //Move controll 0x00  

    uint16_t Cmd2Joint1;
    uint16_t Cmd2Joint2;

    uint8_t CmdBrake2Joint1;
    uint8_t CmdBrake2Joint2;

    // stiffness and control mode of each side, command payload 14/18 (side 2: 32/36), docs/JRCP_PROTOCOL.md 7.1:
    // stiffness in % (100 normally, 0 in manual teaching), mode CTRL_MODE_POSITION / CTRL_MODE_TEACH. Set by
    // teachTick in main.cpp (it also ramps the stiffness up by 5 % per ms, like Intera).
    uint8_t stiff1=100, stiff2=100;
    uint8_t ctrlMode1=CTRL_MODE_POSITION, ctrlMode2=CTRL_MODE_POSITION;

    bool brakeEngaged1;
    bool brakeEngaged2;

    bool moveEnabled1;
    bool moveEnabled2;

    int32_t encoderPos1;
    int32_t encoderPos2;

    int32_t encoderSP1;
    int32_t encoderSP2;

    int32_t jogMoveSP1;   // jog speed: the held target moves by jogMoveSP/HOLD_JOG_DIV counts per frame (ms)
    int32_t jogMoveSP2;

    // position hold (like Intera): while a side reports ENABLED + HOMED its target is fixed and only moved by jog/go-to
    bool    hold1=false, hold2=false;
    int32_t holdSP1=0, holdSP2=0;
    // LED window: display bits of the global command flags (payload 44-47, masked with LED_GLOBAL_MASK) and the
    // cuff (ITB) light bits of each side's joint flags (0x10/0x20 ITB inner/outer, 0x40/0x80 ITB1 inner/outer)
    uint32_t ledFlags=0;
    // gravity compensation feed-forward per side (gravityTick in main.cpp), added to effortSP in the command frame
    int32_t  gravityFF1=0, gravityFF2=0;
    uint8_t  itbLights1=0, itbLights2=0;
    int32_t lastPos1=0, lastPos2=0;   // previous frame's position, to detect sensor wrap-around
    bool    wrapLock1=false, wrapLock2=false;   // after a wrap-around: jog ignored until the side is disabled

    //int16_t effortSP1;
    //int16_t effortSP2;
    int32_t effortSP1; // becouse of ImGui
    int32_t effortSP2;
/////
    int debugIntValue1;
    int debugIntValue2;

    int debugIntValue3;
    int debugIntValue4;

/////////////////////////////////////////////////////////////////
    // Registers related
    int regCommStage;
    
    vector <JointRegisters_t> registerList; 
    vector <int> regNoToUploadList; // list of id to uload 
    
    vector<int> savedSubListregNoFromList;

    //uint8_t ack05Cnt; // ack for recived frame 05 
    uint8_t cmd01Ack05Cnt;
    uint8_t frame04Cnt;
    
    //int sendAckForRegNo;
    //bool isWaitingFor05Ack;
    //int waitingFor05AckTimeOut;
    int retry04Counter;
    int delayCounter;
    
    JointStatus_t jointUploadRegCopy;
   
    uint8_t framePcktCounterUploadReg;  // [0x4]
    uint16_t ackFramePcktCounterUploadReg; // [0x05] - from joint 
    uint8_t retryCntUploadReg;
    
    uint16_t loadRegCounter;
    bool registerAcksent;
    bool registersUploaded;
    
////////////////////////////////////////////////////////////////
    // for graph 
    float valTab1[90];
    float valTab2[90];
    float valTab3[90];
    float valTab4[90];
    int valTabOffset;

    uint8_t privNo;
    uint8_t jointHwNum;
    bool isGatewayed;
    string ipAdd;
    MacAddress mac;

    int enabled;
    int state;
    
    uint16_t framePcktCounter;
   

    string ident1;
    string ident2;
    vector <uint8_t> fileDATABLOCK;
    vector <uint8_t> fileBLI;
    vector <uint8_t> fileGW_BLI;
    vector <uint8_t> fileGW_DATABLOCK;
    JointStatus_t jointRawFrame80Received[5];
    JointStatus_t jointVF;
    int idxRecRead;
    int idxRecWrite;
    bool newFrameRecived;
    int statRecCounter;
    int regFailed=0;      // register upload frames that were never acknowledged
    int regRefused=0;     // single writes the board answered with 53 <code> (refused)
    int statSendCounter;


    std::vector<double> plotData[10];

};