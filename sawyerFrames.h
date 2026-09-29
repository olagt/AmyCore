// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#pragma once

//#ifndef SAWYER_FRAMES
//#define SAWYER_FRAMES 1

#include <algorithm>
#include <cstdint>
#include <string>
#include <sstream>
#include <iostream>

#include <cstdlib>
#include <vector>
#include <iomanip> // for setprecision

//#include "jointJCB.h"


#define JCB_REG_TYPEUNKNOWN 0
#define JCB_REG_TYPE4102 1
#define JCB_REG_TYPE4104 2
#define JCB_REG_TYPE4302 3
#define JCB_REG_TYPE4304 4

#define JCB_REG_TYPE0302 5 
#define JCB_REG_TYPE032a 6

#define JCB_DVAR_TYPE0202 2
#define JCB_DVAR_TYPE0402 4


using namespace std;


#pragma pack (1)
typedef struct //80 bytes from joints and poll from joint
{
    uint8_t dstMAC[6];
    uint8_t srcMAC[6];
    uint8_t type[2]; //"JF"
    union // anonymous
    {
        uint8_t data[66];
        struct // info frame (f8)
        {
           uint8_t subAddress[2];// 01 - gw from node
                                 // 02 - address TO node from PC
           uint8_t jointHwNum[1]; // f8-  info Frame     // 08,0x10 ,0x18,0x20
           uint8_t msgtyp40[1]; // mostly 0x40
           uint8_t msgtyp40_2[1];
           uint8_t someStatus[1]; // 4 or 5 for GW
           uint8_t placeh1[7];
           uint8_t buildInfoStr[17]; //00016 1710252050
           uint8_t placeX[5];
           uint8_t serial2No[7];
           uint8_t placeh2[2];
           uint8_t nodeMac[6];
           uint8_t placeh3[2];
           
        };
        struct // info frame (f8)
        {
           uint8_t subAddress[2];// 01 - gw from node
                                 // 02 - address TO node from PC
           uint8_t jointHwNum[1]; // f8-  info Frame     // 08,0x10 ,0x18,0x20
           uint8_t msgtyp40[1]; // mostly 0x40
           uint8_t msgtyp40_2[1];
           uint8_t someStatus[1]; // 4 or 5 for GW
           uint8_t placeh1[7];
           uint8_t buildInfoStr[17]; //00016 1710252050
           uint8_t placeX[5];
           uint8_t serial2No[7];
           uint8_t placeh2[2];
           uint8_t nodeMac[6];
           uint8_t placeh3[2];
           
        }xx;  
   //                                0   1 2
        //framerec type:22 [45362] 00 00 00 
// 3 4  5   6  7 8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46
//40 22 00 32 b1 00 00 c4 b7 d9 ff ff ff 00 00 b9 ff 0c bf 08 05 00 00 37 fd ff ff ff ff 35 00 dc ff 00 00 00 00 00 00 00 00 b1 09 ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff
        struct // value frame typ 0x22 0x32 etc
        {
           uint8_t subAddress[2]; // 0:
                                  // 1:
           uint8_t jointHwNum[1];    // 2:  
           uint8_t msgtyp40[1];   // 3:
           uint8_t msgtyp40_2[1];  // 4
           uint8_t someStatus[1]; // 5
           uint16_t pcktCounter; //6
                                //7
           
           int8_t ackX05;  //8
           uint8_t dbgPage;    //9  // triangle shape ? no idea always same fluctuations 0x00to 0x15 ?
                                // to sa chyba rejestry jakie sa obecne w ostatnich 16 bajtach
                                //gd 00 poprawne dane enkodera ! no i ponoc to na koncu ffffffffffffff 
           int32_t encoder1;  //10
                                //11
                                //12
                                //13
           int16_t speed1;       // 14:
                                //15
           int16_t effort1; //16:  // ar they are together 
                            //17
           int16_t force1;  //18 force ? 
                            //19
           uint8_t stFlags1;     //20: // statusFlag 
           int8_t encKnob;       //21 absolute knob position for joint0 
           int16_t jErFlags1;    //22: 0x080x05 ? ALWAYS 0000 ?
                            //23
           //uint8_t unk32[2]; //24 
           int16_t speedSpecial1; //24  some other speed /effort 
                            //25


           int32_t encoder2; //26 
                              //27
                              //28
                              //29
           int16_t speed2;   // 30: speed node2 
                            //31
           int16_t effort2;     
           int16_t force2;     
           //uint8_t unk4[4]; //32
                        //33
                        //34
                        //35
           uint8_t stFlags2; //36   // should be 8 bit ? 
           int8_t encKnob2Maybe; 
           int16_t jErFlags2; //38  // zeros al the time ? 
                                //39
           int16_t unkAnalog;  //40 only when moves some analog 
                                //41
           uint8_t globalErrFlag; // JRCP_SEA_CRSP_GLOBAL_ERROR_UNDERVOLTAGE 2
           uint8_t valbtn; // 42 bit button pressed 
           int16_t val7;    //43 SDS offset ?
                            //44
           uint8_t dbgVarBuff20[20]; // DebugVariables 
        }vf;
        struct // registers frame to and from Joint , to joint 0x04 , ack from joint 0x05
        {
           uint8_t subAddress[2]; // 0:
                                  // 1:
           uint8_t jointHwNum[1];    // 2:  
           uint8_t msgtyp40[1];   // 3:
           uint8_t msgtyp40_2[1];  //0x04 from PC to J , 0x05 from Joint
           uint8_t someStatus[1];
           uint8_t pcktCounter8; 
           uint8_t dummy3a[1];    // mostly 3a
           uint8_t regbuff[58];
        }rg;

        struct // keep alive frame to joint and polling Command Frames  name: CCMDFrames}
        {
           uint8_t subAddress[2]; // 0:
                                  // 1:
           uint8_t jointHwNum[1];  
           uint8_t msgtyp40[1];   // 3:
           uint8_t msgtyp40_2[1];  //0x01 - keepAlive frame - 0x04 from PC to J , 0x05 from Joint 
           uint8_t ackFor05[1];  // frame numer cnt for send 04 for ack , otherwise 1 or 58,5c,5a
           uint16_t pcktCounter; // always 0 ? 

           int16_t speedSP1; // effort not speed ?
           int32_t encoderSP1; 
           uint8_t you64[1]; //always 64 - when askig , 00 where poll1 is command 
                              //64 - READ , 00 - WRITE to Joint
                            // this is manualteaching mode !? 
           
           uint8_t bond07[4]; // 00 00 00 07 // samoetime 07 ->0a
           //uint8_t someZeros2[2]; // or 00 01 , when 00 00 - disabled    /emergency stop
           //JRCP_SEA_CCMD_JOINT_FLAG_MANUAL_HOME_AND_UNLOCK_REQUEST: 512
           uint16_t jCmdFlag1; // or 00 01 , when 00 00 - disabled    /emergency stop

           uint8_t brake1[1]; //00 or 02 or 06 , 01 - maybe brake ! (when emergency stop)
           uint8_t someZeros4[4]; //00 00 00 00 00 

           int16_t speedSP2;
           int32_t encoderSP2;
           uint8_t you64_bis[1]; //always 64 ? 
           uint8_t bond07_bis[4]; // 00 00 00 07 
           //uint8_t someZeros2_bis[2]; // mostly 00 01 same as prima
           uint16_t jCmdFlag2; // mostly 00 01 same as prima
           uint8_t brake2[1]; //02 same as prima 
           uint8_t someZeros5_bis[5]; 
           uint8_t lightsBtnBit[1]; // 7c or 00 or 7e or 01 brake ? 00- task is pause , 01 - pause ongoing [not a brake for sure !]
           uint8_t someZero3[1];
           uint8_t lightsHead[1];//00 or 20
           uint8_t unknonwRest[18]; // zeros

        }kaCmd01;
      
    };


}JointStatus_t;
static_assert (sizeof(JointStatus_t) == 80, "Size is not correct JointStatus_t (should be 80)");


typedef struct // 84 bytes
{
    uint8_t dstMAC[6]; // ffffffffff
    uint8_t srcMAC[6];
    uint8_t type[2]; //"JF"
    union // anonymous
    {
        uint8_t data[70];
        struct  // bringing joint into working mode / enable frame broadcast
         {
            uint8_t subAddress[2]; // 0x01 when long packet
                                    
            uint8_t jointHwNum[1]; // f8-  info Frame     // 08,0x10 ,0x18,0x20
            uint8_t msgtyp40[1]; // mostly 0x40
            uint8_t msgtyp40_2[1]; // 20 for enable broadcast, d0 reset/startup
            uint8_t someStatus[1]; // 4 or 5 for GW
            uint16_t pcktCounter; // [lowbyte][highbyte]
            uint8_t mac1Id[1]; //0x80
            uint8_t mac1[6];
            uint8_t mac2Id[1]; //0x81
            uint8_t mac2[6];
            uint8_t mac3Id[1]; //0x82
            uint8_t mac3[6];
            uint8_t mac4Id[1]; //0x83
            uint8_t mac4[6];
            uint8_t dummy2[1];
            uint8_t some80[1];// should be x80 ?
            uint8_t unknown[2];// fillit up
        }en;
         struct  // all day broadcast 
         {
            uint8_t subAddress[2];
                                    
            uint8_t jointHwNum[1]; // f8-  info Frame     // 08,0x10 ,0x18,0x20
            uint8_t msgtyp40[1]; // mostly 0x40
            uint8_t msgtyp40_2[1]; // 20 for enable broadcast, d0 reset/startup
            uint8_t someStatus[1]; // 4 or 5 for GW
            uint16_t pcktCounter; // [lowbyte][highbyte]
            uint8_t fill [1];
            uint8_t rest[61] ;

        }allday; // funny name 
    };

}FrameBroadcast84_t;

static_assert (sizeof(FrameBroadcast84_t) == 84, "Size is not correct FrameBroadcast84_t ");


#pragma pack (0)







typedef  vector < vector <uint8_t>  > PacketList ;

void HexStringToVector(string hexString,std::vector<uint8_t> &vector_);

typedef struct 
{
    uint8_t firstType; //variable type 
    uint8_t secType; //
    uint16_t type16;
    //string name;
    uint8_t len; //
    int32_t value;
    std::vector <uint8_t> rawData;
    //uint8_t msgType;
    //uint8_t jointHwNum;
}JointDVariable_t;





class DebugVariables
{
private:
public:
    DebugVariables
(/* args */);
    ~DebugVariables
();

    vector <uint8_t> rawDataVec[23];
    vector <JointDVariable_t> variableList[23];
    
    int ParseDebugFrame(uint varFrameNo, uint8_t * rawFrame20);

    uint16_t temperature0;
    uint16_t temperature1;
    
    uint16_t busVoltage0;
    uint16_t busVoltage1;

    uint32_t upTime0;
    uint32_t upTime1;

    uint32_t agnoVal[10];

};


typedef struct 
{
    uint8_t type; //register type 
    uint8_t regNo; // reg number 
    uint8_t valSubType; // subtype 
    int32_t value;
    std::vector <uint8_t> rawData;
    uint8_t msgType;
    uint8_t jointHwNum;
}JointRegisters_t;

typedef struct //
{
    uint8_t type; //register type 
    uint8_t regNo; // reg number 
    
    int32_t valSub_2[50]; // subtype
    int32_t valSub_4[50]; // subtype

    bool ackOverwrite;

    int32_t value;
    std::vector <uint8_t> rawData;
    uint8_t msgType;
    uint8_t jointHwNum;
}RegisterRow_t;


typedef struct //
{ // "encoder",11,2,no,j0,j1,j2,j3,j4,j5,
    string name;
    uint8_t subtype;//
    uint8_t length; //
    uint8_t regNo; // reg number 
    
    int32_t value[8]; // 
    bool nonempty[8];
    std::vector <uint8_t> rawData[8];
    
}AmyRegister_t;



//std::string to_strP(const T a_value, const int n = 2)
std::string to_strHEX(uint8_t * b,int n);
std::string to_str(uint32_t i,int n);
void HexStringToVector(string hexString,std::vector<uint8_t> &vector_);

int coutAnsiBraodFromPC(FrameBroadcast84_t f);
int coutAnsiFrame01(JointStatus_t f);
int coutAnsiFrameFromJ(JointStatus_t f);
int coutAnsiFrameRegisters(JointStatus_t f);
string stFlagToString(uint8_t stFlag);
string erFlagToString(uint16_t erFlag);
string globalErFlagToString(uint16_t erFlag);

string parsePayloadComl(char *buff_, int size_ ,vector <JointRegisters_t> &regList_ ,int msgType_,uint8_t jointHwNum_);
string vectorToHexStr(vector<uint8_t> vec_);
bool equalAr(char* a, int l , char* b );
void savePacketsToFile(string filename_,PacketList &packetList_ );
bool loadPacketsFromFile(string filename_,PacketList &packetList_ );

int saveRobotRegisters(string filename );
int loadRobotRegisters(string filename );
int amyRegisters2regList(int jHW_, vector <JointRegisters_t> & regList_);
int prepareReg2Send(JointRegisters_t & reg,vector<uint8_t> &regbufVal,vector<uint8_t> &regbufAck);
// JRCP message CRC (docs/JRCP_PROTOCOL.md): STM32 hardware CRC, CRC-32/MPEG-2 over the 64-byte message as 16 LE words
uint32_t jrcpCrc(const uint8_t *msg64);
void saveJointRegFile(string filename_,vector <JointRegisters_t> &regList_,int mode_);
bool loadJointRegFile(string filename_,vector <JointRegisters_t> &regList_ );

//void initRegistersManualFIX(vector <JointJCB> &jointList);


// auto subtypeToIdx=[&](int x) 
// {  
//     for (int i=0;i<=subList.size();i++)
//     {
//         if (subList[i]==x) return i;
//     }
//     cout << " big error reg type out of range : " <<+x<<" \r\n" <<flush;
//     return -1;
// };



#ifndef _COLORS_
#define _COLORS_



#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

#endif  /* _COLORS_ */
#define COLORS 2
#ifdef COLORS
const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string blue("\033[1;34m");
const std::string magenta("\033[0;35m");
const std::string cyan("\033[0;36m");
const std::string white("\033[0;37m");
const std::string reset("\033[0m");
#else
const std::string red("");
const std::string green("");
const std::string yellow("");
const std::string blue("");
const std::string magenta("");
const std::string cyan("");
const std::string white("");
const std::string reset("");
#endif






//M00016 1710252050IRQ643480 
// broadcast enable with macs
//  ff
// ff ff ff ff ff 00 0a f7 9e 9f c6 4a 46 01 00 f8
// 40 20 00 04 00 
//80 2e f2 c1 1f 36 43 
//81 5e 49 93 1d 14 ae 
//82 fe 93 53 b1 66 0c 
//83 ce 10 5e f4 ce 81
// 00 80 00 00 00 00 00 00 00 00 00 00 00 00 00
//  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 89
//  63 72 ab
 //  frame type :d0 [1174] 01 00 f8 40 d0 00 a1 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 // 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 08 3d dd 70

/* Frame (84 bytes) */
static const unsigned char pkt1[84] = {
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0a, /* ........ */
0xf7, 0x9e, 0x9f, 0xc6, 0x4a, 0x46, 0x01, 0x00, /* ....JF.. */
0xf8, 0x40, 0x20, 0x00, 0x07, 0x00, 0x00, 0x00, /* .@ ..... */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x53, 0x48, 0x2e, 0xb0                          /* SH.. */
};


// pck do jointa pytajcy lampkowy 
/* Frame (127 bytes) */
static const unsigned char pkt26253[127] = {
0x00, 0xe0, 0x4c, 0x38, 0x42, 0xec, 0x00, 0x0c, /* ..L8B... */
0x42, 0xce, 0xd0, 0x15, 0x08, 0x00, 0x45, 0x00, /* B.....E. */
0x00, 0x71, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, /* .q..@.@. */
0x09, 0x22, 0xc0, 0xa8, 0x58, 0x01, 0xc0, 0xa8, /* ."..X... */
0x58, 0x08, 0xd9, 0x36, 0x90, 0x90, 0x00, 0x5d, /* X..6...] */
0x30, 0x0f, 0x01, 0x00, 0x00, 0x01, 0x01, 0x5e, /* 0......^ */
0x49, 0x93, 0x1d, 0x14, 0xae, 0x00, 0x0a, 0xf7, /* I....... */
0x9e, 0x9f, 0xc6, 0x4a, 0x46, 0x00, 0x00, 0x00, /* ...JF... */
0x40, 0x01, 0x58, 0x00, 0x00, 0x00, 0x00, 0x2b, /* @.X....+ */
0x2f, 0xde, 0xff, 0x64, 0x00, 0x00, 0x00, 0x07, /* /..d.... */
0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x37, 0xfd, 0xff, 0xff, 0x64, 0x00, 0x00, /* .7...d.. */
0x00, 0x07, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x7e, 0x00, 0x20, 0x00, 0x00, 0x00, /* ..~. ... */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00        /* ....... */
};


// pytajacy jointa i wtedy 32 z danymi zaswieca lampki 
/* Frame (127 bytes) */
static const unsigned char pkt23769[127] = {
0x00, 0xe0, 0x4c, 0x38, 0x42, 0xec, 0x00, 0x0c, /* ..L8B... */
0x42, 0xce, 0xd0, 0x15, 0x08, 0x00, 0x45, 0x00, /* B.....E. */
0x00, 0x71, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, /* .q..@.@. */
0x09, 0x22, 0xc0, 0xa8, 0x58, 0x01, 0xc0, 0xa8, /* ."..X... */
0x58, 0x08, 0xd9, 0x36, 0x90, 0x90, 0x00, 0x5d, /* X..6...] */
0x46, 0x0f, 0x01, 0x00, 0x00, 0x01, 0x01, 0x5e, /* F......^ */
0x49, 0x93, 0x1d, 0x14, 0xae, 0x00, 0x0a, 0xf7, /* I....... */
0x9e, 0x9f, 0xc6, 0x4a, 0x46, 0x00, 0x00, 0x00, /* ...JF... */
0x40, 0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x2b, /* @.B....+ */
0x2f, 0xde, 0xff, 0x64, 0x00, 0x00, 0x00, 0x07, /* /..d.... */
0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x37, 0xfd, 0xff, 0xff, 0x64, 0x00, 0x00, /* .7...d.. */
0x00, 0x07, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x7e, 0x00, 0x20, 0x00, 0x00, 0x00, /* ..~. ... */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00        /* ....... */
};

// inna wersja pytajaca jointa z enabled 
/* Frame (127 bytes) */
//static const unsigned char pkt19082[127] = {
static const unsigned char pkt19082[127] = {
0x00, 0xe0, 0x4c, 0x38, 0x42, 0xec, 0x00, 0x0c, /* ..L8B... */
0x42, 0xce, 0xd0, 0x15, 0x08, 0x00, 0x45, 0x00, /* B.....E. */
0x00, 0x71, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, /* .q..@.@. */
0x09, 0x22, 0xc0, 0xa8, 0x58, 0x01, 0xc0, 0xa8, /* ."..X... */
0x58, 0x08, 0xd9, 0x36, 0x90, 0x90, 0x00, 0x5d, /* X..6...] */
0x94, 0x21, 0x01, 0x00, 0x00, 0x01, 0x01, 0x5e, /* .!.....^ */
0x49, 0x93, 0x1d, 0x14, 0xae, 0x00, 0x0a, 0xf7, /* I....... */
0x9e, 0x9f, 0xc6, 0x4a, 0x46, 0x00, 0x00, 0x00, /* ...JF... */
0x40, 0x31, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, /* @1B..... */
0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x07, /* ...d.... */
0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, /* .....d.. */
0x00, 0x07, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, /* ..|..... */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00        /* ....... */
};

//pkt715
/* Frame (127 bytes) */
static const unsigned char pktKeepAlive[127] = {
0x00, 0xe0, 0x4c, 0x38, 0x42, 0xec, 0x00, 0x0c, /* ..L8B... */
0x42, 0xce, 0xd0, 0x15, 0x08, 0x00, 0x45, 0x00, /* B.....E. */
0x00, 0x71, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, /* .q..@.@. */
0x09, 0x22, 0xc0, 0xa8, 0x58, 0x01, 0xc0, 0xa8, /* ."..X... */
0x58, 0x08, 0xd9, 0x36, 0x90, 0x90, 0x00, 0x5d, /* X..6...] */
0xd6, 0x51, 0x01, 0x00, 0x00, 0x01, 0x01, 0x5e, /* .Q.....^ */
0x49, 0x93, 0x1d, 0x14, 0xae, 0x00, 0x0a, 0xf7, /* I....... */
0x9e, 0x9f, 0xc6, 0x4a, 0x46, 0x00, 0x00, 0x00, /* ...JF... */
0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* @....... */
0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x07, /* ...d.... */
0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, /* .....d.. */
0x00, 0x07, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, /* ..|..... */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00        /* ....... */
};


// broadcast by miec 32 typ d0
/* Frame (131 bytes) */
static const unsigned char pkt26293[131] = {
0x00, 0xe0, 0x4c, 0x38, 0x42, 0xec, 0x00, 0x0c, /* ..L8B... */
0x42, 0xce, 0xd0, 0x15, 0x08, 0x00, 0x45, 0x00, /* B.....E. */
0x00, 0x75, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, /* .u..@.@. */
0x09, 0x1e, 0xc0, 0xa8, 0x58, 0x01, 0xc0, 0xa8, /* ....X... */
0x58, 0x08, 0xd9, 0x36, 0x90, 0x90, 0x00, 0x61, /* X..6...a */
0x8d, 0xaa, 0x01, 0x00, 0x00, 0x01, 0x01, 0xff, /* ........ */
0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0a, 0xf7, /* ........ */
0x9e, 0x9f, 0xc6, 0x4a, 0x46, 0x01, 0x00, 0xf8, /* ...JF... */
0x40, 0xd0, 0x00, 0x09, 0x11, 0x00, 0x00, 0x00, /* @....... */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, /* .......G */
0x4c, 0x63, 0x7f                                /* Lc. */
};
// another brodcast for 32 / type 0xd0
/* Frame (131 bytes) */
static const unsigned char pkt23768[131] = {
0x00, 0xe0, 0x4c, 0x38, 0x42, 0xec, 0x00, 0x0c, /* ..L8B... */
0x42, 0xce, 0xd0, 0x15, 0x08, 0x00, 0x45, 0x00, /* B.....E. */
0x00, 0x75, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, /* .u..@.@. */
0x09, 0x1e, 0xc0, 0xa8, 0x58, 0x01, 0xc0, 0xa8, /* ....X... */
0x58, 0x08, 0xd9, 0x36, 0x90, 0x90, 0x00, 0x61, /* X..6...a */
0x61, 0x49, 0x01, 0x00, 0x00, 0x01, 0x01, 0xff, /* aI...... */
0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0a, 0xf7, /* ........ */
0x9e, 0x9f, 0xc6, 0x4a, 0x46, 0x01, 0x00, 0xf8, /* ...JF... */
0x40, 0xd0, 0x00, 0x98, 0x0f, 0x00, 0x00, 0x00, /* @....... */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ........ */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, /* ........ */
0x63, 0x5e, 0x96                                /* c^. */
};
//#endif