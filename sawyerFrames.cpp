// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial


#include <string>
#include <sstream>
#include <iostream>

#include <cstdlib>
#include <vector>
#include <iomanip> // for setprecision

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

//#include "jointJCB.h"
#include "sawyerFrames.h"

#include "sawyerHelper.h" // Rethink defs


template <typename T>
std::string to_strP(const T a_value, const int n = 2)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

std::string to_strHEX(uint8_t * b,int n)
{
    char buff[100];
    string ret;
    for (int i=0 ;i<n;i++) 
    {
        sprintf(buff," %.2x",b[i]);
        ret+=buff;
    }
    return ret;
}

std::string to_str(uint32_t i,int n)
{
    char buff[100];
    string ret;
    if (n==5)
        sprintf(buff,"%5d", i);
    if (n==8)
        sprintf(buff,"%8d", i);
    return string(buff);
}


void HexStringToVector(string hexString,std::vector<uint8_t> &vector_)
{
  for (int a=0;a<hexString.size();a+=2)
  {
    string two="";
    two+=(hexString[a]);
    two+=(hexString[a+1]);
    int value = stoi(two, nullptr, 16);
    vector_.push_back(value);
  }
}



int coutAnsiBraodFromPC(FrameBroadcast84_t f)
{
     if (f.en.mac1Id[0]==0x80)  // FIXME : how to disnghs beeeten enable and all day ?
     {
        cout<< red<<"z PC en BrC ";
        
        cout << yellow <<to_strHEX(f.en.jointHwNum,1)<<" ";
        cout << red <<to_strHEX(f.en.msgtyp40_2,1)<<" ";
        cout << green <<"SS"<<to_strHEX(f.en.someStatus,1)<<" ";
        cout << cyan<<",[" << to_str(f.en.pcktCounter,5) <<"]";

        cout << green <<"id:"<<to_strHEX(f.en.mac1Id,1)<<" ";
        cout << red <<","<<to_strHEX(f.en.mac1,6)<<" ";
        cout << green <<"id:"<<to_strHEX(f.en.mac2Id,1)<<" ";
        cout << red <<","<<to_strHEX(f.en.mac2,6)<<" ";
        cout << green <<"id:"<<to_strHEX(f.en.mac3Id,1)<<" ";
        cout << red <<","<<to_strHEX(f.en.mac3,6)<<" ";
    }
    else
    {
        cout<< red<<"z PC alday";
        cout << yellow <<to_strHEX(f.en.jointHwNum,1)<<" ";
        cout << red <<to_strHEX(f.en.msgtyp40_2,1)<<" ";
        cout << green <<"SS"<<to_strHEX(f.en.someStatus,1)<<" ";
        cout << cyan<<",[" << to_str(f.en.pcktCounter,5) <<"]";
        cout << green <<"fill1:"<<to_strHEX(f.allday.fill,1)<<" ";
        cout << green <<"rest:"<<to_strHEX(f.allday.rest,61)<<" ";
    }

    cout <<reset<<"\r\n"<<flush;
    return 0;
}


int coutAnsiFrame01(JointStatus_t f)
{
    if (f.msgtyp40_2[0]!=0x01) 
        cout<< red<<"no 01 frame" <<reset<<flush;
    cout << yellow <<to_strHEX(f.kaCmd01.jointHwNum,1)<<" ";
    cout << red <<to_strHEX(f.kaCmd01.msgtyp40_2,1)<<" ";
    cout << green <<"ack:"<<to_strHEX(f.kaCmd01.ackFor05,1)<<" ";
    cout << ","<<cyan << to_str(f.kaCmd01.pcktCounter,5) ;
    cout << ","<< magenta << to_str(f.kaCmd01.speedSP1,5); 
    cout << ","<< magenta << to_str(f.kaCmd01.encoderSP1,8); 
    cout << red <<to_strHEX(f.kaCmd01.you64,1)<<" ";
    cout << yellow <<to_strHEX(f.kaCmd01.bond07,4)<<" ";
    //cout << green <<to_strHEX(f.kaCmd01.someZeros2,2)<<" ";
    cout << red <<"CCMD1"<<to_str(f.kaCmd01.jCmdFlag1,5)<<" ";
    cout << magenta <<"B:"<<to_strHEX(f.kaCmd01.brake1,1)<<" ";
    cout << green <<to_strHEX(f.kaCmd01.someZeros4,4)<<" ";

    cout << ","<< magenta << to_str(f.kaCmd01.speedSP2,5); 
    cout << ","<< magenta << to_str(f.kaCmd01.encoderSP2,8); 
    cout << red <<to_strHEX(f.kaCmd01.you64_bis,1)<<" ";

    cout << yellow <<to_strHEX(f.kaCmd01.bond07_bis,4)<<" ";
    cout << red <<"CCMD2"<<to_str(f.kaCmd01.jCmdFlag2,5)<<" ";
    cout << magenta <<"B:"<<to_strHEX(f.kaCmd01.brake2,1)<<" ";
    cout << green <<"somez:"<<to_strHEX(f.kaCmd01.brake2,5)<<" ";
    cout << magenta <<"L"<<to_strHEX(f.kaCmd01.lightsBtnBit,1)<<" ";
    cout << green  <<to_strHEX(f.kaCmd01.someZero3,1)<<" ";
    cout << magenta <<"H"<<to_strHEX(f.kaCmd01.lightsHead,1)<<" ";

    cout << green  <<to_strHEX(f.kaCmd01.unknonwRest,18)<<" ";
    ////std::cout << ","<<format("{:#04x}", f.kaCmd01.you64[0]);
    //cout << green << std::setfill('0') << std::setw(2) << hex << f.kaCmd01.you64[0];
    
    cout <<reset<<"\r\n"<<flush;
    return 0;
}

int coutAnsiFrameFromJ(JointStatus_t f)
{
    //if (f.msgtyp40_2[0]!=0x01) 
        //cout<< red<<"no 01 frame" <<reset<<flush;
    cout << yellow <<to_strHEX(f.vf.jointHwNum,1)<<" ";
    cout << green <<to_strHEX(f.vf.msgtyp40_2,1)<<" ";
    cout << green <<"SS"<<to_strHEX(f.vf.someStatus,1)<<" ";
    cout << cyan<<",[" << to_str(f.vf.pcktCounter,5) <<"]";
    cout << green <<" ackX05"<<to_strHEX((uint8_t*)&f.vf.ackX05,1)<<" ";
    cout << green <<"dbgPage"<<to_strHEX((uint8_t*)&f.vf.dbgPage,1)<<" ";
    
    cout << blue<<"," << to_str(f.vf.encoder1,8); 
    cout <<  blue << ","<< to_str(f.vf.speed1,5); 
    cout <<  blue <<","<< to_str(f.vf.effort1,5); 
    cout <<  blue << ","<<to_str(f.vf.force1,5); 
    cout << blue << "Flags:" <<  to_strHEX((uint8_t*)&f.vf.stFlags1,1); 
    cout << blue << "knob:" << to_str(f.vf.encKnob,5); 
    cout << green <<" jE:"<<  to_str(f.vf.jErFlags1,5); 
    cout << green<<"," << to_str(f.vf.speedSpecial1,5); 
    cout << blue<<"," << to_str(f.vf.encoder2,8); 
    cout <<  blue << ","<< to_str(f.vf.speed2,5); 
    cout <<  blue <<","<< to_str(f.vf.effort2,5); 
    cout <<  blue << ","<<to_str(f.vf.force2,5); 
    //cout << green<<",Flags:" << to_str(f.vf.statusFlags,5); 
    cout << red<<",Flags:" << to_strHEX((uint8_t*)&f.vf.stFlags2,1); 
    cout << green<<",jE2:" << to_str(f.vf.jErFlags2,5); 
    
    //cout << green <<"u5:"<<to_strHEX((uint8_t*)&f.vf.unk5,3)<<" ";
    cout << green<<",an:" << to_str(f.vf.unkAnalog,5); 
    cout << green <<"btn:"<<to_strHEX((uint8_t*)&f.vf.valbtn,1)<<" ";
    cout << blue<<",v7:" << to_str(f.vf.val7,5); 
    cout << green <<":"<<to_strHEX((uint8_t*)&f.vf.dbgVarBuff20,20)<<" ";


    cout <<reset<<"\r\n"<<flush;
    return 0;

}

int coutAnsiFrameRegisters(JointStatus_t f)
{
    cout << yellow <<to_strHEX(f.rg.jointHwNum,1)<<" ";
    cout << white <<to_strHEX(f.rg.msgtyp40_2,1)<<" ";
    cout << green <<"SS"<<to_strHEX(f.rg.someStatus,1)<<" ";
    cout << cyan<<",_[" << to_strHEX(&f.rg.pcktCounter8,1) <<"]";
    cout << green <<" dummy3a"<<to_strHEX(f.rg.dummy3a,1)<<" ";
    cout << green <<"regsmpl"<<to_strHEX(f.rg.regbuff,sizeof(f.rg.regbuff))<<" ";
        
    cout <<reset<<"\r\n"<<flush;
    return 0;

}
string stFlagToString(uint8_t stFlag)
{
    string str="";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_CCW_LIMIT) str+="CCW_LIMIT ";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_CURRENT_LIMITING_ACTIVE) str+="CURRENT_LIMITING_ACTIVE ";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_CW_LIMIT) str+="CW_LIMIT ";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_ENABLED) str+="ENABLED ";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_IS_HEAD_PAN_ENABLED) str+="IS_HEAD_PAN_ENABLED ";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_IS_HOMED_AND_UNLOCKED) str+="IS_HOMED_AND_UNLOCKED ";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_IS_HOMING_AND_UNLOCKING) str+="IS_HOMING_AND_UNLOCKING ";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_IS_JOINT_MOVING) str+="IS_JOINT_MOVING ";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_ITB_BUTTON0) str+="ITB_BUTTON0 ";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_ITB_BUTTON1) str+="ITB_BUTTON1 ";
    if ( stFlag & JRCP_SEA_CRSP_JOINT_FLAG_ITB_BUTTON2) str+="ITB_BUTTON2 ";
   
    return str;
}
 
 string erFlagToString(uint16_t erFlag)
{
    string str="";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_HES_CONFIGURATION_ERROR) str+="HES_CONFIGURATION_ERROR ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_LATCHED) str+="LATCHED ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_MAE_CONFIG_ERROR) str+="MAE_CONFIG_ERROR ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_MAE_SENSOR_ERROR) str+="MAE_SENSOR_ERROR ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_MOTOR_CONFIGURATION) str+="MOTOR_CONFIGURATION ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_MOTOR_FAULT) str+="MOTOR_FAULT ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_MOTOR_TEMPERATURE) str+="MOTOR_TEMPERATURE ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_OVER_CURRENT_LIMIT) str+="OVER_CURRENT_LIMIT ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_OVER_TORQUE_LIMIT) str+="OVER_TORQUE_LIMIT ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_SDS_SENSOR_ERROR) str+="SDS_SENSOR_ERROR ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_SERVO_CONFIGURATION_ERROR) str+="SERVO_CONFIGURATION_ERROR ";
    if ( erFlag & JRCP_SEA_CRSP_JOINT_ERROR_SERVO_FAULT) str+="SERVO_FAULT ";

    return str;
}

string globalErFlagToString(uint16_t erFlag)
{
    string str="";
    if ( erFlag & JRCP_SEA_CRSP_GLOBAL_ERROR_HEATSINK_TEMPERATURE) str+="ERROR_HEATSINK_TEMPERATURE ";
    if ( erFlag & JRCP_SEA_CRSP_GLOBAL_ERROR_NO_EXTERNAL_ENABLE) str+="ERROR_NO_EXTERNAL_ENABLE ";
    if ( erFlag & JRCP_SEA_CRSP_GLOBAL_ERROR_OVERVOLTAGE) str+="ERROR_OVERVOLTAGE ";
    if ( erFlag & JRCP_SEA_CRSP_GLOBAL_ERROR_PWR_FAULT) str+="ERROR_PWR_FAULT ";
    if ( erFlag & JRCP_SEA_CRSP_GLOBAL_ERROR_SAFETY_CONTACTOR_OPEN) str+="ERROR_SAFETY_CONTACTOR_OPEN ";
    if ( erFlag & JRCP_SEA_CRSP_GLOBAL_ERROR_SELFTEST_FAIL) str+="ERROR_SELFTEST_FAIL ";
    if ( erFlag & JRCP_SEA_CRSP_GLOBAL_ERROR_UNDERVOLTAGE) str+="ERROR_UNDERVOLTAGE ";
    //if ( erFlag & JRCP_SEA_CRSP_GLOBAL_FLAG_A_BUTTON) str+="FLAG_A_BUTTON ";
    //if ( erFlag & JRCP_SEA_CRSP_GLOBAL_FLAG_B_BUTTON) str+="FLAG_B_BUTTON ";
    return str;
}







DebugVariables::DebugVariables(/* args */)
{
    
    // [varCnt][00][typeFirst][TypeSecond][0402-len][typeFirst][typeSecond][0202]
    //dataVec[0]= { 0x00,'o','c','t','e','t',0x00};
    //HexStringToVector("0500 110e 0402 130e04020c0e04020d0e0402004e040200",rawDataVec[0]);
    HexStringToVector("0500110e0402130e04020c0e04020d0e0402004e040200",rawDataVec[1]); // for all the same ? 
    HexStringToVector("0600034e0202024e0402014e040200220202030e04020b0e040200",rawDataVec[2]);
    HexStringToVector("0600050e04020a0e04020e0e04020f0e0402010602020006020200",rawDataVec[3]);
    HexStringToVector("0700160e0402180e0402140e0402161a02020b1a02020d1a02020a1a020200",rawDataVec[4]);
    HexStringToVector("07000c1a0202041a0202012a0202070e040210060202094a0402004a020200",rawDataVec[5]);
    HexStringToVector("0500030604020153040202530402025204020352040200",rawDataVec[6]);
    //7 6 11 00  - 07 is empty ? 
    HexStringToVector("07000e260202112602020d26020210260202162202020a2604020b26040200",rawDataVec[8]);
    // 09 brak ? 
    //10 6 11 00 - empty
    HexStringToVector("09001b0e02021a0e02021d0e02021c0e02021f0e02021e0e02021206020211060202110e040200",rawDataVec[11]);

    HexStringToVector("0500130e04020c0e04020d0e0402004e0402034e020200",rawDataVec[12]);
    //13 for some empty
    HexStringToVector("0500024e0402014e040200220202030e04020b0e040200",rawDataVec[13]);
    HexStringToVector("0600050e04020a0e04020e0e04020f0e0402010602023206020200",rawDataVec[14]);
    HexStringToVector("0700160e0402180e0402140e0402171a02020f1a0202111a02020e1a020200",rawDataVec[15]);
    HexStringToVector("0700101a0202091a0202012a0202070e0402100602023b4a0402324a020200",rawDataVec[16]);
    HexStringToVector("0500030604020353040204530402165204021752040200",rawDataVec[17]);
    //18? - zero ?
    //19 - zero
    
    HexStringToVector("07001d26040223260202262602022b220202052a020222220202010e040200",rawDataVec[20]);
    //21 zero ? HexStringToVector("",rawDataVec[21]);
    HexStringToVector("08001b0e02021a0e02021d0e02021c0e02021f0e02021e0e0202120602021106020200",rawDataVec[22]);


    for (int a=0;a<22;a++)
    {
        JointDVariable_t v;
        auto rd=rawDataVec[a];
        if (rd.size()<6) continue;
        
        for (int c=2;c<(rd.size()-4);c+=4)
        {
            v.firstType=rd[c];
            v.secType=rd[c+1];
            memcpy(&v.type16,&rd[c],2);
            v.len=rd[c+2];
            variableList[a].push_back(v);
        }
        
    }

    // for (int a=0;a<22;a++)
    // {
    //     auto vl=variableList[a];
    //     cout<<"DebugLineNo:"<<+a<<"\r\n";
    //     for (auto v:vl)
    //     {
    //         cout<<" first,sec["<<+v.firstType<<","<<+v.secType<<"] len:"<<+v.len<<"\r\n";
    //     }
    // }

    // JointDVariable_t v;
    
    // v.len=2;
    // v.firstType=0x11;
    // v.secType=0x0e;
    // variableList[0].push_back(v);

}

DebugVariables::~DebugVariables()
{
}

int DebugVariables::ParseDebugFrame(uint varFrameNo, uint8_t * rawFrame20)
{

        auto vl=variableList[varFrameNo];
//        cout<<"DebugLineNo:"<<+a;
        int pos=0;
        int value=0;
        int cnt=0;
        for (auto v:vl)
        {
            //cout<<" first,sec["<<+v.firstType<<","<<+v.secType<<"] len:"<<+v.len<<"\r\n";
            if (v.len==2)
            {
                uint16_t ui16;
                memcpy(&ui16,&rawFrame20[pos],2);
                pos+=2;
                value=ui16;
            }
            if (v.len==4)
            {
                uint32_t ui32;
                memcpy(&ui32,&rawFrame20[pos],4);
                pos+=4;
                value=ui32;
            }

            switch (v.type16)
            {
            case 0x011a:
                temperature0=value;
                break;
            case 0x0b1a:
                temperature1=value;
                break;
            
            default:
                break;
            }
            
            agnoVal[cnt]=value;
            cnt++;
        }
    

     //auto rf20= rawFrame20[varFrameNo];
    
    //for (int a=0;a<22;a++)
    {
        
      //  auto = rawFrame20[20]

    }
    return cnt;

}


//REGISTERS 
//submode list 
vector <int> subList   ={5 ,17 ,25,29,33,37,41,11}; 
//submode list length 
//vector <int> subListLen={50,100,11,50,10,10,10,11};
vector <int> subListLen={50,100,0,50,10,10,10,11};

AmyRegister_t amyRegisters[110][10];



bool equalAr(char* a, int l , char* b )
{
  for (int k=0;k<l;k++)
    {
        if (a[k]!=b[k])
        {
         return false;
        }
    }    
    return true;
}

string vectorToHexStr(vector<uint8_t> vec_)
{
    string ret;
    stringstream ss;
    for (auto& c : vec_)
    {
        ss << std::setw(2) << std::setfill('0') << std::hex << +c ;
    }
    ss>>ret;
    return ret ;
}


string parsePayloadComl(char *buff_, int size_ ,vector <JointRegisters_t> &regList_ ,int msgType_,uint8_t jointHwNum_)
{
    char strBuf[200];
    JointRegisters_t  reg;
    reg.type=JCB_REG_TYPEUNKNOWN;
    reg.value=0;
    reg.valSubType=0;
    //for (int a=0;a<(size_-4);a++)
    bool FromPC=false;
    reg.msgType=msgType_;
    reg.jointHwNum=jointHwNum_;
    if (msgType_==0x04) FromPC=true;
    
    int a=0;
    bool endOfMsg=false;
    string retstr="";
    while ( (a<(size_-4)) && (endOfMsg==false))
    {
        uint8_t regNo=buff_[a+2];
        reg.regNo=regNo;
        if ((buff_[a]==0x41) && (buff_[a+1]==0x02) )
        {
            reg.valSubType=buff_[a+3];
            reg.type=JCB_REG_TYPE4302;
            sprintf(strBuf,"A%d[%d]",reg.valSubType,regNo);
            retstr+=strBuf;
            if (FromPC) a+=4; 
            else // get value from ack
            {
                int16_t i16;
                memcpy(&i16,&buff_[a+4],2);
                reg.value=i16;
                regList_.push_back(reg);
                a+=6;
            }
        }
        else
        if ((buff_[a]==0x41) && (buff_[a+1]==0x04) )
        {
            reg.valSubType=buff_[a+3];
            reg.type=JCB_REG_TYPE4304;
            sprintf(strBuf,"A%d[%d]",reg.valSubType,regNo);
            retstr+=strBuf;
            if (FromPC) a+=4; 
            else 
            {
                int32_t i32;
                memcpy(&i32,&buff_[a+4],4);
                reg.value=i32;
                regList_.push_back(reg);
                a+=8;
            }
            
        }else
        if ((buff_[a]==0x43) && (buff_[a+1]==0x02) )
        {
            if (a+6>=(size_-1)) 
            {
                cout <<"ERROR Parser OutOf Bonds ! \r\n"<<flush;
                endOfMsg=true;
                break;
            }
            reg.type=JCB_REG_TYPE4302;
            reg.valSubType=buff_[a+3];
            
            if (FromPC)
            {
                int16_t i16;
                memcpy(&i16,&buff_[a+4],2);
                reg.value=i16;
                regList_.push_back(reg);
                a+=6;
            }  
            else a+=4;
            sprintf(strBuf,"V%d[%d]",reg.valSubType,regNo);
            retstr+=strBuf;
        }
        else
        if ((buff_[a]==0x43) && (buff_[a+1]==0x04) )
        {
            if (a+8>=(size_-1)) 
            {
                cout <<"ERROR Parser OutOf Bonds ! \r\n"<<flush;
                endOfMsg=true;
                break;
            }
            reg.type=JCB_REG_TYPE4304;
            reg.valSubType=buff_[a+3];
            sprintf(strBuf,"V%d[%d]",reg.valSubType,regNo);
            retstr+=strBuf;
            if (FromPC) 
            {
                int32_t i32;
                memcpy(&i32,&buff_[a+4],4);
                reg.value=i32;
                regList_.push_back(reg);
                a+=8; 
            }
            
            else a+=4;
        } else
        if ((buff_[a]==0x03) && (buff_[a+1]==0x02) )
        {
            reg.type=JCB_REG_TYPE0302;
            reg.valSubType=buff_[a+3];
            int16_t i16;
            memcpy(&i16,&buff_[a+4],2);
            reg.value=i16;
            regList_.push_back(reg);
            sprintf(strBuf,"X%d[%d]",reg.valSubType,regNo);
            retstr+=strBuf;
            a+=6;
        }else
        if ((buff_[a]==0x03) && (buff_[a+1]==0x2a) )
        {
            reg.type=JCB_REG_TYPE032a;
            reg.valSubType=buff_[a+3];

            
            sprintf(strBuf,"S%d[%d]",reg.valSubType,regNo);   
            retstr+=strBuf;
            for (int b=a+4;b<size_;b++)
            {
                if ( b<(size_-4) )
                {
                    if ((buff_[b]==0) && (buff_[b+1]==0) && (buff_[b+2]==0) && (buff_[b+3]==0) )
                    {
                        //end of message 
                        endOfMsg=true;
                        //a=1000;
                        b=size_;
                    }

                }
                if (endOfMsg!=true)
                {
                    reg.rawData.push_back(buff_[b]);
                    sprintf(strBuf," %.2x",buff_[b]);            
                    retstr+=strBuf;
                }
            }
            
            regList_.push_back(reg);
            
            //a+=4+(reg.rawData.size());
        }else
        {
            //retstr+="UnknownData";
            a++;
        }
    }
    retstr+="END";
    return string(retstr);
}


void savePacketsToFile(string filename_,PacketList &packetList_ )
{
    std::ofstream fout(filename_);
    fout << std::setprecision(10);
    for(auto const& r : packetList_) 
    {
        //fout << to_string(r.regNo)<< " "<<to_string(r.type) <<" "<< to_string(r.valStat)<<" ";
        
        for (auto& c : r)
        {
            // +c 'fools' std::cout c is a number, not a character sign isn't changed
            //fout << "0x"<<  std::setw(2) << std::setfill('0') << std::hex << +c << ' ';
            fout << std::setw(2) << std::setfill('0') << std::hex << +c ;
        }
        fout <<'\n';
    }
   fout.close();        
}

bool loadPacketsFromFile(string filename_,PacketList &packetList_ )
{
   ifstream in( filename_ );
   for ( string line; getline( in, line ); )
   {
        stringstream ss( line );
        string hexstring;
        ss >> hexstring;
        vector <uint8_t> packet;
        for(size_t i = 0; i < hexstring.length(); i += 2) 
        {
            std::istringstream strm(hexstring.substr(i, 2));
            auto substr = hexstring.substr(i, 2);
            uint8_t x= std::stoi(substr, nullptr, 16);
            packet.push_back(x);
        }
      
      packetList_.push_back( packet );
   }
    in.close();
    if (packetList_.size()==0)
    {
        cout << "error loading packetList from file "<<filename_ << " !!! \r\n";
        return false;
    }
    return true;
}



auto subtypeToIdx(int x) 
{  
    for (int i=0;i<=subList.size();i++)
    {
        if (subList[i]==x) return i;
    }
    cout << " big error reg type out of range : " <<+x<<" \r\n" <<flush;
    return -1;
};




int saveRobotRegisters(string filename )
{
    // saving to csv 
    ofstream cout(filename);
  for (int s=0;s<subList.size();s++)
  {
        cout<<"#registers for mode : "<<+subList[s]<<"\r\n";

        for (int i=0;i<subListLen[s];i++)
        {
            cout<<std::setw(30)<<left<< amyRegisters[i][s].name<<right;
            cout<<","<<+subList[s]; //subtype list
            cout<<","<<+amyRegisters[i][s].length; 
            cout<<","<<+i; 

            for (int j=0;j<8;j++)
            {
                auto v=amyRegisters[i][s].value[j];
                auto nonE=amyRegisters[i][s].nonempty[j];

                if (subList[s]==11) 
                {
                    cout<<",";
                    for (auto& c : amyRegisters[i][s].rawData[j])
                    {
                        cout << std::setw(2) << std::setfill('0') << std::hex << +c ;
                    }
        
                }
                else
                {
                if (nonE)
                    //cout<<","<<to_str(v,8);
                    cout<<","<<std::setw(10)<<v;
                else
                    cout<<","<<std::setw(10);
                    
                }
 
            }
            cout<<"\r\n";
        }

}
    return 0;
}

int loadRobotRegisters(string filename )
{
    fstream fin; 
    fin.open(filename, ios::in);
    
    if(!fin)
    {
        cout<<"CAN't open file : "<< filename<<"\r\n"<<flush;
        return -1;
    } 

    int rollnum, roll2, count = 0; 
    
    vector<string> row; 
    string line, word, temp; 
    string name;
    int subType;
    int length;
    int regNo;

    while (!fin.eof()) 
    { 
       row.clear(); 
        getline(fin, line); 
        if (line.at(0)=='#') continue;
        stringstream s(line); 
        // read every column data of a row and         // store it in a string variable, 'word' 
        while (getline(s, word, ',')) 
        { 
            // add all the column data             // of a row to a vector 
            row.push_back(word); 
        } 
        name=row[0];
        subType = stoi(row[1]); 
        int sIdx=subtypeToIdx(subType);
        length = stoi(row[2]); 
        regNo = stoi(row[3]); 

        for (int j=0;j<8;j++)
        {
            string str=row[j+4];
           // str.remove(std::begin(str), std::end(str), '\r');
            if (!str.empty() && str[str.length()-1] == '\r') 
            {
               str.erase(str.length()-1);
            }

            str.erase(remove(str.begin(), str.end(), ' '), str.end());
            //remove(std::begin(str), std::end(str), '\n');

            if (!str.empty())
            {
                
                auto val= stoi(str); 
                auto limb=j;
                amyRegisters[regNo][sIdx].value[limb]=val;
                amyRegisters[regNo][sIdx].nonempty[limb]=true;
            }
        }
    
        amyRegisters[regNo][sIdx].regNo=regNo;
        amyRegisters[regNo][sIdx].subtype=subType;
        amyRegisters[regNo][sIdx].name=name;
        amyRegisters[regNo][sIdx].length=length;
        //cout<<name<<" , "<<subType<<","<<length<<","<<regNo <<"\r\n";
    } 

    fin.close();
    return 0;
}
// export HW JCB based - ie 0,1,2,3 , no more
int amyRegisters2regList(int jHW_, vector <JointRegisters_t> & regList_)
{
    JointRegisters_t r;
    
    for (int sIdx=0;sIdx<subList.size();sIdx++)
    {

            // if (subListLen[sIdx]==0)       //special case 25 aka module 6 
            // {
            //    // if (amyRegisters[no][sIdx].nonempty[(jHW_*2)]==true) 
            //     {
                
            //     }

            
            // }else
           for (int no=0;no<subListLen[sIdx];no++) // iterate over all registers in subtype
           {
                if (amyRegisters[no][sIdx].nonempty[(jHW_*2)]==true) 
                {
                    if (amyRegisters[no][sIdx].length==2) r.type=JCB_REG_TYPE4302;
                    if (amyRegisters[no][sIdx].length==4) r.type=JCB_REG_TYPE4304;
                    r.valSubType=subList[sIdx];
                    r.regNo=no;
                    r.jointHwNum=0; //not needed 
                    r.msgType=0;//not needed 
                    
                    r.value=amyRegisters[no][sIdx].value[(jHW_*2)];
                    regList_.push_back(r);
                }
                //cout <<+no<<"  sidx "<<+sIdx<< " hwd "<<+jHW_<< "\r\n"<<flush;
                if (amyRegisters[no][sIdx].nonempty[(jHW_*2)+1]==true) 
                {
                    if (amyRegisters[no][sIdx].length==2) r.type=JCB_REG_TYPE4302;
                    if (amyRegisters[no][sIdx].length==4) r.type=JCB_REG_TYPE4304;
                    r.jointHwNum=0; //not needed 
                    r.valSubType=subList[sIdx];
                    r.msgType=0;//not needed 

                    r.regNo=no+subListLen[sIdx]; //add offfset
                    r.value=amyRegisters[no][sIdx].value[(jHW_*2)+1];

                    regList_.push_back(r);
                }
           }
    }
    return 0;
}


int prepareReg2Send(JointRegisters_t & reg,vector<uint8_t> &regbufVal,vector<uint8_t> &regbufAck)
{
    // returns 1 when special exlusive is present , one register in one message
    regbufVal.clear();
    regbufAck.clear();
    if  (reg.type==JCB_REG_TYPE4302) 
    {
        uint8_t rawBuf[4];
        regbufVal.push_back(0x43);
        regbufVal.push_back(0x02);
        regbufVal.push_back(reg.regNo);
        regbufVal.push_back(reg.valSubType);
        int16_t i16=(int16_t)reg.value;
        memcpy(&rawBuf[0],&i16,2);
        regbufVal.push_back(rawBuf[0]);
        regbufVal.push_back(rawBuf[1]);

        regbufAck.push_back(0x41);
        regbufAck.push_back(0x02);
        regbufAck.push_back(reg.regNo);
        regbufAck.push_back(reg.valSubType);
        return 0;
    }

if  (reg.type==JCB_REG_TYPE4304) 
    {
        uint8_t rawBuf[4];
        regbufVal.push_back(0x43);
        regbufVal.push_back(0x04);
        regbufVal.push_back(reg.regNo);
        regbufVal.push_back(reg.valSubType);
        int32_t i32=(int32_t)reg.value;
        memcpy(&rawBuf[0],&i32,4);
        regbufVal.push_back(rawBuf[0]);
        regbufVal.push_back(rawBuf[1]);
        regbufVal.push_back(rawBuf[2]);
        regbufVal.push_back(rawBuf[3]);

        regbufAck.push_back(0x41);
        regbufAck.push_back(0x04);
        regbufAck.push_back(reg.regNo);
        regbufAck.push_back(reg.valSubType);
        return 0;
    }

    if (reg.type==JCB_REG_TYPE4102 || reg.type==JCB_REG_TYPE4104)   // read only: "41 <size> <reg> <module>"
    {
        regbufAck.push_back(0x41);
        regbufAck.push_back(reg.type==JCB_REG_TYPE4102 ? 0x02 : 0x04);
        regbufAck.push_back(reg.regNo);
        regbufAck.push_back(reg.valSubType);
        return 0;
    }

    if (reg.type==JCB_REG_TYPE0302) 
    {
        uint8_t rawBuf[4];
        regbufVal.push_back(0x03);
        regbufVal.push_back(0x02);
        regbufVal.push_back(reg.regNo);
        regbufVal.push_back(reg.valSubType);
        int16_t i16=(int16_t)reg.value;
        memcpy(&rawBuf[0],&i16,2);
        regbufVal.push_back(rawBuf[0]);
        regbufVal.push_back(rawBuf[1]);
        return 1;
    }

    if (reg.type==JCB_REG_TYPE032a) // multi vector
    {
        regbufVal.push_back(0x03);
        regbufVal.push_back(0x2a);
        regbufVal.push_back(reg.regNo);
        regbufVal.push_back(reg.valSubType);

        for (uint8_t& c : reg.rawData)
        {
            regbufVal.push_back(c);
        }
        return 1;
    }
}


void saveJointRegFile(string filename_,vector <JointRegisters_t> &regList_,int mode_)
{
    std::ofstream fout(filename_,std::ios_base::app);
    fout << std::setprecision(10);
    for(auto const& r : regList_) 
    {
        if (mode_==1) // csv
        {
            fout <<to_string(r.jointHwNum)<<","<<to_string(r.msgType)<<","<<to_string(r.regNo)<< ","<<to_string(r.type) <<","<< to_string(r.valSubType)<<",";
        }
        else
        {
            fout << to_string(r.regNo)<< " "<<to_string(r.type) <<" "<< to_string(r.valSubType)<<" ";
        }
        
        if (r.type!=JCB_REG_TYPE032a) 
        {
            fout <<to_string(r.value);
        }
        else
        for (auto& c : r.rawData)
        {
            // +c 'fools' std::cout c is a number, not a character
            // sign isn't changed
            //fout << "0x"<<  std::setw(2) << std::setfill('0') << std::hex << +c << ' ';
            fout << std::setw(2) << std::setfill('0') << std::hex << +c ;
        }
        fout <<'\n';
    }
   fout.close();        
}
bool loadJointRegFile(string filename_,vector <JointRegisters_t> &regList_ )
{
   ifstream in( filename_ );
   for ( string line; getline( in, line ); )
   {
      JointRegisters_t reg={0};
      stringstream ss( line );
      int tmp;
      ss >> tmp; reg.regNo=tmp;
      ss >> tmp; reg.type=tmp;
      ss >> tmp; reg.valSubType=tmp;
      if (reg.type!=JCB_REG_TYPE032a) 
      {
            ss >> tmp; reg.value=tmp;
      }
     else
     {
        string hexstring;
        ss >> hexstring;
        for(size_t i = 0; i < hexstring.length(); i += 2) 
        {
            std::istringstream strm(hexstring.substr(i, 2));
            auto substr = hexstring.substr(i, 2);
            uint8_t x= std::stoi(substr, nullptr, 16);
            reg.rawData.push_back(x);
        }
     }
      
      regList_.push_back( reg );
   }
    in.close();
    if (regList_.size()==0)
    {
        cout << "error loading registers from file "<<filename_ << " !!! \r\n";
        return false;
    }
    return true;
}





uint32_t jrcpCrc(const uint8_t *msg64)
{
    uint32_t crc=0xFFFFFFFF;
    for (int w=0; w<16; w++)
    {
        uint32_t word; memcpy(&word, msg64+4*w, 4);
        crc^=word;
        for (int b=0; b<32; b++) crc = (crc & 0x80000000) ? (crc<<1)^0x04C11DB7 : (crc<<1);
    }
    return crc;
}
