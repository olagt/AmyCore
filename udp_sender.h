// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#pragma once
#ifndef UDP_SENDER
#define UDP_SENDER
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <net/if.h>
using namespace std;

#include <vector>

#include "sawyerFrames.h"
#include "jointJCB.h"




#define JCB_STATE_INIT_02 1   // 
#define JCB_STATE_SENDING_SERIAL_06 2   // frametype 0x06
//.715 from PC , frame type :01 [0] 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 7c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//Broadcast ENABLE , frame type :d0 [2028] 01 00 f8 40 d0 00 ec 07 80 5e 49 93 1d 14 ae 82 fe 93 53 b1 66 0c 83 ce 10 5e f4 ce 81 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 4f ea dc 99
#define JCB_STATE_WORKING_12 3
//Broadcast , frame type :d0 [2028] 01 00 f8 40 d0 00 ec 07 80 5e 49 93 1d 14 ae 82 fe 93 53 b1 66 0c 83 ce 10 5e f4 ce 81 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 4f ea dc 99
//.726 from PC , frame type :01 [0] 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 7c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//broadacst 0d ENABLE z macami 
#define JCB_STATE_WORKING_22 3
//Broadcast , frame type :d0 [2029] 01 00 f8 40 d0 00 ed 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 9b 4f 45 76
//joint keep alive packet:  .729 from PC , frame type :01 [0] 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 7c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//.1378 z AE , frame type :22 [256] 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 02 00 3c 0a ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff
//Broadcast , frame type :d0 [2274] 01 00 f8 40 d0 00 e2 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 e3 86 65 49
//.1380 from PC , frame type :01 [0] 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 00 00 00 00 00 64 00 00 00 07 00 00 02 00 00 00 00 00 7c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//ZACZYNA SIE LOADREG .1381 from PC , frame type :04 [14856] 43 04 09 11 00 04 00 00 41 04 09 11 43 04 0a 11 00 00 00 00 41 04 0a 11 43 04 0b 11 00 00 09 3d 41 04 0b 11 43 04 0c 11 00 00 00 00 41 04 0c 11 43 04 0d 11 a0 86 01 00 00 00


#define JCB_STATE_WORKING_NO_REGLOADED 3
#define JCB_STATE_WORKING 4
#define JCB_STATE_HOT_START 5


#define TFTP_OPCODE_READ     1
#define TFTP_OPCODE_WRITE    2
#define TFTP_OPCODE_DATA     3
#define TFTP_OPCODE_ACK      4
#define TFTP_OPCODE_ERROR    5



class UDPSender
{
    public:
    void openRawEthListener (string name_);

    void openRawEth (string name_);
    int sendRawEth (MacAddress dstmac_, uint16_t ether_type_ ,char* data_, int datalen_);
    void closeRawEth();

    void openIpUDP (string srcIP_,int src_port_);
    ssize_t sendIPUDP( string dstIP_, int dest_port, char* msg,size_t msg_size);
    int send_IpUdp_fd;

    int receiveFileTFTP(string strIP,string fileName,vector<uint8_t> &recVect);

//sockaddr_in
    int sockMacUdpfd;
	struct ifreq if_idx;
	struct ifreq if_mac;

    MacAddress RawEthListenerMac;

 

};


#endif