// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial



using namespace std;

#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/ether.h>

#include <fstream>
#include <chrono>
#include <thread>

#include "udp_sender.h"
#include "sawyerFrames.h"




#define BUF_SIZ		1024

void UDPSender::openRawEthListener (string name_)
{
    char sender[INET6_ADDRSTRLEN];
	int ret, i;
	int sockopt;
	ssize_t numbytes;
	struct ifreq ifopts;	/* set promiscuous mode */
	struct ifreq if_ip;	/* get ip addr */
	struct sockaddr_storage their_addr;
	uint8_t buf[BUF_SIZ];
	char ifName[IFNAMSIZ];
	
    strncpy(ifName,name_.c_str(),IFNAMSIZ-1);

    #define ETHER_TYPE	0x0800

	memset(&if_ip, 0, sizeof(struct ifreq));

	/* Open PF_PACKET socket, listening for EtherType ETHER_TYPE */
	//if ((sockMacUdpfd = socket(PF_PACKET, SOCK_RAW, htons(ETHER_TYPE))) == -1) {
    if ((sockMacUdpfd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL))) == -1) {
		perror("listener: socket");	
		return ;
	}


 struct packet_mreq mreq = {0};
  mreq.mr_ifindex = if_nametoindex(ifName);
    mreq.mr_type = PACKET_MR_PROMISC;
 	int action;
 
    if (mreq.mr_ifindex == 0) {
        perror("unable to get interface index");
        //return;
    }

    //if (enable)
        //action = PACKET_ADD_MEMBERSHIP;
    //else
       action = PACKET_DROP_MEMBERSHIP;

    if (setsockopt(sockMacUdpfd, SOL_PACKET, action, &mreq, sizeof(mreq)) != 0) {
        perror("unable to enter promiscouous mode");
        //return;
    }

	/* Set interface to promiscuous mode - do we need to do this every time? */
	// strncpy(ifopts.ifr_name, ifName, IFNAMSIZ-1);
	// ioctl(sockMacUdpfd, SIOCGIFFLAGS, &ifopts);

	// ifopts.ifr_flags |= IFF_PROMISC;
	// ioctl(sockMacUdpfd, SIOCSIFFLAGS, &ifopts);
   
	/* Get the MAC address of the interface to send on */
	memset(&if_mac, 0, sizeof(struct ifreq));
	strncpy(if_mac.ifr_name, ifName, IFNAMSIZ-1);
	if (ioctl(sockMacUdpfd, SIOCGIFHWADDR, &if_mac) < 0)
	    perror("SIOCGIFHWADDR");
    
    RawEthListenerMac.clear();
    for  (int m=0 ; m<6;m++)
    {
        RawEthListenerMac.push_back(if_mac.ifr_hwaddr.sa_data[m]);
    }

	/* Allow the socket to be reused - incase connection is closed prematurely */
	if (setsockopt(sockMacUdpfd, SOL_SOCKET, SO_REUSEADDR, &sockopt, sizeof sockopt) == -1) {
		perror("setsockopt");
		////close(sockMacUdpfd);
		exit(EXIT_FAILURE);
	}
	/* Bind to device */
	if (setsockopt(sockMacUdpfd, SOL_SOCKET, SO_BINDTODEVICE, ifName, IFNAMSIZ-1) == -1)	{
		perror("SO_BINDTODEVICE");
		//close(sockMacUdpfd);
		exit(EXIT_FAILURE);
	}
}


void UDPSender::openRawEth (string name_)
{
/* Open RAW socket to send on */
	/*if ((sockMacUdpfd = socket(AF_PACKET, SOCK_RAW, IPPROTO_RAW)) == -1) {
	    perror("Raw Ethsocket");
	}*/
    if ((sockMacUdpfd = socket(PF_PACKET, SOCK_RAW, IPPROTO_RAW)) == -1) {
	    perror("Raw Ethsocket");
	}
    /*int one = 1;
    const int *val = &one;
    if (setsockopt(sockMacUdpfd, IPPROTO_IP, IP_HDRINCL,  val, sizeof (one)) < 0) 
    {
        printf ("Error setting IP_HDRINCL. Error number : %d . Error message : %s \n" , errno , strerror(errno));
	    exit(0);
    }*/
    char ifName[IFNAMSIZ];
    strncpy(ifName,name_.c_str(),IFNAMSIZ-1);
	/* Get the index of the interface to send on */
	memset(&if_idx, 0, sizeof(struct ifreq));
	strncpy(if_idx.ifr_name, ifName, IFNAMSIZ-1);
	if (ioctl(sockMacUdpfd, SIOCGIFINDEX, &if_idx) < 0)
	    perror("SIOCGIFINDEX");
	/* Get the MAC address of the interface to send on */
	memset(&if_mac, 0, sizeof(struct ifreq));
	strncpy(if_mac.ifr_name, ifName, IFNAMSIZ-1);
	if (ioctl(sockMacUdpfd, SIOCGIFHWADDR, &if_mac) < 0)
	    perror("SIOCGIFHWADDR");

}

int UDPSender::sendRawEth (MacAddress dstmac_,uint16_t ether_type_ , char* data_, int datalen_)
{
    int tx_len = 0;
	char sendbuf[BUF_SIZ];
	struct ether_header *eh = (struct ether_header *) sendbuf;
	struct iphdr *iph = (struct iphdr *) (sendbuf + sizeof(struct ether_header));
	struct sockaddr_ll socket_address;


/* Construct the Ethernet header */
	memset(sendbuf, 0, BUF_SIZ);
	/* Ethernet header */
	eh->ether_shost[0] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[0];
	eh->ether_shost[1] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[1];
	eh->ether_shost[2] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[2];
	eh->ether_shost[3] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[3];
	eh->ether_shost[4] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[4];
	eh->ether_shost[5] = ((uint8_t *)&if_mac.ifr_hwaddr.sa_data)[5];
    
    eh->ether_dhost[0] = dstmac_[0];
	eh->ether_dhost[1] = dstmac_[1];
	eh->ether_dhost[2] = dstmac_[2];
	eh->ether_dhost[3] = dstmac_[3];
	eh->ether_dhost[4] = dstmac_[4];
	eh->ether_dhost[5] = dstmac_[5];
	/* Ethertype field */
	// eh->ether_type = htons(ETH_P_IP);
    // RR specyfic 
    // ethernet2 Type 
    
    //eh->ether_type=(0x464a);// 4a46 = JB Joint Boot ?
    eh->ether_type=ether_type_;

	tx_len += sizeof(struct ether_header);

    /* Packet data */
    memcpy((char*)&sendbuf[tx_len],data_,datalen_);
    tx_len+=datalen_;

	/* Index of the network device */
	socket_address.sll_ifindex = if_idx.ifr_ifindex;
	/* Address length*/
	socket_address.sll_halen = ETH_ALEN;
	// /* Destination MAC */
    socket_address.sll_addr[0] = dstmac_[0];
	socket_address.sll_addr[1] = dstmac_[1];
	socket_address.sll_addr[2] = dstmac_[2];
	socket_address.sll_addr[3] = dstmac_[3];
	socket_address.sll_addr[4] = dstmac_[4];
	socket_address.sll_addr[5] = dstmac_[5];
	/* Send packet */
	int ret = sendto(sockMacUdpfd, sendbuf, tx_len, 0, (struct sockaddr*)&socket_address, sizeof(struct sockaddr_ll));
    if (ret<0)
	    printf("Send failed\n");
    else
    {
     //   printf("Send %d \n",ret);
    }

    return ret;
}

void UDPSender::openIpUDP (string srcIP_, int src_port_)
{
    send_IpUdp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(send_IpUdp_fd < 0)
        throw std::runtime_error("Failed to create a send socket.");

    int enable = 1;
    if (setsockopt(send_IpUdp_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
        throw std::runtime_error("Failed to assign REUSADDR option.");
    }
    
    if (setsockopt(send_IpUdp_fd, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(int)) < 0) {
        throw std::runtime_error("Failed to assign SO_BROADCAST option.");
    }

    struct sockaddr_in srcAddr{};
    memset(&srcAddr, 0, sizeof(srcAddr));
    srcAddr.sin_family = AF_INET;
    //srcAddr.sin_port = htons(0); // Any port will do.
    srcAddr.sin_port = htons(src_port_);
   // inet_pton(AF_INET, my_ip, &srcAddr.sin_addr);
    inet_pton(AF_INET, srcIP_.c_str(), &srcAddr.sin_addr);
    if(bind(send_IpUdp_fd, (struct sockaddr*)&srcAddr, sizeof(srcAddr)) < 0) 
    {
        cout << "Failed to bind sending socket " << strerror(errno)<<"\r\n"<<flush;
        //throw std::runtime_error("Failed to bind a send socket.");
    }
}
ssize_t UDPSender::sendIPUDP( string dstIP_, int dest_port, char* msg,size_t msg_size)
{
    struct sockaddr_in dest_addr{};
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(dest_port);
    //inet_pton(AF_INET, dest_ip, &dest_addr.sin_addr);
    inet_pton(AF_INET, dstIP_.c_str(), &dest_addr.sin_addr);
    
    ssize_t bytes_sent = sendto(
            send_IpUdp_fd,
            msg,
            msg_size,
            0,
            (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (bytes_sent < 0) 
    {
        std::string error_msg = "Failed to send " +
                dstIP_+ " errno: " + std::to_string(errno) + " ";
        error_msg += strerror(errno);

        if (errno == EPERM) {
            error_msg += "; error was due iptables bock rule - expected for some targets.";
            cout << error_msg;
            return 0;
        } else {
            cout << error_msg;
            throw std::runtime_error(error_msg);
        }
    }

     if (bytes_sent != msg_size) {
        cout << "Failed to send the expected number of bytes, wanted :"
            << msg_size << " sent " << bytes_sent;
        throw std::runtime_error("Failed to send the expected number of bytes.");
    }

    return bytes_sent;
}

 int UDPSender::receiveFileTFTP(string strIP,string fileName,vector<uint8_t> &recVect)
 {
        vector <unsigned char> vecBuf;
        vecBuf.insert(vecBuf.end(), { 0x00,TFTP_OPCODE_READ});
        //FIXME : messy
        for (auto c: fileName)
        {
           vecBuf.push_back(c);
        }
        vecBuf.insert(vecBuf.end(), { 0x00,'o','c','t','e','t',0x00});
     
        sendIPUDP(strIP,69,(char*)&vecBuf[0],vecBuf.size());
        int timeout=0;
        char recBuf[1000];
        sockaddr_in srcaddr;
        while ( (timeout < 300))
        {
            socklen_t addrlen = sizeof(struct sockaddr_in);
           int rercBytes=recvfrom(send_IpUdp_fd, recBuf, sizeof(recBuf), MSG_DONTWAIT, (struct sockaddr *)&srcaddr, &addrlen);
            if (rercBytes>0)
            {
                string resStr(recBuf);
                resStr.resize(rercBytes);
                printf("got something %s\n",inet_ntoa(srcaddr.sin_addr));
                cout <<"seenTFT "<<rercBytes<<" : "<<resStr<<" \r\n";
                if (recBuf[1]==TFTP_OPCODE_DATA)
                {
                    uint16_t blockNo=(uint16_t)recBuf[3];
                    cout <<"blockNo "<<blockNo<<" \r\n";
                    for (int a=4;a<rercBytes;a++)
                    {
                       recVect.push_back(recBuf[a]);
                    }
                    // send ACK
                    vector <unsigned char> vecBuf;
                    vecBuf.insert(vecBuf.end(), { 0x00,TFTP_OPCODE_ACK,0x00,(unsigned char)blockNo,0,0,0,0,0,0,0,0});
                    
                    sendIPUDP(strIP,htons(srcaddr.sin_port),(char*)&vecBuf[0],12);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                if (rercBytes<500)
                {
                    cout <<"END RECING DATA TFT : " <<recVect.size()<< "\r\n";
                    break; 
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            timeout++;
        }
    return recVect.size();
 }
