// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#pragma once
// linkdiag: why a joint board sends no frames.
// Host side (checked by a background thread every second): link (carrier) on the robot interface, its IPv4 address,
// and whether the DHCP server (udhcpd) runs (boards in their bootloader need one to get an IP and answer IDENTIFY).
// Robot side: the receive thread reports every packet it sees on the robot interface, and linkdiag keeps per sender
// MAC what arrived: joint frames AmyCore accepted, packets it ignored (and why: size, destination, unknown sender,
// ethertype), DHCP requests (= board in its bootloader), TFTP, UDP 51200 (IDENT / boot commands).
#include <string>
#include <vector>
#include <cstdint>

struct LinkDiagBoard { std::string name; std::vector<uint8_t> mac; std::string ident; };

void linkDiagInit(const char *ifname);          // starts the host check thread and prints the first host check
// receive thread, every packet: sll_ifindex / sll_pkttype from recvfrom, accepted = taken as a joint frame
void linkDiagPacket(const uint8_t *buf, int len, int ifindex, int pkttype, bool accepted);
std::string linkDiagHost();                     // one line: link, IP, DHCP server (problems marked with "!")
bool linkDiagHostOk();                          // link up, IP set, udhcpd running
// one-line reason why a board is silent; everAliveAgoMs = ms since its last accepted frame, -1 = never
std::string linkDiagBoard(const LinkDiagBoard &b, long long everAliveAgoMs);
std::string linkDiagReport(const std::vector<LinkDiagBoard> &boards);   // multi-line: host + every sender MAC
std::string macStr(const uint8_t *m);
