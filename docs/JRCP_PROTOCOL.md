# JRCP: the Sawyer joint-board protocol

This describes the Ethernet protocol between the PC and the Sawyer joint controller boards (JCBs), as AmyCore needs it. Statements taken from these sources are marked in the text:

- **[fw]**: constants from the firmware API in Intera's ROS parameter dump (`param_dump.yaml`, not distributed) (`/firmware_api/...`).
- **[cap]**: the 2023 pcap captures and AmyCore's own tests on the robot.

The joint-specific layouts (section 7) come from [cap] and [fw].

## 1. Link layer

- Raw Ethernet, **ethertype 0x4a46** (bytes `4a 46`, "JF"). No IP or UDP.
- The PC sends to each board's MAC and to the broadcast MAC. Frames are at most 220 bytes (the receive buffer).
- Up to **31 nodes (0–30)**. Node 31 means "all nodes" and is used by the broadcast.
- One master cycle sends one broadcast and serves every node. Intera runs it at **1 kHz** [cap].

## 2. Frame and message layout

Offsets are **payload offsets**, counted from the byte after the ethertype. AmyCore's structs (`JointStatus_t::data[]` in `sawyerFrames.h`) use the same numbering.

| payload | meaning |
|---|---|
| 0 | **message count** N. 0 = one message without a CRC (legacy form, still accepted). N ≥ 1 = N blocks of 68 bytes follow, each ending in a CRC |
| 1 | always 0 (AmyCore's older notes call it a sub-address) |
| 2 … | first block |

Each 68-byte block is a **64-byte message plus a 4-byte CRC**, so with N = 1 the CRC sits at payload 66–69. The master puts **up to three messages for one node into one frame** (e.g. CCmd + ACmd, count 2, 138 payload bytes) [cap].

Message header, the first bytes of each message (offsets for the first block):

| payload | meaning |
|---|---|
| 2 | **node byte** = node << 3 \| sub-address (low 3 bits: used behind a gateway; the Axolotl answers as `0x28`). Our joint boards appear as `0x00/0x08/0x10/0x18` (AmyCore's own numbering) or `0x08…0x20` (Intera's) |
| 3 | always `0x40` |
| 4 | **type byte** = network mode << 4 \| message kind (sections 3 and 4) |
| 5 … 65 | kind-specific |

### CRC

The CRC is the **STM32 hardware CRC**: CRC-32/MPEG-2 (polynomial 0x04C11DB7, initial value 0xFFFFFFFF, no reflection, no final XOR) over the 64-byte message **taken as 16 little-endian 32-bit words**. It's stored little-endian after the message. Verified: it matches the CRC of every counted frame in the 2023 capture, in both directions.

```cpp
uint32_t jrcpCrc(const uint8_t *msg64)          // 64 bytes, returns the CRC to store at msg64[64..67]
{
    uint32_t crc = 0xFFFFFFFF;
    for (int w = 0; w < 16; w++) {
        uint32_t word; memcpy(&word, msg64 + 4*w, 4);   // little-endian word
        crc ^= word;
        for (int b = 0; b < 32; b++)
            crc = (crc & 0x80000000) ? (crc << 1) ^ 0x04C11DB7 : (crc << 1);
    }
    return crc;
}
```

**AmyCore** (`jrcpCrc()` in `sawyerFrames.cpp`) puts the CRC into its broadcast (count 1), like Intera; `AMY_NO_BCAST_CRC=1` turns it off. Before 2026-09-26 the broadcast went out without a valid CRC and the joints accepted it anyway. AmyCore's node frames (CCmd, ACmd) still use the count-0 form without a CRC, which the joints also accept.

## 3. Network modes

The high nibble of the type byte is a network mode (0–15):

| mode | name | seen as |
|---|---|---|
| 1 | INITIALIZATION [fw] | joints answer `0x12` right after they get their node number |
| 2 | CONFIGURATION [fw] | broadcast `0x20`; joints answer `0x22`; the PC sends `0x21` to put a node in this mode. **Register writes are only accepted in this mode** |
| 3 | OPERATIONAL [fw] | PC `0x31`, joints `0x32` with real encoder values; configuration registers are locked (writes answered with error `fb`) |
| 12, 15 | unnamed | a node going from 1 or 12 to 2 or 15 triggers the debug-stream configuration; service requests are allowed in 2, 3 and 15 |
| 13 (0xd) | **reboot to bootloader** (per node) | the master's start mode. As the **global** mode in the broadcast (`0xd0`) it is harmless: Intera's and AmyCore's broadcast carry it all the time. As a **per-node** request (CCmd type `0xd1`) it sends the board back to its bootloader: that is what `jcb-reboot` does (section 9) [cap] |
| 0 | — | IDENT frames (`0x06`) and the first broadcast (`0x00`) |

How modes change:

- The **global** mode is carried by the broadcast. When it changes, every node's CCmd also carries it once.
- A mode for **one node** is requested by putting it into that node's next CCmd type byte once (e.g. `0x21`, `0x31`).
- The mode a node is actually in is the high nibble of its responses.

So AmyCore's CONFIG sequence (one `0x21` CCmd per joint, register writes, then `0x31`) is exactly this mechanism: "enter CONFIGURATION", then "enter OPERATIONAL".

## 4. Message kinds (low nibble of the type byte)

| kind | direction | name | section |
|---|---|---|---|
| 0 | PC → all | broadcast / sync | 5 |
| 1 | PC → node | **CCmd**, cyclic command | 7.1 |
| 2 | node → PC | **CRsp**, cyclic response | 7.2 |
| 3 | node → PC | 64-byte node data block, stored per node (use not decoded) | — |
| 4 | PC → node | **ACmd**, asynchronous command: register reads and writes | 6 |
| 5 | node → PC | **ARsp**, asynchronous response | 6 |
| 6 | node → PC | **IDENT** / announce | 5 |
| 8, 9, 10, 13, 15 | both | bootloader: firmware, parameter-block and calibration-block download | — |

## 5. Broadcast and node assignment

The broadcast (84-byte frame: count 1, one message and CRC):

| payload | meaning |
|---|---|
| 0 | 1 (one message) |
| 2 | `0xf8` = node 31 (all) |
| 3 | `0x40` |
| 4 | global mode << 4 \| 0 |
| 6–7 | **16-bit cycle counter**, +1 per master cycle |
| 8–35 | **node-assignment table**: up to 4 entries of 7 bytes, `[0x80 \| node] [MAC ×6]` |
| 36, 37 | two status bytes set by the master every cycle (meaning not decoded) |
| 66–69 | CRC |

Assignment:

1. An unassigned board sends **IDENT (kind 6)** frames. Its MAC is at payload 44–49; version, build number and board type are also in the frame.
2. The master looks the MAC up (Intera uses `board_config` / `nodeId` from the robot configuration) and gets a node number.
3. It adds `[0x80 | node][MAC]` to the broadcast table (up to 4 entries at a time) until the board answers with its node number. The board then leaves mode 0.
4. A node counts as **lost after 1000 cycles (≈1 s) without a frame**.

Board type codes from the IDENT frame:

| code | name | on this robot |
|---|---|---|
| 0x89 | LITTLE EFT | **all four joint boards** |
| 0x8a | IO CTRLR | torso board (missing on our robot) |
| 0x8b | AXOLOTL | gripper/end-effector board behind the wrist board |
| 0x80 / 0x81 | EFT / P2 EFT | |
| 0x82 | NEWBOT EFT | |
| 0x88 | ANOLE | |
| 0x100/0x101, 0x200/0x201, 0x400/0x401, 0x800/0x801, 0x1000/0x1001 | NEWT, TADPOLE, HEAD, SONAR, TORSO (and P2 variants) | Baxter-era boards |
| 1, 2, 4, 8, 0x10, 0x20, 0x40 | JCB0, JCB_GRANDE, JCBLDC, STM3210C_EVAL, WRISTPP_107, WRISTPP_103, STM3220G_EVAL | development boards |

Image types: `0xfe` = application, `1` = primary bootloader, `2` = secondary bootloader.

## 6. Register access (ACmd / ARsp, kinds 4 and 5)

This is how configuration registers and variables are read and written.

### 6.1 ACmd message

| payload | meaning |
|---|---|
| 2–4 | header (node byte, `0x40`, mode << 4 \| 4) |
| 6 | **sequence number**, +1 for every new ACmd to this node |
| 7 | **`0x3a` = 58**, the length of the request area |
| 8–65 | 58 bytes of requests, zero-padded |

### 6.2 Request encoding

Each request starts with a 4-byte header:

| byte | meaning |
|---|---|
| 0 | `tag << 5 \| op`: **op 1 = read, op 3 = write**; tag = a 3-bit caller context, echoed back |
| 1 | size of the value in bytes (2 or 4; 0x2a for a debug-stream page) |
| 2–3 | **16-bit address, little-endian** = `module_byte << 8 \| variable` |
| 4… | the value (writes only) |

- `module_byte` = firmware module × 4 + group (group 1 = configuration variables, 2 = debug variables, 3 = debug-stream pages). Example: `0x21` = MOTOR module (8) group 1.
- Intera uses **tag 2** for configuration access, so the bytes on the wire are `0x41` (read) and `0x43` (write). The debug-stream configuration uses **tag 0**: `0x03`.
- Intera sends each configuration write **followed by a read of the same address** (`43 … 41 …`), so the reply shows the value the board actually holds.
- Two budgets per frame: the requests must fit into 58 bytes, and the **expected replies must also fit into 58 bytes**. A read costs 4 bytes to send and 4 + size in the reply; a write costs 4 + size to send and 4 in the reply.
- Up to 128 queued requests per node; requests are accepted only while the node is in mode 2, 3 or 15.

### 6.3 ARsp message (kind 5)

The same layout as the ACmd: payload 6 = the sequence number being answered, 7 = `0x3a`, then the replies in request order:

- write OK: `43 <size> <addr lo> <addr hi>` (the request header echoed, no value);
- read OK: `41 <size> <addr lo> <addr hi> <value>`;
- **error: the request byte with bit 4 set (`0x10`) and the error code in place of the size**, no value. `0x53` = refused write, `0x51` = refused read.

Error codes seen [cap]:

| code | meaning |
|---|---|
| `fb` (−5) | **configuration locked**: any configuration write while the node is OPERATIONAL (mode 3) |
| `fa` (−6) | variable can't be read with this size (e.g. MOTOR `0x21` regs 0/3/4/5 read with size 2 instead of 4) |
| `fe` (−2) | unknown or read-only variable (debug variables read directly; module 5 reg 0x34 written) |

### 6.4 Flow control

- One ACmd is in flight per node; it is resent until an ARsp with its sequence number arrives [cap: Intera and AmyCore both behave this way].
- The joint keeps repeating its last ARsp until the PC acknowledges it: **every CCmd carries, in payload 5, the sequence number of the last ARsp received**. The CRsp carries in payload 8 the last ACmd sequence number the node processed.
- If the PC restarts in the middle of an exchange, the joint keeps repeating the old ARsp (e.g. sequence 177) and ignores new ACmds. AmyCore now acknowledges that number and continues from the next one (`ProcessRegistersComm05`, "0x05 sequence resync").

## 7. Joint boards (LITTLE EFT, two joints per board)

The layouts below are from [cap] and AmyCore; the flag values are [fw].

### 7.1 CCmd (PC → joint, kind 1)

The joint command is 42 bytes, payload 6–47.

| payload | meaning |
|---|---|
| 5 | **acknowledgement: last ARsp sequence number received** |
| 6–7 | counter |
| 8–9 | side 1 **torque feed-forward** (int16, **1/250 Nm**; Intera: gravity compensation, e.g. j1 ≈ −6000) [cap: Intera's values = 249–255 × Pinocchio's gravity torque for j1–j5, R² 0.97–1.00 over 10,262 poses] |
| 10–13 | side 1 **position setpoint** (int32, 1/1024 mrad) |
| 14 | side 1 **stiffness in %** (100 normally; 0 in manual teaching; on leaving it Intera ramps 0 → 100 in steps of 5 per ms) [cap] |
| 15–17 | `00 00 00` (constant in every 2023 capture) |
| 18 | side 1 **control mode**: 7 = position (impedance around the setpoint), 10 = manual teaching / free drive (only sent with stiffness 0) [cap] |
| **20–21** | side 1 **joint command flags** (LE16, see below) |
| 26–27 | side 2 torque feed-forward |
| 28–31 | side 2 position setpoint |
| 32, 36 | side 2 stiffness %, control mode |
| **38–39** | side 2 joint command flags |
| **44–47** | **global command flags** (LE32, see below) |

Joint command flags (`JRCP_SEA_CCMD_JOINT_FLAG_*`):

| bit | name |
|---|---|
| 0x0001 | ENABLE |
| 0x0002 | RESET_LATCHED_ERRORS |
| 0x0004 | TARE |
| 0x0008 | DO_HEAD_NOD |
| 0x0100 | AUTO_HOME_AND_UNLOCK_REQUEST |
| 0x0200 | MANUAL_HOME_AND_UNLOCK_REQUEST |
| 0x0400 | ENABLE_HEAD_PAN_REQUEST |
| 0x0010 / 0x0020 | ITB lights inner / outer |

Global command flags (`JRCP_SEA_CCMD_GLOBAL_FLAG_*`, the ones that matter here): 0x40 DISENGAGE_BRAKE, 0x200 CAMERA_STROBE_ENABLE, 0x7c00 nav LEDs A–E, 0x8000/0x10000/0x20000 nav LED red/green/blue, 0x40000 LCD_DISPLAY_POWER, 0x8000000 ASSERT_ESTOP, 0x10000000 LAMP_TEST, **0x20000000 SAFETY_CTRLR_RESUME**, **0x40000000 SELECT_LOW_VMOTOR**.

### 7.2 CRsp (joint → PC, kind 2)

Payload 10–45 are the joint's 36 response bytes; payload 46–65 carry the debug stream (section 8).

| payload | meaning |
|---|---|
| 6–7 | counter |
| 8 | **last ACmd sequence number processed** |
| 9 | **debug-stream page index** |
| 10–13 | side 1 position (int32, 1/1024 mrad) |
| 14–15 | side 1 speed (int16, **1 mrad/s** = 1024 counts/s; fitted against d(position)/dt, j4: 1013 counts/s per unit, corr 0.995) [cap] |
| 16–17 | side 1 effort (torque sensor) |
| 18–19 | side 1 second torque-like value ("force") |
| 20 | side 1 **status flags** |
| 21 | knob position (board 0) |
| 22–23 | side 1 **joint error flags** |
| 24–25 | side 1 extra value (int16; swings strongly while a joint is moved by hand in manual teaching; meaning unknown) |
| 26–29, 30–31, 32–33, 34–35 | side 2 position, speed, effort, force |
| 36 | side 2 status flags |
| 38–39 | side 2 joint error flags |
| 40–41 | analog value (changes during motion) |
| 42 | **global error byte** |
| 43 | buttons (single bits, not sensor data: in the 2023 manual-teach capture board 5e49/fe93 always 0, the elbow board 0x80 for one press, the wrist board 3ee5 bit 0x01 toggling steadily at ~12 Hz / 13 % duty from 60 s after boot, independent of motion; maybe the hand camera's strobe) [cap] |
| 44 | slowly varying (0x14–0x28 in 2023), probably a temperature |
| 45 | unknown (9–13, per board) |
| 46–65 | **20 bytes of debug-stream data** for the page in payload 9 |

Status flags (`JRCP_SEA_CRSP_JOINT_FLAG_*`): 0x01 CURRENT_LIMITING_ACTIVE, 0x02 **ENABLED**, 0x04 IS_JOINT_MOVING, 0x08 CCW_LIMIT, 0x10 **IS_HOMED_AND_UNLOCKED**, 0x20 **IS_HOMING_AND_UNLOCKING**, 0x40 IS_HEAD_PAN_ENABLED. (The firmware list also gives 0x10 as CW_LIMIT and 0x04/0x08 as JAS_HOME_STATE. On this robot 0x10 behaves as HOMED, and 0x04 and 0x08 are set in **every** homed reply of the 2023 captures (status 0x1e), so they are not "moving"/"CCW limit" but most likely the JAS home state.)

Joint error flags (`JRCP_SEA_CRSP_JOINT_ERROR_*`): 0x001 LATCHED, 0x002 MOTOR_TEMPERATURE, 0x004 OVER_CURRENT_LIMIT, 0x008 OVER_TORQUE_LIMIT, 0x010 MAE_SENSOR_ERROR, 0x020 MAE_CONFIG_ERROR, 0x040 SERVO_FAULT, 0x080 **MOTOR_CONFIGURATION**, 0x100 HES_CONFIGURATION_ERROR, 0x200 SERVO_CONFIGURATION_ERROR, 0x400 SDS_SENSOR_ERROR, 0x800 MOTOR_FAULT.

Global error byte (`JRCP_SEA_CRSP_GLOBAL_ERROR_*`): 0x01 HEATSINK_TEMPERATURE, 0x02 UNDERVOLTAGE, 0x04 OVERVOLTAGE, 0x08 NO_EXTERNAL_ENABLE, 0x10 PWR_FAULT, 0x20 SELFTEST_FAIL, 0x40 SAFETY_CONTACTOR_OPEN.

### 7.3 Axolotl (end-effector board, node byte 0x28 behind the wrist board)

Its CRsp (kind 2, with CRC) carries at payload 14/16/18 three int16 values that are an **accelerometer**: they are the
gravity vector in the wrist frame `right_l6`, axes turned 90° about z: payload 14 ≈ +515·g_y, 16 ≈ −497·g_x,
18 ≈ +479·g_z (~500 counts per g, R² 0.94–0.98 against Pinocchio's wrist orientation over 3961 samples; at rest the
vector length is 497 ± 15 in every orientation) [cap]. Its command is the second message in
the PC's frames to the wrist board.

## 8. Debug-variable stream

Each board can stream up to 20 bytes of chosen variables per CRsp, cycling through configured "pages".

Configuration, two kinds of ACmd write with tag 0 to module byte `0x0b` (DEBUG_VAR_STREAMING, group 3):

1. **page count**: `03 02 00 0b <count u16>` (address `0x0b00`, size 2);
2. **each page** 1…count: `03 2a <page> 0b <42 bytes>` (address `0x0b00 | page`, size 0x2a).

Page content: `<n> 00` then n entries `<variable> <module_byte> <size> 02`, then `00`, zero-padded to 42 bytes. `module_byte` = module × 4 + 2 (debug group); for example `0x22` = MOTOR debug, `0x26` = BLDC debug. The data of all entries must fit into 20 bytes.

The joint then sends page by page: CRsp payload 9 = page index (0-based), payload 46–65 = that page's values in entry order.

**Intera sends this configuration automatically** when a node moves into CONFIGURATION. AmyCore sends its own two pages at the end of every CONFIG (`dbgStreamQueue()` in `main.cpp`): page 1 = MOTOR diagnostics (fault status, diagnostic flags, bus voltage, 5 V, enabled, temperature for both motors), page 2 = hall sensor state and counters. The values appear by name in `/tmp/amycore.dbgvars` ("board N decoded: …") and in the Expert window. `dbgpage <board|all> <page> <hex>` still sets pages by hand until the next CONFIG. Intera's own 22 pages are in `DebugVariables` in `sawyerFrames.cpp`.

### 8.1 Torque sensor: zero and scale (measured 2026-09-26)

Each joint's torque comes from a Hall sensor on its spring: **torque [mNm] = (RawHes0 − HesOffset0) × SDSCalibrationSlope / 100**.
`HesOffset0` (config module byte 0x19, reg 0x00 side 1 / 0x02 side 2) is the zero, `SDSCalibrationSlope` (0x19 reg
0x0b / 0x17) the gain; the factory values are in each board's DATABLOCK (`data: jN: {HesOffset0: …}`). Checked on the
wrist board: j6 `RawHes0` 2599, `HesOffset0` 2088, slope 651 → 3327 mNm = debug `Torque0` 3326 = `MeasuredTorque` 3320 =
the CRsp effort 828 × 4 mNm. Useful debug variables (module byte, variable, size; side 1 / side 2):
`RawHes0` 0x1a/0x02 and 0x1a/0x07 (2 B), `Torque0` 0x1a/0x03 and 0x1a/0x08 (4 B, mNm), `MeasuredTorque` 0x12/0x0a and
0x12/0x3c (4 B, mNm), `RefTorque` 0x12/0x0b and 0x12/0x3d, `LoadCellValue` 0x2e/0x00 (2 B). Example (wrist board):
`dbgpage 3 3 0400021a0202071a0202081a04023c12040200` then `dbgpage 3 0 03`; values in `/tmp/amycore.dbgvars` line
`board 3 page  2`. A joint's zero = its `RawHes0` while unloaded and still; AmyCore writes local corrections from
`AmyConfig/joint_calibration.txt` at every CONFIG.

## 9. Bootloaders, TFTP and rebooting a running board

Source: a 2023 capture of the original robot booting [cap].

**Two bootloader generations.** Old boards ran "JRCP" bootloaders (tools `eth-jfiload`, `eth-datablock`, raw Ethernet). `upgrade-jrcp-to-tftp` replaces them with **TFTP bootloaders** (IP/UDP); the 2023 robot and today's boards have those. Each board holds three images, listed in its `BLI` file (YAML): `PRI_BOOT`, `SEC_BOOT` and `JCB_APP` (`image_type`, `board_name`, `board_variant`, `firmware_crc`, `build_info`, `boot_priority`, `firmware_version`). 2023: all 5.2.0, build `00016 1710252050`; Little_Eft = variant 137, Axolotl = 139. "Side jump" switches between the primary and secondary bootloader.

**Bootloader commands: UDP broadcast to port 51200**, 12 bytes = the command padded with spaces to 11 characters + NUL [cap]:

| command | effect |
|---|---|
| `IDENTIFY   ` | every board in a bootloader answers `IDENT,<variant>,<board>,<image>,<ip>,<mac>,DATABLOCK_OK\|BAD,` + 7 more fields for a board gatewayed behind it (the wrist board reports the Axolotl there, `-` otherwise): 14 fields |
| `LOAD APP   ` | start the application (a board with nothing to start answers `NO IMAGE`, e.g. the wrist board for its Axolotl slot) |
| `SIDE JUMP  ` | switch to the other bootloader |
| `AX LOAD APP`, `AX REBOOT  `, `AX SIDEJUMP` | the same for the Axolotl behind the wrist board |

There is **no UDP reboot for a board running the application**: `jcb-tftp-boot -r` calls `jcb-reboot` instead.

**TFTP** (`echo "binary\nrexmt 15\ntimeout 20\nget|put <remote> <local>\n" | tftp <board ip>`), files on each board; prefix `axolotl/` reaches the Axolotl through the wrist board:

| file | access |
|---|---|
| `BLI` | read: image list (YAML, above) |
| `DATABLOCK` | read/write: gzipped YAML. `type` (scapula/humerus/ulna/carpus/hand), `serial`, `model` (mp4), `class` (sawyer), `robot_serial` (Axolotl only), `data` (per-joint `HesOffset0`, `JointAssy2RobotOffset_mRads_q10`, `SDSCalibrationSlope`), `config` (camera IDs and intrinsics: head camera `port:<MAC>`, hand camera `port:203.0.113.130`), and `urdf`: **the per-joint calibrated URDF origins** (xyz/rpy of each joint), i.e. this robot's kinematic calibration |
| `eeprom0/config`, `eeprom1/config` | read; write only after `put <key file> FW_KEY` (unlock) |
| `eeprom0/calibration`, `eeprom1/calibration` | read; write after `FW_KEY` |
| `eeprom0/odometer`, `eeprom1/odometer` | read only ("joint accumulator") |

`eeprom0`/`eeprom1` = joint side 0/1 of a Little_Eft (index > 1 refused).

**Rebooting a running board into its bootloader (`jcb-reboot`)** [cap: 172.8 s and 188.5 s]:

1. run the 1 kHz master in global network mode 2 for 1.5 s, then 300 more cycles to find the nodes;
2. for 100 cycles: CCmd type `0xd1` (mode 13) to every **gatewayed** node (Axolotl, node `0x28`);
3. for 100 cycles: global mode 13 and CCmd `0xd1` to every other board;
4. stop, wait 5 s. The boards go silent at once and come back in their bootloader (answering `IDENTIFY`).

So the boards can be restarted **without cutting their power**: CCmd `0xd1` for ~100 ms, then `IDENTIFY` / `LOAD APP`. Intera did this at every start (2023: `IDENTIFY` → BLI + DATABLOCK reads → `LOAD APP` → reboot → BLI reads (firmware check) → `LOAD APP` → CONFIG).

**2023 addresses**: PC 203.0.113.1, joint boards 203.0.113.13–16, head camera .17 (UDP 51234), hand camera .130, torso IO_Controller 203.0.113.x; default interface `net1`, overridden by `JRCP_INTERFACE`.

## 10. What is still unknown

- The two status bytes in the broadcast (payload 36/37) and modes 12, 13 and 15.
- CCmd payload 15–17/19 (always 0) and CRsp payload 24–25/40–41 ("extra"), 44–45.
- What makes a joint start MANUAL homing by itself: it happened on this robot only after the missing registers were fixed (2026-09-26), so a register was probably involved.
- The CCmd/CRsp layouts of the torso board (0x8a) and the Axolotl (0x8b).
