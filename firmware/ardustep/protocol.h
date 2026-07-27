/*
 * protocol.h  --  THE FROZEN CONTRACT
 *
 * This is the single source of truth for the wire format between the LinuxCNC
 * host driver and the Arduino. It is shared verbatim by:
 *    - the firmware (this file, included by ardustep.ino)
 *    - the C HAL component, if/when it is written (it will #include this file)
 * and mirrored byte-for-byte by linuxcnc/protocol.py (Python struct formats).
 *
 * Rules for keeping the contract intact:
 *    - AVR is little-endian, structs are __attribute__((packed)) -> no padding.
 *    - Python uses struct format '<' (little-endian, no alignment) to match.
 *    - If you change ANY field here, update protocol.py in the same commit and
 *      bump PROTO_VERSION so a mismatched pair refuses to run.
 */
#ifndef ARDUSTEP_PROTOCOL_H
#define ARDUSTEP_PROTOCOL_H

#include <stdint.h>

#define PROTO_VERSION   1

#define NUM_AXES        3

#define CMD_MAGIC       0xA5    /* PC -> MCU */
#define FB_MAGIC        0x5A    /* MCU -> PC */

/* CommandPacket.flags bits */
#define FLAG_ENABLE     0x01    /* drivers/motion enabled                  */
#define FLAG_ESTOP      0x02    /* emergency stop asserted -> hard zero vel */
#define FLAG_SPINDLE    0x04    /* spindle on/off (PWM duty = spindle field)*/

/* FeedbackPacket.status bits */
#define ST_RUNNING      0x01    /* at least one axis is moving             */
#define ST_ENABLED      0x02    /* echo of FLAG_ENABLE as latched by MCU   */
#define ST_FAULT        0x04    /* watchdog tripped / estop latched        */
#define ST_UNDERRUN     0x08    /* a commanded move exceeded step ceiling  */

/* FeedbackPacket.limits bits (active = switch pressed) */
#define LIM_X           0x01
#define LIM_Y           0x02
#define LIM_Z           0x04

/* PC -> MCU, 18 bytes. Sent once per host cycle (nominally 1 kHz). */
typedef struct __attribute__((packed)) {
    uint8_t  magic;          /* = CMD_MAGIC                                 */
    uint8_t  seq;            /* rolling sequence, echoed back for drop det. */
    uint8_t  flags;          /* FLAG_*                                      */
    int32_t  pos_cmd[NUM_AXES]; /* absolute commanded position, in STEPS    */
    uint16_t spindle;        /* 0..65535 spindle duty (future PWM)          */
    uint8_t  crc;            /* CRC-8 over bytes [0 .. sizeof-2]            */
} CommandPacket;

/* MCU -> PC, 19 bytes. Sent in reply to every CommandPacket. */
typedef struct __attribute__((packed)) {
    uint8_t  magic;          /* = FB_MAGIC                                  */
    uint8_t  seq;            /* echo of the CommandPacket.seq we acted on   */
    uint8_t  status;         /* ST_*                                        */
    uint8_t  limits;         /* LIM_*                                       */
    int32_t  pos_fb[NUM_AXES];  /* actual step count generated, in STEPS    */
    uint16_t underruns;      /* cumulative underrun events (diagnostic)     */
    uint8_t  crc;            /* CRC-8 over bytes [0 .. sizeof-2]            */
} FeedbackPacket;

#define CMD_PACKET_LEN  18
#define FB_PACKET_LEN   19

/* CRC-8, polynomial 0x07 (CRC-8/SMBus), init 0x00. Table-free. */
static inline uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

#endif /* ARDUSTEP_PROTOCOL_H */
