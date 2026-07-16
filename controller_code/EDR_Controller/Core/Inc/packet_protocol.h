#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---------------- Framing ---------------- */
#define PKT_STX             0x02
#define PKT_ETX             0x03

#define PKT_HEADER_DATA      'P'   /* Laptop -> PCB  : data packet      */
#define PKT_HEADER_ACK       'A'   /* PCB    -> Laptop: acknowledgment  */

#define PKT_PAYLOAD_SIZE     8     /* X(2) + Y(2) + Timestamp(4)        */
#define PKT_FRAME_SIZE       (1 + 1 + PKT_PAYLOAD_SIZE + 1 + 1) /* = 12 */

/* ---------------- Data model ---------------- */
typedef struct {
    uint8_t  header;      /* PKT_HEADER_DATA or PKT_HEADER_ACK */
    int16_t  x;
    int16_t  y;
    uint32_t timestamp;   /* ms, originated by the laptop      */
} PacketData_t;

/* ---------------- Init ---------------- */
void Packet_Init(void);

/* Feed raw bytes coming from CDC_Receive_FS() into the parser.
 * Call this directly from the USB CDC receive callback. */
void Packet_ReceiveBytes(const uint8_t *buf, uint32_t len);

/* Build + transmit a 'P' data packet PCB -> laptop (optional/test use) */
uint8_t Packet_SendData(int16_t x, int16_t y, uint32_t timestamp);

/* Build + transmit an 'A' ack packet PCB -> laptop */
uint8_t Packet_SendAck(int16_t x, int16_t y, uint32_t timestamp);

extern volatile uint8_t CDC_IsInReceiveCallback;

/* ---------------- User callbacks (weak, override as needed) ---------------- */

/* Called whenever a full, valid 'P' packet is received FROM the laptop */
void Packet_OnDataFromLaptop(const PacketData_t *pkt);

/* Called whenever an 'A' ack packet is TRANSMITTED back to the laptop */
void Packet_OnAckToLaptop(const PacketData_t *ack);

/* Called whenever a byte stream fails checksum/framing validation */
void Packet_OnFrameError(void);

/* ---------------------------------------------------------------------
 * Raw CDC-level callbacks — fire on every raw byte chunk in/out of USB
 * CDC, independent of whether it parses into a valid packet. Useful for
 * low-level debugging (e.g. seeing exactly what hit the wire).
 * ------------------------------------------------------------------- */
void Packet_OnRawBytesReceived(const uint8_t *buf, uint32_t len);
void Packet_OnRawBytesTransmitted(const uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* PACKET_PROTOCOL_H */
