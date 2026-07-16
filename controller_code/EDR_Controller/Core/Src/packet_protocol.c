#include "packet_protocol.h"
#include "usbd_cdc_if.h"   /* CDC_Transmit_FS() */
#include <stdio.h>
#include <string.h>

extern volatile uint8_t CDC_IsInReceiveCallback;

/* ---------------------------------------------------------------------
 * Debug printing helper — builds a string with sprintf and pushes it
 * out over USB CDC instead of a UART-backed printf.
 * ------------------------------------------------------------------- */
#define DEBUG_BUF_SIZE 256

static void Packet_DebugPrint(const char *str)
{
    if (CDC_IsInReceiveCallback != 0U) {
        return;
    }
    CDC_Transmit_FS((uint8_t *)str, (uint16_t)strlen(str));
}

/* ---------------------------------------------------------------------
 * Parser state machine
 * Byte-by-byte so partial USB packets (fragmented across multiple
 * CDC_Receive_FS calls) are handled correctly.
 * ------------------------------------------------------------------- */
typedef enum {
    PKT_STATE_WAIT_STX = 0,
    PKT_STATE_HEADER,
    PKT_STATE_PAYLOAD,
    PKT_STATE_CHECKSUM,
    PKT_STATE_WAIT_ETX
} PacketParseState_t;

static PacketParseState_t s_state = PKT_STATE_WAIT_STX;
static uint8_t  s_header;
static uint8_t  s_payload[PKT_PAYLOAD_SIZE];
static uint8_t  s_payloadIdx;
static uint8_t  s_rxChecksum;
static uint8_t  s_calcChecksum;

/* ---------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------- */
static uint8_t Packet_Checksum(uint8_t header, const uint8_t *payload)
{
    uint8_t chk = header;
    for (int i = 0; i < PKT_PAYLOAD_SIZE; i++) {
        chk ^= payload[i];
    }
    return chk;
}

static void Packet_SerializePayload(uint8_t *out, int16_t x, int16_t y, uint32_t ts)
{
    out[0] = (uint8_t)(x & 0xFF);
    out[1] = (uint8_t)((x >> 8) & 0xFF);
    out[2] = (uint8_t)(y & 0xFF);
    out[3] = (uint8_t)((y >> 8) & 0xFF);
    out[4] = (uint8_t)(ts & 0xFF);
    out[5] = (uint8_t)((ts >> 8) & 0xFF);
    out[6] = (uint8_t)((ts >> 16) & 0xFF);
    out[7] = (uint8_t)((ts >> 24) & 0xFF);
}

static void Packet_DeserializePayload(const uint8_t *in, PacketData_t *pkt)
{
    pkt->x = (int16_t)(in[0] | (in[1] << 8));
    pkt->y = (int16_t)(in[2] | (in[3] << 8));
    pkt->timestamp = (uint32_t)in[4] |
                      ((uint32_t)in[5] << 8) |
                      ((uint32_t)in[6] << 16) |
                      ((uint32_t)in[7] << 24);
}

static void Packet_ResetParser(void)
{
    s_state = PKT_STATE_WAIT_STX;
    s_payloadIdx = 0;
}

/* Transmit a fully framed packet over USB CDC */
static uint8_t Packet_SendFrame(uint8_t header, int16_t x, int16_t y, uint32_t timestamp)
{
    uint8_t frame[PKT_FRAME_SIZE];
    uint8_t payload[PKT_PAYLOAD_SIZE];

    Packet_SerializePayload(payload, x, y, timestamp);

    frame[0] = PKT_STX;
    frame[1] = header;
    memcpy(&frame[2], payload, PKT_PAYLOAD_SIZE);
    frame[2 + PKT_PAYLOAD_SIZE] = Packet_Checksum(header, payload);
    frame[3 + PKT_PAYLOAD_SIZE] = PKT_ETX;

    Packet_OnRawBytesTransmitted(frame, PKT_FRAME_SIZE);

    uint8_t result = CDC_Transmit_FS(frame, PKT_FRAME_SIZE);

    if (result == USBD_OK) {
        PacketData_t sent = { .header = header, .x = x, .y = y, .timestamp = timestamp };
        if (header == PKT_HEADER_ACK) {
            Packet_OnAckToLaptop(&sent);
        }
    }

    return result;
}

/* ---------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------- */
void Packet_Init(void)
{
    Packet_ResetParser();
}

uint8_t Packet_SendData(int16_t x, int16_t y, uint32_t timestamp)
{
    return Packet_SendFrame(PKT_HEADER_DATA, x, y, timestamp);
}

uint8_t Packet_SendAck(int16_t x, int16_t y, uint32_t timestamp)
{
    return Packet_SendFrame(PKT_HEADER_ACK, x, y, timestamp);
}

/* Feed raw incoming USB CDC bytes through the parser one at a time */
void Packet_ReceiveBytes(const uint8_t *buf, uint32_t len)
{
    Packet_OnRawBytesReceived(buf, len);

    for (uint32_t i = 0; i < len; i++) {
        uint8_t b = buf[i];

        switch (s_state) {

        case PKT_STATE_WAIT_STX:
            if (b == PKT_STX) {
                s_state = PKT_STATE_HEADER;
            }
            break;

        case PKT_STATE_HEADER:
            if (b == PKT_HEADER_DATA || b == PKT_HEADER_ACK) {
                s_header = b;
                s_payloadIdx = 0;
                s_state = PKT_STATE_PAYLOAD;
            } else {
                /* Unexpected header, resync on next STX */
                Packet_OnFrameError();
                Packet_ResetParser();
            }
            break;

        case PKT_STATE_PAYLOAD:
            s_payload[s_payloadIdx++] = b;
            if (s_payloadIdx >= PKT_PAYLOAD_SIZE) {
                s_calcChecksum = Packet_Checksum(s_header, s_payload);
                s_state = PKT_STATE_CHECKSUM;
            }
            break;

        case PKT_STATE_CHECKSUM:
            s_rxChecksum = b;
            s_state = PKT_STATE_WAIT_ETX;
            break;

        case PKT_STATE_WAIT_ETX:
            if (b == PKT_ETX && s_rxChecksum == s_calcChecksum) {
                PacketData_t pkt;
                pkt.header = s_header;
                Packet_DeserializePayload(s_payload, &pkt);

                if (s_header == PKT_HEADER_DATA) {
                    Packet_OnDataFromLaptop(&pkt);
                    /* Auto-acknowledge every data packet received from laptop */
                    Packet_SendAck(pkt.x, pkt.y, pkt.timestamp);
                }
                /* If you also expect the laptop to send 'A' packets for some
                 * other reason, handle s_header == PKT_HEADER_ACK here too. */
            } else {
                Packet_OnFrameError();
            }
            Packet_ResetParser();
            break;

        default:
            Packet_ResetParser();
            break;
        }
    }
}

/* ---------------------------------------------------------------------
 * Weak default debug callbacks — override in your application code by
 * defining a non-weak function with the same signature elsewhere.
 * Assumes printf is retargeted to a UART (e.g. via _write in syscalls.c).
 * ------------------------------------------------------------------- */
__attribute__((weak)) void Packet_OnDataFromLaptop(const PacketData_t *pkt)
{
    char buf[DEBUG_BUF_SIZE];
    sprintf(buf, "[RX <- LAPTOP] P  X=%6d  Y=%6d  T=%lu ms\r\n",
            pkt->x, pkt->y, (unsigned long)pkt->timestamp);
    Packet_DebugPrint(buf);
}

__attribute__((weak)) void Packet_OnAckToLaptop(const PacketData_t *ack)
{
    char buf[DEBUG_BUF_SIZE];
    sprintf(buf, "[TX -> LAPTOP] A  X=%6d  Y=%6d  T=%lu ms\r\n",
            ack->x, ack->y, (unsigned long)ack->timestamp);
    Packet_DebugPrint(buf);
}

__attribute__((weak)) void Packet_OnFrameError(void)
{
    char buf[DEBUG_BUF_SIZE];
    sprintf(buf, "[PKT ERROR] Bad checksum or framing, dropping frame\r\n");
    Packet_DebugPrint(buf);
}

__attribute__((weak)) void Packet_OnRawBytesReceived(const uint8_t *buf, uint32_t len)
{
    char msg[DEBUG_BUF_SIZE];
    int  offset = sprintf(msg, "[CDC RX] %lu bytes: ", (unsigned long)len);

    for (uint32_t i = 0; i < len && offset < (DEBUG_BUF_SIZE - 4); i++) {
        offset += sprintf(&msg[offset], "%02X ", buf[i]);
    }
    offset += sprintf(&msg[offset], "\r\n");

    Packet_DebugPrint(msg);
}

__attribute__((weak)) void Packet_OnRawBytesTransmitted(const uint8_t *buf, uint32_t len)
{
    char msg[DEBUG_BUF_SIZE];
    int  offset = sprintf(msg, "[CDC TX] %lu bytes: ", (unsigned long)len);

    for (uint32_t i = 0; i < len && offset < (DEBUG_BUF_SIZE - 4); i++) {
        offset += sprintf(&msg[offset], "%02X ", buf[i]);
    }
    offset += sprintf(&msg[offset], "\r\n");

    Packet_DebugPrint(msg);
}
