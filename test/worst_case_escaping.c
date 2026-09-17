// Regression test for the TX frame buffer sizing in main.c.
//
// main.c accumulates a full encoded SLIP frame in slip_tx_buf before writing
// it to the serial port in one shot. Its size must cover the worst case: a
// full MTU payload where every byte is a special byte (END/ESC) and therefore
// escapes to two bytes on the wire. This test builds exactly that frame and
// checks that:
//   1. the encoded frame fits in SLIP_TX_BUFFER_SIZE,
//   2. the frame is delimited by a raw END on both sides with no interior END
//      (the framing assumption slip_write_byte() relies on),
//   3. it decodes back to the original payload.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "slip/slip.h"

// Mirror the sizing from main.c.
#define BUFFER_SIZE         1600
#define SLIP_TX_BUFFER_SIZE ((BUFFER_SIZE + 2) * 2 + 2)

static uint8_t encoded[SLIP_TX_BUFFER_SIZE * 2];
static size_t  encoded_len;

static uint8_t decoded[BUFFER_SIZE + 4];
static size_t  decoded_len;
static int     decoded_seen;

static uint8_t decode_buf[BUFFER_SIZE + 8];

static uint8_t capture_write_byte(uint8_t byte) {
    if (encoded_len < sizeof(encoded)) {
        encoded[encoded_len++] = byte;
    }
    return 1;
}

static void capture_recv_message(uint8_t *data, uint32_t size) {
    decoded_seen = 1;
    decoded_len = size;
    if (size <= sizeof(decoded)) {
        memcpy(decoded, data, size);
    }
}

static const slip_descriptor_s descriptor = {
    .buf = decode_buf,
    .buf_size = sizeof(decode_buf),
    .crc_seed = 0xFFFF,
    .recv_message = capture_recv_message,
    .write_byte = capture_write_byte,
};

static int fail(const char *msg) {
    fprintf(stderr, "FAIL: %s\n", msg);
    return 1;
}

int main(void) {
    slip_handler_s slip;
    uint8_t payload[BUFFER_SIZE];

    // Worst case for escaping: every byte is a special byte.
    for (size_t i = 0; i < sizeof(payload); i++) {
        payload[i] = (i % 2 == 0) ? SLIP_SPECIAL_BYTE_END : SLIP_SPECIAL_BYTE_ESC;
    }

    slip_init(&slip, &descriptor);

    if (slip_send_message(&slip, payload, sizeof(payload)) != SLIP_NO_ERROR) {
        return fail("slip_send_message returned an error");
    }

    // 1. The whole encoded frame must fit the TX buffer main.c allocates.
    if (encoded_len > SLIP_TX_BUFFER_SIZE) {
        fprintf(stderr, "encoded_len=%zu exceeds SLIP_TX_BUFFER_SIZE=%d\n",
                encoded_len, SLIP_TX_BUFFER_SIZE);
        return fail("encoded frame does not fit TX buffer");
    }

    // 2. Framing: a raw END delimits both ends, and never appears in between.
    if (encoded_len < 2 ||
        encoded[0] != SLIP_SPECIAL_BYTE_END ||
        encoded[encoded_len - 1] != SLIP_SPECIAL_BYTE_END) {
        return fail("frame is not END-delimited on both sides");
    }
    for (size_t i = 1; i < encoded_len - 1; i++) {
        if (encoded[i] == SLIP_SPECIAL_BYTE_END) {
            return fail("unexpected interior END byte");
        }
    }

    // 3. Round-trip: decoding the frame yields the original payload.
    for (size_t i = 0; i < encoded_len; i++) {
        if (slip_read_byte(&slip, encoded[i]) != SLIP_NO_ERROR) {
            return fail("slip_read_byte returned an error");
        }
    }

    if (!decoded_seen) {
        return fail("decoder never produced a message");
    }
    if (decoded_len != sizeof(payload)) {
        fprintf(stderr, "decoded_len=%zu expected=%zu\n",
                decoded_len, sizeof(payload));
        return fail("decoded length mismatch");
    }
    if (memcmp(decoded, payload, sizeof(payload)) != 0) {
        return fail("decoded payload mismatch");
    }

    printf("PASS: worst-case escaping (payload=%zu, encoded=%zu, limit=%d)\n",
           sizeof(payload), encoded_len, SLIP_TX_BUFFER_SIZE);
    return 0;
}
