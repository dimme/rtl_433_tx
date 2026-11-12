// Bootstrap a valid Toyota TPMS capture.json for end-to-end TX testing.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bitbuffer.h"
#include "capture.h"
#include "pulse_tx.h"
#include "r_device.h"
#include "r_util.h"
#include "bit_util.h"

extern r_device const tpms_toyota;

static void set_bit(uint8_t *row, unsigned idx, int val)
{
    if (val) row[idx / 8] |= (uint8_t)(1 << (7 - (idx % 8)));
    else     row[idx / 8] &= (uint8_t)~(1 << (7 - (idx % 8)));
}

int main(void)
{
    // message bytes
    unsigned id = 0x01234567;
    int pressure1 = (int)((32.0 + 7.0) * 4.0 + 0.5); // 156
    int temp = (int)(25.0 + 40.0 + 0.5);             // 65
    int status = 0;
    uint8_t b[9];
    b[0] = id >> 24; b[1] = id >> 16; b[2] = id >> 8; b[3] = id;
    b[4] = (uint8_t)((status & 0x80) | ((pressure1 >> 1) & 0x7f));
    b[5] = (uint8_t)(((pressure1 & 1) << 7) | ((temp >> 1) & 0x7f));
    b[6] = (uint8_t)(((temp & 1) << 7) | (status & 0x7f));
    b[7] = (uint8_t)(pressure1 ^ 0xff);
    b[8] = crc8(b, 8, 0x07, 0x80);

    // Build full raw bit row: preamble + 1 lead bit + 144 diff-manchester half-bits + trailer.
    // preamble pattern searched is {0xa9,0xe0} (12 bits), decode starts at bitpos+11.
    uint8_t row[64] = {0};
    unsigned pos = 0;
    // leading idle/sync: use 0x55 0x55 then the 0xa9 0xe0 preamble.
    // Put preamble aligned at byte boundary.
    // preamble bits: 1010 1001 1110 0000 -> 0xa9 0xe0
    row[0] = 0xa9; row[1] = 0xe0; pos = 12;
    // The decode starts at bitpos+11 = 11. bit at index 10 is the "prev" seed.
    unsigned pstart = 11;
    // seed prev from bit index 10 (within 0xa9 0xe0: byte1 bit2). 0xe0=11100000, index8=1,9=1,10=1
    int prev = 1;
    pos = pstart;
    for (unsigned i = 0; i < 72; ++i) {
        int d  = (b[i / 8] >> (7 - (i % 8))) & 1;
        int h1 = 1 - prev;
        int h2 = d ? h1 : (1 - h1);
        set_bit(row, pos++, h1);
        set_bit(row, pos++, h2);
        prev = h2;
    }
    // trailer: mark bits (1s) so they survive PCM slicing, giving the decoder
    // enough length (needs preamble_pos + 156 bits). Ends before the reset gap.
    unsigned total = pos + 8;
    for (unsigned i = pos; i < total; ++i) set_bit(row, i, 1);

    bitbuffer_t bits = {0};
    bits.num_rows = 1;
    bits.bits_per_row[0] = (uint16_t)total;
    memcpy(bits.bb[0], row, (total + 7) / 8);

    // Verify the decoder accepts it.
    capture_t cap = {0};
    extern int encoder_decode_once(r_device const *, bitbuffer_t *, capture_t *);
    r_device devcopy = tpms_toyota; // writable copy (global is const/.rodata)
    int ev = encoder_decode_once(&devcopy, &bits, &cap);
    fprintf(stderr, "decode events=%d, pressure=%s temp=%s id=%s\n", ev,
            capture_get_field(&cap, "pressure_PSI"),
            capture_get_field(&cap, "temperature_C"),
            capture_get_field(&cap, "id"));
    if (ev <= 0) { fprintf(stderr, "FAILED to construct valid frame\n"); return 1; }

    // Fill capture metadata and pulses.
    cap.protocol = 0;
    free(cap.model); cap.model = strdup("Toyota TPMS");
    cap.modulation_num = tpms_toyota.modulation;
    cap.is_fsk = 1;
    cap.freq_hz = 315000000; // Toyota TPMS in NA is 315 MHz
    cap.sample_rate = 250000;
    cap.short_us = 52; cap.long_us = 52; cap.reset_us = 150;
    cap.bits = bits;
    cap.pulses.sample_rate = cap.sample_rate;
    if (pulse_tx_from_bits(&bits, &devcopy, &cap.pulses, cap.sample_rate) < 0) {
        fprintf(stderr, "pulse_tx_from_bits failed\n");
        return 1;
    }

    FILE *f = fopen("toyota_cap.json", "w");
    capture_write_json(f, &cap);
    fclose(f);
    fprintf(stderr, "wrote toyota_cap.json (%u pulses)\n", cap.pulses.num_pulses);
    return 0;
}
