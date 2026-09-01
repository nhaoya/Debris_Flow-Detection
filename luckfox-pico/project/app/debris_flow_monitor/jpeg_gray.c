#include "jpeg_gray.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Standard JPEG luminance quantization table, natural 8x8 order. */
static const uint8_t k_q_luma[64] = {
    16,11,10,16,24,40,51,61,
    12,12,14,19,26,58,60,55,
    14,13,16,24,40,57,69,56,
    14,17,22,29,51,87,80,62,
    18,22,37,56,68,109,103,77,
    24,35,55,64,81,104,113,92,
    49,64,78,87,103,121,120,101,
    72,92,95,98,112,100,103,99
};

static const uint8_t k_zigzag[64] = {
     0, 1, 8,16, 9, 2, 3,10,
    17,24,32,25,18,11, 4, 5,
    12,19,26,33,40,48,41,34,
    27,20,13, 6, 7,14,21,28,
    35,42,49,56,57,50,43,36,
    29,22,15,23,30,37,44,51,
    58,59,52,45,38,31,39,46,
    53,60,61,54,47,55,62,63
};

/* JPEG Annex K default Huffman tables for luminance. bits[0] is unused. */
static const uint8_t k_dc_bits[17] = {
    0, 0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0
};
static const uint8_t k_dc_vals[12] = {
    0,1,2,3,4,5,6,7,8,9,10,11
};
static const uint8_t k_ac_bits[17] = {
    0, 0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d
};
static const uint8_t k_ac_vals[162] = {
    0x01,0x02,
    0x03,
    0x00,0x04,0x11,
    0x05,0x12,0x21,
    0x31,0x41,
    0x06,0x13,0x51,0x61,
    0x07,0x22,0x71,
    0x14,0x32,0x81,0x91,0xa1,
    0x08,0x23,0x42,0xb1,0xc1,
    0x15,0x52,0xd1,0xf0,
    0x24,0x33,0x62,0x72,
    0x82,
    0x09,0x0a,0x16,0x17,0x18,0x19,0x1a,0x25,
    0x26,0x27,0x28,0x29,0x2a,0x34,0x35,0x36,
    0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,
    0x47,0x48,0x49,0x4a,0x53,0x54,0x55,0x56,
    0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,
    0x67,0x68,0x69,0x6a,0x73,0x74,0x75,0x76,
    0x77,0x78,0x79,0x7a,0x83,0x84,0x85,0x86,
    0x87,0x88,0x89,0x8a,0x92,0x93,0x94,0x95,
    0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,
    0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,
    0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,
    0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,
    0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,
    0xda,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
    0xe8,0xe9,0xea,0xf1,0xf2,0xf3,0xf4,0xf5,
    0xf6,0xf7,0xf8,0xf9,0xfa
};

typedef struct {
    uint16_t code[256];
    uint8_t size[256];
} HuffTable;

typedef struct {
    uint8_t *buf;
    size_t cap;
    size_t pos;
    uint32_t bit_buf;
    unsigned bit_count;
    int error;
} JpegWriter;

static double g_basis[8][8];
static int g_basis_ready = 0;

static void init_basis(void) {
    int u, x;
    if (g_basis_ready) return;
    for (u = 0; u < 8; ++u) {
        const double c = (u == 0) ? (1.0 / sqrt(2.0)) : 1.0;
        for (x = 0; x < 8; ++x)
            g_basis[u][x] = 0.5 * c * cos(((2.0 * x + 1.0) * u * M_PI) / 16.0);
    }
    g_basis_ready = 1;
}

static void build_huffman(HuffTable *table,
                          const uint8_t bits[17],
                          const uint8_t *vals,
                          size_t val_count) {
    uint16_t code = 0;
    size_t k = 0;
    int len;
    memset(table, 0, sizeof(*table));
    for (len = 1; len <= 16; ++len) {
        int i;
        for (i = 0; i < bits[len] && k < val_count; ++i) {
            table->code[vals[k]] = code;
            table->size[vals[k]] = (uint8_t)len;
            ++code;
            ++k;
        }
        code <<= 1;
    }
}

static void jw_byte(JpegWriter *w, uint8_t v) {
    if (w->error) return;
    if (w->pos >= w->cap) { w->error = 1; return; }
    w->buf[w->pos++] = v;
}

static void jw_u16be(JpegWriter *w, uint16_t v) {
    jw_byte(w, (uint8_t)(v >> 8));
    jw_byte(w, (uint8_t)(v & 0xffU));
}

static void jw_marker(JpegWriter *w, uint8_t marker) {
    jw_byte(w, 0xffU);
    jw_byte(w, marker);
}

static void jw_entropy_byte(JpegWriter *w, uint8_t v) {
    jw_byte(w, v);
    if (v == 0xffU) jw_byte(w, 0x00U);
}

static void jw_bits(JpegWriter *w, uint16_t bits, unsigned count) {
    if (w->error || count == 0U) return;
    w->bit_buf = (w->bit_buf << count) | (uint32_t)(bits & ((1U << count) - 1U));
    w->bit_count += count;
    while (w->bit_count >= 8U) {
        unsigned shift = w->bit_count - 8U;
        uint8_t out = (uint8_t)((w->bit_buf >> shift) & 0xffU);
        jw_entropy_byte(w, out);
        w->bit_count -= 8U;
        if (w->bit_count == 0U) w->bit_buf = 0U;
        else w->bit_buf &= (1U << w->bit_count) - 1U;
    }
}

static void jw_flush_bits(JpegWriter *w) {
    if (w->bit_count > 0U) {
        unsigned pad = 8U - w->bit_count;
        uint16_t ones = (uint16_t)((1U << pad) - 1U);
        jw_bits(w, ones, pad);
    }
}

static int magnitude_category(int value) {
    unsigned a;
    int n = 0;
    if (value == 0) return 0;
    a = (unsigned)(value < 0 ? -value : value);
    while (a) { ++n; a >>= 1; }
    return n;
}

static uint16_t magnitude_bits(int value, int category) {
    if (category == 0) return 0U;
    if (value >= 0) return (uint16_t)value;
    return (uint16_t)(value + ((1 << category) - 1));
}

static void write_dht(JpegWriter *w,
                      uint8_t table_class_and_id,
                      const uint8_t bits[17],
                      const uint8_t *vals,
                      size_t val_count) {
    int i;
    jw_marker(w, 0xc4U);
    jw_u16be(w, (uint16_t)(2U + 1U + 16U + val_count));
    jw_byte(w, table_class_and_id);
    for (i = 1; i <= 16; ++i) jw_byte(w, bits[i]);
    for (i = 0; i < (int)val_count; ++i) jw_byte(w, vals[i]);
}

static void scaled_quant_table(int quality, uint8_t q[64]) {
    int scale;
    int i;
    if (quality < 1) quality = 1;
    if (quality > 100) quality = 100;
    scale = quality < 50 ? 5000 / quality : 200 - quality * 2;
    for (i = 0; i < 64; ++i) {
        int v = (k_q_luma[i] * scale + 50) / 100;
        if (v < 1) v = 1;
        if (v > 255) v = 255;
        q[i] = (uint8_t)v;
    }
}

static uint8_t sample_scaled(const GrayImage *src,
                             int ox, int oy,
                             int out_width, int out_height) {
    int sx = (int)(((int64_t)ox * src->width) / out_width);
    int sy = (int)(((int64_t)oy * src->height) / out_height);
    if (sx < 0) sx = 0;
    if (sy < 0) sy = 0;
    if (sx >= src->width) sx = src->width - 1;
    if (sy >= src->height) sy = src->height - 1;
    return src->data[(size_t)sy * (size_t)src->stride + (size_t)sx];
}

static void encode_block(JpegWriter *w,
                         const GrayImage *src,
                         int block_x,
                         int block_y,
                         int out_width,
                         int out_height,
                         const uint8_t q[64],
                         const HuffTable *dc,
                         const HuffTable *ac,
                         int *prev_dc) {
    double block[8][8];
    double temp[8][8];
    int coeff[64];
    int x, y, u, v;
    int dc_diff, cat;
    int run = 0;
    int k;

    for (y = 0; y < 8; ++y) {
        int oy = block_y + y;
        if (oy >= out_height) oy = out_height - 1;
        for (x = 0; x < 8; ++x) {
            int ox = block_x + x;
            if (ox >= out_width) ox = out_width - 1;
            block[y][x] = (double)sample_scaled(src, ox, oy, out_width, out_height) - 128.0;
        }
    }

    /* Separable 8x8 DCT: row transform, then column transform. */
    for (y = 0; y < 8; ++y) {
        for (u = 0; u < 8; ++u) {
            double sum = 0.0;
            for (x = 0; x < 8; ++x) sum += block[y][x] * g_basis[u][x];
            temp[y][u] = sum;
        }
    }
    for (v = 0; v < 8; ++v) {
        for (u = 0; u < 8; ++u) {
            double sum = 0.0;
            int idx = v * 8 + u;
            for (y = 0; y < 8; ++y) sum += temp[y][u] * g_basis[v][y];
            coeff[idx] = (int)lrint(sum / (double)q[idx]);
        }
    }

    dc_diff = coeff[0] - *prev_dc;
    *prev_dc = coeff[0];
    cat = magnitude_category(dc_diff);
    jw_bits(w, dc->code[cat], dc->size[cat]);
    if (cat) jw_bits(w, magnitude_bits(dc_diff, cat), (unsigned)cat);

    for (k = 1; k < 64; ++k) {
        int value = coeff[k_zigzag[k]];
        if (value == 0) {
            ++run;
            continue;
        }
        while (run >= 16) {
            jw_bits(w, ac->code[0xf0], ac->size[0xf0]);
            run -= 16;
        }
        cat = magnitude_category(value);
        if (cat > 10) {
            /* Baseline 8-bit JPEG AC magnitude is expected to fit in 10 bits. */
            if (value > 0) value = 1023;
            else value = -1023;
            cat = 10;
        }
        {
            uint8_t symbol = (uint8_t)((run << 4) | cat);
            jw_bits(w, ac->code[symbol], ac->size[symbol]);
            jw_bits(w, magnitude_bits(value, cat), (unsigned)cat);
        }
        run = 0;
    }
    if (run > 0) jw_bits(w, ac->code[0x00], ac->size[0x00]);
}

int jpeg_gray_encode_scaled(const GrayImage *src,
                            int out_width,
                            int out_height,
                            int quality,
                            uint8_t *out,
                            size_t out_capacity,
                            size_t *out_size) {
    JpegWriter w;
    HuffTable dc, ac;
    uint8_t q[64];
    int bx, by;
    int prev_dc = 0;
    int i;

    if (out_size) *out_size = 0U;
    if (!src || !src->data || src->width <= 0 || src->height <= 0 || src->stride < src->width ||
        out_width <= 0 || out_height <= 0 || out_width > 65535 || out_height > 65535 ||
        !out || out_capacity < 512U || !out_size)
        return -1;

    init_basis();
    scaled_quant_table(quality, q);
    build_huffman(&dc, k_dc_bits, k_dc_vals, sizeof(k_dc_vals));
    build_huffman(&ac, k_ac_bits, k_ac_vals, sizeof(k_ac_vals));

    memset(&w, 0, sizeof(w));
    w.buf = out;
    w.cap = out_capacity;

    /* SOI */
    jw_marker(&w, 0xd8U);

    /* APP0/JFIF */
    jw_marker(&w, 0xe0U);
    jw_u16be(&w, 16U);
    jw_byte(&w, 'J'); jw_byte(&w, 'F'); jw_byte(&w, 'I'); jw_byte(&w, 'F'); jw_byte(&w, 0);
    jw_byte(&w, 1); jw_byte(&w, 1);
    jw_byte(&w, 0);
    jw_u16be(&w, 1); jw_u16be(&w, 1);
    jw_byte(&w, 0); jw_byte(&w, 0);

    /* DQT, table 0, values serialized in zig-zag order. */
    jw_marker(&w, 0xdbU);
    jw_u16be(&w, 67U);
    jw_byte(&w, 0x00U);
    for (i = 0; i < 64; ++i) jw_byte(&w, q[k_zigzag[i]]);

    /* SOF0, one grayscale component. */
    jw_marker(&w, 0xc0U);
    jw_u16be(&w, 11U);
    jw_byte(&w, 8U);
    jw_u16be(&w, (uint16_t)out_height);
    jw_u16be(&w, (uint16_t)out_width);
    jw_byte(&w, 1U);
    jw_byte(&w, 1U);       /* component id */
    jw_byte(&w, 0x11U);    /* H/V sampling */
    jw_byte(&w, 0U);       /* quant table */

    write_dht(&w, 0x00U, k_dc_bits, k_dc_vals, sizeof(k_dc_vals));
    write_dht(&w, 0x10U, k_ac_bits, k_ac_vals, sizeof(k_ac_vals));

    /* SOS */
    jw_marker(&w, 0xdaU);
    jw_u16be(&w, 8U);
    jw_byte(&w, 1U);
    jw_byte(&w, 1U);
    jw_byte(&w, 0x00U);    /* DC0 / AC0 */
    jw_byte(&w, 0U);
    jw_byte(&w, 63U);
    jw_byte(&w, 0U);

    for (by = 0; by < out_height; by += 8)
        for (bx = 0; bx < out_width; bx += 8)
            encode_block(&w, src, bx, by, out_width, out_height, q, &dc, &ac, &prev_dc);

    jw_flush_bits(&w);
    jw_marker(&w, 0xd9U);
    if (w.error) return -2;
    *out_size = w.pos;
    return 0;
}

static int pgm_read_token(FILE *fp, char *buf, size_t cap) {
    int c;
    size_t n = 0U;
    if (!fp || !buf || cap < 2U) return -1;
    do {
        c = fgetc(fp);
        if (c == '#') {
            do { c = fgetc(fp); } while (c != '\n' && c != EOF);
        }
    } while (c != EOF && isspace((unsigned char)c));
    if (c == EOF) return -1;
    while (c != EOF && !isspace((unsigned char)c) && c != '#') {
        if (n + 1U < cap) buf[n++] = (char)c;
        c = fgetc(fp);
    }
    if (c == '#') {
        do { c = fgetc(fp); } while (c != '\n' && c != EOF);
    }
    buf[n] = '\0';
    return n > 0U ? 0 : -1;
}

int jpeg_gray_encode_pgm_scaled(const char *pgm_path,
                                int out_width,
                                int out_height,
                                int quality,
                                uint8_t *out,
                                size_t out_capacity,
                                size_t *out_size) {
    FILE *fp;
    char tok[32];
    int width, height, maxval;
    uint8_t *pixels = NULL;
    size_t bytes;
    GrayImage gray;
    int ret = -1;

    if (!pgm_path) return -1;
    fp = fopen(pgm_path, "rb");
    if (!fp) return -2;
    if (pgm_read_token(fp, tok, sizeof(tok)) != 0 || strcmp(tok, "P5") != 0) goto done;
    if (pgm_read_token(fp, tok, sizeof(tok)) != 0) goto done;
    width = atoi(tok);
    if (pgm_read_token(fp, tok, sizeof(tok)) != 0) goto done;
    height = atoi(tok);
    if (pgm_read_token(fp, tok, sizeof(tok)) != 0) goto done;
    maxval = atoi(tok);
    if (width <= 0 || height <= 0 || maxval != 255) goto done;
    bytes = (size_t)width * (size_t)height;
    pixels = (uint8_t *)malloc(bytes);
    if (!pixels) { ret = -3; goto done; }
    if (fread(pixels, 1, bytes, fp) != bytes) { ret = -4; goto done; }
    gray.data = pixels;
    gray.width = width;
    gray.height = height;
    gray.stride = width;
    ret = jpeg_gray_encode_scaled(&gray, out_width, out_height, quality,
                                  out, out_capacity, out_size);
done:
    free(pixels);
    fclose(fp);
    return ret;
}
