/** @file
    Capture file format implementation.

    Copyright (C) 2024 rtl_433_tx contributors

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "capture.h"
#include "fatal.h"
#include "jsmn.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static char *dup_str(char const *s)
{
    if (!s)
        return NULL;
    size_t n = strlen(s);
    char *r = malloc(n + 1);
    if (!r)
        FATAL_MALLOC("dup_str");
    memcpy(r, s, n + 1);
    return r;
}

static char *dup_strn(char const *s, size_t n)
{
    char *r = malloc(n + 1);
    if (!r)
        FATAL_MALLOC("dup_strn");
    memcpy(r, s, n);
    r[n] = '\0';
    return r;
}

static char const *modulation_name(int mod)
{
    switch (mod) {
    case OOK_PULSE_PCM:                 return "OOK_PULSE_PCM";
    case OOK_PULSE_PPM:                 return "OOK_PULSE_PPM";
    case OOK_PULSE_PWM:                 return "OOK_PULSE_PWM";
    case OOK_PULSE_MANCHESTER_ZEROBIT:  return "OOK_PULSE_MANCHESTER_ZEROBIT";
    case OOK_PULSE_PIWM_RAW:            return "OOK_PULSE_PIWM_RAW";
    case OOK_PULSE_PIWM_DC:             return "OOK_PULSE_PIWM_DC";
    case OOK_PULSE_DMC:                 return "OOK_PULSE_DMC";
    case OOK_PULSE_PWM_OSV1:            return "OOK_PULSE_PWM_OSV1";
    case OOK_PULSE_NRZS:               return "OOK_PULSE_NRZS";
    case OOK_PULSE_RZI:                 return "OOK_PULSE_RZI";
    case FSK_PULSE_PCM:                 return "FSK_PULSE_PCM";
    case FSK_PULSE_PWM:                 return "FSK_PULSE_PWM";
    case FSK_PULSE_MANCHESTER_ZEROBIT:  return "FSK_PULSE_MANCHESTER_ZEROBIT";
    case LORA:                          return "LORA";
    default:                            return "UNKNOWN";
    }
}

void capture_clear(capture_t *cap)
{
    if (!cap)
        return;
    free(cap->time);
    free(cap->model);
    free(cap->modulation);
    for (unsigned i = 0; i < cap->num_fields; ++i) {
        free(cap->fields[i].key);
        free(cap->fields[i].val);
    }
    free(cap->fields);
    memset(cap, 0, sizeof(*cap));
}

char const *capture_get_field(capture_t const *cap, char const *key)
{
    for (unsigned i = 0; i < cap->num_fields; ++i) {
        if (strcmp(cap->fields[i].key, key) == 0)
            return cap->fields[i].val;
    }
    return NULL;
}

void capture_set_field(capture_t *cap, char const *key, char const *val)
{
    for (unsigned i = 0; i < cap->num_fields; ++i) {
        if (strcmp(cap->fields[i].key, key) == 0) {
            free(cap->fields[i].val);
            cap->fields[i].val = dup_str(val);
            return;
        }
    }
    cap->fields = realloc(cap->fields, (cap->num_fields + 1) * sizeof(capture_field_t));
    if (!cap->fields)
        FATAL_MALLOC("capture_set_field");
    cap->fields[cap->num_fields].key       = dup_str(key);
    cap->fields[cap->num_fields].val       = dup_str(val);
    cap->fields[cap->num_fields].is_string = 1;
    cap->num_fields++;
}

static void field_add(capture_t *cap, char const *key, char const *val, int is_string)
{
    cap->fields = realloc(cap->fields, (cap->num_fields + 1) * sizeof(capture_field_t));
    if (!cap->fields)
        FATAL_MALLOC("field_add");
    cap->fields[cap->num_fields].key       = dup_str(key);
    cap->fields[cap->num_fields].val       = dup_str(val);
    cap->fields[cap->num_fields].is_string = is_string;
    cap->num_fields++;
}

void capture_from_decode(capture_t *cap, r_device const *dev,
        pulse_data_t const *pulses, bitbuffer_t const *bits, data_t const *data,
        uint32_t freq_hz)
{
    memset(cap, 0, sizeof(*cap));
    cap->protocol       = dev ? (int)dev->protocol_num : 0;
    cap->model          = dup_str(dev ? dev->name : "");
    cap->modulation_num = dev ? (int)dev->modulation : 0;
    cap->modulation     = dup_str(modulation_name(cap->modulation_num));
    cap->freq_hz        = freq_hz;
    cap->sample_rate    = pulses ? pulses->sample_rate : 0;
    cap->short_us       = dev ? dev->short_width : 0;
    cap->long_us        = dev ? dev->long_width : 0;
    cap->reset_us       = dev ? dev->reset_limit : 0;
    cap->gap_us         = dev ? dev->gap_limit : 0;
    cap->sync_us        = dev ? dev->sync_width : 0;
    cap->tolerance_us   = dev ? dev->tolerance : 0;
    cap->is_fsk         = (cap->modulation_num >= FSK_DEMOD_MIN_VAL && cap->modulation_num < LORA_DEMOD_MIN_VAL);
    if (pulses) {
        cap->pulses     = *pulses;
        cap->fsk_f1_hz  = pulses->freq1_hz;
        cap->fsk_f2_hz  = pulses->freq2_hz;
    }
    if (bits)
        cap->bits = *bits;

    capture_fields_from_data(cap, data);
}

void capture_fields_from_data(capture_t *cap, data_t const *data)
{
    for (data_t const *d = data; d; d = d->next) {
        char buf[256];
        int is_string = 0;
        switch (d->type) {
        case DATA_INT:
            snprintf(buf, sizeof(buf), "%d", d->value.v_int);
            break;
        case DATA_DOUBLE:
            snprintf(buf, sizeof(buf), "%.4f", d->value.v_dbl);
            break;
        case DATA_STRING:
            snprintf(buf, sizeof(buf), "%s", (char *)d->value.v_ptr);
            is_string = 1;
            break;
        default:
            continue; // skip arrays/nested for now
        }
        field_add(cap, d->key, buf, is_string);
    }
}

static void json_escape(FILE *f, char const *s)
{
    for (; s && *s; ++s) {
        switch (*s) {
        case '"':  fputs("\\\"", f); break;
        case '\\': fputs("\\\\", f); break;
        case '\n': fputs("\\n", f); break;
        case '\r': fputs("\\r", f); break;
        case '\t': fputs("\\t", f); break;
        default:   fputc(*s, f); break;
        }
    }
}

int capture_write_json(FILE *f, capture_t const *cap)
{
    fputc('{', f);
    fprintf(f, "\"time\":\"");
    json_escape(f, cap->time ? cap->time : "");
    fprintf(f, "\",\"protocol\":%d,\"model\":\"", cap->protocol);
    json_escape(f, cap->model ? cap->model : "");
    fprintf(f, "\",\"modulation\":\"%s\",\"modulation_num\":%d",
            cap->modulation ? cap->modulation : "UNKNOWN", cap->modulation_num);
    fprintf(f, ",\"freq_hz\":%u,\"sample_rate\":%u", cap->freq_hz, cap->sample_rate);
    fprintf(f, ",\"short_us\":%.2f,\"long_us\":%.2f,\"reset_us\":%.2f,\"gap_us\":%.2f,\"sync_us\":%.2f,\"tolerance_us\":%.2f",
            cap->short_us, cap->long_us, cap->reset_us, cap->gap_us, cap->sync_us, cap->tolerance_us);
    fprintf(f, ",\"is_fsk\":%d,\"fsk_f1_hz\":%.1f,\"fsk_f2_hz\":%.1f",
            cap->is_fsk, cap->fsk_f1_hz, cap->fsk_f2_hz);

    // raw bits as a parseable code string, rows separated by '/'
    fprintf(f, ",\"rows\":\"");
    for (unsigned r = 0; r < (unsigned)cap->bits.num_rows; ++r) {
        char hex[BITBUF_COLS * 2 + 16];
        int off = snprintf(hex, sizeof(hex), "{%u}", cap->bits.bits_per_row[r]);
        bitrow_snprint(cap->bits.bb[r], cap->bits.bits_per_row[r], hex + off, sizeof(hex) - off);
        if (r)
            fputc('/', f);
        fputs(hex, f);
    }
    fputc('"', f);

    // raw pulses in microseconds (alternating pulse,gap)
    double to_us = cap->sample_rate ? 1.0e6 / cap->sample_rate : 0.0;
    fprintf(f, ",\"pulses_us\":[");
    for (unsigned n = 0; n < cap->pulses.num_pulses; ++n) {
        if (n)
            fputc(',', f);
        fprintf(f, "%.2f,%.2f", cap->pulses.pulse[n] * to_us, cap->pulses.gap[n] * to_us);
    }
    fputc(']', f);

    // decoded fields
    fprintf(f, ",\"decoded\":{");
    for (unsigned i = 0; i < cap->num_fields; ++i) {
        if (i)
            fputc(',', f);
        fputc('"', f);
        json_escape(f, cap->fields[i].key);
        fputs("\":", f);
        if (cap->fields[i].is_string) {
            fputc('"', f);
            json_escape(f, cap->fields[i].val);
            fputc('"', f);
        }
        else {
            fputs(cap->fields[i].val, f);
        }
    }
    fputc('}', f);

    fputs("}\n", f);
    return 0;
}

/* ---- JSON parsing (jsmn) ---- */

static int tok_eq(char const *js, jsmntok_t const *t, char const *s)
{
    size_t len = (size_t)(t->end - t->start);
    return t->type == JSMN_STRING && strlen(s) == len && strncmp(js + t->start, s, len) == 0;
}

static double tok_dbl(char const *js, jsmntok_t const *t)
{
    return strtod(js + t->start, NULL);
}

static long tok_long(char const *js, jsmntok_t const *t)
{
    return strtol(js + t->start, NULL, 0);
}

// count tokens belonging to the value at index i (recursive)
static int tok_span(jsmntok_t const *toks, int i)
{
    int n = 1;
    if (toks[i].type == JSMN_OBJECT) {
        int kids = toks[i].size;
        int j    = i + 1;
        for (int k = 0; k < kids; ++k) {
            j += 1;                 // key
            j += tok_span(toks, j); // value
        }
        n = j - i;
    }
    else if (toks[i].type == JSMN_ARRAY) {
        int kids = toks[i].size;
        int j    = i + 1;
        for (int k = 0; k < kids; ++k)
            j += tok_span(toks, j);
        n = j - i;
    }
    return n;
}

int capture_parse_json(capture_t *cap, char const *json, size_t len)
{
    memset(cap, 0, sizeof(*cap));

    jsmn_parser p;
    jsmn_init(&p);
    int ntok = jsmn_parse(&p, json, len, NULL, 0);
    if (ntok < 1)
        return -1;
    jsmntok_t *toks = calloc(ntok, sizeof(jsmntok_t));
    if (!toks)
        FATAL_MALLOC("capture_parse_json");
    jsmn_init(&p);
    int r = jsmn_parse(&p, json, len, toks, ntok);
    if (r < 1 || toks[0].type != JSMN_OBJECT) {
        free(toks);
        return -1;
    }

    char *rows_code = NULL;
    double pulses_us[2 * PD_MAX_PULSES];
    int pulses_n = 0;

    int i = 1;
    int kids = toks[0].size;
    for (int k = 0; k < kids; ++k) {
        jsmntok_t *key = &toks[i];
        int vi = i + 1;
        jsmntok_t *val = &toks[vi];

        if (tok_eq(json, key, "time"))
            cap->time = dup_strn(json + val->start, val->end - val->start);
        else if (tok_eq(json, key, "protocol"))
            cap->protocol = (int)tok_long(json, val);
        else if (tok_eq(json, key, "model"))
            cap->model = dup_strn(json + val->start, val->end - val->start);
        else if (tok_eq(json, key, "modulation"))
            cap->modulation = dup_strn(json + val->start, val->end - val->start);
        else if (tok_eq(json, key, "modulation_num"))
            cap->modulation_num = (int)tok_long(json, val);
        else if (tok_eq(json, key, "freq_hz"))
            cap->freq_hz = (uint32_t)tok_long(json, val);
        else if (tok_eq(json, key, "sample_rate"))
            cap->sample_rate = (uint32_t)tok_long(json, val);
        else if (tok_eq(json, key, "short_us"))
            cap->short_us = tok_dbl(json, val);
        else if (tok_eq(json, key, "long_us"))
            cap->long_us = tok_dbl(json, val);
        else if (tok_eq(json, key, "reset_us"))
            cap->reset_us = tok_dbl(json, val);
        else if (tok_eq(json, key, "gap_us"))
            cap->gap_us = tok_dbl(json, val);
        else if (tok_eq(json, key, "sync_us"))
            cap->sync_us = tok_dbl(json, val);
        else if (tok_eq(json, key, "tolerance_us"))
            cap->tolerance_us = tok_dbl(json, val);
        else if (tok_eq(json, key, "is_fsk"))
            cap->is_fsk = (int)tok_long(json, val);
        else if (tok_eq(json, key, "fsk_f1_hz"))
            cap->fsk_f1_hz = tok_dbl(json, val);
        else if (tok_eq(json, key, "fsk_f2_hz"))
            cap->fsk_f2_hz = tok_dbl(json, val);
        else if (tok_eq(json, key, "rows"))
            rows_code = dup_strn(json + val->start, val->end - val->start);
        else if (tok_eq(json, key, "pulses_us") && val->type == JSMN_ARRAY) {
            int cnt = val->size;
            for (int e = 0; e < cnt && pulses_n < 2 * PD_MAX_PULSES; ++e)
                pulses_us[pulses_n++] = tok_dbl(json, &toks[vi + 1 + e]);
        }
        else if (tok_eq(json, key, "decoded") && val->type == JSMN_OBJECT) {
            int fcount = val->size;
            int fi     = vi + 1;
            for (int e = 0; e < fcount; ++e) {
                jsmntok_t *fk = &toks[fi];
                jsmntok_t *fv = &toks[fi + 1];
                char *fkey = dup_strn(json + fk->start, fk->end - fk->start);
                char *fval = dup_strn(json + fv->start, fv->end - fv->start);
                field_add(cap, fkey, fval, fv->type == JSMN_STRING);
                free(fkey);
                free(fval);
                fi += 1 + tok_span(toks, fi + 1);
            }
        }

        i = vi + tok_span(toks, vi);
    }

    // reconstruct bits
    if (rows_code) {
        bitbuffer_parse(&cap->bits, rows_code);
        free(rows_code);
    }

    // reconstruct pulses (us -> samples)
    pulse_data_clear(&cap->pulses);
    cap->pulses.sample_rate = cap->sample_rate;
    double to_samp = cap->sample_rate / 1.0e6;
    for (int n = 0; n + 1 < pulses_n && cap->pulses.num_pulses < PD_MAX_PULSES; n += 2) {
        unsigned idx = cap->pulses.num_pulses;
        cap->pulses.pulse[idx] = (int)(pulses_us[n] * to_samp + 0.5);
        cap->pulses.gap[idx]   = (int)(pulses_us[n + 1] * to_samp + 0.5);
        cap->pulses.num_pulses++;
    }
    if (cap->is_fsk) {
        cap->pulses.fsk_f1_est = 1;
        cap->pulses.fsk_f2_est = 1;
    }

    free(toks);
    return 0;
}

int capture_read_next(FILE *f, capture_t *cap)
{
    // read one line (may be long); grow buffer as needed
    size_t cap_sz = 65536;
    char *line = malloc(cap_sz);
    if (!line)
        FATAL_MALLOC("capture_read_next");
    size_t len = 0;
    int c;
    int got = 0;
    while ((c = fgetc(f)) != EOF) {
        got = 1;
        if (c == '\n')
            break;
        if (len + 1 >= cap_sz) {
            cap_sz *= 2;
            line = realloc(line, cap_sz);
            if (!line)
                FATAL_MALLOC("capture_read_next");
        }
        line[len++] = (char)c;
    }
    line[len] = '\0';

    if (!got && len == 0) {
        free(line);
        return 1; // EOF
    }
    // skip blank lines
    size_t s = 0;
    while (s < len && (line[s] == ' ' || line[s] == '\t' || line[s] == '\r'))
        s++;
    if (s >= len) {
        free(line);
        return capture_read_next(f, cap);
    }

    int r = capture_parse_json(cap, line + s, len - s);
    free(line);
    return r;
}
