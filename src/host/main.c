#include <SDL2/SDL.h>

#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gba_core.h"
#include "gba_keys.h"

#define GBA_FPS 59.7275
#define MAX_HEADLESS_HOLDS 128
#define MAX_HEADLESS_DUMPS 32

typedef struct {
  uint16_t keymask;
  int start_frame;
  int len;
} HeadlessHold;

typedef struct {
  int frame_no;
  const char *path;
  bool png;
} HeadlessDump;

typedef struct {
  const char *bios_path;
  const char *rom_path;
  const char *save_path;
  int scale;
  bool mute;
  bool trace;
  bool use_bios;
  int headless_frames;
  HeadlessHold headless_holds[MAX_HEADLESS_HOLDS];
  int headless_hold_count;
  HeadlessDump headless_dumps[MAX_HEADLESS_DUMPS];
  int headless_dump_count;
  bool headless_debug;
} CliOptions;

static const char *gba_init_error_reason(int rc) {
  switch (rc) {
    case -1:
      return "invalid init arguments";
    case -2:
      return "allocation failure";
    case -3:
      return "ROM load failure";
    case -4:
      return "BIOS load failure";
    case -5:
      return "invalid BIOS size (expected 16384 bytes)";
    default:
      return "unknown init failure";
  }
}

static void print_usage(const char *prog) {
  fprintf(stderr,
          "Usage: %s --rom <path> [options]\n"
          "Options:\n"
          "  --bios <path>            Path to gba_bios.bin\n"
          "  --save <path>            Save file path (default: ROM basename .sav)\n"
          "  --scale <N>              Integer window scale (default: 3)\n"
          "  --mute                   Disable audio output\n"
          "  --trace                  Enable core trace mode\n"
          "  --no-bios                Skip BIOS load even if --bios provided\n"
          "  --headless-frames <N>    Run N frames without SDL window\n"
          "  --headless-press-start <frame> <len>\n"
          "                           Hold START for len frames (headless/window)\n"
          "  --headless-hold <key> <frame> <len>\n"
          "                           Hold key for len frames (headless/window)\n"
          "  --headless-dump-bmp <frame> <path>\n"
          "                           Dump one headless frame as 32-bit BMP\n"
          "  --headless-dump-png <frame> <path>\n"
          "                           Dump one headless frame as RGBA PNG\n"
          "  --headless-debug         Print core debug state after headless run\n"
          "  --help                   Print this message\n",
          prog);
}

static bool parse_i32(const char *s, int *out) {
  char *end = NULL;
  long value = 0;

  if (!s || !out) {
    return false;
  }

  errno = 0;
  value = strtol(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0') {
    return false;
  }

  if (value < INT32_MIN || value > INT32_MAX) {
    return false;
  }

  *out = (int)value;
  return true;
}

static bool streq_nocase(const char *a, const char *b) {
  unsigned char ca = 0;
  unsigned char cb = 0;

  if (!a || !b) {
    return false;
  }

  while (*a && *b) {
    ca = (unsigned char)*a++;
    cb = (unsigned char)*b++;
    if (tolower(ca) != tolower(cb)) {
      return false;
    }
  }

  return (*a == '\0') && (*b == '\0');
}

static bool parse_headless_key(const char *s, uint16_t *out_mask) {
  if (!s || !out_mask) {
    return false;
  }

  if (streq_nocase(s, "A")) {
    *out_mask = GBA_KEY_A;
    return true;
  }
  if (streq_nocase(s, "B")) {
    *out_mask = GBA_KEY_B;
    return true;
  }
  if (streq_nocase(s, "SELECT")) {
    *out_mask = GBA_KEY_SELECT;
    return true;
  }
  if (streq_nocase(s, "START")) {
    *out_mask = GBA_KEY_START;
    return true;
  }
  if (streq_nocase(s, "RIGHT")) {
    *out_mask = GBA_KEY_RIGHT;
    return true;
  }
  if (streq_nocase(s, "LEFT")) {
    *out_mask = GBA_KEY_LEFT;
    return true;
  }
  if (streq_nocase(s, "UP")) {
    *out_mask = GBA_KEY_UP;
    return true;
  }
  if (streq_nocase(s, "DOWN")) {
    *out_mask = GBA_KEY_DOWN;
    return true;
  }
  if (streq_nocase(s, "R")) {
    *out_mask = GBA_KEY_R;
    return true;
  }
  if (streq_nocase(s, "L")) {
    *out_mask = GBA_KEY_L;
    return true;
  }

  return false;
}

static bool add_headless_hold(CliOptions *opts, uint16_t keymask, int start_frame, int len) {
  int idx = 0;

  if (!opts || keymask == 0 || start_frame <= 0 || len <= 0) {
    return false;
  }
  if (opts->headless_hold_count >= MAX_HEADLESS_HOLDS) {
    return false;
  }

  idx = opts->headless_hold_count++;
  opts->headless_holds[idx].keymask = keymask;
  opts->headless_holds[idx].start_frame = start_frame;
  opts->headless_holds[idx].len = len;
  return true;
}

static bool add_headless_dump(CliOptions *opts, int frame_no, const char *path, bool png) {
  int idx = 0;

  if (!opts || frame_no <= 0 || !path) {
    return false;
  }
  if (opts->headless_dump_count >= MAX_HEADLESS_DUMPS) {
    return false;
  }

  idx = opts->headless_dump_count++;
  opts->headless_dumps[idx].frame_no = frame_no;
  opts->headless_dumps[idx].path = path;
  opts->headless_dumps[idx].png = png;
  return true;
}

static bool write_u16_le(FILE *f, uint16_t v) {
  uint8_t b[2];
  b[0] = (uint8_t)(v & 0xFFu);
  b[1] = (uint8_t)((v >> 8) & 0xFFu);
  return fwrite(b, 1, sizeof(b), f) == sizeof(b);
}

static bool write_u32_le(FILE *f, uint32_t v) {
  uint8_t b[4];
  b[0] = (uint8_t)(v & 0xFFu);
  b[1] = (uint8_t)((v >> 8) & 0xFFu);
  b[2] = (uint8_t)((v >> 16) & 0xFFu);
  b[3] = (uint8_t)((v >> 24) & 0xFFu);
  return fwrite(b, 1, sizeof(b), f) == sizeof(b);
}

static bool write_u32_be(FILE *f, uint32_t v) {
  uint8_t b[4];
  b[0] = (uint8_t)((v >> 24) & 0xFFu);
  b[1] = (uint8_t)((v >> 16) & 0xFFu);
  b[2] = (uint8_t)((v >> 8) & 0xFFu);
  b[3] = (uint8_t)(v & 0xFFu);
  return fwrite(b, 1, sizeof(b), f) == sizeof(b);
}

static uint32_t crc32_step(uint32_t crc, const uint8_t *buf, size_t len) {
  size_t i = 0;
  int bit = 0;

  for (i = 0; i < len; ++i) {
    crc ^= (uint32_t)buf[i];
    for (bit = 0; bit < 8; ++bit) {
      if (crc & 1u) {
        crc = (crc >> 1) ^ 0xEDB88320u;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static uint32_t adler32_calc(const uint8_t *data, size_t len) {
  uint32_t s1 = 1;
  uint32_t s2 = 0;
  size_t i = 0;

  for (i = 0; i < len; ++i) {
    s1 += data[i];
    if (s1 >= 65521u) {
      s1 -= 65521u;
    }
    s2 += s1;
    s2 %= 65521u;
  }

  return (s2 << 16) | s1;
}

static bool write_png_chunk(FILE *f, const char type[4], const uint8_t *data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFu;

  if (!write_u32_be(f, len)) {
    return false;
  }
  if (fwrite(type, 1, 4, f) != 4) {
    return false;
  }
  if (len && fwrite(data, 1, len, f) != len) {
    return false;
  }

  crc = crc32_step(crc, (const uint8_t *)type, 4);
  if (len) {
    crc = crc32_step(crc, data, len);
  }
  crc ^= 0xFFFFFFFFu;

  return write_u32_be(f, crc);
}

static bool write_png32(const char *path, const GbaFrame *frame) {
  static const uint8_t sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
  uint8_t ihdr[13];
  uint8_t *raw = NULL;
  uint8_t *idat = NULL;
  size_t raw_len = 0;
  size_t idat_len = 0;
  size_t max_blocks = 0;
  size_t block_count = 0;
  size_t in_off = 0;
  size_t out_off = 0;
  FILE *f = NULL;
  uint32_t adler = 0;
  uint32_t x = 0;
  uint32_t y = 0;

  if (!path || !frame || !frame->pixels || frame->width == 0 || frame->height == 0) {
    return false;
  }
  if (frame->width > UINT32_MAX / 4u) {
    return false;
  }
  if (frame->height > (SIZE_MAX - 1) / (size_t)(frame->width * 4u + 1u)) {
    return false;
  }

  raw_len = (size_t)frame->height * (size_t)(frame->width * 4u + 1u);
  max_blocks = (raw_len + 65534u) / 65535u;
  if (raw_len > SIZE_MAX - (size_t)(2u + max_blocks * 5u + 4u)) {
    return false;
  }
  idat_len = 2u + max_blocks * 5u + raw_len + 4u;

  raw = (uint8_t *)malloc(raw_len);
  if (!raw) {
    return false;
  }

  out_off = 0;
  for (y = 0; y < frame->height; ++y) {
    raw[out_off++] = 0;
    for (x = 0; x < frame->width; ++x) {
      const uint32_t p = frame->pixels[y * frame->width + x];
      raw[out_off++] = (uint8_t)((p >> 16) & 0xFFu);
      raw[out_off++] = (uint8_t)((p >> 8) & 0xFFu);
      raw[out_off++] = (uint8_t)(p & 0xFFu);
      raw[out_off++] = (uint8_t)((p >> 24) & 0xFFu);
    }
  }
  if (out_off != raw_len) {
    free(raw);
    return false;
  }

  idat = (uint8_t *)malloc(idat_len);
  if (!idat) {
    free(raw);
    return false;
  }

  out_off = 0;
  idat[out_off++] = 0x78;
  idat[out_off++] = 0x01;

  in_off = 0;
  block_count = 0;
  while (in_off < raw_len) {
    const size_t remain = raw_len - in_off;
    const uint16_t len = (uint16_t)(remain > 65535u ? 65535u : remain);
    const uint16_t nlen = (uint16_t)(~len);
    const bool final_block = (in_off + len == raw_len);
    idat[out_off++] = final_block ? 0x01u : 0x00u;
    idat[out_off++] = (uint8_t)(len & 0xFFu);
    idat[out_off++] = (uint8_t)((len >> 8) & 0xFFu);
    idat[out_off++] = (uint8_t)(nlen & 0xFFu);
    idat[out_off++] = (uint8_t)((nlen >> 8) & 0xFFu);
    memcpy(idat + out_off, raw + in_off, len);
    out_off += len;
    in_off += len;
    block_count++;
  }

  adler = adler32_calc(raw, raw_len);
  idat[out_off++] = (uint8_t)((adler >> 24) & 0xFFu);
  idat[out_off++] = (uint8_t)((adler >> 16) & 0xFFu);
  idat[out_off++] = (uint8_t)((adler >> 8) & 0xFFu);
  idat[out_off++] = (uint8_t)(adler & 0xFFu);

  if (out_off != idat_len) {
    free(idat);
    free(raw);
    return false;
  }

  f = fopen(path, "wb");
  if (!f) {
    free(idat);
    free(raw);
    return false;
  }

  if (fwrite(sig, 1, sizeof(sig), f) != sizeof(sig)) {
    fclose(f);
    free(idat);
    free(raw);
    return false;
  }

  ihdr[0] = (uint8_t)((frame->width >> 24) & 0xFFu);
  ihdr[1] = (uint8_t)((frame->width >> 16) & 0xFFu);
  ihdr[2] = (uint8_t)((frame->width >> 8) & 0xFFu);
  ihdr[3] = (uint8_t)(frame->width & 0xFFu);
  ihdr[4] = (uint8_t)((frame->height >> 24) & 0xFFu);
  ihdr[5] = (uint8_t)((frame->height >> 16) & 0xFFu);
  ihdr[6] = (uint8_t)((frame->height >> 8) & 0xFFu);
  ihdr[7] = (uint8_t)(frame->height & 0xFFu);
  ihdr[8] = 8;
  ihdr[9] = 6;
  ihdr[10] = 0;
  ihdr[11] = 0;
  ihdr[12] = 0;

  if (!write_png_chunk(f, "IHDR", ihdr, sizeof(ihdr)) ||
      !write_png_chunk(f, "IDAT", idat, (uint32_t)idat_len) ||
      !write_png_chunk(f, "IEND", NULL, 0)) {
    fclose(f);
    free(idat);
    free(raw);
    return false;
  }

  fclose(f);
  free(idat);
  free(raw);
  return true;
}

static bool write_bmp32(const char *path, const GbaFrame *frame) {
  FILE *f = NULL;
  uint32_t row_bytes = 0;
  uint32_t pixel_bytes = 0;
  uint32_t file_bytes = 0;
  int y = 0;

  if (!path || !frame || !frame->pixels || frame->width == 0 || frame->height == 0) {
    return false;
  }

  row_bytes = frame->width * 4u;
  if (frame->height > UINT32_MAX / row_bytes) {
    return false;
  }
  pixel_bytes = row_bytes * frame->height;
  if (pixel_bytes > UINT32_MAX - 54u) {
    return false;
  }
  file_bytes = 54u + pixel_bytes;

  f = fopen(path, "wb");
  if (!f) {
    return false;
  }

  if (!write_u16_le(f, 0x4D42u) ||
      !write_u32_le(f, file_bytes) ||
      !write_u16_le(f, 0u) ||
      !write_u16_le(f, 0u) ||
      !write_u32_le(f, 54u) ||
      !write_u32_le(f, 40u) ||
      !write_u32_le(f, frame->width) ||
      !write_u32_le(f, frame->height) ||
      !write_u16_le(f, 1u) ||
      !write_u16_le(f, 32u) ||
      !write_u32_le(f, 0u) ||
      !write_u32_le(f, pixel_bytes) ||
      !write_u32_le(f, 0u) ||
      !write_u32_le(f, 0u) ||
      !write_u32_le(f, 0u) ||
      !write_u32_le(f, 0u)) {
    fclose(f);
    return false;
  }

  for (y = (int)frame->height - 1; y >= 0; --y) {
    const uint8_t *row = (const uint8_t *)&frame->pixels[(uint32_t)y * frame->width];
    if (fwrite(row, 1, row_bytes, f) != row_bytes) {
      fclose(f);
      return false;
    }
  }

  fclose(f);
  return true;
}

static char *derive_default_save_path(const char *rom_path) {
  const char *dot = NULL;
  size_t base_len = 0;
  char *out = NULL;

  if (!rom_path) {
    return NULL;
  }

  dot = strrchr(rom_path, '.');
  if (!dot || dot < strrchr(rom_path, '/')) {
    base_len = strlen(rom_path);
  } else {
    base_len = (size_t)(dot - rom_path);
  }

  out = (char *)malloc(base_len + 5);
  if (!out) {
    return NULL;
  }

  memcpy(out, rom_path, base_len);
  memcpy(out + base_len, ".sav", 5);
  return out;
}

static bool parse_args(int argc, char **argv, CliOptions *out, char **owned_save_path) {
  int i = 0;

  if (!out || !owned_save_path) {
    return false;
  }

  memset(out, 0, sizeof(*out));
  *owned_save_path = NULL;

  out->scale = 3;
  out->use_bios = true;

  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--help") == 0) {
      print_usage(argv[0]);
      return false;
    }

    if (strcmp(argv[i], "--rom") == 0 && (i + 1) < argc) {
      out->rom_path = argv[++i];
      continue;
    }

    if (strcmp(argv[i], "--bios") == 0 && (i + 1) < argc) {
      out->bios_path = argv[++i];
      continue;
    }

    if (strcmp(argv[i], "--save") == 0 && (i + 1) < argc) {
      out->save_path = argv[++i];
      continue;
    }

    if (strcmp(argv[i], "--scale") == 0 && (i + 1) < argc) {
      int parsed = 0;
      if (!parse_i32(argv[++i], &parsed) || parsed <= 0 || parsed > 10) {
        fprintf(stderr, "Invalid --scale value\n");
        return false;
      }
      out->scale = parsed;
      continue;
    }

    if (strcmp(argv[i], "--headless-frames") == 0 && (i + 1) < argc) {
      int parsed = 0;
      if (!parse_i32(argv[++i], &parsed) || parsed <= 0) {
        fprintf(stderr, "Invalid --headless-frames value\n");
        return false;
      }
      out->headless_frames = parsed;
      continue;
    }

    if (strcmp(argv[i], "--headless-press-start") == 0 && (i + 2) < argc) {
      int frame_start = 0;
      int len = 0;
      if (!parse_i32(argv[++i], &frame_start) || frame_start <= 0) {
        fprintf(stderr, "Invalid --headless-press-start frame value\n");
        return false;
      }
      if (!parse_i32(argv[++i], &len) || len <= 0) {
        fprintf(stderr, "Invalid --headless-press-start len value\n");
        return false;
      }
      if (!add_headless_hold(out, GBA_KEY_START, frame_start, len)) {
        fprintf(stderr, "Failed to add --headless-press-start event\n");
        return false;
      }
      continue;
    }

    if (strcmp(argv[i], "--headless-hold") == 0 && (i + 3) < argc) {
      uint16_t keymask = 0;
      int frame_start = 0;
      int len = 0;
      if (!parse_headless_key(argv[++i], &keymask)) {
        fprintf(stderr, "Invalid --headless-hold key (use A/B/SELECT/START/RIGHT/LEFT/UP/DOWN/R/L)\n");
        return false;
      }
      if (!parse_i32(argv[++i], &frame_start) || frame_start <= 0) {
        fprintf(stderr, "Invalid --headless-hold frame value\n");
        return false;
      }
      if (!parse_i32(argv[++i], &len) || len <= 0) {
        fprintf(stderr, "Invalid --headless-hold len value\n");
        return false;
      }
      if (!add_headless_hold(out, keymask, frame_start, len)) {
        fprintf(stderr, "Too many --headless-hold events\n");
        return false;
      }
      continue;
    }

    if (strcmp(argv[i], "--headless-dump-bmp") == 0 && (i + 2) < argc) {
      int frame_no = 0;
      const char *path = NULL;
      if (!parse_i32(argv[++i], &frame_no) || frame_no <= 0) {
        fprintf(stderr, "Invalid --headless-dump-bmp frame value\n");
        return false;
      }
      path = argv[++i];
      if (!add_headless_dump(out, frame_no, path, false)) {
        fprintf(stderr, "Too many --headless-dump-* events\n");
        return false;
      }
      continue;
    }

    if (strcmp(argv[i], "--headless-dump-png") == 0 && (i + 2) < argc) {
      int frame_no = 0;
      const char *path = NULL;
      if (!parse_i32(argv[++i], &frame_no) || frame_no <= 0) {
        fprintf(stderr, "Invalid --headless-dump-png frame value\n");
        return false;
      }
      path = argv[++i];
      if (!add_headless_dump(out, frame_no, path, true)) {
        fprintf(stderr, "Too many --headless-dump-* events\n");
        return false;
      }
      continue;
    }

    if (strcmp(argv[i], "--headless-debug") == 0) {
      out->headless_debug = true;
      continue;
    }

    if (strcmp(argv[i], "--mute") == 0) {
      out->mute = true;
      continue;
    }

    if (strcmp(argv[i], "--trace") == 0) {
      out->trace = true;
      continue;
    }

    if (strcmp(argv[i], "--no-bios") == 0) {
      out->use_bios = false;
      continue;
    }

    fprintf(stderr, "Unknown argument: %s\n", argv[i]);
    return false;
  }

  if (!out->rom_path) {
    fprintf(stderr, "Missing required --rom\n");
    return false;
  }

  if (!out->save_path) {
    *owned_save_path = derive_default_save_path(out->rom_path);
    if (!*owned_save_path) {
      fprintf(stderr, "Failed to derive default save path\n");
      return false;
    }
    out->save_path = *owned_save_path;
  }

  return true;
}

static uint16_t gather_keymask(void) {
  const Uint8 *k = SDL_GetKeyboardState(NULL);
  uint16_t mask = 0;

  if (k[SDL_SCANCODE_Z]) {
    mask |= GBA_KEY_A;
  }
  if (k[SDL_SCANCODE_X]) {
    mask |= GBA_KEY_B;
  }
  if (k[SDL_SCANCODE_BACKSPACE]) {
    mask |= GBA_KEY_SELECT;
  }
  if (k[SDL_SCANCODE_RETURN]) {
    mask |= GBA_KEY_START;
  }
  if (k[SDL_SCANCODE_RIGHT]) {
    mask |= GBA_KEY_RIGHT;
  }
  if (k[SDL_SCANCODE_LEFT]) {
    mask |= GBA_KEY_LEFT;
  }
  if (k[SDL_SCANCODE_UP]) {
    mask |= GBA_KEY_UP;
  }
  if (k[SDL_SCANCODE_DOWN]) {
    mask |= GBA_KEY_DOWN;
  }
  if (k[SDL_SCANCODE_S]) {
    mask |= GBA_KEY_R;
  }
  if (k[SDL_SCANCODE_A]) {
    mask |= GBA_KEY_L;
  }

  return mask;
}

static uint16_t scripted_keymask_for_frame(const CliOptions *opts, int frame_no) {
  int hold_i = 0;
  uint16_t mask = 0;

  if (!opts || frame_no <= 0) {
    return 0;
  }

  for (hold_i = 0; hold_i < opts->headless_hold_count; ++hold_i) {
    const HeadlessHold *hold = &opts->headless_holds[hold_i];
    if (frame_no >= hold->start_frame && frame_no < (hold->start_frame + hold->len)) {
      mask |= hold->keymask;
    }
  }

  return mask;
}

static uint64_t hash_frame(const GbaFrame *frame) {
  uint64_t hash = 1469598103934665603ULL;
  const uint64_t prime = 1099511628211ULL;
  uint32_t i = 0;
  uint32_t limit = 0;

  if (!frame || !frame->pixels) {
    return 0;
  }

  limit = frame->width * frame->height;
  for (i = 0; i < limit; i += 97) {
    hash ^= (uint64_t)frame->pixels[i];
    hash *= prime;
  }

  return hash;
}

static int run_headless(GbaCore *core, const CliOptions *opts) {
  int i = 0;
  int dump_i = 0;
  GbaFrame frame = {0};
  GbaAudioChunk audio = {0};
  uint64_t agg_hash = 0;
  int frames = 0;

  if (!opts) {
    return 1;
  }
  frames = opts->headless_frames;

  for (i = 0; i < frames; ++i) {
    const int frame_no = i + 1;
    const uint16_t keymask = scripted_keymask_for_frame(opts, frame_no);
    gba_set_input(core, keymask);
    if (gba_step_frame(core, &frame, &audio) != 0) {
      fprintf(stderr, "gba_step_frame failed in headless mode at frame %d\n", i);
      return 1;
    }

    for (dump_i = 0; dump_i < opts->headless_dump_count; ++dump_i) {
      const HeadlessDump *dump = &opts->headless_dumps[dump_i];
      if (dump->frame_no != frame_no || !dump->path) {
        continue;
      }
      if (dump->png) {
        if (!write_png32(dump->path, &frame)) {
          fprintf(stderr, "Failed to dump PNG frame %d to %s\n", frame_no, dump->path);
          return 1;
        }
      } else {
        if (!write_bmp32(dump->path, &frame)) {
          fprintf(stderr, "Failed to dump BMP frame %d to %s\n", frame_no, dump->path);
          return 1;
        }
      }
    }

    agg_hash ^= hash_frame(&frame) + (uint64_t)i;
  }

  printf("HEADLESS_OK frames=%d hash=%016" PRIx64 "\n", frames, agg_hash);
  if (opts->headless_debug) {
    GbaDebugState dbg;
    memset(&dbg, 0, sizeof(dbg));
    if (gba_debug_state(core, &dbg) == 0) {
      printf("HEADLESS_DEBUG pc=%08" PRIx32 " cpsr=%08" PRIx32 " spsr=%08" PRIx32
             " halted=%" PRIu32 " cpu_unknown=%" PRIu32
             " vcount=%" PRIu32 " scanline_cycles=%" PRIu32
             " disp=%04" PRIx32 " stat=%04" PRIx32
             " ime=%04" PRIx32 " ie=%04" PRIx32 " if=%04" PRIx32 " key=%04" PRIx32
             " frame_count=%08" PRIx32 "%08" PRIx32
             " ticks=%08" PRIx32 "%08" PRIx32
             " eeprom_mode=%" PRIu32 " eeprom_bits=%" PRIu32
             " irq1178=%04" PRIx32 " irq7ff8=%04" PRIx32
             " keycnt=%04" PRIx32 "\n",
             dbg.pc, dbg.cpsr, dbg.spsr,
             dbg.halted, dbg.cpu_unknown,
             dbg.vcount, dbg.scanline_cycles,
             dbg.dispcnt, dbg.dispstat,
             dbg.ime, dbg.ie, dbg.iflag, dbg.keyinput,
             dbg.frame_count_hi, dbg.frame_count_lo,
             dbg.ticks_hi, dbg.ticks_lo,
             dbg.eeprom_mode, dbg.eeprom_bit_count,
             dbg.irq_latch_1178, dbg.irq_latch_7ff8,
             dbg.keycnt);
    } else {
      printf("HEADLESS_DEBUG unavailable\n");
    }
  }
  return 0;
}

static int run_interactive(GbaCore *core, const CliOptions *opts) {
  SDL_Window *window = NULL;
  SDL_Renderer *renderer = NULL;
  SDL_Texture *texture = NULL;
  SDL_AudioDeviceID audio_dev = 0;
  SDL_AudioSpec want = {0};
  SDL_AudioSpec have = {0};
  uint64_t perf_freq = 0;
  bool running = true;
  int rc = 1;
  int frame_no = 0;
  int scale = 3;
  bool mute = false;

  if (!opts) {
    return 1;
  }
  scale = opts->scale;
  mute = opts->mute;

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_EVENTS) != 0) {
    fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
    return 1;
  }

  window = SDL_CreateWindow("gbaemu", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                            240 * scale, 160 * scale, SDL_WINDOW_SHOWN);
  if (!window) {
    fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
    goto cleanup;
  }

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer) {
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  }
  if (!renderer) {
    fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
    goto cleanup;
  }

  texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
                              SDL_TEXTUREACCESS_STREAMING, 240, 160);
  if (!texture) {
    fprintf(stderr, "SDL_CreateTexture failed: %s\n", SDL_GetError());
    goto cleanup;
  }

  if (!mute) {
    want.freq = 32768;
    want.format = AUDIO_S16SYS;
    want.channels = 2;
    want.samples = 1024;
    want.callback = NULL;

    audio_dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (audio_dev == 0) {
      fprintf(stderr, "Audio disabled: %s\n", SDL_GetError());
      mute = true;
    } else {
      SDL_PauseAudioDevice(audio_dev, 0);
    }
  }

  perf_freq = (uint64_t)SDL_GetPerformanceFrequency();

  while (running) {
    SDL_Event ev;
    uint64_t frame_start = (uint64_t)SDL_GetPerformanceCounter();
    GbaFrame frame = {0};
    GbaAudioChunk audio = {0};
    uint16_t keymask = 0;

    while (SDL_PollEvent(&ev)) {
      if (ev.type == SDL_QUIT) {
        running = false;
      }
      if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) {
        running = false;
      }
    }

    keymask = gather_keymask();
    frame_no++;
    keymask |= scripted_keymask_for_frame(opts, frame_no);
    gba_set_input(core, keymask);

    if (gba_step_frame(core, &frame, &audio) != 0) {
      fprintf(stderr, "gba_step_frame failed\n");
      goto cleanup;
    }

    if (SDL_UpdateTexture(texture, NULL, frame.pixels, (int)(frame.width * sizeof(uint32_t))) != 0) {
      fprintf(stderr, "SDL_UpdateTexture failed: %s\n", SDL_GetError());
      goto cleanup;
    }

    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);

    if (!mute && audio_dev != 0 && audio.samples) {
      const uint32_t byte_count = audio.sample_count * audio.channels * (uint32_t)sizeof(int16_t);
      if (SDL_GetQueuedAudioSize(audio_dev) > (uint32_t)(have.freq * have.channels * sizeof(int16_t))) {
        SDL_ClearQueuedAudio(audio_dev);
      }
      SDL_QueueAudio(audio_dev, audio.samples, byte_count);
    }

    if (perf_freq != 0) {
      const double target_ms = 1000.0 / GBA_FPS;
      const uint64_t frame_end = (uint64_t)SDL_GetPerformanceCounter();
      const double elapsed_ms = ((double)(frame_end - frame_start) * 1000.0) / (double)perf_freq;
      if (elapsed_ms < target_ms) {
        SDL_Delay((Uint32)(target_ms - elapsed_ms));
      }
    }
  }

  rc = 0;

cleanup:
  if (audio_dev != 0) {
    SDL_CloseAudioDevice(audio_dev);
  }
  if (texture) {
    SDL_DestroyTexture(texture);
  }
  if (renderer) {
    SDL_DestroyRenderer(renderer);
  }
  if (window) {
    SDL_DestroyWindow(window);
  }
  SDL_Quit();
  return rc;
}

int main(int argc, char **argv) {
  CliOptions opts;
  char *owned_save = NULL;
  GbaConfig cfg;
  GbaCore *core = NULL;
  int rc = 0;

  if (!parse_args(argc, argv, &opts, &owned_save)) {
    free(owned_save);
    return 2;
  }

  memset(&cfg, 0, sizeof(cfg));
  cfg.bios_path = opts.bios_path;
  cfg.rom_path = opts.rom_path;
  cfg.save_path = opts.save_path;
  cfg.use_bios = (uint8_t)(opts.use_bios ? 1 : 0);
  cfg.trace = (uint8_t)(opts.trace ? 1 : 0);

  rc = gba_init(&cfg, &core);
  if (rc != 0 || !core) {
    fprintf(stderr, "gba_init failed: %d (%s)\n", rc, gba_init_error_reason(rc));
    if (rc == -4 && opts.use_bios && opts.bios_path) {
      fprintf(stderr, "BIOS path: %s\n", opts.bios_path);
    }
    free(owned_save);
    return 1;
  }

  if (opts.headless_frames > 0) {
    rc = run_headless(core, &opts);
  } else {
    rc = run_interactive(core, &opts);
  }

  if (gba_save(core, opts.save_path) != 0) {
    fprintf(stderr, "warning: save write failed for %s\n", opts.save_path ? opts.save_path : "(null)");
  }

  gba_destroy(core);
  free(owned_save);
  return rc;
}
