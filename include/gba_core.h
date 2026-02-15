#ifndef GBA_CORE_H
#define GBA_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct GbaCore GbaCore;

typedef struct {
  const char *bios_path;
  const char *rom_path;
  const char *save_path;
  uint8_t use_bios;
  uint8_t trace;
  uint16_t reserved;
} GbaConfig;

typedef struct {
  uint32_t *pixels;
  uint32_t width;
  uint32_t height;
} GbaFrame;

typedef struct {
  int16_t *samples;
  uint32_t sample_count;
  uint32_t channels;
  uint32_t sample_rate;
} GbaAudioChunk;

typedef struct {
  uint32_t pc;
  uint32_t cpsr;
  uint32_t spsr;
  uint32_t halted;
  uint32_t cpu_unknown;
  uint32_t scanline_cycles;
  uint32_t vcount;
  uint32_t dispcnt;
  uint32_t dispstat;
  uint32_t ie;
  uint32_t iflag;
  uint32_t ime;
  uint32_t keyinput;
  uint32_t frame_count_lo;
  uint32_t frame_count_hi;
  uint32_t ticks_lo;
  uint32_t ticks_hi;
  uint32_t eeprom_mode;
  uint32_t eeprom_bit_count;
  uint32_t irq_latch_1178;
  uint32_t irq_latch_7ff8;
  uint32_t keycnt;
} GbaDebugState;

int gba_init(const GbaConfig *cfg, GbaCore **out_core);
int gba_step_frame(GbaCore *core, GbaFrame *out_frame, GbaAudioChunk *out_audio);
void gba_set_input(GbaCore *core, uint16_t keymask);
int gba_save(GbaCore *core, const char *path);
int gba_debug_state(GbaCore *core, GbaDebugState *out_state);
void gba_destroy(GbaCore *core);

#ifdef __cplusplus
}
#endif

#endif
