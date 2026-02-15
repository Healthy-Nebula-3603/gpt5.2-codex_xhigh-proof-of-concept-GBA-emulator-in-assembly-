%ifndef GBA_CORE_INC
%define GBA_CORE_INC

%define GBA_MAGIC 0x31414247        ; 'GBA1'

%define GBA_SCREEN_W 240
%define GBA_SCREEN_H 160
%define GBA_FB_PIXELS (GBA_SCREEN_W * GBA_SCREEN_H)

%define GBA_AUDIO_SAMPLES_PER_FRAME 1024
%define GBA_AUDIO_CHANNELS 2
%define GBA_AUDIO_RATE 32768
%define GBA_AUDIO_TOTAL_SAMPLES (GBA_AUDIO_SAMPLES_PER_FRAME * GBA_AUDIO_CHANNELS)

%define GBA_SAVE_SIZE 65536
%define GBA_EEPROM_MAX_SIZE 8192

%define GBA_EWRAM_SIZE 262144
%define GBA_IWRAM_SIZE 32768
%define GBA_IO_SIZE 1024
%define GBA_PRAM_SIZE 1024
%define GBA_VRAM_SIZE 98304
%define GBA_OAM_SIZE 1024

%define GBA_ROM_BASE 0x08000000
%define GBA_EEPROM_BASE 0x0D000000
%define GBA_SRAM_BASE 0x0E000000

%define IO_DISPCNT 0x000
%define IO_DISPSTAT 0x004
%define IO_VCOUNT  0x006
%define IO_BG0CNT  0x008
%define IO_BG1CNT  0x00A
%define IO_BG2CNT  0x00C
%define IO_BG3CNT  0x00E
%define IO_BG0HOFS 0x010
%define IO_BG0VOFS 0x012
%define IO_BG1HOFS 0x014
%define IO_BG1VOFS 0x016
%define IO_BG2HOFS 0x018
%define IO_BG2VOFS 0x01A
%define IO_BG3HOFS 0x01C
%define IO_BG3VOFS 0x01E
%define IO_BG2PA   0x020
%define IO_BG2PB   0x022
%define IO_BG2PC   0x024
%define IO_BG2PD   0x026
%define IO_BG2X    0x028
%define IO_BG2Y    0x02C
%define IO_BG3PA   0x030
%define IO_BG3PB   0x032
%define IO_BG3PC   0x034
%define IO_BG3PD   0x036
%define IO_BG3X    0x038
%define IO_BG3Y    0x03C
%define IO_WIN0H  0x040
%define IO_WIN1H  0x042
%define IO_WIN0V  0x044
%define IO_WIN1V  0x046
%define IO_WININ  0x048
%define IO_WINOUT 0x04A
%define IO_BLDCNT 0x050
%define IO_BLDALPHA 0x052
%define IO_BLDY 0x054
%define IO_DMA0SAD 0x0B0
%define IO_DMA_STRIDE 12
%define IO_TM0CNT_L 0x100
%define IO_TM0CNT_H 0x102
%define IO_TM1CNT_L 0x104
%define IO_TM1CNT_H 0x106
%define IO_TM2CNT_L 0x108
%define IO_TM2CNT_H 0x10A
%define IO_TM3CNT_L 0x10C
%define IO_TM3CNT_H 0x10E
%define IO_KEYINPUT 0x130
%define IO_KEYCNT 0x132
%define IO_IE 0x200
%define IO_IF 0x202
%define IO_IME 0x208

%define CPSR_N 0x80000000
%define CPSR_Z 0x40000000
%define CPSR_C 0x20000000
%define CPSR_V 0x10000000
%define CPSR_I 0x00000080
%define CPSR_T 0x00000020

%define IRQ_VBLANK 0x0001
%define IRQ_HBLANK 0x0002
%define IRQ_VCOUNT 0x0004
%define IRQ_TIMER0 0x0008
%define IRQ_TIMER1 0x0010
%define IRQ_TIMER2 0x0020
%define IRQ_TIMER3 0x0040
%define IRQ_DMA0   0x0100
%define IRQ_DMA1   0x0200
%define IRQ_DMA2   0x0400
%define IRQ_DMA3   0x0800
%define IRQ_KEYPAD 0x1000

struc GBA_CONFIG
  .bios_path:  resq 1
  .rom_path:   resq 1
  .save_path:  resq 1
  .use_bios:   resb 1
  .trace:      resb 1
  .reserved:   resw 1
  .pad:        resd 1
endstruc

struc GBA_FRAME
  .pixels:     resq 1
  .width:      resd 1
  .height:     resd 1
endstruc

struc GBA_AUDIO_CHUNK
  .samples:       resq 1
  .sample_count:  resd 1
  .channels:      resd 1
  .sample_rate:   resd 1
  .pad:           resd 1
endstruc

struc GBA_DEBUG_STATE
  .pc:               resd 1
  .cpsr:             resd 1
  .spsr:             resd 1
  .halted:           resd 1
  .cpu_unknown:      resd 1
  .scanline_cycles:  resd 1
  .vcount:           resd 1
  .dispcnt:          resd 1
  .dispstat:         resd 1
  .ie:               resd 1
  .iflag:            resd 1
  .ime:              resd 1
  .keyinput:         resd 1
  .frame_count_lo:   resd 1
  .frame_count_hi:   resd 1
  .ticks_lo:         resd 1
  .ticks_hi:         resd 1
  .eeprom_mode:      resd 1
  .eeprom_bit_count: resd 1
  .irq_latch_1178:   resd 1
  .irq_latch_7ff8:   resd 1
  .keycnt:           resd 1
endstruc

struc GBA_CORE
  .magic:         resd 1
  .trace:         resd 1
  .frame_count:   resq 1
  .input_mask:    resw 1
  .pad_input:     resw 1
  .save_dirty:    resd 1
  .halted:        resd 1

  .bios_ptr:      resq 1
  .bios_size:     resq 1
  .rom_ptr:       resq 1
  .rom_size:      resq 1
  .save_ptr:      resq 1
  .save_size:     resq 1
  .save_path:     resq 1

  .cpu_regs:      resd 16
  .cpu_cpsr:      resd 1
  .cpu_spsr:      resd 1
  .cpu_pc:        resd 1
  .cpu_unknown:   resd 1
  .bank_r13_usr:  resd 1
  .bank_r14_usr:  resd 1
  .bank_r13_irq:  resd 1
  .bank_r14_irq:  resd 1
  .bank_r13_svc:  resd 1
  .bank_r14_svc:  resd 1

  .ticks:         resq 1
  .bg_phase:      resd 1
  .scanline_cycles: resd 1
  .timer_accum:   resq 4
  .timer_reload:  resw 4
  .timer_latched_en: resb 4
  .eeprom_mode:   resd 1
  .eeprom_addr_bits: resd 1
  .eeprom_bit_count: resd 1
  .eeprom_read_addr: resd 1
  .eeprom_read_bit: resd 1
  .eeprom_bits:   resb 96
  .keypad_irq_latched: resb 1
  .pad_key_irq:   resb 3

  .ewram:         resb GBA_EWRAM_SIZE
  .iwram:         resb GBA_IWRAM_SIZE
  .io:            resb GBA_IO_SIZE
  .pram:          resb GBA_PRAM_SIZE
  .vram:          resb GBA_VRAM_SIZE
  .oam:           resb GBA_OAM_SIZE

  .framebuffer:   resd GBA_FB_PIXELS
  .audio:         resw GBA_AUDIO_TOTAL_SAMPLES
endstruc

%endif
