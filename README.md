Working ( quite well ! ) GBA emulator written in assembly!
I used codex-cli with GPT 5.3 codex xhigh - It was building, testing (taking screenshots, playing itself ) and debugging itself for around 5 hours. Finally giving working GBA emulator.

PROMPT
"Build a fully working Nintendo GBA emulator in a pure assembly that would rum games like SuperMarioAdvance for instance  ( I dumped that and put in the folder with dumped bios)"

GPT made a plan first:
"
Proposed Plan


  # GBA Emulator Plan (Linux x86-64, Assembly Core, SDL2 Frontend)

  ## Summary

  Build a Game Boy Advance emulator where emulation subsystems are in x86-64 assembly, with a minimal C host layer only for SDL2 window/audio/input,
  Primary target is broad compatibility with a concrete acceptance gate: SuperMarioAdvance.gba boots from gba_bios.bin, reaches gameplay, and is
  stable/playable.

  ## Scope and Non-Scope

  - In scope:
      - ARM7TDMI CPU core (ARM + Thumb), interrupts, timers, DMA, keypad, cartridge bus, BIOS-backed boot path.
      - GBA memory map, waitstates, save memory handling (SRAM/Flash/EEPROM basic support).
      - PPU modes/layers/sprites/window/blending sufficient for Mario Advance and common games.
      - APU channels + mixer with practical timing (not cycle-perfect first pass).
      - Deterministic frame stepping and regression tests.
  - Out of scope for first major milestone:
      - Full cycle-accurate bus contention for every edge case.
      - Netplay, rewind, shader filters.

  ## Architecture (Decision Complete)

  1. Repository layout:
      - src/asm/mem/ memory map, regions, waitstate logic.
      - src/asm/apu/ channel synthesis + mixer.
      - src/asm/sys/ DMA, timers, IRQ, keypad, scheduler.
      - src/host/ C SDL2 bootstrap + CLI + file I/O.
      - tests/ test ROM harness + SMA smoke scripts.
  2. Build system:
      - CMake + nasm for .asm and clang/gcc for host C.
      - Targets: gbaemu (release), gbaemu_dbg (trace enabled).
  3. Runtime threading model:
      - Single emulation thread (deterministic core).
      - SDL audio callback pulls from ring buffer filled by emu thread.
  4. Frame scheduling:
      - Present one frame every VBlank to SDL texture.
      - Autodetect by ROM ID + probing fallback.
      - Persist to <romname>.sav in ROM directory.
      
## Public Interfaces / Types / ABIs

  - CLI contract:
  - Host-to-core C ABI (stable):
      - int gba_init(const GbaConfig* cfg, GbaCore** out);
      - int gba_step_frame(GbaCore* core, GbaFrame* out_frame, GbaAudioChunk* out_audio);
      - void gba_set_input(GbaCore* core, uint16_t keymask);
      - int gba_save(GbaCore* core, const char* path);
      - void gba_destroy(GbaCore* core);
  - Core-owned state structures:
      - GbaCpuState, GbaMemState, GbaPpuState, GbaApuState, GbaSysState.
  2. CPU correctness baseline:
      - IRQ/exception entry/return.
  3. Memory + system timing:
      - Full memory map, MMIO register behavior, timers, DMA channels, IRQ flags.
  4. PPU bring-up:
      - Mode 0/1/2 tile backgrounds, sprites, palette/VRAM/OAM rules, VBlank/HBlank timing.
      - Then Mode 3/4/5 and blending/windowing.
      - PSG channels + FIFO DMA audio path, downmix to SDL sample format.
  6. Compatibility expansion:
      - Save memory variants, waitstate tuning, edge-case MMIO behavior.
  7. Performance pass:
      - Hotpath decode table optimization, branch prediction-aware dispatch, cache-friendly memory handlers.
  8. Stabilization:
      - Fixes from test ROM failures + SMA scripted smoke failures.

  ## Testing and Acceptance

  - Automated test suites:
      - ARM/Thumb instruction tests (flags, shifts, memory addressing, exceptions).
      - GBA timing/MMIO-focused homebrew test ROMs.
  - SMA smoke automation:
      - Boot from BIOS to title screen within bounded frames.
      - Scripted input sequence reaches in-game control.
      - No crash/hang for 10+ minutes gameplay loop.
  - Determinism checks:
      - Same ROM + input log produces identical frame hashes and audio checksums across runs.
  - Performance gate:
      - On Linux x86-64 baseline machine: sustained 59.7 FPS average with audio enabled for SMA.

  ## Tooling and Debug Strategy

  - Debug modes:
      - Instruction trace (--trace) with PC/opcode/CPSR.
      - MMIO and IRQ event logs (toggle flags).
  - Golden artifacts:
      - Frame hash snapshots at known milestones (boot logo, title screen).
      - Save-state-like debug dumps for bisecting regressions.

  ## Risks and Mitigations

  - Risk: “Pure assembly” complexity slows progress.
      - Mitigation: strict assembly core boundaries + tiny stable C platform shim.
  - Risk: PPU/APU timing bugs block compatibility.
      - Mitigation: incremental test ROM gating before SMA smoke tests.
  - Risk: ABI drift between C and assembly.
      - Mitigation: one canonical header + CI check building both debug/release targets.

  ## Assumptions and Defaults

  - Host OS/arch: Linux x86-64.
  - Frontend stack: SDL2 for video/audio/input.
  - Purity policy: emulation core in assembly; host glue in C only.
  - BIOS: use provided gba_bios.bin dump from workspace.
  - Initial success definition: SuperMarioAdvance.gba boots and is playable, while architecture remains expandable for broader compatibility.

"

Thenjust made the emulator....


<img width="739" height="531" alt="Screenshot from 2026-02-08 19-14-41" src="https://github.com/user-attachments/assets/1d396a66-14b5-40ea-a57b-85181907280b" />
<img width="736" height="527" alt="Screenshot from 2026-02-15 01-12-49" src="https://github.com/user-attachments/assets/6c91ada4-cbc1-497d-b8d9-522e80ba74bf" />
<img width="736" height="527" alt="Screenshot from 2026-02-15 01-12-34" src="https://github.com/user-attachments/assets/31f3cbb4-7ac9-42eb-a889-cbdee045fa83" />
<img width="736" height="527" alt="Screenshot from 2026-02-15 01-11-59" src="https://github.com/user-attachments/assets/060603ab-f055-4667-b06f-acae3f66b385" />
<img width="736" height="527" alt="Screenshot from 2026-02-15 01-11-46" src="https://github.com/user-attachments/assets/2fbc3a9d-6067-4a31-b5d5-0e5e3159b875" />
<img width="736" height="527" alt="Screenshot from 2026-02-15 01-11-38" src="https://github.com/user-attachments/assets/f9d94fa0-17a6-4f2c-9b48-d7291122674c" />
<img width="736" height="527" alt="Screenshot from 2026-02-15 01-11-31" src="https://github.com/user-attachments/assets/cd4c022c-e7f7-4b67-9914-229524835d17" />




# gbaemu (assembly-core prototype)

This project implements a Game Boy Advance emulator prototype with an assembly-exported core ABI and an SDL2 host frontend.
The CPU/PPU/APU/system paths are ASM modules.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run

Interactive:

```bash
./build/gbaemu --bios ./gba_bios.bin --rom ./SuperMarioAdvance.gba --scale 3
```

Interactive with scripted startup (auto-boot, then take over manually):

```bash
./build/gbaemu --bios ./gba_bios.bin --rom ./SuperMarioAdvance.gba --scale 3 \
```

Headless smoke run:

```bash
./build/gbaemu --bios ./gba_bios.bin --rom ./SuperMarioAdvance.gba --headless-frames 120 --mute
```

Headless scripted input + frame dump:

```bash
./build/gbaemu --bios ./gba_bios.bin --rom ./SuperMarioAdvance.gba \
  --headless-frames 1200 \
  --headless-hold START 900 8 \
  --headless-dump-png 900 /tmp/frame900.png \
  --headless-dump-png 1200 /tmp/frame1200.png \
  --mute
```

Deterministic menu progression example (title -> file select):

```bash
./build/gbaemu --bios ./gba_bios.bin --rom /tmp/SMA_clean.gba \
  --headless-frames 7000 \
  --headless-hold START 1450 2 \
  --headless-hold START 1600 2 \
  --headless-hold A 1600 2 \
  --headless-hold A 1700 2 \
  --headless-dump-png 7000 /tmp/file_select.png \
  --mute
```

## Controls

- `Z`: A
- `X`: B
- `Backspace`: Select
- `Enter`: Start
- Arrow keys: D-pad
- `A`: L
- `S`: R
- `Esc`: Quit

## Notes

- Core-facing API is exposed from assembly (`gba_init`, `gba_step_frame`, `gba_set_input`, `gba_save`, `gba_destroy`).
- Save data defaults to `<romname>.sav` unless overridden with `--save`.
- `--headless-hold` / `--headless-press-start` work in both headless and interactive modes.
- Implemented emulation subset includes:
  - ARM + Thumb interpreter subsets with conditional execution, branches/BL/BX, push/pop, and common arithmetic/load-store ops.
  - ARM block transfer and multiply family subsets.
  - BIOS HLE SWI subset for `Div`, `Sqrt`, `CpuSet`, `CpuFastSet`, and LZ77 decompress paths used by many commercial games.
  - Memory-mapped EWRAM/IWRAM/IO/PRAM/VRAM/OAM/ROM/SRAM regions.
  - PPU mode 3 and mode 4 framebuffer rendering paths.
- This is still not full game-compatibility yet.
