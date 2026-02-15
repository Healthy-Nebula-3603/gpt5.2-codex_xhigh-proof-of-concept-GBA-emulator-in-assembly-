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
