default rel
bits 64

%include "gba_core.inc"

extern gba_host_alloc
extern gba_host_free
extern gba_host_memset
extern gba_host_memcpy
extern gba_host_load_file
extern gba_host_save_file
extern gba_host_file_exists

extern gba_cpu_step_chunk
extern gba_ppu_render
extern gba_apu_mix
extern gba_sys_tick

global gba_init
global gba_step_frame
global gba_set_input
global gba_save
global gba_debug_state
global gba_destroy

section .text

; int gba_init(const GbaConfig *cfg, GbaCore **out_core)
gba_init:
    push rbp
    mov rbp, rsp
    sub rsp, 80

    ; locals
    ; [rbp-8]  = core
    ; [rbp-16] = tmp_ptr
    ; [rbp-24] = tmp_size
    ; [rbp-32] = cfg
    ; [rbp-40] = out_core
    ; [rbp-48] = error_code

    mov qword [rbp-8], 0
    mov qword [rbp-16], 0
    mov qword [rbp-24], 0
    mov qword [rbp-32], rdi
    mov qword [rbp-40], rsi
    mov dword [rbp-48], -1

    test rdi, rdi
    jz .fail
    test rsi, rsi
    jz .fail

    mov rax, [rdi + GBA_CONFIG.rom_path]
    test rax, rax
    jz .fail

    mov edi, GBA_CORE_size
    call gba_host_alloc
    test rax, rax
    jz .alloc_fail

    mov [rbp-8], rax

    mov dword [rax + GBA_CORE.magic], GBA_MAGIC
    mov rcx, [rbp-32]
    movzx edx, byte [rcx + GBA_CONFIG.trace]
    mov dword [rax + GBA_CORE.trace], edx

    ; Load ROM (required)
    mov rcx, [rbp-32]
    mov rdi, [rcx + GBA_CONFIG.rom_path]
    lea rsi, [rbp-16]
    lea rdx, [rbp-24]
    call gba_host_load_file
    test eax, eax
    jnz .rom_load_fail

    mov rcx, [rbp-8]
    mov rax, [rbp-16]
    mov [rcx + GBA_CORE.rom_ptr], rax
    mov rax, [rbp-24]
    mov [rcx + GBA_CORE.rom_size], rax

    ; Load BIOS (optional unless --bios path is explicitly provided)
    mov rcx, [rbp-32]
    movzx eax, byte [rcx + GBA_CONFIG.use_bios]
    test eax, eax
    jz .skip_bios

    mov rdi, [rcx + GBA_CONFIG.bios_path]
    test rdi, rdi
    jz .skip_bios

    mov qword [rbp-16], 0
    mov qword [rbp-24], 0
    lea rsi, [rbp-16]
    lea rdx, [rbp-24]
    call gba_host_load_file
    test eax, eax
    jnz .bios_load_fail

    mov rax, [rbp-24]
    cmp rax, 0x4000
    jne .bios_size_fail

    mov rcx, [rbp-8]
    mov rax, [rbp-16]
    mov [rcx + GBA_CORE.bios_ptr], rax
    mov rax, [rbp-24]
    mov [rcx + GBA_CORE.bios_size], rax

.skip_bios:
    ; Allocate save RAM
    mov edi, GBA_SAVE_SIZE
    call gba_host_alloc
    test rax, rax
    jz .alloc_fail

    mov rcx, [rbp-8]
    mov [rcx + GBA_CORE.save_ptr], rax
    mov qword [rcx + GBA_CORE.save_size], GBA_SAVE_SIZE
    mov rdi, rax
    mov esi, 0xFF
    mov edx, GBA_SAVE_SIZE
    call gba_host_memset

    ; Save path pointer is owned by host process argv (stable for process lifetime)
    mov rdx, [rbp-32]
    mov rax, [rdx + GBA_CONFIG.save_path]
    mov [rcx + GBA_CORE.save_path], rax

    ; Try to load existing save file if present
    test rax, rax
    jz .state_init

    mov rdi, rax
    call gba_host_file_exists
    test eax, eax
    jz .state_init

    mov rcx, [rbp-32]
    mov rdi, [rcx + GBA_CONFIG.save_path]
    mov qword [rbp-16], 0
    mov qword [rbp-24], 0
    lea rsi, [rbp-16]
    lea rdx, [rbp-24]
    call gba_host_load_file
    test eax, eax
    jnz .state_init

    mov rcx, [rbp-8]
    mov rdi, [rcx + GBA_CORE.save_ptr]
    mov rsi, [rbp-16]
    mov rdx, [rbp-24]
    cmp rdx, GBA_SAVE_SIZE
    jbe .save_copy_size_ok
    mov rdx, GBA_SAVE_SIZE

.save_copy_size_ok:
    call gba_host_memcpy

    mov rdi, [rbp-16]
    call gba_host_free

.state_init:
    mov rcx, [rbp-8]

    mov dword [rcx + GBA_CORE.save_dirty], 0
    mov dword [rcx + GBA_CORE.halted], 0
    mov dword [rcx + GBA_CORE.cpu_unknown], 0
    mov qword [rcx + GBA_CORE.frame_count], 0
    mov qword [rcx + GBA_CORE.ticks], 0
    mov dword [rcx + GBA_CORE.bg_phase], 0
    mov dword [rcx + GBA_CORE.eeprom_mode], 0
    mov dword [rcx + GBA_CORE.eeprom_addr_bits], 0
    mov dword [rcx + GBA_CORE.eeprom_bit_count], 0
    mov dword [rcx + GBA_CORE.eeprom_read_addr], 0
    mov dword [rcx + GBA_CORE.eeprom_read_bit], 0
    mov byte [rcx + GBA_CORE.keypad_irq_latched], 0

    ; Boot from BIOS when loaded, otherwise fall back to cartridge entry.
    mov eax, GBA_ROM_BASE
    mov rdx, [rcx + GBA_CORE.bios_ptr]
    test rdx, rdx
    jz .boot_pc_ready
    mov rdx, [rcx + GBA_CORE.bios_size]
    cmp rdx, 0x4000
    jb .boot_pc_ready
    xor eax, eax
.boot_pc_ready:
    ; CPU defaults.
    ; BIOS boot starts in SVC ARM mode with IRQ/FIQ disabled (CPSR=0xD3).
    ; No-BIOS fallback keeps the historical SYS mode setup.
    mov dword [rcx + GBA_CORE.cpu_spsr], 0

    mov dword [rcx + GBA_CORE.bank_r13_usr], 0x03007F00
    mov dword [rcx + GBA_CORE.bank_r14_usr], 0
    mov dword [rcx + GBA_CORE.bank_r13_irq], 0x03007FA0
    mov dword [rcx + GBA_CORE.bank_r14_irq], 0
    mov dword [rcx + GBA_CORE.bank_r13_svc], 0x03007FE0
    mov dword [rcx + GBA_CORE.bank_r14_svc], 0

    test eax, eax
    jnz .cpu_no_bios_mode

    mov dword [rcx + GBA_CORE.cpu_cpsr], 0x000000D3
    mov dword [rcx + GBA_CORE.cpu_regs + (13 * 4)], 0x03007FE0
    mov dword [rcx + GBA_CORE.cpu_regs + (14 * 4)], 0
    jmp .cpu_mode_ready

.cpu_no_bios_mode:
    mov dword [rcx + GBA_CORE.cpu_cpsr], 0x0000001F
    mov dword [rcx + GBA_CORE.cpu_regs + (13 * 4)], 0x03007F00
    mov dword [rcx + GBA_CORE.cpu_regs + (14 * 4)], 0

.cpu_mode_ready:
    mov dword [rcx + GBA_CORE.cpu_regs + (15 * 4)], eax
    mov dword [rcx + GBA_CORE.cpu_pc], eax

    ; Keypad starts with no buttons pressed (active low, upper bits read as 1).
    mov word [rcx + GBA_CORE.io + IO_KEYINPUT], 0xFFFF
    mov word [rcx + GBA_CORE.io + IO_KEYCNT], 0x0000

    ; When booting without BIOS, keep legacy direct-render startup behavior.
    cmp eax, GBA_ROM_BASE
    jne .display_init_done
    mov word [rcx + GBA_CORE.io + IO_DISPCNT], 0x0403
.display_init_done:

    mov rax, [rbp-40]
    mov rdx, [rbp-8]
    mov [rax], rdx

    xor eax, eax
    leave
    ret

.alloc_fail:
    mov dword [rbp-48], -2
    jmp .fail

.rom_load_fail:
    mov dword [rbp-48], -3
    jmp .fail

.bios_load_fail:
    mov dword [rbp-48], -4
    jmp .fail

.bios_size_fail:
    mov rdi, [rbp-16]
    test rdi, rdi
    jz .bios_size_fail_set
    call gba_host_free
    mov qword [rbp-16], 0
.bios_size_fail_set:
    mov dword [rbp-48], -5
    jmp .fail

.fail:
    mov rax, [rbp-40]
    test rax, rax
    jz .fail_cleanup
    mov qword [rax], 0

.fail_cleanup:
    mov rdi, [rbp-8]
    test rdi, rdi
    jz .ret_fail
    call gba_destroy

.ret_fail:
    mov eax, [rbp-48]
    leave
    ret

; int gba_step_frame(GbaCore *core, GbaFrame *out_frame, GbaAudioChunk *out_audio)
gba_step_frame:
    push rbp
    mov rbp, rsp
    sub rsp, 32

    mov [rbp-8], rdi
    mov [rbp-16], rsi
    mov [rbp-24], rdx

    test rdi, rdi
    jz .bad_args
    cmp dword [rdi + GBA_CORE.magic], GBA_MAGIC
    jne .bad_args

    ; Emulate roughly one GBA frame worth of cycles in interleaved slices,
    ; so CPU wait loops can observe timer/VCOUNT/IRQ state changes.
    mov dword [rbp-28], 280896   ; cycles remaining

.step_loop:
    mov eax, [rbp-28]
    test eax, eax
    jz .step_done

    ; Use instruction-granular interleave so IRQ/timer side effects can
    ; occur inside tight ROM wait loops.
    cmp eax, 1
    jbe .slice_ready
    mov eax, 1

.slice_ready:
    mov [rbp-32], eax
    sub dword [rbp-28], eax

    mov rdi, [rbp-8]
    mov esi, [rbp-32]
    call gba_cpu_step_chunk

    mov rdi, [rbp-8]
    mov esi, [rbp-32]
    call gba_sys_tick
    jmp .step_loop

.step_done:

    mov rdi, [rbp-8]
    call gba_ppu_render

    mov rdi, [rbp-8]
    call gba_apu_mix

    ; Fill output frame descriptor.
    mov rax, [rbp-16]
    test rax, rax
    jz .skip_frame_desc

    mov rcx, [rbp-8]
    lea rdx, [rcx + GBA_CORE.framebuffer]
    mov [rax + GBA_FRAME.pixels], rdx
    mov dword [rax + GBA_FRAME.width], GBA_SCREEN_W
    mov dword [rax + GBA_FRAME.height], GBA_SCREEN_H

.skip_frame_desc:
    mov rax, [rbp-24]
    test rax, rax
    jz .ok

    mov rcx, [rbp-8]
    lea rdx, [rcx + GBA_CORE.audio]
    mov [rax + GBA_AUDIO_CHUNK.samples], rdx
    mov dword [rax + GBA_AUDIO_CHUNK.sample_count], GBA_AUDIO_SAMPLES_PER_FRAME
    mov dword [rax + GBA_AUDIO_CHUNK.channels], GBA_AUDIO_CHANNELS
    mov dword [rax + GBA_AUDIO_CHUNK.sample_rate], GBA_AUDIO_RATE

.ok:
    xor eax, eax
    leave
    ret

.bad_args:
    mov eax, -1
    leave
    ret

; void gba_set_input(GbaCore *core, uint16_t keymask)
gba_set_input:
    test rdi, rdi
    jz .done
    cmp dword [rdi + GBA_CORE.magic], GBA_MAGIC
    jne .done

    mov [rdi + GBA_CORE.input_mask], si
.done:
    ret

; int gba_save(GbaCore *core, const char *path)
gba_save:
    push rbp
    mov rbp, rsp
    sub rsp, 16

    mov [rbp-8], rdi
    mov [rbp-16], rsi

    test rdi, rdi
    jz .bad
    cmp dword [rdi + GBA_CORE.magic], GBA_MAGIC
    jne .bad

    mov rax, [rbp-16]
    test rax, rax
    jnz .path_ok

    mov rax, [rdi + GBA_CORE.save_path]

.path_ok:
    test rax, rax
    jz .bad

    mov rdx, [rdi + GBA_CORE.save_ptr]
    test rdx, rdx
    jz .bad

    mov rdi, rax
    mov rsi, rdx
    mov rdx, [rbp-8]
    mov rdx, [rdx + GBA_CORE.save_size]
    call gba_host_save_file
    test eax, eax
    jnz .save_fail

    mov rcx, [rbp-8]
    mov dword [rcx + GBA_CORE.save_dirty], 0

    xor eax, eax
    leave
    ret

.save_fail:
    mov eax, -2
    leave
    ret

.bad:
    mov eax, -1
    leave
    ret

; int gba_debug_state(GbaCore *core, GbaDebugState *out_state)
gba_debug_state:
    test rdi, rdi
    jz .bad
    test rsi, rsi
    jz .bad
    cmp dword [rdi + GBA_CORE.magic], GBA_MAGIC
    jne .bad

    mov eax, [rdi + GBA_CORE.cpu_regs + (15 * 4)]
    mov [rsi + GBA_DEBUG_STATE.pc], eax
    mov eax, [rdi + GBA_CORE.cpu_cpsr]
    mov [rsi + GBA_DEBUG_STATE.cpsr], eax
    mov eax, [rdi + GBA_CORE.cpu_spsr]
    mov [rsi + GBA_DEBUG_STATE.spsr], eax
    mov eax, [rdi + GBA_CORE.halted]
    mov [rsi + GBA_DEBUG_STATE.halted], eax
    mov eax, [rdi + GBA_CORE.cpu_unknown]
    mov [rsi + GBA_DEBUG_STATE.cpu_unknown], eax
    mov eax, [rdi + GBA_CORE.scanline_cycles]
    mov [rsi + GBA_DEBUG_STATE.scanline_cycles], eax
    movzx eax, word [rdi + GBA_CORE.io + IO_VCOUNT]
    mov [rsi + GBA_DEBUG_STATE.vcount], eax
    movzx eax, word [rdi + GBA_CORE.io + IO_DISPCNT]
    mov [rsi + GBA_DEBUG_STATE.dispcnt], eax
    movzx eax, word [rdi + GBA_CORE.io + IO_DISPSTAT]
    mov [rsi + GBA_DEBUG_STATE.dispstat], eax
    movzx eax, word [rdi + GBA_CORE.io + IO_IE]
    mov [rsi + GBA_DEBUG_STATE.ie], eax
    movzx eax, word [rdi + GBA_CORE.io + IO_IF]
    mov [rsi + GBA_DEBUG_STATE.iflag], eax
    movzx eax, word [rdi + GBA_CORE.io + IO_IME]
    mov [rsi + GBA_DEBUG_STATE.ime], eax
    movzx eax, word [rdi + GBA_CORE.io + IO_KEYINPUT]
    mov [rsi + GBA_DEBUG_STATE.keyinput], eax
    mov rax, [rdi + GBA_CORE.frame_count]
    mov [rsi + GBA_DEBUG_STATE.frame_count_lo], eax
    shr rax, 32
    mov [rsi + GBA_DEBUG_STATE.frame_count_hi], eax
    mov rax, [rdi + GBA_CORE.ticks]
    mov [rsi + GBA_DEBUG_STATE.ticks_lo], eax
    shr rax, 32
    mov [rsi + GBA_DEBUG_STATE.ticks_hi], eax
    mov eax, [rdi + GBA_CORE.eeprom_mode]
    mov [rsi + GBA_DEBUG_STATE.eeprom_mode], eax
    mov eax, [rdi + GBA_CORE.eeprom_bit_count]
    mov [rsi + GBA_DEBUG_STATE.eeprom_bit_count], eax
    movzx eax, word [rdi + GBA_CORE.iwram + 0x1178]
    mov [rsi + GBA_DEBUG_STATE.irq_latch_1178], eax
    movzx eax, word [rdi + GBA_CORE.iwram + 0x7FF8]
    mov [rsi + GBA_DEBUG_STATE.irq_latch_7ff8], eax
    movzx eax, word [rdi + GBA_CORE.io + IO_KEYCNT]
    mov [rsi + GBA_DEBUG_STATE.keycnt], eax

    xor eax, eax
    ret

.bad:
    mov eax, -1
    ret

; void gba_destroy(GbaCore *core)
gba_destroy:
    push rbp
    mov rbp, rsp
    sub rsp, 16

    mov [rbp-8], rdi

    test rdi, rdi
    jz .done

    cmp dword [rdi + GBA_CORE.magic], GBA_MAGIC
    jne .free_only

    mov rax, [rdi + GBA_CORE.bios_ptr]
    test rax, rax
    jz .skip_bios_free
    mov rdi, rax
    call gba_host_free

.skip_bios_free:
    mov rdi, [rbp-8]

    mov rax, [rdi + GBA_CORE.rom_ptr]
    test rax, rax
    jz .skip_rom_free
    mov rdi, rax
    call gba_host_free

.skip_rom_free:
    mov rdi, [rbp-8]
    mov rax, [rdi + GBA_CORE.save_ptr]
    test rax, rax
    jz .skip_save_free
    mov rdi, rax
    call gba_host_free

.skip_save_free:
    mov rdi, [rbp-8]
    mov dword [rdi + GBA_CORE.magic], 0

.free_only:
    call gba_host_free

.done:
    leave
    ret
