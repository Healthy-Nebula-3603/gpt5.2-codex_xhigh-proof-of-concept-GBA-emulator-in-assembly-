default rel
bits 64

%include "gba_core.inc"

section .rodata
; theta = (angle>>8) * (pi / 128)
k_theta_scale: dd 0.024543693

section .text
global gba_cpu_step_chunk
global gba_mem_read8
global gba_mem_read16
global gba_mem_read32
global gba_mem_write8
global gba_mem_write16
global gba_mem_write32

; -----------------------------------------------------------------------------
; Helpers: memory map
; -----------------------------------------------------------------------------

cpu_eeprom_read_bit:
    ; in:  rdi=core
    ; out: eax=bit (0/1)
    mov eax, [rdi + GBA_CORE.eeprom_mode]
    cmp eax, 1
    jne .eeprom_read_idle

    mov edx, [rdi + GBA_CORE.eeprom_read_bit]
    cmp edx, 68
    jae .eeprom_read_done_stream

    cmp edx, 4
    jb .eeprom_read_dummy

    mov eax, edx
    sub eax, 4
    mov ecx, eax
    and ecx, 7                    ; bit within byte
    mov esi, eax
    shr esi, 3                    ; byte within 64-bit payload

    mov eax, [rdi + GBA_CORE.eeprom_read_addr]
    mov ecx, [rdi + GBA_CORE.eeprom_addr_bits]
    cmp ecx, 14
    jne .eeprom_read_addr_small
    and eax, 0x3FF
    jmp .eeprom_read_addr_masked

.eeprom_read_addr_small:
    and eax, 0x3F

.eeprom_read_addr_masked:
    shl eax, 3
    add eax, esi

    mov rcx, [rdi + GBA_CORE.save_ptr]
    test rcx, rcx
    jz .eeprom_read_idle
    movzx eax, byte [rcx + rax]

    mov ecx, edx
    sub ecx, 4
    and ecx, 7
    mov esi, 7
    sub esi, ecx
    mov ecx, esi
    shr eax, cl
    and eax, 1
    jmp .eeprom_read_advance

.eeprom_read_dummy:
    xor eax, eax
    jmp .eeprom_read_advance

.eeprom_read_advance:
    mov edx, [rdi + GBA_CORE.eeprom_read_bit]
    inc edx
    mov [rdi + GBA_CORE.eeprom_read_bit], edx
    ret

.eeprom_read_done_stream:
    mov eax, 1
    ret

.eeprom_read_idle:
    mov eax, 1
    ret

cpu_eeprom_write_bit:
    ; in: rdi=core, edx=value (bit in LSB)
    ; If read stream is active, a write starts a new command.
    cmp dword [rdi + GBA_CORE.eeprom_mode], 1
    jne .eeprom_write_mode_ready
    mov dword [rdi + GBA_CORE.eeprom_mode], 0
    mov dword [rdi + GBA_CORE.eeprom_bit_count], 0

.eeprom_write_mode_ready:
    mov eax, [rdi + GBA_CORE.eeprom_bit_count]
    cmp eax, 96
    jb .eeprom_write_store
    mov dword [rdi + GBA_CORE.eeprom_bit_count], 0
    xor eax, eax

.eeprom_write_store:
    mov ecx, edx
    and ecx, 1
    mov [rdi + GBA_CORE.eeprom_bits + rax], cl
    inc eax
    mov [rdi + GBA_CORE.eeprom_bit_count], eax

    cmp eax, 2
    jb .eeprom_write_ret

    ; Commands must start with leading 1 bit.
    movzx ecx, byte [rdi + GBA_CORE.eeprom_bits + 0]
    cmp ecx, 1
    je .eeprom_prefix_ok
    mov dword [rdi + GBA_CORE.eeprom_bit_count], 0
    jmp .eeprom_write_ret

.eeprom_prefix_ok:
    movzx ecx, byte [rdi + GBA_CORE.eeprom_bits + 1]
    cmp ecx, 1
    je .eeprom_cmd_read
    cmp ecx, 0
    je .eeprom_cmd_write
    mov dword [rdi + GBA_CORE.eeprom_bit_count], 0
    jmp .eeprom_write_ret

.eeprom_cmd_read:
    mov ecx, [rdi + GBA_CORE.eeprom_addr_bits]
    test ecx, ecx
    jnz .eeprom_cmd_read_known
    cmp eax, 9
    je .eeprom_cmd_read_short
    cmp eax, 17
    jne .eeprom_write_ret
    mov ecx, 14
    mov [rdi + GBA_CORE.eeprom_addr_bits], ecx
    jmp .eeprom_cmd_read_ready

.eeprom_cmd_read_short:
    mov ecx, 6
    mov [rdi + GBA_CORE.eeprom_addr_bits], ecx
    jmp .eeprom_cmd_read_ready

.eeprom_cmd_read_known:
    mov edx, ecx
    add edx, 3
    cmp eax, edx
    jne .eeprom_write_ret

.eeprom_cmd_read_ready:
    ; Decode address bits [2 .. 2+addr_bits-1], MSB-first.
    xor eax, eax
    xor edx, edx

.eeprom_read_addr_loop:
    cmp edx, ecx
    jae .eeprom_read_addr_done
    shl eax, 1
    movzx esi, byte [rdi + GBA_CORE.eeprom_bits + rdx + 2]
    and esi, 1
    or eax, esi
    inc edx
    jmp .eeprom_read_addr_loop

.eeprom_read_addr_done:
    cmp ecx, 14
    jne .eeprom_read_addr_mask
    shr eax, 4
    and eax, 0x3FF
    jmp .eeprom_read_store

.eeprom_read_addr_mask:
    and eax, 0x3F

.eeprom_read_store:
    mov [rdi + GBA_CORE.eeprom_read_addr], eax
    mov dword [rdi + GBA_CORE.eeprom_read_bit], 0
    mov dword [rdi + GBA_CORE.eeprom_mode], 1
    mov dword [rdi + GBA_CORE.eeprom_bit_count], 0
    jmp .eeprom_write_ret

.eeprom_cmd_write:
    mov ecx, [rdi + GBA_CORE.eeprom_addr_bits]
    test ecx, ecx
    jnz .eeprom_cmd_write_known
    cmp eax, 73
    je .eeprom_cmd_write_short
    cmp eax, 81
    jne .eeprom_write_ret
    mov ecx, 14
    mov [rdi + GBA_CORE.eeprom_addr_bits], ecx
    jmp .eeprom_cmd_write_ready

.eeprom_cmd_write_short:
    mov ecx, 6
    mov [rdi + GBA_CORE.eeprom_addr_bits], ecx
    jmp .eeprom_cmd_write_ready

.eeprom_cmd_write_known:
    mov edx, ecx
    add edx, 67
    cmp eax, edx
    jne .eeprom_write_ret

.eeprom_cmd_write_ready:
    ; Decode address bits [2 .. 2+addr_bits-1], MSB-first.
    xor r8d, r8d
    xor edx, edx

.eeprom_write_addr_loop:
    cmp edx, ecx
    jae .eeprom_write_addr_done
    shl r8d, 1
    movzx esi, byte [rdi + GBA_CORE.eeprom_bits + rdx + 2]
    and esi, 1
    or r8d, esi
    inc edx
    jmp .eeprom_write_addr_loop

.eeprom_write_addr_done:
    cmp ecx, 14
    jne .eeprom_write_addr_mask
    shr r8d, 4
    and r8d, 0x3FF
    jmp .eeprom_write_data

.eeprom_write_addr_mask:
    and r8d, 0x3F

.eeprom_write_data:
    mov r10, [rdi + GBA_CORE.save_ptr]
    test r10, r10
    jz .eeprom_write_finalize

    mov r9d, r8d
    shl r9d, 3                    ; destination byte offset
    mov r11d, ecx
    add r11d, 2                   ; first data bit index
    xor edx, edx                  ; byte index 0..7

.eeprom_write_byte_loop:
    cmp edx, 8
    jae .eeprom_write_done
    xor r8d, r8d                  ; assembled byte
    xor esi, esi                  ; bit index within byte

.eeprom_write_bit_loop:
    cmp esi, 8
    jae .eeprom_write_store_byte
    shl r8d, 1
    mov ecx, r11d
    mov eax, edx
    shl eax, 3
    add ecx, eax
    add ecx, esi
    movzx eax, byte [rdi + GBA_CORE.eeprom_bits + rcx]
    and eax, 1
    or r8d, eax
    inc esi
    jmp .eeprom_write_bit_loop

.eeprom_write_store_byte:
    mov eax, r9d
    add eax, edx
    mov [r10 + rax], r8b
    inc edx
    jmp .eeprom_write_byte_loop

.eeprom_write_done:
    mov dword [rdi + GBA_CORE.save_dirty], 1

.eeprom_write_finalize:
    mov dword [rdi + GBA_CORE.eeprom_mode], 0
    mov dword [rdi + GBA_CORE.eeprom_bit_count], 0

.eeprom_write_ret:
    ret

cpu_mem_read8:
    ; in:  rdi=core, esi=addr
    ; out: eax=byte (0..255)
    mov eax, esi
    cmp eax, 0x00004000
    jb .bios

    cmp eax, 0x02000000
    jb .zero
    cmp eax, 0x03000000
    jb .ewram

    cmp eax, 0x04000000
    jb .iwram

    cmp eax, 0x04000000
    jb .zero
    cmp eax, 0x04000400
    jb .io

    cmp eax, 0x05000000
    jb .zero
    cmp eax, 0x05000400
    jb .pram

    cmp eax, 0x06000000
    jb .zero
    cmp eax, 0x07000000
    jb .vram

    cmp eax, 0x07000000
    jb .zero
    cmp eax, 0x07000400
    jb .oam

    cmp eax, GBA_ROM_BASE
    jb .zero
    cmp eax, GBA_EEPROM_BASE
    jb .rom
    cmp eax, GBA_SRAM_BASE
    jb .eeprom

    cmp eax, GBA_SRAM_BASE + GBA_SAVE_SIZE
    jb .sram
    jmp .zero

.bios:
    mov rcx, [rdi + GBA_CORE.bios_ptr]
    test rcx, rcx
    jz .zero
    mov rdx, [rdi + GBA_CORE.bios_size]
    cmp rax, rdx
    jae .zero
    movzx eax, byte [rcx + rax]
    ret

.ewram:
    sub eax, 0x02000000
    and eax, (GBA_EWRAM_SIZE - 1)
    movzx eax, byte [rdi + GBA_CORE.ewram + rax]
    ret

.iwram:
    sub eax, 0x03000000
    and eax, (GBA_IWRAM_SIZE - 1)
    movzx eax, byte [rdi + GBA_CORE.iwram + rax]
    ret

.io:
    sub eax, 0x04000000
    movzx eax, byte [rdi + GBA_CORE.io + rax]
    ret

.pram:
    sub eax, 0x05000000
    movzx eax, byte [rdi + GBA_CORE.pram + rax]
    ret

.vram:
    sub eax, 0x06000000
    and eax, 0x1FFFF
    cmp eax, 0x18000
    jb .vram_read_index_ready
    sub eax, 0x8000
.vram_read_index_ready:
    movzx eax, byte [rdi + GBA_CORE.vram + rax]
    ret

.oam:
    sub eax, 0x07000000
    movzx eax, byte [rdi + GBA_CORE.oam + rax]
    ret

.rom:
    mov rcx, [rdi + GBA_CORE.rom_size]
    test rcx, rcx
    jz .zero
    sub eax, GBA_ROM_BASE
    xor edx, edx
    div ecx
    mov rcx, [rdi + GBA_CORE.rom_ptr]
    movzx eax, byte [rcx + rdx]
    ret

.eeprom:
    call cpu_eeprom_read_bit
    ret

.sram:
    mov rcx, [rdi + GBA_CORE.save_ptr]
    test rcx, rcx
    jz .zero
    sub eax, GBA_SRAM_BASE
    movzx eax, byte [rcx + rax]
    ret

.zero:
    xor eax, eax
    ret

cpu_mem_read16:
    ; in: rdi=core, esi=addr
    ; out: eax=word
    mov eax, esi
    cmp eax, GBA_EEPROM_BASE
    jb .read16_normal
    cmp eax, GBA_SRAM_BASE
    jb .read16_eeprom

.read16_normal:
    push rbx
    push r12
    mov ebx, esi

    call cpu_mem_read8
    mov r12d, eax

    mov esi, ebx
    add esi, 1
    call cpu_mem_read8

    shl eax, 8
    or eax, r12d
    pop r12
    pop rbx
    ret

.read16_eeprom:
    call cpu_eeprom_read_bit
    ret

cpu_mem_read32:
    ; in: rdi=core, esi=addr
    ; out: eax=dword
    mov eax, esi
    cmp eax, GBA_EEPROM_BASE
    jb .read32_normal
    cmp eax, GBA_SRAM_BASE
    jb .read32_eeprom

.read32_normal:
    push rbx
    push r12

    mov ebx, esi

    call cpu_mem_read8
    mov r12d, eax

    mov esi, ebx
    add esi, 1
    call cpu_mem_read8
    shl eax, 8
    or r12d, eax

    mov esi, ebx
    add esi, 2
    call cpu_mem_read8
    shl eax, 16
    or r12d, eax

    mov esi, ebx
    add esi, 3
    call cpu_mem_read8
    shl eax, 24
    or eax, r12d

    pop r12
    pop rbx
    ret

.read32_eeprom:
    call cpu_eeprom_read_bit
    ret

cpu_mem_write8:
    ; in: rdi=core, esi=addr, edx=value
    mov eax, esi
    cmp eax, 0x02000000
    jb .done
    cmp eax, 0x03000000
    jb .ewram

    cmp eax, 0x04000000
    jb .iwram

    cmp eax, 0x04000000
    jb .done
    cmp eax, 0x04000400
    jb .io

    cmp eax, 0x05000000
    jb .done
    cmp eax, 0x05000400
    jb .pram

    cmp eax, 0x06000000
    jb .done
    cmp eax, 0x07000000
    jb .vram

    cmp eax, 0x07000000
    jb .done
    cmp eax, 0x07000400
    jb .oam

    cmp eax, GBA_EEPROM_BASE
    jb .check_sram
    cmp eax, GBA_SRAM_BASE
    jb .eeprom

.check_sram:
    cmp eax, GBA_SRAM_BASE
    jb .done
    cmp eax, GBA_SRAM_BASE + GBA_SAVE_SIZE
    jb .sram
    jmp .done

.ewram:
    sub eax, 0x02000000
    and eax, (GBA_EWRAM_SIZE - 1)
    mov [rdi + GBA_CORE.ewram + rax], dl
    ret

.iwram:
    sub eax, 0x03000000
    and eax, (GBA_IWRAM_SIZE - 1)
    mov [rdi + GBA_CORE.iwram + rax], dl
    ret

.io:
    mov r8d, edx
    sub eax, 0x04000000
    cmp eax, IO_KEYINPUT
    je .done
    cmp eax, IO_KEYINPUT + 1
    je .done

    ; IF is write-1-to-clear on GBA hardware.
    cmp eax, IO_IF
    je .io_if_low
    cmp eax, IO_IF + 1
    je .io_if_high

    ; Track timer reload values when TMxCNT_L bytes are written.
    cmp eax, IO_TM0CNT_L
    jb .io_store
    cmp eax, IO_TM3CNT_H + 1
    ja .io_store

    mov ecx, eax
    sub ecx, IO_TM0CNT_L
    mov r9d, ecx
    and r9d, 3
    cmp r9d, 2
    jae .io_store
    shr ecx, 2
    lea r10, [rdi + GBA_CORE.timer_reload + rcx * 2]
    cmp r9d, 0
    jne .tm_reload_high
    mov [r10], r8b
    jmp .io_store

.tm_reload_high:
    mov [r10 + 1], r8b
    jmp .io_store

.io_if_low:
    movzx ecx, word [rdi + GBA_CORE.io + IO_IF]
    mov edx, r8d
    and edx, 0xFF
    not edx
    and ecx, edx
    mov [rdi + GBA_CORE.io + IO_IF], cx
    jmp .io_irq_latch_sync

.io_if_high:
    movzx ecx, word [rdi + GBA_CORE.io + IO_IF]
    mov edx, r8d
    and edx, 0xFF
    shl edx, 8
    not edx
    and ecx, edx
    mov [rdi + GBA_CORE.io + IO_IF], cx
    jmp .io_irq_latch_sync

.io_irq_latch_sync:
    ; BIOS software IRQ latches are sticky until software clears them.
    ; Reflect pending IE&IF bits, but do not auto-clear here.
    movzx eax, word [rdi + GBA_CORE.io + IO_IE]
    movzx edx, word [rdi + GBA_CORE.io + IO_IF]
    and eax, edx
    test eax, eax
    jz .io_irq_latch_done
    movzx ecx, word [rdi + GBA_CORE.iwram + 0x1178]
    movzx edx, word [rdi + GBA_CORE.iwram + 0x7FF8]
    or ecx, eax
    or edx, eax
    mov [rdi + GBA_CORE.iwram + 0x1178], cx
    mov [rdi + GBA_CORE.iwram + 0x7FF8], dx
.io_irq_latch_done:
    ret

.io_store:
    mov [rdi + GBA_CORE.io + rax], r8b
    ret

.pram:
    sub eax, 0x05000000
    and eax, 0x3FE
    mov [rdi + GBA_CORE.pram + rax], dl
    mov [rdi + GBA_CORE.pram + rax + 1], dl
    ret

.vram:
    mov r8d, edx
    sub eax, 0x06000000
    and eax, 0x1FFFF
    cmp eax, 0x18000
    jb .vram_write_index_ready
    sub eax, 0x8000
.vram_write_index_ready:
    ; Align to halfword while preserving all mirrored VRAM address bits.
    and eax, 0x1FFFE
    mov [rdi + GBA_CORE.vram + rax], r8b
    mov [rdi + GBA_CORE.vram + rax + 1], r8b
    ret

.oam:
    sub eax, 0x07000000
    and eax, 0x3FE
    mov [rdi + GBA_CORE.oam + rax], dl
    mov [rdi + GBA_CORE.oam + rax + 1], dl
    ret

.eeprom:
    call cpu_eeprom_write_bit
    ret

.sram:
    sub eax, GBA_SRAM_BASE
    mov rcx, [rdi + GBA_CORE.save_ptr]
    test rcx, rcx
    jz .done
    mov [rcx + rax], dl
    mov dword [rdi + GBA_CORE.save_dirty], 1

.done:
    ret

cpu_mem_write16:
    ; in: rdi=core, esi=addr, edx=value
    mov eax, esi

    ; EEPROM serial bus on 0x0Dxxxxxx, one bit per halfword transfer.
    cmp eax, GBA_EEPROM_BASE
    jb .write16_not_eeprom
    cmp eax, GBA_SRAM_BASE
    jb .write16_eeprom

.write16_not_eeprom:

    ; PRAM halfword write (byte writes are handled separately).
    cmp eax, 0x05000000
    jb .write16_fallback
    cmp eax, 0x05000400
    jb .write16_pram

    ; VRAM halfword write with 128KB mirroring.
    cmp eax, 0x06000000
    jb .write16_fallback
    cmp eax, 0x07000000
    jb .write16_vram

    ; OAM halfword write.
    cmp eax, 0x07000000
    jb .write16_fallback
    cmp eax, 0x07000400
    jb .write16_oam

.write16_fallback:
    push rbx
    push r12
    mov ebx, esi
    mov r12d, edx

    mov esi, ebx
    mov edx, r12d
    call cpu_mem_write8

    mov edx, r12d
    shr edx, 8
    mov esi, ebx
    add esi, 1
    call cpu_mem_write8

    pop r12
    pop rbx
    ret

.write16_pram:
    sub eax, 0x05000000
    and eax, 0x3FE
    mov [rdi + GBA_CORE.pram + rax], dl
    mov [rdi + GBA_CORE.pram + rax + 1], dh
    ret

.write16_vram:
    sub eax, 0x06000000
    and eax, 0x1FFFF
    cmp eax, 0x18000
    jb .write16_vram_mapped
    sub eax, 0x8000
.write16_vram_mapped:
    ; Align to halfword while preserving all mirrored VRAM address bits.
    and eax, 0x1FFFE
    mov [rdi + GBA_CORE.vram + rax], dl
    mov [rdi + GBA_CORE.vram + rax + 1], dh
    ret

.write16_oam:
    sub eax, 0x07000000
    and eax, 0x3FE
    mov [rdi + GBA_CORE.oam + rax], dl
    mov [rdi + GBA_CORE.oam + rax + 1], dh
    ret

.write16_eeprom:
    call cpu_eeprom_write_bit
    ret

cpu_mem_write32:
    ; in: rdi=core, esi=addr, edx=value
    mov eax, esi
    cmp eax, GBA_EEPROM_BASE
    jb .write32_normal
    cmp eax, GBA_SRAM_BASE
    jb .write32_eeprom

.write32_normal:
    push rbx
    push r12

    mov ebx, esi
    mov r12d, edx

    mov edx, r12d
    mov esi, ebx
    call cpu_mem_write16

    mov edx, r12d
    shr edx, 16
    mov esi, ebx
    add esi, 2
    call cpu_mem_write16

    pop r12
    pop rbx
    ret

.write32_eeprom:
    call cpu_eeprom_write_bit
    ret

gba_mem_read8:
    jmp cpu_mem_read8

gba_mem_read16:
    jmp cpu_mem_read16

gba_mem_read32:
    jmp cpu_mem_read32

gba_mem_write8:
    jmp cpu_mem_write8

gba_mem_write16:
    jmp cpu_mem_write16

gba_mem_write32:
    jmp cpu_mem_write32

; -----------------------------------------------------------------------------
; Helpers: flags / condition checks
; -----------------------------------------------------------------------------

cpu_update_nz:
    ; in: eax=result, rbx=core
    ; preserves C/V
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    and ecx, ~(CPSR_N | CPSR_Z)

    test eax, eax
    jnz .nz_not_zero
    or ecx, CPSR_Z

.nz_not_zero:
    test eax, 0x80000000
    jz .nz_store
    or ecx, CPSR_N

.nz_store:
    mov [rbx + GBA_CORE.cpu_cpsr], ecx
    ret

cpu_set_flags_add:
    ; in: ecx=a, edx=b, eax=result, rbx=core
    mov r8d, [rbx + GBA_CORE.cpu_cpsr]
    and r8d, ~(CPSR_N | CPSR_Z | CPSR_C | CPSR_V)

    ; Carry
    mov r9d, ecx
    add r9d, edx
    setc r10b
    test r10b, r10b
    jz .no_c
    or r8d, CPSR_C

.no_c:
    ; Overflow
    mov r9d, ecx
    add r9d, edx
    seto r10b
    test r10b, r10b
    jz .no_v
    or r8d, CPSR_V

.no_v:
    test eax, eax
    jnz .no_z
    or r8d, CPSR_Z

.no_z:
    test eax, 0x80000000
    jz .store
    or r8d, CPSR_N

.store:
    mov [rbx + GBA_CORE.cpu_cpsr], r8d
    ret

cpu_set_flags_sub:
    ; in: ecx=a, edx=b, eax=result, rbx=core
    mov r8d, [rbx + GBA_CORE.cpu_cpsr]
    and r8d, ~(CPSR_N | CPSR_Z | CPSR_C | CPSR_V)

    mov r9d, ecx
    sub r9d, edx
    setnc r10b               ; no borrow => C set
    test r10b, r10b
    jz .no_c
    or r8d, CPSR_C

.no_c:
    mov r9d, ecx
    sub r9d, edx
    seto r10b
    test r10b, r10b
    jz .no_v
    or r8d, CPSR_V

.no_v:
    test eax, eax
    jnz .no_z
    or r8d, CPSR_Z

.no_z:
    test eax, 0x80000000
    jz .store
    or r8d, CPSR_N

.store:
    mov [rbx + GBA_CORE.cpu_cpsr], r8d
    ret

cpu_arm_cond_pass:
    ; in: edi=cond, esi=cpsr
    ; out: al=0/1
    mov eax, 0
    cmp edi, 14
    je .true
    cmp edi, 15
    je .false

    ; Precompute bits.
    mov ecx, esi
    and ecx, CPSR_Z
    mov edx, esi
    and edx, CPSR_C
    mov r8d, esi
    and r8d, CPSR_N
    mov r9d, esi
    and r9d, CPSR_V

    cmp edi, 0
    je .eq
    cmp edi, 1
    je .ne
    cmp edi, 2
    je .cs
    cmp edi, 3
    je .cc
    cmp edi, 4
    je .mi
    cmp edi, 5
    je .pl
    cmp edi, 6
    je .vs
    cmp edi, 7
    je .vc
    cmp edi, 8
    je .hi
    cmp edi, 9
    je .ls
    cmp edi, 10
    je .ge
    cmp edi, 11
    je .lt
    cmp edi, 12
    je .gt
    cmp edi, 13
    je .le
    jmp .false

.eq:
    test ecx, ecx
    jnz .true
    jmp .false

.ne:
    test ecx, ecx
    jz .true
    jmp .false

.cs:
    test edx, edx
    jnz .true
    jmp .false

.cc:
    test edx, edx
    jz .true
    jmp .false

.mi:
    test r8d, r8d
    jnz .true
    jmp .false

.pl:
    test r8d, r8d
    jz .true
    jmp .false

.vs:
    test r9d, r9d
    jnz .true
    jmp .false

.vc:
    test r9d, r9d
    jz .true
    jmp .false

.hi:
    test edx, edx
    jz .false
    test ecx, ecx
    jz .true
    jmp .false

.ls:
    test edx, edx
    jz .true
    test ecx, ecx
    jnz .true
    jmp .false

.ge:
    xor eax, eax
    test r8d, r8d
    setnz al
    xor edx, edx
    test r9d, r9d
    setnz dl
    cmp al, dl
    je .true
    jmp .false

.lt:
    xor eax, eax
    test r8d, r8d
    setnz al
    xor edx, edx
    test r9d, r9d
    setnz dl
    cmp al, dl
    jne .true
    jmp .false

.gt:
    test ecx, ecx
    jnz .false
    xor eax, eax
    test r8d, r8d
    setnz al
    xor edx, edx
    test r9d, r9d
    setnz dl
    cmp al, dl
    je .true
    jmp .false

.le:
    test ecx, ecx
    jnz .true
    xor eax, eax
    test r8d, r8d
    setnz al
    xor edx, edx
    test r9d, r9d
    setnz dl
    cmp al, dl
    jne .true
    jmp .false

.true:
    mov eax, 1
    ret

.false:
    xor eax, eax
    ret

cpu_irq_pending:
    ; in: rbx=core
    ; out: eax=0/1
    movzx eax, word [rbx + GBA_CORE.io + IO_IME]
    test eax, 1
    jz .no
    movzx ecx, word [rbx + GBA_CORE.io + IO_IE]
    movzx edx, word [rbx + GBA_CORE.io + IO_IF]
    and ecx, edx
    test ecx, ecx
    jz .no
    mov eax, 1
    ret
.no:
    xor eax, eax
    ret

cpu_switch_mode:
    ; in: rbx=core, ecx=new mode bits[4:0]
    ; swaps banked r13/r14 for USR/SYS, IRQ, and SVC modes.
    mov eax, [rbx + GBA_CORE.cpu_cpsr]
    and eax, 0x1F
    mov edx, ecx
    and edx, 0x1F
    cmp eax, edx
    je .done

    ; Save old mode bank.
    cmp eax, 0x12
    je .save_old_irq
    cmp eax, 0x13
    je .save_old_svc

    ; Save USR/SYS bank.
    mov ecx, [rbx + GBA_CORE.cpu_regs + (13 * 4)]
    mov [rbx + GBA_CORE.bank_r13_usr], ecx
    mov ecx, [rbx + GBA_CORE.cpu_regs + (14 * 4)]
    mov [rbx + GBA_CORE.bank_r14_usr], ecx
    jmp .restore_new_mode

.save_old_irq:
    ; Save IRQ bank.
    mov ecx, [rbx + GBA_CORE.cpu_regs + (13 * 4)]
    mov [rbx + GBA_CORE.bank_r13_irq], ecx
    mov ecx, [rbx + GBA_CORE.cpu_regs + (14 * 4)]
    mov [rbx + GBA_CORE.bank_r14_irq], ecx
    jmp .restore_new_mode

.save_old_svc:
    ; Save SVC bank.
    mov ecx, [rbx + GBA_CORE.cpu_regs + (13 * 4)]
    mov [rbx + GBA_CORE.bank_r13_svc], ecx
    mov ecx, [rbx + GBA_CORE.cpu_regs + (14 * 4)]
    mov [rbx + GBA_CORE.bank_r14_svc], ecx

.restore_new_mode:
    cmp edx, 0x12
    je .restore_irq_bank
    cmp edx, 0x13
    je .restore_svc_bank

    ; Restore USR/SYS bank.
    mov ecx, [rbx + GBA_CORE.bank_r13_usr]
    mov [rbx + GBA_CORE.cpu_regs + (13 * 4)], ecx
    mov ecx, [rbx + GBA_CORE.bank_r14_usr]
    mov [rbx + GBA_CORE.cpu_regs + (14 * 4)], ecx
    jmp .done

.restore_irq_bank:
    mov ecx, [rbx + GBA_CORE.bank_r13_irq]
    mov [rbx + GBA_CORE.cpu_regs + (13 * 4)], ecx
    mov ecx, [rbx + GBA_CORE.bank_r14_irq]
    mov [rbx + GBA_CORE.cpu_regs + (14 * 4)], ecx
    jmp .done

.restore_svc_bank:
    mov ecx, [rbx + GBA_CORE.bank_r13_svc]
    mov [rbx + GBA_CORE.cpu_regs + (13 * 4)], ecx
    mov ecx, [rbx + GBA_CORE.bank_r14_svc]
    mov [rbx + GBA_CORE.cpu_regs + (14 * 4)], ecx

.done:
    ret

cpu_enter_irq:
    ; in: rbx=core
    ; Enter IRQ exception mode and branch to vector 0x18.
    mov r10, [rbx + GBA_CORE.bios_ptr]
    test r10, r10
    jnz .irq_with_bios

    ; No BIOS vector code: avoid jumping into 0x18 and keep game code running.
    or dword [rbx + GBA_CORE.cpu_cpsr], CPSR_I
    mov dword [rbx + GBA_CORE.halted], 0
    ret

.irq_with_bios:
    mov eax, [rbx + GBA_CORE.cpu_cpsr]
    mov [rbx + GBA_CORE.cpu_spsr], eax

    mov edx, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    ; BIOS returns with SUBS PC, LR, #4, so LR_irq must be next_pc + 4.
    add edx, 4
    mov r9d, edx
    mov ecx, 0x12
    call cpu_switch_mode
    mov [rbx + GBA_CORE.cpu_regs + (14 * 4)], r9d

    mov eax, [rbx + GBA_CORE.cpu_cpsr]
    and eax, ~(CPSR_T | 0x1F)
    or eax, (CPSR_I | 0x12)
    mov [rbx + GBA_CORE.cpu_cpsr], eax

    mov dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 0x00000018
    mov dword [rbx + GBA_CORE.halted], 0
    ret

; -----------------------------------------------------------------------------
; Helpers: SWI HLE
; -----------------------------------------------------------------------------

cpu_swi_div_core:
    ; in: rbx=core, ecx=numerator, edx=denominator
    ; out: r0/r1/r3 updated
    test edx, edx
    jnz .div_ok
    mov dword [rbx + GBA_CORE.cpu_regs + (0 * 4)], 0
    mov dword [rbx + GBA_CORE.cpu_regs + (1 * 4)], 0
    mov dword [rbx + GBA_CORE.cpu_regs + (3 * 4)], 0
    ret

.div_ok:
    ; x86 idiv traps on INT_MIN / -1; model ARM/BIOS behavior instead.
    cmp ecx, 0x80000000
    jne .div_do
    cmp edx, -1
    jne .div_do
    mov eax, 0x80000000
    xor edx, edx
    jmp .div_store

.div_do:
    mov r8d, edx
    mov eax, ecx
    cdq
    idiv r8d

.div_store:
    mov [rbx + GBA_CORE.cpu_regs + (0 * 4)], eax
    mov [rbx + GBA_CORE.cpu_regs + (1 * 4)], edx
    mov ecx, eax
    test ecx, ecx
    jns .div_abs_store
    neg ecx

.div_abs_store:
    mov [rbx + GBA_CORE.cpu_regs + (3 * 4)], ecx
    ret

cpu_swi_sqrt:
    ; in: rbx=core, r0=input
    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    movd xmm0, eax
    cvtdq2ps xmm0, xmm0
    sqrtss xmm0, xmm0
    cvttss2si eax, xmm0
    mov [rbx + GBA_CORE.cpu_regs + (0 * 4)], eax
    ret

cpu_swi_cpuset:
    ; r0=src, r1=dst, r2=control
    push rbp
    mov rbp, rsp
    sub rsp, 32

    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    mov [rbp-4], eax      ; src
    mov eax, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    mov [rbp-8], eax      ; dst
    mov eax, [rbx + GBA_CORE.cpu_regs + (2 * 4)]
    mov [rbp-12], eax     ; control

    mov ecx, eax
    and ecx, 0x001FFFFF
    test ecx, ecx
    jz .done
    mov [rbp-16], ecx     ; count

    mov ecx, eax
    and ecx, (1 << 26)    ; 32-bit mode?
    mov [rbp-20], ecx

    mov ecx, eax
    and ecx, (1 << 24)    ; fixed source?
    mov [rbp-24], ecx

.loop:
    mov ecx, [rbp-16]
    test ecx, ecx
    jz .done

    mov eax, [rbp-20]
    test eax, eax
    jz .copy16

    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read32
    mov edx, eax
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write32

    mov eax, [rbp-24]
    test eax, eax
    jnz .cpuset_dst_inc32
    add dword [rbp-4], 4

.cpuset_dst_inc32:
    add dword [rbp-8], 4
    jmp .cpuset_dec

.copy16:
    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read16
    mov edx, eax
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write16

    mov eax, [rbp-24]
    test eax, eax
    jnz .cpuset_dst_inc16
    add dword [rbp-4], 2

.cpuset_dst_inc16:
    add dword [rbp-8], 2

.cpuset_dec:
    sub dword [rbp-16], 1
    jmp .loop

.done:
    leave
    ret

cpu_swi_cpufastset:
    ; r0=src, r1=dst, r2=control
    push rbp
    mov rbp, rsp
    sub rsp, 32

    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    mov [rbp-4], eax      ; src
    mov eax, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    mov [rbp-8], eax      ; dst
    mov eax, [rbx + GBA_CORE.cpu_regs + (2 * 4)]
    mov [rbp-12], eax     ; control

    mov ecx, eax
    and ecx, 0x001FFFFF   ; words
    test ecx, ecx
    jz .done
    mov [rbp-16], ecx

    mov ecx, eax
    and ecx, (1 << 24)    ; fixed source
    mov [rbp-20], ecx

.loop:
    mov ecx, [rbp-16]
    test ecx, ecx
    jz .done

    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read32
    mov edx, eax
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write32

    mov eax, [rbp-20]
    test eax, eax
    jnz .dst_inc
    add dword [rbp-4], 4

.dst_inc:
    add dword [rbp-8], 4
    sub dword [rbp-16], 1
    jmp .loop

.done:
    leave
    ret

cpu_swi_lz77:
    ; r0=src, r1=dst, ecx=output width (1=WRAM, 2=VRAM)
    push rbp
    mov rbp, rsp
    sub rsp, 80

    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    mov [rbp-4], eax      ; src
    mov eax, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    mov [rbp-8], eax      ; dst
    mov [rbp-44], ecx     ; output width
    mov dword [rbp-52], 0 ; pending low-byte flag (for width=2)

    ; header
    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read32
    add dword [rbp-4], 4

    mov ecx, eax
    shr ecx, 8
    mov [rbp-12], ecx     ; decompressed size

    mov dword [rbp-16], 0 ; produced

.lz_group:
    mov eax, [rbp-16]
    cmp eax, [rbp-12]
    jae .lz_done

    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read8
    add dword [rbp-4], 1
    mov [rbp-20], eax     ; flags
    mov dword [rbp-24], 0 ; bit index

.lz_bit:
    mov eax, [rbp-16]
    cmp eax, [rbp-12]
    jae .lz_done

    mov ecx, [rbp-24]
    cmp ecx, 8
    jae .lz_group

    mov eax, [rbp-20]
    mov ecx, [rbp-24]
    mov edx, 7
    sub edx, ecx
    mov ecx, edx
    shr eax, cl
    and eax, 1
    test eax, eax
    jnz .lz_compressed

    ; literal byte
    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read8
    add dword [rbp-4], 1

    ; Emit one decompressed byte (eax).
    cmp dword [rbp-44], 2
    jne .lz_emit_literal_w1
    cmp dword [rbp-52], 0
    jne .lz_emit_literal_hi
    mov [rbp-48], eax
    mov dword [rbp-52], 1
    jmp .lz_emit_literal_done

.lz_emit_literal_hi:
    mov edx, [rbp-48]
    mov ecx, eax
    shl ecx, 8
    or edx, ecx
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write16
    add dword [rbp-8], 2
    mov dword [rbp-52], 0
    jmp .lz_emit_literal_done

.lz_emit_literal_w1:
    mov edx, eax
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write8
    add dword [rbp-8], 1

.lz_emit_literal_done:
    add dword [rbp-16], 1
    add dword [rbp-24], 1
    jmp .lz_bit

.lz_compressed:
    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read8
    mov [rbp-28], eax
    add dword [rbp-4], 1

    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read8
    mov [rbp-32], eax
    add dword [rbp-4], 1

    mov eax, [rbp-28]
    shr eax, 4
    and eax, 0xF
    add eax, 3
    mov [rbp-36], eax     ; len

    mov eax, [rbp-28]
    and eax, 0xF
    shl eax, 8
    mov ecx, [rbp-32]
    or eax, ecx
    add eax, 1
    mov [rbp-40], eax     ; disp

.lz_copy_loop:
    mov eax, [rbp-36]
    test eax, eax
    jz .lz_copy_done

    mov eax, [rbp-16]
    cmp eax, [rbp-12]
    jae .lz_done

    mov esi, [rbp-8]
    sub esi, [rbp-40]
    mov rdi, rbx
    call cpu_mem_read8

    ; Emit one decompressed byte (eax).
    cmp dword [rbp-44], 2
    jne .lz_emit_copy_w1
    cmp dword [rbp-52], 0
    jne .lz_emit_copy_hi
    mov [rbp-48], eax
    mov dword [rbp-52], 1
    jmp .lz_emit_copy_done

.lz_emit_copy_hi:
    mov edx, [rbp-48]
    mov ecx, eax
    shl ecx, 8
    or edx, ecx
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write16
    add dword [rbp-8], 2
    mov dword [rbp-52], 0
    jmp .lz_emit_copy_done

.lz_emit_copy_w1:
    mov edx, eax
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write8
    add dword [rbp-8], 1

.lz_emit_copy_done:
    add dword [rbp-16], 1
    sub dword [rbp-36], 1
    jmp .lz_copy_loop

.lz_copy_done:
    add dword [rbp-24], 1
    jmp .lz_bit

.lz_done:
    ; VRAM variant writes halfwords; pad odd trailing byte with zero high-byte.
    cmp dword [rbp-44], 2
    jne .lz_ret
    cmp dword [rbp-52], 0
    je .lz_ret
    mov edx, [rbp-48]
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write16
    add dword [rbp-8], 2

.lz_ret:
    mov eax, [rbp-4]
    mov [rbx + GBA_CORE.cpu_regs + (0 * 4)], eax
    mov eax, [rbp-8]
    mov [rbx + GBA_CORE.cpu_regs + (1 * 4)], eax
    mov dword [rbx + GBA_CORE.cpu_regs + (3 * 4)], 0
    leave
    ret

cpu_swi_rl:
    ; r0=src, r1=dst, ecx=output width (1=WRAM, 2=VRAM)
    push rbp
    mov rbp, rsp
    sub rsp, 96

    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    mov [rbp-4], eax      ; src
    mov eax, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    mov [rbp-8], eax      ; dst
    mov [rbp-12], ecx     ; output width
    mov dword [rbp-16], 0 ; pending low-byte flag (width=2)

    ; Header: [31:8]=output size
    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read32
    add dword [rbp-4], 4
    mov ecx, eax
    shr ecx, 8
    mov [rbp-20], ecx     ; remaining output bytes

.rl_block_loop:
    cmp dword [rbp-20], 0
    jle .rl_done

    ; control byte
    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read8
    add dword [rbp-4], 1
    mov [rbp-24], eax

    test eax, 0x80
    jz .rl_raw_block

    ; compressed run: next byte repeated (ctrl&0x7F)+3 times
    mov ecx, [rbp-24]
    and ecx, 0x7F
    add ecx, 3
    mov [rbp-28], ecx     ; run_len

    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read8
    add dword [rbp-4], 1
    mov [rbp-32], eax     ; run_value

.rl_comp_emit_loop:
    cmp dword [rbp-28], 0
    jle .rl_block_loop
    cmp dword [rbp-20], 0
    jle .rl_done

    mov eax, [rbp-32]
    jmp .rl_emit_byte

.rl_raw_block:
    ; raw run: (ctrl&0x7F)+1 literal bytes
    mov ecx, [rbp-24]
    and ecx, 0x7F
    add ecx, 1
    mov [rbp-28], ecx     ; run_len

.rl_raw_emit_loop:
    cmp dword [rbp-28], 0
    jle .rl_block_loop
    cmp dword [rbp-20], 0
    jle .rl_done

    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read8
    add dword [rbp-4], 1

.rl_emit_byte:
    ; Emit one output byte in eax.
    cmp dword [rbp-12], 2
    jne .rl_emit_w1

    cmp dword [rbp-16], 0
    jne .rl_emit_hi
    mov [rbp-36], eax
    mov dword [rbp-16], 1
    jmp .rl_emit_done

.rl_emit_hi:
    mov edx, [rbp-36]
    mov ecx, eax
    shl ecx, 8
    or edx, ecx
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write16
    add dword [rbp-8], 2
    mov dword [rbp-16], 0
    jmp .rl_emit_done

.rl_emit_w1:
    mov edx, eax
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write8
    add dword [rbp-8], 1

.rl_emit_done:
    sub dword [rbp-20], 1
    sub dword [rbp-28], 1

    test dword [rbp-24], 0x80
    jnz .rl_comp_emit_loop
    jmp .rl_raw_emit_loop

.rl_done:
    ; VRAM variant writes halfwords; pad odd trailing byte with zero high-byte.
    cmp dword [rbp-12], 2
    jne .rl_ret
    cmp dword [rbp-16], 0
    je .rl_ret
    mov edx, [rbp-36]
    mov rdi, rbx
    mov esi, [rbp-8]
    call cpu_mem_write16
    add dword [rbp-8], 2

.rl_ret:
    mov eax, [rbp-4]
    mov [rbx + GBA_CORE.cpu_regs + (0 * 4)], eax
    mov eax, [rbp-8]
    mov [rbx + GBA_CORE.cpu_regs + (1 * 4)], eax
    mov dword [rbx + GBA_CORE.cpu_regs + (3 * 4)], 0
    leave
    ret

cpu_swi_huffman:
    ; r0=src, r1=dst
    push rbp
    mov rbp, rsp
    sub rsp, 112

    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    and eax, 0xFFFFFFFC
    mov [rbp-4], eax      ; src
    mov eax, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    mov [rbp-8], eax      ; dst

    ; header: [31:8]=remaining bytes, [3:0]=bits per symbol
    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read32
    mov ecx, eax
    shr ecx, 8
    mov [rbp-12], ecx     ; remaining
    and eax, 0xF
    test eax, eax
    jnz .huff_bits_ok
    mov eax, 8
.huff_bits_ok:
    mov [rbp-16], eax     ; bits

    ; only aligned output widths are supported (matching BIOS behavior here)
    mov ecx, 32
    xor edx, edx
    div dword [rbp-16]
    test edx, edx
    jnz .huff_ret
    cmp dword [rbp-16], 1
    je .huff_ret

    ; tree size and base
    mov rdi, rbx
    mov esi, [rbp-4]
    add esi, 4
    call cpu_mem_read8
    mov ecx, eax
    lea ecx, [ecx * 2 + 1]
    mov [rbp-20], ecx     ; tree_size

    mov eax, [rbp-4]
    add eax, 5
    mov [rbp-24], eax     ; tree_base

    mov eax, [rbp-4]
    add eax, 5
    add eax, [rbp-20]
    mov [rbp-4], eax      ; src points to bitstream

    mov eax, [rbp-24]
    mov [rbp-28], eax     ; node_ptr
    mov rdi, rbx
    mov esi, [rbp-28]
    call cpu_mem_read8
    mov [rbp-32], eax     ; node byte
    mov dword [rbp-36], 0 ; block accumulator
    mov dword [rbp-40], 0 ; bits_seen

.huff_stream_loop:
    cmp dword [rbp-12], 0
    jle .huff_done

    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read32
    mov [rbp-44], eax     ; bitstream
    add dword [rbp-4], 4
    mov dword [rbp-48], 32 ; bits_remaining

.huff_bit_loop:
    cmp dword [rbp-48], 0
    jle .huff_stream_loop
    cmp dword [rbp-12], 0
    jle .huff_done

    ; next = (node_ptr & ~1) + (offset*2) + 2
    mov eax, [rbp-28]
    and eax, 0xFFFFFFFE
    mov ecx, [rbp-32]
    and ecx, 0x3F
    lea eax, [eax + ecx * 2 + 2]
    mov [rbp-52], eax     ; next

    ; branch by next bit (MSB first)
    mov eax, [rbp-44]
    test eax, 0x80000000
    jz .huff_go_left

    ; right
    mov eax, [rbp-32]
    test eax, 0x40
    jz .huff_descend_right
    mov rdi, rbx
    mov esi, [rbp-52]
    add esi, 1
    call cpu_mem_read8
    mov [rbp-56], eax     ; readBits
    jmp .huff_symbol_ready

.huff_descend_right:
    mov eax, [rbp-52]
    add eax, 1
    mov [rbp-28], eax
    mov rdi, rbx
    mov esi, eax
    call cpu_mem_read8
    mov [rbp-32], eax
    jmp .huff_shift_next

.huff_go_left:
    mov eax, [rbp-32]
    test eax, 0x80
    jz .huff_descend_left
    mov rdi, rbx
    mov esi, [rbp-52]
    call cpu_mem_read8
    mov [rbp-56], eax     ; readBits
    jmp .huff_symbol_ready

.huff_descend_left:
    mov eax, [rbp-52]
    mov [rbp-28], eax
    mov rdi, rbx
    mov esi, eax
    call cpu_mem_read8
    mov [rbp-32], eax
    jmp .huff_shift_next

.huff_symbol_ready:
    ; block |= (readBits & ((1<<bits)-1)) << bits_seen
    mov eax, 1
    mov ecx, [rbp-16]
    shl eax, cl
    dec eax
    and eax, [rbp-56]
    mov ecx, [rbp-40]
    shl eax, cl
    or [rbp-36], eax

    mov eax, [rbp-40]
    add eax, [rbp-16]
    mov [rbp-40], eax

    ; reset node to root
    mov eax, [rbp-24]
    mov [rbp-28], eax
    mov rdi, rbx
    mov esi, eax
    call cpu_mem_read8
    mov [rbp-32], eax

    cmp dword [rbp-40], 32
    jne .huff_shift_next

    mov rdi, rbx
    mov esi, [rbp-8]
    mov edx, [rbp-36]
    call cpu_mem_write32
    add dword [rbp-8], 4
    sub dword [rbp-12], 4
    mov dword [rbp-36], 0
    mov dword [rbp-40], 0

.huff_shift_next:
    shl dword [rbp-44], 1
    sub dword [rbp-48], 1
    jmp .huff_bit_loop

.huff_done:
    mov eax, [rbp-4]
    mov [rbx + GBA_CORE.cpu_regs + (0 * 4)], eax
    mov eax, [rbp-8]
    mov [rbx + GBA_CORE.cpu_regs + (1 * 4)], eax
    mov dword [rbx + GBA_CORE.cpu_regs + (3 * 4)], 0

.huff_ret:
    leave
    ret

cpu_swi_unfilter:
    ; r0=src, r1=dst, ecx=inwidth (1/2), edx=outwidth (1/2)
    push rbp
    mov rbp, rsp
    sub rsp, 96

    mov [rbp-4], ecx      ; inwidth
    mov [rbp-8], edx      ; outwidth

    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    and eax, 0xFFFFFFFC
    mov [rbp-12], eax     ; src
    mov eax, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    mov [rbp-16], eax     ; dst

    mov rdi, rbx
    mov esi, [rbp-12]
    call cpu_mem_read32
    mov ecx, eax
    shr ecx, 8
    mov [rbp-20], ecx     ; remaining
    mov dword [rbp-24], 0 ; halfword scratch
    mov dword [rbp-28], 0 ; old value
    add dword [rbp-12], 4

.uf_loop:
    cmp dword [rbp-20], 0
    jle .uf_done

    cmp dword [rbp-4], 1
    jne .uf_read16
    mov rdi, rbx
    mov esi, [rbp-12]
    call cpu_mem_read8
    movzx eax, al
    jmp .uf_read_done

.uf_read16:
    mov rdi, rbx
    mov esi, [rbp-12]
    call cpu_mem_read16
    movzx eax, ax

.uf_read_done:
    add eax, [rbp-28]
    and eax, 0xFFFF
    mov [rbp-32], eax     ; new

    ; outwidth > inwidth path (8->16)
    mov eax, [rbp-8]
    cmp eax, [rbp-4]
    jbe .uf_not_expand

    mov eax, [rbp-24]
    shr eax, 8
    mov [rbp-24], eax
    mov ecx, [rbp-32]
    shl ecx, 8
    or [rbp-24], ecx

    mov eax, [rbp-12]
    test eax, 1
    jz .uf_store_done
    mov rdi, rbx
    mov esi, [rbp-16]
    mov edx, [rbp-24]
    call cpu_mem_write16
    add dword [rbp-16], 2
    sub dword [rbp-20], 2
    jmp .uf_store_done

.uf_not_expand:
    cmp dword [rbp-8], 1
    jne .uf_store16
    mov rdi, rbx
    mov esi, [rbp-16]
    mov edx, [rbp-32]
    call cpu_mem_write8
    add dword [rbp-16], 1
    sub dword [rbp-20], 1
    jmp .uf_store_done

.uf_store16:
    mov rdi, rbx
    mov esi, [rbp-16]
    mov edx, [rbp-32]
    call cpu_mem_write16
    add dword [rbp-16], 2
    sub dword [rbp-20], 2

.uf_store_done:
    mov eax, [rbp-32]
    mov [rbp-28], eax
    mov eax, [rbp-4]
    add [rbp-12], eax
    jmp .uf_loop

.uf_done:
    mov eax, [rbp-12]
    mov [rbx + GBA_CORE.cpu_regs + (0 * 4)], eax
    mov eax, [rbp-16]
    mov [rbx + GBA_CORE.cpu_regs + (1 * 4)], eax
    mov dword [rbx + GBA_CORE.cpu_regs + (3 * 4)], 0
    leave
    ret

cpu_swi_bgaffineset:
    ; r0=src, r1=dst, r2=count
    push rbp
    mov rbp, rsp
    sub rsp, 128

    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    mov [rbp-4], eax       ; src
    mov eax, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    mov [rbp-8], eax       ; dst
    mov eax, [rbx + GBA_CORE.cpu_regs + (2 * 4)]
    mov [rbp-12], eax      ; count

.bg_affine_loop:
    cmp dword [rbp-12], 0
    jle .bg_affine_done

    ; ox_raw (s32, 8.8)
    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read32
    mov [rbp-16], eax

    ; oy_raw (s32, 8.8)
    mov rdi, rbx
    mov esi, [rbp-4]
    add esi, 4
    call cpu_mem_read32
    mov [rbp-20], eax

    ; cx (s16)
    mov rdi, rbx
    mov esi, [rbp-4]
    add esi, 8
    call cpu_mem_read16
    movsx eax, ax
    mov [rbp-24], eax

    ; cy (s16)
    mov rdi, rbx
    mov esi, [rbp-4]
    add esi, 10
    call cpu_mem_read16
    movsx eax, ax
    mov [rbp-28], eax

    ; sx_raw (s16, 8.8)
    mov rdi, rbx
    mov esi, [rbp-4]
    add esi, 12
    call cpu_mem_read16
    movsx eax, ax
    mov [rbp-32], eax

    ; sy_raw (s16, 8.8)
    mov rdi, rbx
    mov esi, [rbp-4]
    add esi, 14
    call cpu_mem_read16
    movsx eax, ax
    mov [rbp-36], eax

    ; theta = (angle>>8) * (pi/128)
    mov rdi, rbx
    mov esi, [rbp-4]
    add esi, 16
    call cpu_mem_read16
    movzx eax, ax
    shr eax, 8
    cvtsi2ss xmm0, eax
    mulss xmm0, dword [rel k_theta_scale]
    movss dword [rbp-40], xmm0

    ; sin/cos(theta)
    fld dword [rbp-40]
    fsin
    fstp dword [rbp-44]    ; sin

    fld dword [rbp-40]
    fcos
    fstp dword [rbp-48]    ; cos

    ; pa = trunc(cos * sx_raw)
    cvtsi2ss xmm0, dword [rbp-32]
    mulss xmm0, dword [rbp-48]
    cvttss2si eax, xmm0
    mov [rbp-52], eax

    ; pb = trunc(-sin * sx_raw)
    cvtsi2ss xmm0, dword [rbp-32]
    mulss xmm0, dword [rbp-44]
    cvttss2si eax, xmm0
    neg eax
    mov [rbp-56], eax

    ; pc = trunc(sin * sy_raw)
    cvtsi2ss xmm0, dword [rbp-36]
    mulss xmm0, dword [rbp-44]
    cvttss2si eax, xmm0
    mov [rbp-60], eax

    ; pd = trunc(cos * sy_raw)
    cvtsi2ss xmm0, dword [rbp-36]
    mulss xmm0, dword [rbp-48]
    cvttss2si eax, xmm0
    mov [rbp-64], eax

    ; rx256 = ox_raw - (pa*cx + pb*cy)
    mov eax, [rbp-52]
    imul eax, [rbp-24]
    mov ecx, [rbp-56]
    imul ecx, [rbp-28]
    add eax, ecx
    mov ecx, [rbp-16]
    sub ecx, eax
    mov [rbp-68], ecx

    ; ry256 = oy_raw - (pc*cx + pd*cy)
    mov eax, [rbp-60]
    imul eax, [rbp-24]
    mov ecx, [rbp-64]
    imul ecx, [rbp-28]
    add eax, ecx
    mov ecx, [rbp-20]
    sub ecx, eax
    mov [rbp-72], ecx

    ; store pa/pb/pc/pd
    mov rdi, rbx
    mov esi, [rbp-8]
    mov edx, [rbp-52]
    call cpu_mem_write16

    mov rdi, rbx
    mov esi, [rbp-8]
    add esi, 2
    mov edx, [rbp-56]
    call cpu_mem_write16

    mov rdi, rbx
    mov esi, [rbp-8]
    add esi, 4
    mov edx, [rbp-60]
    call cpu_mem_write16

    mov rdi, rbx
    mov esi, [rbp-8]
    add esi, 6
    mov edx, [rbp-64]
    call cpu_mem_write16

    ; store rx/ry (24.8)
    mov rdi, rbx
    mov esi, [rbp-8]
    add esi, 8
    mov edx, [rbp-68]
    call cpu_mem_write32

    mov rdi, rbx
    mov esi, [rbp-8]
    add esi, 12
    mov edx, [rbp-72]
    call cpu_mem_write32

    add dword [rbp-4], 20
    add dword [rbp-8], 16
    sub dword [rbp-12], 1
    jmp .bg_affine_loop

.bg_affine_done:
    leave
    ret

cpu_swi_objaffineset:
    ; r0=src, r1=dst, r2=count, r3=diff
    push rbp
    mov rbp, rsp
    sub rsp, 112

    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    mov [rbp-4], eax       ; src
    mov eax, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    mov [rbp-8], eax       ; dst
    mov eax, [rbx + GBA_CORE.cpu_regs + (2 * 4)]
    mov [rbp-12], eax      ; count
    mov eax, [rbx + GBA_CORE.cpu_regs + (3 * 4)]
    mov [rbp-16], eax      ; diff

.obj_affine_loop:
    cmp dword [rbp-12], 0
    jle .obj_affine_done

    ; sx_raw (s16, 8.8)
    mov rdi, rbx
    mov esi, [rbp-4]
    call cpu_mem_read16
    movsx eax, ax
    mov [rbp-20], eax

    ; sy_raw (s16, 8.8)
    mov rdi, rbx
    mov esi, [rbp-4]
    add esi, 2
    call cpu_mem_read16
    movsx eax, ax
    mov [rbp-24], eax

    ; theta = (angle>>8) * (pi/128)
    mov rdi, rbx
    mov esi, [rbp-4]
    add esi, 4
    call cpu_mem_read16
    movzx eax, ax
    shr eax, 8
    cvtsi2ss xmm0, eax
    mulss xmm0, dword [rel k_theta_scale]
    movss dword [rbp-28], xmm0

    ; sin/cos(theta)
    fld dword [rbp-28]
    fsin
    fstp dword [rbp-32]    ; sin

    fld dword [rbp-28]
    fcos
    fstp dword [rbp-36]    ; cos

    ; pa = trunc(cos * sx_raw)
    cvtsi2ss xmm0, dword [rbp-20]
    mulss xmm0, dword [rbp-36]
    cvttss2si eax, xmm0
    mov [rbp-40], eax

    ; pb = trunc(-sin * sx_raw)
    cvtsi2ss xmm0, dword [rbp-20]
    mulss xmm0, dword [rbp-32]
    cvttss2si eax, xmm0
    neg eax
    mov [rbp-44], eax

    ; pc = trunc(sin * sy_raw)
    cvtsi2ss xmm0, dword [rbp-24]
    mulss xmm0, dword [rbp-32]
    cvttss2si eax, xmm0
    mov [rbp-48], eax

    ; pd = trunc(cos * sy_raw)
    cvtsi2ss xmm0, dword [rbp-24]
    mulss xmm0, dword [rbp-36]
    cvttss2si eax, xmm0
    mov [rbp-52], eax

    ; store pa at dst
    mov rdi, rbx
    mov esi, [rbp-8]
    mov edx, [rbp-40]
    call cpu_mem_write16

    ; store pb at dst + diff
    mov ecx, [rbp-16]
    mov esi, [rbp-8]
    add esi, ecx
    mov rdi, rbx
    mov edx, [rbp-44]
    call cpu_mem_write16

    ; store pc at dst + diff*2
    mov ecx, [rbp-16]
    lea eax, [ecx + ecx]
    mov esi, [rbp-8]
    add esi, eax
    mov rdi, rbx
    mov edx, [rbp-48]
    call cpu_mem_write16

    ; store pd at dst + diff*3
    mov ecx, [rbp-16]
    lea eax, [ecx + ecx * 2]
    mov esi, [rbp-8]
    add esi, eax
    mov rdi, rbx
    mov edx, [rbp-52]
    call cpu_mem_write16

    add dword [rbp-4], 8
    mov eax, [rbp-16]
    shl eax, 2
    add dword [rbp-8], eax
    sub dword [rbp-12], 1
    jmp .obj_affine_loop

.obj_affine_done:
    leave
    ret

cpu_swi_bitunpack:
    ; r0=src, r1=dst, r2=BitUnPackInfo*
    push rbp
    mov rbp, rsp
    sub rsp, 96

    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    mov [rbp-4], eax       ; src
    mov eax, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    mov [rbp-8], eax       ; dst
    mov eax, [rbx + GBA_CORE.cpu_regs + (2 * 4)]
    mov [rbp-12], eax      ; info ptr

    ; src_len (u16)
    mov rdi, rbx
    mov esi, [rbp-12]
    call cpu_mem_read16
    movzx eax, ax
    mov [rbp-16], eax

    ; src_width (u8)
    mov rdi, rbx
    mov esi, [rbp-12]
    add esi, 2
    call cpu_mem_read8
    mov [rbp-20], eax

    ; dst_width (u8)
    mov rdi, rbx
    mov esi, [rbp-12]
    add esi, 3
    call cpu_mem_read8
    mov [rbp-24], eax

    ; data_offset (u32), bit31=offset-zero flag, bits0..30=offset
    mov rdi, rbx
    mov esi, [rbp-12]
    add esi, 4
    call cpu_mem_read32
    mov [rbp-28], eax
    mov ecx, eax
    shr ecx, 31
    mov [rbp-32], ecx      ; zero flag
    and eax, 0x7FFFFFFF
    mov [rbp-36], eax      ; offset value

    ; Validate widths.
    mov eax, [rbp-20]
    cmp eax, 1
    je .bitunpack_width_ok
    cmp eax, 2
    je .bitunpack_width_ok
    cmp eax, 4
    je .bitunpack_width_ok
    cmp eax, 8
    je .bitunpack_width_ok
    jmp .bitunpack_done

.bitunpack_width_ok:
    mov eax, [rbp-24]
    cmp eax, 1
    je .bitunpack_dst_ok
    cmp eax, 2
    je .bitunpack_dst_ok
    cmp eax, 4
    je .bitunpack_dst_ok
    cmp eax, 8
    je .bitunpack_dst_ok
    cmp eax, 16
    je .bitunpack_dst_ok
    cmp eax, 32
    jne .bitunpack_done

.bitunpack_dst_ok:
    mov dword [rbp-40], 0  ; bytes processed
    mov dword [rbp-44], 0  ; src bit offset [0..7]
    mov dword [rbp-48], 0  ; dst accumulator
    mov dword [rbp-52], 0  ; dst bits filled

.bitunpack_src_loop:
    mov eax, [rbp-40]
    cmp eax, [rbp-16]
    jae .bitunpack_flush

    mov rdi, rbx
    mov esi, [rbp-4]
    add esi, eax
    call cpu_mem_read8
    mov [rbp-56], eax      ; current source byte
    mov dword [rbp-44], 0

.bitunpack_group_loop:
    mov ecx, [rbp-44]
    cmp ecx, 8
    jae .bitunpack_next_src

    ; chunk = (src_byte >> bitoff) & ((1<<src_width)-1)
    mov eax, 1
    mov ecx, [rbp-20]
    shl eax, cl
    dec eax
    mov [rbp-60], eax      ; src mask

    mov eax, [rbp-56]
    mov ecx, [rbp-44]
    shr eax, cl
    and eax, [rbp-60]
    mov [rbp-64], eax      ; chunk

    ; zero suppression unless zero flag set
    cmp dword [rbp-64], 0
    jne .bitunpack_apply_offset
    cmp dword [rbp-32], 0
    jne .bitunpack_apply_offset
    jmp .bitunpack_chunk_ready

.bitunpack_apply_offset:
    mov eax, [rbp-64]
    add eax, [rbp-36]
    mov [rbp-64], eax

.bitunpack_chunk_ready:
    ; mask chunk to dst_width bits
    mov eax, 1
    mov ecx, [rbp-24]
    shl eax, cl
    dec eax
    and [rbp-64], eax

    ; dst_acc |= (chunk << dst_bits_filled)
    mov eax, [rbp-64]
    mov ecx, [rbp-52]
    shl eax, cl
    or [rbp-48], eax

    ; dst_bits_filled += dst_width
    mov eax, [rbp-52]
    add eax, [rbp-24]
    mov [rbp-52], eax

    ; write complete words
    cmp dword [rbp-52], 32
    jb .bitunpack_advance_chunk

    mov rdi, rbx
    mov esi, [rbp-8]
    mov edx, [rbp-48]
    call cpu_mem_write32
    add dword [rbp-8], 4
    mov dword [rbp-48], 0
    mov dword [rbp-52], 0

.bitunpack_advance_chunk:
    mov eax, [rbp-44]
    add eax, [rbp-20]
    mov [rbp-44], eax
    jmp .bitunpack_group_loop

.bitunpack_next_src:
    add dword [rbp-40], 1
    jmp .bitunpack_src_loop

.bitunpack_flush:
    cmp dword [rbp-52], 0
    je .bitunpack_done
    mov rdi, rbx
    mov esi, [rbp-8]
    mov edx, [rbp-48]
    call cpu_mem_write32

.bitunpack_done:
    leave
    ret

cpu_handle_swi:
    ; in: rbx=core, edi=swi number
    cmp edi, 0x00
    jne .not_softreset
    mov dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], GBA_ROM_BASE
    and dword [rbx + GBA_CORE.cpu_cpsr], ~CPSR_T
    ret

.not_softreset:
    cmp edi, 0x01
    jne .not_ramreset
    ; RegisterRamReset(r0 flags):
    ; bit0 EWRAM, bit1 IWRAM (except top 0x200 bytes), bit2 PRAM,
    ; bit3 VRAM, bit4 OAM. Bits 5..7 are IO reset groups (not yet modeled).
    mov r10d, [rbx + GBA_CORE.cpu_regs + (0 * 4)]

    test r10d, 0x01
    jz .ramreset_no_ewram
    lea rdi, [rbx + GBA_CORE.ewram]
    xor eax, eax
    mov ecx, GBA_EWRAM_SIZE
    rep stosb

.ramreset_no_ewram:
    test r10d, 0x02
    jz .ramreset_no_iwram
    lea rdi, [rbx + GBA_CORE.iwram]
    xor eax, eax
    mov ecx, (GBA_IWRAM_SIZE - 0x200)
    rep stosb

.ramreset_no_iwram:
    test r10d, 0x04
    jz .ramreset_no_pram
    lea rdi, [rbx + GBA_CORE.pram]
    xor eax, eax
    mov ecx, GBA_PRAM_SIZE
    rep stosb

.ramreset_no_pram:
    test r10d, 0x08
    jz .ramreset_no_vram
    lea rdi, [rbx + GBA_CORE.vram]
    xor eax, eax
    mov ecx, GBA_VRAM_SIZE
    rep stosb

.ramreset_no_vram:
    test r10d, 0x10
    jz .ramreset_done
    lea rdi, [rbx + GBA_CORE.oam]
    xor eax, eax
    mov ecx, GBA_OAM_SIZE
    rep stosb

.ramreset_done:
    ret

.not_ramreset:
    cmp edi, 0x02
    jne .not_halt
    ; HALT: stop CPU until an enabled interrupt becomes pending.
    mov dword [rbx + GBA_CORE.halted], 1
    ret

.not_halt:
    cmp edi, 0x03
    jne .not_stop
    mov dword [rbx + GBA_CORE.halted], 1
    ret

.not_stop:
    cmp edi, 0x04
    jne .not_intrwait
    ; IntrWait(r0=discard_old, r1=irq_mask)
    mov ecx, [rbx + GBA_CORE.cpu_regs + (1 * 4)] ; mask
    and ecx, 0x3FFF
    jnz .intrwait_mask_ok
    mov ecx, 0x3FFF

.intrwait_mask_ok:
    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)] ; discard_old?
    test eax, eax
    jnz .intrwait_check

    ; Clear requested IF bits before waiting.
    movzx edx, word [rbx + GBA_CORE.io + IO_IF]
    mov eax, ecx
    not eax
    and edx, eax
    mov [rbx + GBA_CORE.io + IO_IF], dx

.intrwait_check:
    movzx edx, word [rbx + GBA_CORE.io + IO_IF]
    mov eax, edx
    and eax, ecx
    test eax, eax
    jz .intrwait_sleep

    ; Consume matching IF bits and continue execution.
    not eax
    and edx, eax
    mov [rbx + GBA_CORE.io + IO_IF], dx
    mov dword [rbx + GBA_CORE.halted], 0
    ret

.intrwait_sleep:
    mov dword [rbx + GBA_CORE.halted], 1
    ret

.not_intrwait:
    cmp edi, 0x05
    jne .not_div
    ; VBlankIntrWait: wait specifically for VBlank IRQ.
    mov ecx, IRQ_VBLANK
    mov eax, [rbx + GBA_CORE.cpu_regs + (0 * 4)] ; discard_old?
    test eax, eax
    jnz .vblank_wait_check
    ; discard_old=0: clear stale VBlank IF before sleeping.
    movzx edx, word [rbx + GBA_CORE.io + IO_IF]
    and edx, ~IRQ_VBLANK
    mov [rbx + GBA_CORE.io + IO_IF], dx

.vblank_wait_check:
    movzx edx, word [rbx + GBA_CORE.io + IO_IF]
    test edx, ecx
    jz .vblank_wait_sleep

    and edx, ~IRQ_VBLANK
    mov [rbx + GBA_CORE.io + IO_IF], dx
    mov dword [rbx + GBA_CORE.halted], 0
    ret

.vblank_wait_sleep:
    mov dword [rbx + GBA_CORE.halted], 1
    ret

.not_div:
    cmp edi, 0x06
    jne .not_divarm
    mov ecx, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    mov edx, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    call cpu_swi_div_core
    ret

.not_divarm:
    cmp edi, 0x07
    jne .not_sqrt
    mov ecx, [rbx + GBA_CORE.cpu_regs + (1 * 4)]
    mov edx, [rbx + GBA_CORE.cpu_regs + (0 * 4)]
    call cpu_swi_div_core
    ret

.not_sqrt:
    cmp edi, 0x08
    jne .not_cpuset
    call cpu_swi_sqrt
    ret

.not_cpuset:
    cmp edi, 0x0B
    jne .not_cpufast
    call cpu_swi_cpuset
    ret

.not_cpufast:
    cmp edi, 0x0C
    jne .not_cpufast_dispatch
    call cpu_swi_cpufastset
    ret

.not_cpufast_dispatch:
    cmp edi, 0x0E
    jne .not_bgaffine
    call cpu_swi_bgaffineset
    ret

.not_bgaffine:
    cmp edi, 0x0F
    jne .not_objaffine
    call cpu_swi_objaffineset
    ret

.not_objaffine:
    cmp edi, 0x10
    jne .not_bitunpack
    call cpu_swi_bitunpack
    ret

.not_bitunpack:
    cmp edi, 0x11
    jne .not_lz77_wram
    mov ecx, 1
    call cpu_swi_lz77
    ret

.not_lz77_wram:
    cmp edi, 0x12
    jne .not_lz77_vram
    mov ecx, 2
    call cpu_swi_lz77
    ret

.not_lz77_vram:
    cmp edi, 0x13
    jne .not_huffman
    call cpu_swi_huffman
    ret

.not_huffman:
    cmp edi, 0x14
    jne .not_rl_wram_dispatch
    mov ecx, 1
    call cpu_swi_rl
    ret

.not_rl_wram_dispatch:
    cmp edi, 0x15
    jne .not_rl_vram_dispatch
    mov ecx, 2
    call cpu_swi_rl
    ret

.not_rl_vram_dispatch:
    cmp edi, 0x16
    jne .not_diff8_vram
    mov ecx, 1
    mov edx, 1
    call cpu_swi_unfilter
    ret

.not_diff8_vram:
    cmp edi, 0x17
    jne .not_diff16
    mov ecx, 1
    mov edx, 2
    call cpu_swi_unfilter
    ret

.not_diff16:
    cmp edi, 0x18
    jne .ret_noop
    mov ecx, 2
    mov edx, 2
    call cpu_swi_unfilter
    ret

.ret_noop:
    ret

; -----------------------------------------------------------------------------
; ARM decode/execute subset
; -----------------------------------------------------------------------------

cpu_exec_arm:
    ; in: rbx=core, edi=instruction
    push rbp
    mov rbp, rsp
    sub rsp, 64

    mov [rbp-4], edi      ; instr

    ; Condition check.
    mov eax, edi
    shr eax, 28
    mov edi, eax
    mov esi, [rbx + GBA_CORE.cpu_cpsr]
    call cpu_arm_cond_pass
    test eax, eax
    jnz .cond_ok
    jmp .done

.cond_ok:
    mov eax, [rbp-4]

    ; BX / BLX register pattern
    mov ecx, eax
    and ecx, 0x0FFFFFF0
    cmp ecx, 0x012FFF10
    jne .not_bx

    mov ecx, eax
    and ecx, 0xF
    mov edx, [rbx + GBA_CORE.cpu_regs + rcx * 4]
    test edx, 1
    jz .bx_to_arm

    or dword [rbx + GBA_CORE.cpu_cpsr], CPSR_T
    and edx, 0xFFFFFFFE
    mov [rbx + GBA_CORE.cpu_regs + (15 * 4)], edx
    jmp .done

.bx_to_arm:
    and dword [rbx + GBA_CORE.cpu_cpsr], ~CPSR_T
    and edx, 0xFFFFFFFC
    mov [rbx + GBA_CORE.cpu_regs + (15 * 4)], edx
    jmp .done

.not_bx:
    ; PSR transfer (MRS/MSR), used by ROM startup/IRQ plumbing.
    mov ecx, eax
    and ecx, 0x0FBF0FFF
    cmp ecx, 0x010F0000
    jne .not_mrs

    mov ecx, eax
    shr ecx, 12
    and ecx, 0xF
    mov edx, [rbx + GBA_CORE.cpu_cpsr]
    test dword [rbp-4], 0x00400000
    jz .mrs_store
    mov edx, [rbx + GBA_CORE.cpu_spsr]

.mrs_store:
    mov [rbx + GBA_CORE.cpu_regs + rcx * 4], edx
    jmp .done

.not_mrs:
    ; MSR (register source)
    mov ecx, eax
    and ecx, 0x0FB0FFF0
    cmp ecx, 0x0120F000
    jne .not_msr_imm
    test dword [rbp-4], 0x00400000
    jnz .done

    mov ecx, eax
    and ecx, 0xF
    mov esi, [rbx + GBA_CORE.cpu_regs + rcx * 4]
    cmp ecx, 15
    jne .msr_reg_src_ready
    add esi, 4

.msr_reg_src_ready:
    mov ecx, eax
    shr ecx, 16
    and ecx, 0xF
    jmp .msr_apply

.not_msr_imm:
    ; MSR (immediate source)
    mov ecx, eax
    and ecx, 0x0FB0F000
    cmp ecx, 0x0320F000
    jne .not_psr
    test dword [rbp-4], 0x00400000
    jnz .done

    mov ecx, eax
    and ecx, 0xFF
    mov esi, ecx

    mov ecx, eax
    shr ecx, 8
    and ecx, 0xF
    shl ecx, 1
    test ecx, ecx
    jz .msr_imm_src_ready
    ror esi, cl

.msr_imm_src_ready:
    mov ecx, eax
    shr ecx, 16
    and ecx, 0xF

.msr_apply:
    ; in: ecx=field mask nibble (cxsf), esi=source value
    xor edx, edx
    test ecx, 1
    jz .msr_no_c
    or edx, 0x000000FF
.msr_no_c:
    test ecx, 2
    jz .msr_no_x
    or edx, 0x0000FF00
.msr_no_x:
    test ecx, 4
    jz .msr_no_s
    or edx, 0x00FF0000
.msr_no_s:
    test ecx, 8
    jz .msr_mask_ready
    or edx, 0xFF000000

.msr_mask_ready:
    mov eax, [rbx + GBA_CORE.cpu_cpsr]
    mov edi, eax
    and edi, 0x1F

    mov r8d, esi
    and r8d, edx
    not edx
    and eax, edx
    or eax, r8d
    mov [rbp-52], eax

    and eax, 0x1F
    cmp eax, edi
    je .msr_store
    mov ecx, eax
    call cpu_switch_mode

.msr_store:
    mov eax, [rbp-52]
    mov [rbx + GBA_CORE.cpu_cpsr], eax
    jmp .done

.not_psr:
    ; Software interrupt
    mov ecx, eax
    and ecx, 0x0F000000
    cmp ecx, 0x0F000000
    jne .not_swi

    mov edi, eax
    and edi, 0x00FFFFFF
    call cpu_handle_swi
    jmp .done

.not_swi:
    ; Branch / Branch with link
    mov ecx, eax
    and ecx, 0x0E000000
    cmp ecx, 0x0A000000
    jne .not_branch

    test eax, 0x01000000
    jz .branch_no_link
    mov ecx, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    mov [rbx + GBA_CORE.cpu_regs + (14 * 4)], ecx

.branch_no_link:
    mov ecx, eax
    shl ecx, 8
    sar ecx, 6
    add [rbx + GBA_CORE.cpu_regs + (15 * 4)], ecx
    add dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 4
    jmp .done

.not_branch:
    ; Halfword/signed transfer (LDRH/STRH/LDRSB/LDRSH).
    mov ecx, eax
    and ecx, 0x0E000090
    cmp ecx, 0x00000090
    jne .not_hdt
    mov ecx, eax
    and ecx, 0x00000060
    jz .not_hdt

    mov ecx, eax
    shr ecx, 16
    and ecx, 0xF
    mov [rbp-56], ecx      ; rn

    mov ecx, eax
    shr ecx, 12
    and ecx, 0xF
    mov [rbp-60], ecx      ; rd

    mov ecx, [rbp-56]
    mov r8d, ecx
    mov esi, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    cmp r8d, 15
    jne .hdt_base_ready
    add esi, 4
.hdt_base_ready:
    mov [rbp-64], esi      ; base

    ; Offset source.
    test dword [rbp-4], (1 << 22)
    jz .hdt_offset_reg
    mov ecx, eax
    and ecx, 0xF
    mov edx, eax
    shr edx, 8
    and edx, 0xF
    shl edx, 4
    or ecx, edx
    jmp .hdt_offset_ready

.hdt_offset_reg:
    mov ecx, eax
    and ecx, 0xF
    mov r8d, ecx
    mov ecx, [rbx + GBA_CORE.cpu_regs + r8 * 4]

.hdt_offset_ready:
    mov [rbp-48], ecx      ; offset

    ; Effective address.
    mov edx, [rbp-64]
    test dword [rbp-4], (1 << 24)
    jz .hdt_addr_post
    test dword [rbp-4], (1 << 23)
    jz .hdt_pre_sub
    add edx, ecx
    jmp .hdt_addr_ready
.hdt_pre_sub:
    sub edx, ecx
    jmp .hdt_addr_ready
.hdt_addr_post:
    ; post-index uses base as effective address
.hdt_addr_ready:
    mov [rbp-52], edx      ; effective address

    mov r9d, eax
    shr r9d, 5
    and r9d, 0x3           ; transfer kind

    test dword [rbp-4], (1 << 20)
    jnz .hdt_load

    ; Store form: only STRH is valid (kind=1).
    cmp r9d, 1
    jne .unknown
    mov ecx, [rbp-60]
    mov r8d, ecx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    cmp r8d, 15
    jne .hdt_store_go
    add edx, 4
.hdt_store_go:
    mov rdi, rbx
    mov esi, [rbp-52]
    call cpu_mem_write16
    jmp .hdt_writeback

.hdt_load:
    cmp r9d, 1
    je .hdt_load_u16
    cmp r9d, 2
    je .hdt_load_s8
    cmp r9d, 3
    je .hdt_load_s16
    jmp .unknown

.hdt_load_u16:
    mov rdi, rbx
    mov esi, [rbp-52]
    call cpu_mem_read16
    jmp .hdt_store_loaded

.hdt_load_s8:
    mov rdi, rbx
    mov esi, [rbp-52]
    call cpu_mem_read8
    movsx eax, al
    jmp .hdt_store_loaded

.hdt_load_s16:
    mov rdi, rbx
    mov esi, [rbp-52]
    call cpu_mem_read16
    movsx eax, ax

.hdt_store_loaded:
    mov ecx, [rbp-60]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    cmp r8d, 15
    jne .hdt_writeback
    test eax, 1
    jz .hdt_load_pc_arm
    or dword [rbx + GBA_CORE.cpu_cpsr], CPSR_T
    and dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 0xFFFFFFFE
    jmp .hdt_writeback

.hdt_load_pc_arm:
    and dword [rbx + GBA_CORE.cpu_cpsr], ~CPSR_T
    and dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 0xFFFFFFFC

.hdt_writeback:
    ; Writeback for post-index or explicit W bit.
    test dword [rbp-4], (1 << 24)
    jz .hdt_wb_do
    test dword [rbp-4], (1 << 21)
    jz .done

.hdt_wb_do:
    mov edx, [rbp-64]
    mov ecx, [rbp-48]
    test dword [rbp-4], (1 << 23)
    jz .hdt_wb_sub
    add edx, ecx
    jmp .hdt_wb_store

.hdt_wb_sub:
    sub edx, ecx

.hdt_wb_store:
    mov ecx, [rbp-56]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], edx
    jmp .done

.not_hdt:
    ; Block data transfer (LDM/STM)
    mov ecx, eax
    and ecx, 0x0E000000
    cmp ecx, 0x08000000
    jne .not_bdt

    mov ecx, eax
    shr ecx, 16
    and ecx, 0xF
    mov [rbp-8], ecx       ; rn

    mov ecx, eax
    and ecx, 0xFFFF
    mov [rbp-12], ecx      ; rlist

    mov ecx, [rbp-8]
    mov esi, [rbx + GBA_CORE.cpu_regs + rcx * 4]
    cmp ecx, 15
    jne .bdt_base_done
    add esi, 4
.bdt_base_done:
    mov [rbp-16], esi      ; base

    ; Count registers in list.
    mov edx, [rbp-12]
    xor ecx, ecx
.bdt_count_loop:
    test edx, edx
    jz .bdt_count_done
    lea eax, [rdx - 1]
    and edx, eax
    inc ecx
    jmp .bdt_count_loop

.bdt_count_done:
    mov [rbp-24], ecx      ; transfer count

    ; Compute first transfer address (lowest register maps to lowest address).
    mov esi, [rbp-16]
    test ecx, ecx
    jz .bdt_addr_ready
    mov edx, ecx
    shl edx, 2             ; count * 4

    test dword [rbp-4], (1 << 23) ; U
    jnz .bdt_inc_addr

    ; Decrementing modes.
    test dword [rbp-4], (1 << 24) ; P
    jz .bdt_addr_da
    sub esi, edx           ; DB: base - count*4
    jmp .bdt_addr_ready

.bdt_addr_da:
    sub esi, edx
    add esi, 4             ; DA: base - count*4 + 4
    jmp .bdt_addr_ready

.bdt_inc_addr:
    ; Incrementing modes.
    test dword [rbp-4], (1 << 24) ; P
    jz .bdt_addr_ready      ; IA: base
    add esi, 4              ; IB: base + 4

.bdt_addr_ready:
    mov [rbp-20], esi      ; current transfer address

    mov ecx, 0
.bdt_loop:
    cmp ecx, 16
    jae .bdt_writeback
    bt dword [rbp-12], ecx
    jnc .bdt_next

    test dword [rbp-4], (1 << 20)
    jz .bdt_store

    ; load
    mov rdi, rbx
    mov esi, [rbp-20]
    mov [rbp-48], ecx
    call cpu_mem_read32
    mov ecx, [rbp-48]
    cmp ecx, 15
    jne .bdt_store_loaded
    test eax, 1
    jz .bdt_load_pc_arm
    or dword [rbx + GBA_CORE.cpu_cpsr], CPSR_T
    and eax, 0xFFFFFFFE
    jmp .bdt_store_loaded

.bdt_load_pc_arm:
    and dword [rbx + GBA_CORE.cpu_cpsr], ~CPSR_T
    and eax, 0xFFFFFFFC

.bdt_store_loaded:
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .bdt_advance

.bdt_store:
    mov r8d, ecx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    cmp ecx, 15
    jne .bdt_store_val_ok
    add edx, 4

.bdt_store_val_ok:
    mov rdi, rbx
    mov esi, [rbp-20]
    mov [rbp-48], ecx
    call cpu_mem_write32
    mov ecx, [rbp-48]

.bdt_advance:
    add dword [rbp-20], 4

.bdt_next:
    inc ecx
    jmp .bdt_loop

.bdt_writeback:
    test dword [rbp-4], (1 << 21)
    jz .done

    mov ecx, [rbp-24]
    shl ecx, 2
    mov esi, [rbp-16]
    test dword [rbp-4], (1 << 23)
    jz .bdt_wb_sub
    add esi, ecx
    jmp .bdt_wb_store

.bdt_wb_sub:
    sub esi, ecx

.bdt_wb_store:
    mov ecx, [rbp-8]
    mov [rbx + GBA_CORE.cpu_regs + rcx * 4], esi
    jmp .done

.not_bdt:
    ; Single data transfer
    mov ecx, eax
    and ecx, 0x0C000000
    cmp ecx, 0x04000000
    jne .not_sdt

    ; Decode fields
    mov edx, eax
    shr edx, 16
    and edx, 0xF
    mov [rbp-8], edx      ; rn

    mov edx, eax
    shr edx, 12
    and edx, 0xF
    mov [rbp-12], edx     ; rd

    mov ecx, [rbp-8]
    mov esi, [rbx + GBA_CORE.cpu_regs + rcx * 4]
    cmp ecx, 15
    jne .sdt_base_done
    add esi, 4
.sdt_base_done:
    mov [rbp-16], esi     ; base

    ; offset
    test eax, 0x02000000
    jz .sdt_offset_imm

    mov ecx, eax
    and ecx, 0xF
    mov edx, [rbx + GBA_CORE.cpu_regs + rcx * 4]
    cmp ecx, 15
    jne .sdt_off_rm_done
    add edx, 4
.sdt_off_rm_done:

    mov ecx, eax
    shr ecx, 7
    and ecx, 0x1F

    mov r8d, eax
    shr r8d, 5
    and r8d, 0x3

    cmp r8d, 0
    jne .sdt_shift_lsr
    shl edx, cl
    jmp .sdt_offset_done

.sdt_shift_lsr:
    cmp r8d, 1
    jne .sdt_shift_asr
    test ecx, ecx
    jz .sdt_lsr_32
    shr edx, cl
    jmp .sdt_offset_done

.sdt_lsr_32:
    xor edx, edx
    jmp .sdt_offset_done

.sdt_shift_asr:
    cmp r8d, 2
    jne .sdt_shift_ror
    test ecx, ecx
    jz .sdt_asr_32
    sar edx, cl
    jmp .sdt_offset_done

.sdt_asr_32:
    sar edx, 31
    jmp .sdt_offset_done

.sdt_shift_ror:
    test ecx, ecx
    jz .sdt_offset_done
    ror edx, cl
    jmp .sdt_offset_done

.sdt_offset_imm:
    mov edx, eax
    and edx, 0xFFF

.sdt_offset_done:
    mov [rbp-20], edx     ; offset

    mov esi, [rbp-16]
    mov edx, [rbp-20]

    test eax, 0x01000000
    jz .sdt_post_index

    test eax, 0x00800000
    jz .sdt_pre_sub
    add esi, edx
    jmp .sdt_addr_ready

.sdt_pre_sub:
    sub esi, edx
    jmp .sdt_addr_ready

.sdt_post_index:
    ; post-index uses base as transfer address

.sdt_addr_ready:
    mov [rbp-24], esi     ; address used for transfer

    ; Load/store op
    test eax, 0x00100000
    jz .sdt_store

    ; LDR / LDRB
    test eax, 0x00400000
    jz .sdt_load_word

    mov rdi, rbx
    mov esi, [rbp-24]
    call cpu_mem_read8
    jmp .sdt_load_store_reg

.sdt_load_word:
    mov rdi, rbx
    mov esi, [rbp-24]
    call cpu_mem_read32

.sdt_load_store_reg:
    mov ecx, [rbp-12]
    mov [rbx + GBA_CORE.cpu_regs + rcx * 4], eax

    cmp ecx, 15
    jne .sdt_writeback
    test dword [rbx + GBA_CORE.cpu_cpsr], CPSR_T
    jz .pc_align_arm
    and dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 0xFFFFFFFE
    jmp .sdt_writeback

.pc_align_arm:
    and dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 0xFFFFFFFC
    jmp .sdt_writeback

.sdt_store:
    mov ecx, [rbp-12]
    mov r8d, ecx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]

    test eax, 0x00400000
    jz .sdt_store_word

    mov rdi, rbx
    mov esi, [rbp-24]
    call cpu_mem_write8
    jmp .sdt_writeback

.sdt_store_word:
    mov rdi, rbx
    mov esi, [rbp-24]
    call cpu_mem_write32

.sdt_writeback:
    ; Writeback if post-index or W bit set.
    test dword [rbp-4], 0x01000000
    jz .sdt_do_writeback
    test dword [rbp-4], 0x00200000
    jz .done

.sdt_do_writeback:
    mov esi, [rbp-16]
    mov edx, [rbp-20]
    test dword [rbp-4], 0x00800000
    jz .sdt_wb_sub
    add esi, edx
    jmp .sdt_wb_store

.sdt_wb_sub:
    sub esi, edx

.sdt_wb_store:
    mov ecx, [rbp-8]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], esi
    jmp .done

.not_sdt:
    ; Multiply / Multiply-Accumulate
    mov ecx, eax
    and ecx, 0x0FC000F0
    cmp ecx, 0x00000090
    jne .not_mul

    mov ecx, eax
    and ecx, 0xF
    mov r8d, [rbx + GBA_CORE.cpu_regs + rcx * 4] ; rm

    mov ecx, eax
    shr ecx, 8
    and ecx, 0xF
    mov r9d, [rbx + GBA_CORE.cpu_regs + rcx * 4] ; rs

    mov r10d, r8d
    imul r10d, r9d

    test eax, (1 << 21)
    jz .mul_no_acc
    mov ecx, eax
    shr ecx, 12
    and ecx, 0xF
    add r10d, [rbx + GBA_CORE.cpu_regs + rcx * 4]

.mul_no_acc:
    mov ecx, eax
    shr ecx, 16
    and ecx, 0xF
    mov [rbx + GBA_CORE.cpu_regs + rcx * 4], r10d

    test eax, (1 << 20)
    jz .done
    mov eax, r10d
    call cpu_update_nz
    jmp .done

.not_mul:
    ; Data processing (subset)
    mov ecx, eax
    and ecx, 0x0C000000
    cmp ecx, 0x00000000
    jne .unknown

    ; Operand2 with ARM shifter carry semantics.
    ; [rbp-28] stores shifter carry as 0/1.
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    shr ecx, 29
    and ecx, 1
    mov [rbp-28], ecx

    test eax, 0x02000000
    jz .dp_op2_reg

    ; Immediate rotate.
    mov ecx, eax
    and ecx, 0xFF
    mov edx, eax
    shr edx, 8
    and edx, 0xF
    shl edx, 1

    mov esi, ecx
    test edx, edx
    jz .dp_op2_imm_done
    mov ecx, edx
    ror esi, cl
    mov ecx, esi
    shr ecx, 31
    and ecx, 1
    mov [rbp-28], ecx

.dp_op2_imm_done:
    mov [rbp-32], esi
    jmp .dp_operands_ready

.dp_op2_reg:
    mov ecx, eax
    and ecx, 0xF
    mov r8d, ecx
    mov esi, [rbx + GBA_CORE.cpu_regs + r8 * 4]   ; Rm
    cmp r8d, 15
    jne .dp_op2_rm_done
    add esi, 4
.dp_op2_rm_done:
    mov r9d, esi                                   ; rm original value

    mov edx, eax
    shr edx, 5
    and edx, 0x3                                   ; shift type

    test eax, 0x10
    jnz .dp_shift_reg

    ; Shift by immediate.
    mov ecx, eax
    shr ecx, 7
    and ecx, 0x1F                                  ; imm5

    cmp edx, 0
    jne .dp_lsr_imm
    ; LSL #imm
    test ecx, ecx
    jz .dp_reg_shift_done                          ; carry unchanged
    mov r11d, ecx
    mov esi, r9d
    shl esi, cl
    mov edx, r9d
    mov ecx, 32
    sub ecx, r11d
    shr edx, cl
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_lsr_imm:
    cmp edx, 1
    jne .dp_asr_imm
    ; LSR #imm (imm==0 means 32)
    test ecx, ecx
    jnz .dp_lsr_imm_nz
    mov edx, r9d
    shr edx, 31
    and edx, 1
    mov [rbp-28], edx
    xor esi, esi
    jmp .dp_reg_shift_done

.dp_lsr_imm_nz:
    mov r11d, ecx
    mov esi, r9d
    shr esi, cl
    mov edx, r9d
    mov ecx, r11d
    dec ecx
    shr edx, cl
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_asr_imm:
    cmp edx, 2
    jne .dp_ror_imm
    ; ASR #imm (imm==0 means 32)
    test ecx, ecx
    jnz .dp_asr_imm_nz
    mov edx, r9d
    shr edx, 31
    and edx, 1
    mov [rbp-28], edx
    mov esi, r9d
    sar esi, 31
    jmp .dp_reg_shift_done

.dp_asr_imm_nz:
    mov r11d, ecx
    mov esi, r9d
    sar esi, cl
    mov edx, r9d
    mov ecx, r11d
    dec ecx
    shr edx, cl
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_ror_imm:
    ; ROR #imm (imm==0 means RRX)
    test ecx, ecx
    jnz .dp_ror_imm_nz
    mov esi, r9d
    shr esi, 1
    mov edx, [rbp-28]
    test edx, edx
    jz .dp_rrx_carry_done
    or esi, 0x80000000
.dp_rrx_carry_done:
    mov edx, r9d
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_ror_imm_nz:
    mov esi, r9d
    ror esi, cl
    mov edx, esi
    shr edx, 31
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_shift_reg:
    ; Shift amount from Rs[7:0].
    mov ecx, eax
    shr ecx, 8
    and ecx, 0xF
    mov r8d, [rbx + GBA_CORE.cpu_regs + rcx * 4]
    cmp ecx, 15
    jne .dp_shift_rs_ready
    add r8d, 4
.dp_shift_rs_ready:
    and r8d, 0xFF

    cmp edx, 0
    jne .dp_lsr_reg
    ; LSL Rs
    test r8d, r8d
    jz .dp_reg_shift_done
    cmp r8d, 32
    jb .dp_lsl_reg_lt32
    je .dp_lsl_reg_eq32
    xor esi, esi
    mov dword [rbp-28], 0
    jmp .dp_reg_shift_done

.dp_lsl_reg_lt32:
    mov ecx, r8d
    mov esi, r9d
    shl esi, cl
    mov edx, r9d
    mov ecx, 32
    sub ecx, r8d
    shr edx, cl
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_lsl_reg_eq32:
    xor esi, esi
    mov edx, r9d
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_lsr_reg:
    cmp edx, 1
    jne .dp_asr_reg
    ; LSR Rs
    test r8d, r8d
    jz .dp_reg_shift_done
    cmp r8d, 32
    jb .dp_lsr_reg_lt32
    je .dp_lsr_reg_eq32
    xor esi, esi
    mov dword [rbp-28], 0
    jmp .dp_reg_shift_done

.dp_lsr_reg_lt32:
    mov ecx, r8d
    mov esi, r9d
    shr esi, cl
    mov edx, r9d
    mov ecx, r8d
    dec ecx
    shr edx, cl
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_lsr_reg_eq32:
    xor esi, esi
    mov edx, r9d
    shr edx, 31
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_asr_reg:
    cmp edx, 2
    jne .dp_ror_reg
    ; ASR Rs
    test r8d, r8d
    jz .dp_reg_shift_done
    cmp r8d, 32
    jb .dp_asr_reg_lt32
    mov edx, r9d
    shr edx, 31
    and edx, 1
    mov [rbp-28], edx
    mov esi, r9d
    sar esi, 31
    jmp .dp_reg_shift_done

.dp_asr_reg_lt32:
    mov ecx, r8d
    mov esi, r9d
    sar esi, cl
    mov edx, r9d
    mov ecx, r8d
    dec ecx
    shr edx, cl
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_ror_reg:
    ; ROR Rs
    test r8d, r8d
    jz .dp_reg_shift_done
    mov ecx, r8d
    and ecx, 31
    jz .dp_ror_reg_mod0
    mov esi, r9d
    ror esi, cl
    mov edx, esi
    shr edx, 31
    and edx, 1
    mov [rbp-28], edx
    jmp .dp_reg_shift_done

.dp_ror_reg_mod0:
    mov esi, r9d
    mov edx, r9d
    shr edx, 31
    and edx, 1
    mov [rbp-28], edx

.dp_reg_shift_done:
    mov [rbp-32], esi

.dp_operands_ready:
    mov ecx, eax
    shr ecx, 16
    and ecx, 0xF
    mov [rbp-36], ecx     ; rn

    mov ecx, eax
    shr ecx, 12
    and ecx, 0xF
    mov [rbp-40], ecx     ; rd

    mov ecx, [rbp-36]
    mov r8d, ecx
    mov ecx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    cmp r8d, 15
    jne .dp_op1_done
    add ecx, 4
.dp_op1_done:
    mov [rbp-44], ecx     ; op1

    mov edx, [rbp-32]     ; op2
    mov esi, [rbp-44]     ; op1

    mov ecx, eax
    shr ecx, 21
    and ecx, 0xF

    ; opcode dispatch
    cmp ecx, 0
    je .dp_and
    cmp ecx, 1
    je .dp_eor
    cmp ecx, 2
    je .dp_sub
    cmp ecx, 3
    je .dp_rsb
    cmp ecx, 4
    je .dp_add
    cmp ecx, 5
    je .dp_adc
    cmp ecx, 6
    je .dp_sbc
    cmp ecx, 8
    je .dp_tst
    cmp ecx, 9
    je .dp_teq
    cmp ecx, 10
    je .dp_cmp
    cmp ecx, 11
    je .dp_cmn
    cmp ecx, 12
    je .dp_orr
    cmp ecx, 13
    je .dp_mov
    cmp ecx, 14
    je .dp_bic
    cmp ecx, 15
    je .dp_mvn
    jmp .unknown

.dp_and:
    mov eax, esi
    and eax, edx
    jmp .dp_write_result_logic

.dp_eor:
    mov eax, esi
    xor eax, edx
    jmp .dp_write_result_logic

.dp_orr:
    mov eax, esi
    or eax, edx
    jmp .dp_write_result_logic

.dp_mov:
    mov eax, edx
    jmp .dp_write_result_logic

.dp_mvn:
    mov eax, edx
    not eax
    jmp .dp_write_result_logic

.dp_sub:
    mov ecx, esi
    mov eax, esi
    sub eax, edx

    ; Write result
    mov r8d, [rbp-40]
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax

    ; Flags if S
    test dword [rbp-4], 0x00100000
    jz .dp_pc_fix
    call cpu_set_flags_sub
    jmp .dp_pc_fix

.dp_rsb:
    mov ecx, edx
    mov eax, edx
    sub eax, esi

    mov r8d, [rbp-40]
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax

    test dword [rbp-4], 0x00100000
    jz .dp_pc_fix
    mov edx, esi
    call cpu_set_flags_sub
    jmp .dp_pc_fix

.dp_add:
    mov ecx, esi
    mov eax, esi
    add eax, edx

    mov r8d, [rbp-40]
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax

    test dword [rbp-4], 0x00100000
    jz .dp_pc_fix
    call cpu_set_flags_add
    jmp .dp_pc_fix

.dp_adc:
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    test ecx, CPSR_C
    jz .dp_adc_c0
    stc
    jmp .dp_adc_go
.dp_adc_c0:
    clc
.dp_adc_go:
    mov eax, esi
    adc eax, edx
    setc r10b
    seto r11b

    mov r8d, [rbp-40]
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax

    test dword [rbp-4], 0x00100000
    jz .dp_pc_fix

    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    and ecx, ~(CPSR_N | CPSR_Z | CPSR_C | CPSR_V)
    test eax, eax
    jnz .dp_adc_no_z
    or ecx, CPSR_Z
.dp_adc_no_z:
    test eax, 0x80000000
    jz .dp_adc_no_n
    or ecx, CPSR_N
.dp_adc_no_n:
    test r10b, r10b
    jz .dp_adc_no_c
    or ecx, CPSR_C
.dp_adc_no_c:
    test r11b, r11b
    jz .dp_adc_store
    or ecx, CPSR_V
.dp_adc_store:
    mov [rbx + GBA_CORE.cpu_cpsr], ecx
    jmp .dp_pc_fix

.dp_sbc:
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    test ecx, CPSR_C
    jz .dp_sbc_c0
    clc
    jmp .dp_sbc_go
.dp_sbc_c0:
    stc
.dp_sbc_go:
    mov eax, esi
    sbb eax, edx
    setc r10b
    seto r11b

    mov r8d, [rbp-40]
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax

    test dword [rbp-4], 0x00100000
    jz .dp_pc_fix

    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    and ecx, ~(CPSR_N | CPSR_Z | CPSR_C | CPSR_V)
    test eax, eax
    jnz .dp_sbc_no_z
    or ecx, CPSR_Z
.dp_sbc_no_z:
    test eax, 0x80000000
    jz .dp_sbc_no_n
    or ecx, CPSR_N
.dp_sbc_no_n:
    test r10b, r10b
    jnz .dp_sbc_no_c
    or ecx, CPSR_C
.dp_sbc_no_c:
    test r11b, r11b
    jz .dp_sbc_store
    or ecx, CPSR_V
.dp_sbc_store:
    mov [rbx + GBA_CORE.cpu_cpsr], ecx
    jmp .dp_pc_fix

.dp_cmp:
    mov ecx, esi
    mov eax, esi
    sub eax, edx
    call cpu_set_flags_sub
    jmp .done

.dp_tst:
    mov eax, esi
    and eax, edx
    call cpu_update_nz
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    and ecx, ~CPSR_C
    mov edx, [rbp-28]
    test edx, edx
    jz .dp_tst_store
    or ecx, CPSR_C
.dp_tst_store:
    mov [rbx + GBA_CORE.cpu_cpsr], ecx
    jmp .done

.dp_teq:
    mov eax, esi
    xor eax, edx
    call cpu_update_nz
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    and ecx, ~CPSR_C
    mov edx, [rbp-28]
    test edx, edx
    jz .dp_teq_store
    or ecx, CPSR_C
.dp_teq_store:
    mov [rbx + GBA_CORE.cpu_cpsr], ecx
    jmp .done

.dp_cmn:
    mov ecx, esi
    mov eax, esi
    add eax, edx
    call cpu_set_flags_add
    jmp .done

.dp_bic:
    mov eax, edx
    not eax
    and eax, esi
    jmp .dp_write_result_logic

.dp_write_result_logic:
    mov r8d, [rbp-40]
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax

    test dword [rbp-4], 0x00100000
    jz .dp_pc_fix

    call cpu_update_nz

    ; Logical ops with S set C from Operand2 shifter carry.
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    and ecx, ~CPSR_C
    mov edx, [rbp-28]
    test edx, edx
    jz .dp_logic_store
    or ecx, CPSR_C
.dp_logic_store:
    mov [rbx + GBA_CORE.cpu_cpsr], ecx

.dp_pc_fix:
    mov r8d, [rbp-40]
    cmp r8d, 15
    jne .done

    ; Data-processing with S and Rd=PC performs an exception return:
    ; restore CPSR from SPSR before final PC alignment.
    test dword [rbp-4], 0x00100000
    jz .dp_align_by_state
    mov eax, [rbx + GBA_CORE.cpu_spsr]
    mov [rbp-52], eax

    mov ecx, eax
    and ecx, 0x1F
    mov edx, [rbx + GBA_CORE.cpu_cpsr]
    and edx, 0x1F
    cmp ecx, edx
    je .dp_restore_cpsr
    call cpu_switch_mode

.dp_restore_cpsr:
    mov eax, [rbp-52]
    mov [rbx + GBA_CORE.cpu_cpsr], eax

.dp_align_by_state:
    test dword [rbx + GBA_CORE.cpu_cpsr], CPSR_T
    jz .dp_align_arm
    and dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 0xFFFFFFFE
    jmp .done

.dp_align_arm:
    and dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 0xFFFFFFFC
    jmp .done

.unknown:
    add dword [rbx + GBA_CORE.cpu_unknown], 1

.done:
    leave
    ret

; -----------------------------------------------------------------------------
; Thumb decode/execute subset
; -----------------------------------------------------------------------------

cpu_exec_thumb:
    ; in: rbx=core, edi=op16
    push rbp
    mov rbp, rsp
    sub rsp, 96

    mov [rbp-4], edi

    ; BL first/second half (11110 / 11111)
    mov eax, edi
    and eax, 0xF800
    cmp eax, 0xF000
    jne .not_bl_hi

    ; BL high offset: LR = (PC + 2) + sign_extend(imm11) << 12
    mov eax, [rbp-4]
    and eax, 0x07FF
    shl eax, 21
    sar eax, 21
    shl eax, 12
    mov ecx, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    add ecx, 2
    add ecx, eax
    mov [rbx + GBA_CORE.cpu_regs + (14 * 4)], ecx
    jmp .done

.not_bl_hi:
    cmp eax, 0xF800
    jne .not_bl_lo

    ; BL low offset: PC = LR + (imm11 << 1), LR = return|1
    mov eax, [rbp-4]
    and eax, 0x07FF
    shl eax, 1
    mov ecx, [rbx + GBA_CORE.cpu_regs + (14 * 4)]
    add ecx, eax
    mov edx, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    or edx, 1
    mov [rbx + GBA_CORE.cpu_regs + (14 * 4)], edx
    and ecx, 0xFFFFFFFE
    mov [rbx + GBA_CORE.cpu_regs + (15 * 4)], ecx
    or dword [rbx + GBA_CORE.cpu_cpsr], CPSR_T
    jmp .done

.not_bl_lo:
    ; Unconditional branch (11100)
    mov eax, edi
    and eax, 0xF800
    cmp eax, 0xE000
    jne .not_uncond_b

    mov eax, [rbp-4]
    and eax, 0x07FF
    shl eax, 21
    sar eax, 21
    shl eax, 1
    mov ecx, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    add ecx, 2
    add ecx, eax
    mov [rbx + GBA_CORE.cpu_regs + (15 * 4)], ecx
    jmp .done

.not_uncond_b:
    ; Conditional branch / SWI
    mov eax, edi
    and eax, 0xF000
    cmp eax, 0xD000
    jne .not_cond_b

    mov ecx, [rbp-4]
    shr ecx, 8
    and ecx, 0xF
    cmp ecx, 0xF
    jne .thumb_cond_branch

    mov edi, [rbp-4]
    and edi, 0xFF
    call cpu_handle_swi
    jmp .done

.thumb_cond_branch:
    mov edi, ecx
    mov esi, [rbx + GBA_CORE.cpu_cpsr]
    call cpu_arm_cond_pass
    test eax, eax
    jz .done

    mov eax, [rbp-4]
    and eax, 0xFF
    shl eax, 24
    sar eax, 24
    shl eax, 1
    mov ecx, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    add ecx, 2
    add ecx, eax
    mov [rbx + GBA_CORE.cpu_regs + (15 * 4)], ecx
    jmp .done

.not_cond_b:
    ; PUSH/POP register list (1011x10x)
    mov eax, [rbp-4]
    and eax, 0xF600
    cmp eax, 0xB400
    jne .not_push_pop

    mov eax, [rbp-4]
    and eax, 0xFF
    mov [rbp-8], eax        ; rlist
    mov eax, [rbx + GBA_CORE.cpu_regs + (13 * 4)]
    mov [rbp-12], eax       ; sp

    test dword [rbp-4], (1 << 11)
    jnz .thumb_pop

    ; PUSH
    test dword [rbp-4], (1 << 8)
    jz .push_regs
    sub dword [rbp-12], 4
    mov rdi, rbx
    mov esi, [rbp-12]
    mov edx, [rbx + GBA_CORE.cpu_regs + (14 * 4)]
    call cpu_mem_write32

.push_regs:
    mov ecx, 7
.push_loop:
    bt dword [rbp-8], ecx
    jnc .push_next
    sub dword [rbp-12], 4
    mov rdi, rbx
    mov esi, [rbp-12]
    mov r8d, ecx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov [rbp-44], ecx
    call cpu_mem_write32
    mov ecx, [rbp-44]

.push_next:
    dec ecx
    jns .push_loop
    mov eax, [rbp-12]
    mov [rbx + GBA_CORE.cpu_regs + (13 * 4)], eax
    jmp .done

.thumb_pop:
    mov ecx, 0
.pop_loop:
    cmp ecx, 8
    jae .pop_pc
    bt dword [rbp-8], ecx
    jnc .pop_next
    mov rdi, rbx
    mov esi, [rbp-12]
    mov [rbp-44], ecx
    call cpu_mem_read32
    mov ecx, [rbp-44]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    add dword [rbp-12], 4

.pop_next:
    inc ecx
    jmp .pop_loop

.pop_pc:
    test dword [rbp-4], (1 << 8)
    jz .pop_done

    mov rdi, rbx
    mov esi, [rbp-12]
    call cpu_mem_read32
    add dword [rbp-12], 4
    test eax, 1
    jz .pop_to_arm
    or dword [rbx + GBA_CORE.cpu_cpsr], CPSR_T
    and eax, 0xFFFFFFFE
    mov [rbx + GBA_CORE.cpu_regs + (15 * 4)], eax
    jmp .pop_done

.pop_to_arm:
    and dword [rbx + GBA_CORE.cpu_cpsr], ~CPSR_T
    and eax, 0xFFFFFFFC
    mov [rbx + GBA_CORE.cpu_regs + (15 * 4)], eax

.pop_done:
    mov eax, [rbp-12]
    mov [rbx + GBA_CORE.cpu_regs + (13 * 4)], eax
    jmp .done

.not_push_pop:
    ; ADD/SUB SP immediate (10110000)
    mov eax, [rbp-4]
    and eax, 0xFF00
    cmp eax, 0xB000
    jne .not_add_sp_imm

    mov eax, [rbp-4]
    and eax, 0x7F
    shl eax, 2
    test dword [rbp-4], (1 << 7)
    jnz .sub_sp_imm
    add [rbx + GBA_CORE.cpu_regs + (13 * 4)], eax
    jmp .done

.sub_sp_imm:
    sub [rbx + GBA_CORE.cpu_regs + (13 * 4)], eax
    jmp .done

.not_add_sp_imm:
    ; ADD Rd, PC/SP, #imm (1010)
    mov eax, [rbp-4]
    and eax, 0xF000
    cmp eax, 0xA000
    jne .not_add_pc_sp

    mov ecx, [rbp-4]
    shr ecx, 8
    and ecx, 0x7
    mov [rbp-16], ecx       ; rd

    mov eax, [rbp-4]
    and eax, 0xFF
    shl eax, 2
    mov [rbp-20], eax       ; imm

    test dword [rbp-4], (1 << 11)
    jz .add_from_pc
    mov eax, [rbx + GBA_CORE.cpu_regs + (13 * 4)]
    jmp .add_addr_store

.add_from_pc:
    mov eax, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    add eax, 2
    and eax, 0xFFFFFFFC

.add_addr_store:
    add eax, [rbp-20]
    mov ecx, [rbp-16]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.not_add_pc_sp:
    ; SP-relative load/store (1001)
    mov eax, [rbp-4]
    and eax, 0xF000
    cmp eax, 0x9000
    jne .not_sp_rel_ls

    mov ecx, [rbp-4]
    shr ecx, 8
    and ecx, 0x7
    mov [rbp-16], ecx       ; rd

    mov eax, [rbp-4]
    and eax, 0xFF
    shl eax, 2
    add eax, [rbx + GBA_CORE.cpu_regs + (13 * 4)]
    mov [rbp-20], eax       ; addr

    test dword [rbp-4], (1 << 11)
    jnz .sp_rel_ldr

    mov ecx, [rbp-16]
    mov r8d, ecx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov rdi, rbx
    mov esi, [rbp-20]
    call cpu_mem_write32
    jmp .done

.sp_rel_ldr:
    mov rdi, rbx
    mov esi, [rbp-20]
    call cpu_mem_read32
    mov ecx, [rbp-16]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.not_sp_rel_ls:
    ; LDMIA/STMIA (1100)
    mov eax, [rbp-4]
    and eax, 0xF000
    cmp eax, 0xC000
    jne .not_ldm_stm

    mov ecx, [rbp-4]
    shr ecx, 8
    and ecx, 0x7
    mov [rbp-16], ecx       ; rb
    mov eax, [rbp-4]
    and eax, 0xFF
    mov [rbp-8], eax        ; rlist
    mov ecx, [rbp-16]
    mov r8d, ecx
    mov eax, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov [rbp-20], eax       ; addr

    test dword [rbp-4], (1 << 11)
    jnz .ldm_loop

    ; STMIA
    mov ecx, 0
.stm_loop:
    cmp ecx, 8
    jae .ldmstm_done
    bt dword [rbp-8], ecx
    jnc .stm_next
    mov r8d, ecx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov rdi, rbx
    mov esi, [rbp-20]
    mov [rbp-44], ecx
    call cpu_mem_write32
    mov ecx, [rbp-44]
    add dword [rbp-20], 4

.stm_next:
    inc ecx
    jmp .stm_loop

.ldm_loop:
    mov ecx, 0
.ldm_iter:
    cmp ecx, 8
    jae .ldmstm_done
    bt dword [rbp-8], ecx
    jnc .ldm_next
    mov rdi, rbx
    mov esi, [rbp-20]
    mov [rbp-44], ecx
    call cpu_mem_read32
    mov ecx, [rbp-44]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    add dword [rbp-20], 4

.ldm_next:
    inc ecx
    jmp .ldm_iter

.ldmstm_done:
    mov ecx, [rbp-16]
    mov r8d, ecx
    mov eax, [rbp-20]
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.not_ldm_stm:
    ; Shift by immediate (000xx)
    mov eax, [rbp-4]
    and eax, 0xF800
    cmp eax, 0x1800
    jae .not_shift_imm

    mov eax, [rbp-4]
    mov ecx, eax
    shr ecx, 11
    and ecx, 0x3             ; op
    mov [rbp-24], ecx
    mov edx, eax
    shr edx, 6
    and edx, 0x1F            ; shift
    mov [rbp-28], edx
    mov esi, eax
    shr esi, 3
    and esi, 0x7             ; rs
    mov [rbp-32], esi
    and eax, 0x7             ; rd
    mov [rbp-36], eax

    mov ecx, [rbp-32]
    mov r8d, ecx
    mov eax, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov [rbp-40], eax        ; src

    mov ecx, [rbp-24]
    cmp ecx, 0
    je .shift_lsl
    cmp ecx, 1
    je .shift_lsr
    cmp ecx, 2
    je .shift_asr
    jmp .done

.shift_lsl:
    mov ecx, [rbp-28]
    mov eax, [rbp-40]
    test ecx, ecx
    jz .shift_store_no_c
    cmp ecx, 32
    jb .shift_lsl_norm
    je .shift_lsl_32
    xor eax, eax
    xor edx, edx
    jmp .shift_store_with_c

.shift_lsl_norm:
    mov edx, eax
    mov r8d, ecx
    mov ecx, 32
    sub ecx, r8d
    shr edx, cl
    and edx, 1
    mov ecx, r8d
    shl eax, cl
    jmp .shift_store_with_c

.shift_lsl_32:
    mov edx, eax
    and edx, 1
    xor eax, eax
    jmp .shift_store_with_c

.shift_lsr:
    mov ecx, [rbp-28]
    mov eax, [rbp-40]
    test ecx, ecx
    jnz .shift_lsr_has
    mov ecx, 32

.shift_lsr_has:
    cmp ecx, 32
    jb .shift_lsr_norm
    mov edx, eax
    shr edx, 31
    and edx, 1
    xor eax, eax
    jmp .shift_store_with_c

.shift_lsr_norm:
    mov edx, eax
    mov r8d, ecx
    dec r8d
    mov ecx, r8d
    shr edx, cl
    and edx, 1
    mov ecx, [rbp-28]
    shr eax, cl
    jmp .shift_store_with_c

.shift_asr:
    mov ecx, [rbp-28]
    mov eax, [rbp-40]
    test ecx, ecx
    jnz .shift_asr_has
    mov ecx, 32

.shift_asr_has:
    cmp ecx, 32
    jb .shift_asr_norm
    mov edx, eax
    shr edx, 31
    and edx, 1
    sar eax, 31
    jmp .shift_store_with_c

.shift_asr_norm:
    mov edx, eax
    mov r8d, ecx
    dec r8d
    mov ecx, r8d
    shr edx, cl
    and edx, 1
    mov ecx, [rbp-28]
    sar eax, cl
    jmp .shift_store_with_c

.shift_store_no_c:
    ; keep old C/V, update N/Z only.
    mov ecx, [rbp-36]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    call cpu_update_nz
    jmp .done

.shift_store_with_c:
    mov ecx, [rbp-36]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    and ecx, ~(CPSR_N | CPSR_Z | CPSR_C)
    test eax, eax
    jnz .shift_nz_check
    or ecx, CPSR_Z

.shift_nz_check:
    test eax, 0x80000000
    jz .shift_carry
    or ecx, CPSR_N

.shift_carry:
    test edx, 1
    jz .shift_store_flags
    or ecx, CPSR_C

.shift_store_flags:
    mov [rbx + GBA_CORE.cpu_cpsr], ecx
    jmp .done

.not_shift_imm:
    ; BX / Hi register ops (010001)
    mov eax, edi
    and eax, 0xFC00
    cmp eax, 0x4400
    jne .not_hi_ops

    mov eax, [rbp-4]
    mov ecx, eax
    shr ecx, 8
    and ecx, 0x3               ; op

    mov edx, eax
    shr edx, 6
    and edx, 0x1               ; H2

    mov esi, eax
    shr esi, 7
    and esi, 0x1               ; H1

    mov r8d, eax
    shr r8d, 3
    and r8d, 0x7               ; Rs low
    shl edx, 3
    or r8d, edx                ; Rs full

    mov r9d, eax
    and r9d, 0x7               ; Rd low
    shl esi, 3
    or r9d, esi                ; Rd full

    mov r10d, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    cmp r8d, 15
    jne .thumb_hi_rs_ready
    add r10d, 2                ; Thumb PC reads as current instruction + 4.
.thumb_hi_rs_ready:
    mov r11d, [rbx + GBA_CORE.cpu_regs + r9 * 4]
    cmp r9d, 15
    jne .thumb_hi_vals_ready
    add r11d, 2
.thumb_hi_vals_ready:

    cmp ecx, 3
    je .thumb_bx
    cmp ecx, 0
    je .thumb_hi_add
    cmp ecx, 1
    je .thumb_hi_cmp

    ; MOV
    mov [rbx + GBA_CORE.cpu_regs + r9 * 4], r10d
    cmp r9d, 15
    jne .done
    and dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 0xFFFFFFFE
    jmp .done

.thumb_hi_add:
    add r11d, r10d
    mov [rbx + GBA_CORE.cpu_regs + r9 * 4], r11d
    cmp r9d, 15
    jne .done
    and dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 0xFFFFFFFE
    jmp .done

.thumb_hi_cmp:
    mov ecx, r11d
    mov edx, r10d
    mov eax, r11d
    sub eax, r10d
    call cpu_set_flags_sub
    jmp .done

.thumb_bx:
    test r10d, 1
    jz .thumb_bx_arm
    or dword [rbx + GBA_CORE.cpu_cpsr], CPSR_T
    and r10d, 0xFFFFFFFE
    mov [rbx + GBA_CORE.cpu_regs + (15 * 4)], r10d
    jmp .done

.thumb_bx_arm:
    and dword [rbx + GBA_CORE.cpu_cpsr], ~CPSR_T
    and r10d, 0xFFFFFFFC
    mov [rbx + GBA_CORE.cpu_regs + (15 * 4)], r10d
    jmp .done

.not_hi_ops:
    ; Move/Compare/Add/Sub immediate (001x)
    mov eax, edi
    and eax, 0xE000
    cmp eax, 0x2000
    jne .not_mcas_imm

    mov ecx, [rbp-4]
    shr ecx, 11
    and ecx, 0x3

    mov edx, [rbp-4]
    shr edx, 8
    and edx, 0x7             ; Rd

    mov esi, [rbp-4]
    and esi, 0xFF            ; imm8

    cmp ecx, 0
    je .thumb_mov_imm
    cmp ecx, 1
    je .thumb_cmp_imm
    cmp ecx, 2
    je .thumb_add_imm

    ; SUB imm
    mov eax, [rbx + GBA_CORE.cpu_regs + rdx * 4]
    mov ecx, eax
    mov edx, esi
    sub eax, edx
    mov r8d, [rbp-4]
    shr r8d, 8
    and r8d, 0x7
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    call cpu_set_flags_sub
    jmp .done

.thumb_mov_imm:
    mov [rbx + GBA_CORE.cpu_regs + rdx * 4], esi
    mov eax, esi
    call cpu_update_nz
    jmp .done

.thumb_cmp_imm:
    mov eax, [rbx + GBA_CORE.cpu_regs + rdx * 4]
    mov ecx, eax
    mov edx, esi
    sub eax, edx
    call cpu_set_flags_sub
    jmp .done

.thumb_add_imm:
    mov eax, [rbx + GBA_CORE.cpu_regs + rdx * 4]
    mov ecx, eax
    mov r8d, edx
    mov edx, esi
    add eax, edx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    call cpu_set_flags_add
    jmp .done

.not_mcas_imm:
    ; Add/sub register/immediate (00011)
    mov eax, edi
    and eax, 0xF800
    cmp eax, 0x1800
    jb .not_addsub
    cmp eax, 0x2000
    jae .not_addsub

    mov eax, [rbp-4]
    mov ecx, eax
    and ecx, 0x7              ; Rd

    mov edx, eax
    shr edx, 3
    and edx, 0x7              ; Rs

    mov esi, eax
    shr esi, 6
    and esi, 0x7              ; Rn/imm3

    mov r8d, [rbx + GBA_CORE.cpu_regs + rdx * 4] ; op1

    test eax, 0x0400
    jz .thumb_addsub_reg
    mov r9d, esi
    jmp .thumb_addsub_do

.thumb_addsub_reg:
    mov r9d, [rbx + GBA_CORE.cpu_regs + rsi * 4]

.thumb_addsub_do:
    test eax, 0x0200
    jz .thumb_addsub_add

    mov ecx, r8d
    mov edx, r9d
    mov eax, r8d
    sub eax, r9d
    mov r10d, [rbp-4]
    and r10d, 0x7
    mov [rbx + GBA_CORE.cpu_regs + r10 * 4], eax
    call cpu_set_flags_sub
    jmp .done

.thumb_addsub_add:
    mov ecx, r8d
    mov edx, r9d
    mov eax, r8d
    add eax, r9d
    mov r10d, [rbp-4]
    and r10d, 0x7
    mov [rbx + GBA_CORE.cpu_regs + r10 * 4], eax
    call cpu_set_flags_add
    jmp .done

.not_addsub:
    ; ALU operations (010000)
    mov eax, edi
    and eax, 0xFC00
    cmp eax, 0x4000
    jne .not_alu

    mov eax, [rbp-4]
    mov ecx, eax
    shr ecx, 6
    and ecx, 0xF             ; op

    mov edx, eax
    shr edx, 3
    and edx, 0x7             ; Rs

    mov esi, eax
    and esi, 0x7             ; Rd

    mov r8d, [rbx + GBA_CORE.cpu_regs + rsi * 4] ; rd val
    mov r9d, [rbx + GBA_CORE.cpu_regs + rdx * 4] ; rs val

    cmp ecx, 0
    je .thumb_alu_and
    cmp ecx, 1
    je .thumb_alu_eor
    cmp ecx, 2
    je .thumb_alu_lsl
    cmp ecx, 3
    je .thumb_alu_lsr
    cmp ecx, 4
    je .thumb_alu_asr
    cmp ecx, 5
    je .thumb_alu_adc
    cmp ecx, 6
    je .thumb_alu_sbc
    cmp ecx, 7
    je .thumb_alu_ror
    cmp ecx, 8
    je .thumb_alu_tst
    cmp ecx, 9
    je .thumb_alu_neg
    cmp ecx, 10
    je .thumb_alu_cmp
    cmp ecx, 11
    je .thumb_alu_cmn
    cmp ecx, 12
    je .thumb_alu_orr
    cmp ecx, 13
    je .thumb_alu_mul
    cmp ecx, 14
    je .thumb_alu_bic
    cmp ecx, 15
    je .thumb_alu_mvn
    jmp .done

.thumb_alu_and:
    mov eax, r8d
    and eax, r9d
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    call cpu_update_nz
    jmp .done

.thumb_alu_eor:
    mov eax, r8d
    xor eax, r9d
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    call cpu_update_nz
    jmp .done

.thumb_alu_lsl:
    mov ecx, r9d
    and ecx, 0xFF
    mov eax, r8d
    shl eax, cl
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    call cpu_update_nz
    jmp .done

.thumb_alu_lsr:
    mov ecx, r9d
    and ecx, 0xFF
    mov eax, r8d
    shr eax, cl
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    call cpu_update_nz
    jmp .done

.thumb_alu_asr:
    mov ecx, r9d
    and ecx, 0xFF
    mov eax, r8d
    sar eax, cl
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    call cpu_update_nz
    jmp .done

.thumb_alu_tst:
    mov eax, r8d
    and eax, r9d
    call cpu_update_nz
    jmp .done

.thumb_alu_cmp:
    mov ecx, r8d
    mov edx, r9d
    mov eax, r8d
    sub eax, r9d
    call cpu_set_flags_sub
    jmp .done

.thumb_alu_orr:
    mov eax, r8d
    or eax, r9d
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    call cpu_update_nz
    jmp .done

.thumb_alu_mvn:
    mov eax, r9d
    not eax
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    call cpu_update_nz
    jmp .done

.thumb_alu_adc:
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    test ecx, CPSR_C
    jz .adc_c0
    stc
    jmp .adc_go
.adc_c0:
    clc
.adc_go:
    mov eax, r8d
    adc eax, r9d
    setc r10b
    seto r11b
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    and ecx, ~(CPSR_N | CPSR_Z | CPSR_C | CPSR_V)
    test eax, eax
    jnz .adc_no_z
    or ecx, CPSR_Z
.adc_no_z:
    test eax, 0x80000000
    jz .adc_no_n
    or ecx, CPSR_N
.adc_no_n:
    test r10b, r10b
    jz .adc_no_c
    or ecx, CPSR_C
.adc_no_c:
    test r11b, r11b
    jz .adc_store
    or ecx, CPSR_V
.adc_store:
    mov [rbx + GBA_CORE.cpu_cpsr], ecx
    jmp .done

.thumb_alu_sbc:
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    test ecx, CPSR_C
    jz .sbc_c0
    clc
    jmp .sbc_go
.sbc_c0:
    stc
.sbc_go:
    mov eax, r8d
    sbb eax, r9d
    setc r10b                 ; borrow
    seto r11b
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    and ecx, ~(CPSR_N | CPSR_Z | CPSR_C | CPSR_V)
    test eax, eax
    jnz .sbc_no_z
    or ecx, CPSR_Z
.sbc_no_z:
    test eax, 0x80000000
    jz .sbc_no_n
    or ecx, CPSR_N
.sbc_no_n:
    test r10b, r10b
    jnz .sbc_no_c
    or ecx, CPSR_C
.sbc_no_c:
    test r11b, r11b
    jz .sbc_store
    or ecx, CPSR_V
.sbc_store:
    mov [rbx + GBA_CORE.cpu_cpsr], ecx
    jmp .done

.thumb_alu_ror:
    mov ecx, r9d
    and ecx, 0xFF
    mov eax, r8d
    test ecx, ecx
    jz .thumb_alu_ror_noc
    and ecx, 31
    jnz .thumb_alu_ror_do
    mov ecx, 32
.thumb_alu_ror_do:
    ror eax, cl
    mov edx, eax
    shr edx, 31
    jmp .thumb_alu_ror_store

.thumb_alu_ror_noc:
    mov edx, [rbx + GBA_CORE.cpu_cpsr]
    and edx, CPSR_C
    shr edx, 29

.thumb_alu_ror_store:
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    mov ecx, [rbx + GBA_CORE.cpu_cpsr]
    and ecx, ~(CPSR_N | CPSR_Z | CPSR_C)
    test eax, eax
    jnz .thumb_alu_ror_nz
    or ecx, CPSR_Z
.thumb_alu_ror_nz:
    test eax, 0x80000000
    jz .thumb_alu_ror_c
    or ecx, CPSR_N
.thumb_alu_ror_c:
    test edx, 1
    jz .thumb_alu_ror_flags
    or ecx, CPSR_C
.thumb_alu_ror_flags:
    mov [rbx + GBA_CORE.cpu_cpsr], ecx
    jmp .done

.thumb_alu_neg:
    xor ecx, ecx
    mov edx, r9d
    mov eax, ecx
    sub eax, edx
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    call cpu_set_flags_sub
    jmp .done

.thumb_alu_cmn:
    mov ecx, r8d
    mov edx, r9d
    mov eax, r8d
    add eax, r9d
    call cpu_set_flags_add
    jmp .done

.thumb_alu_mul:
    mov eax, r8d
    imul eax, r9d
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    call cpu_update_nz
    jmp .done

.thumb_alu_bic:
    mov eax, r8d
    not r9d
    and eax, r9d
    mov [rbx + GBA_CORE.cpu_regs + rsi * 4], eax
    call cpu_update_nz
    jmp .done

.not_alu:
    ; PC-relative LDR (01001)
    mov eax, edi
    and eax, 0xF800
    cmp eax, 0x4800
    jne .not_pc_rel_ldr

    mov ecx, [rbp-4]
    shr ecx, 8
    and ecx, 0x7             ; Rd
    mov [rbp-44], ecx

    mov edx, [rbp-4]
    and edx, 0xFF
    shl edx, 2

    mov esi, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    add esi, 2
    and esi, 0xFFFFFFFC
    add esi, edx

    mov rdi, rbx
    call cpu_mem_read32
    mov ecx, [rbp-44]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.not_pc_rel_ldr:
    ; Register-offset and signed/halfword load/store (0101)
    mov eax, [rbp-4]
    and eax, 0xF000
    cmp eax, 0x5000
    jne .not_ls_reg

    mov eax, [rbp-4]
    mov ecx, eax
    shr ecx, 9
    and ecx, 0x7
    mov [rbp-24], ecx       ; subop
    mov edx, eax
    shr edx, 6
    and edx, 0x7
    mov [rbp-28], edx       ; rm
    mov esi, eax
    shr esi, 3
    and esi, 0x7
    mov [rbp-32], esi       ; rn
    and eax, 0x7
    mov [rbp-36], eax       ; rd

    mov ecx, [rbp-32]
    mov r8d, ecx
    mov r9d, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov ecx, [rbp-28]
    mov r8d, ecx
    add r9d, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov [rbp-40], r9d       ; addr

    mov ecx, [rbp-24]
    cmp ecx, 0
    je .ls_reg_str
    cmp ecx, 1
    je .ls_reg_strh
    cmp ecx, 2
    je .ls_reg_strb
    cmp ecx, 3
    je .ls_reg_ldrsb
    cmp ecx, 4
    je .ls_reg_ldr
    cmp ecx, 5
    je .ls_reg_ldrh
    cmp ecx, 6
    je .ls_reg_ldrb
    ; 7 -> LDRSH
    jmp .ls_reg_ldrsh

.ls_reg_str:
    mov ecx, [rbp-36]
    mov r8d, ecx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov rdi, rbx
    mov esi, [rbp-40]
    call cpu_mem_write32
    jmp .done

.ls_reg_strh:
    mov ecx, [rbp-36]
    mov r8d, ecx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov rdi, rbx
    mov esi, [rbp-40]
    call cpu_mem_write16
    jmp .done

.ls_reg_strb:
    mov ecx, [rbp-36]
    mov r8d, ecx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov rdi, rbx
    mov esi, [rbp-40]
    call cpu_mem_write8
    jmp .done

.ls_reg_ldrsb:
    mov rdi, rbx
    mov esi, [rbp-40]
    call cpu_mem_read8
    movsx eax, al
    mov ecx, [rbp-36]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.ls_reg_ldr:
    mov rdi, rbx
    mov esi, [rbp-40]
    call cpu_mem_read32
    mov ecx, [rbp-36]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.ls_reg_ldrh:
    mov rdi, rbx
    mov esi, [rbp-40]
    call cpu_mem_read16
    mov ecx, [rbp-36]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.ls_reg_ldrb:
    mov rdi, rbx
    mov esi, [rbp-40]
    call cpu_mem_read8
    mov ecx, [rbp-36]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.ls_reg_ldrsh:
    mov rdi, rbx
    mov esi, [rbp-40]
    call cpu_mem_read16
    movsx eax, ax
    mov ecx, [rbp-36]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.not_ls_reg:
    ; Load/store immediate (011x / 1000 subset)
    mov eax, edi
    and eax, 0xE000
    cmp eax, 0x6000
    jne .not_ls_imm

    mov eax, [rbp-4]
    mov ecx, eax
    shr ecx, 11
    and ecx, 0x3             ; op: STR/LDR/STRB/LDRB

    mov edx, eax
    shr edx, 6
    and edx, 0x1F            ; imm5

    mov esi, eax
    shr esi, 3
    and esi, 0x7             ; Rn

    mov r8d, eax
    and r8d, 0x7             ; Rd

    mov r9d, [rbx + GBA_CORE.cpu_regs + rsi * 4] ; base

    cmp ecx, 0
    je .thumb_ls_str
    cmp ecx, 1
    je .thumb_ls_ldr
    cmp ecx, 2
    je .thumb_ls_strb

    ; LDRB
    add r9d, edx
    mov [rbp-44], r8d
    mov rdi, rbx
    mov esi, r9d
    call cpu_mem_read8
    mov r8d, [rbp-44]
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.thumb_ls_strb:
    add r9d, edx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov rdi, rbx
    mov esi, r9d
    call cpu_mem_write8
    jmp .done

.thumb_ls_str:
    shl edx, 2
    add r9d, edx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov rdi, rbx
    mov esi, r9d
    call cpu_mem_write32
    jmp .done

.thumb_ls_ldr:
    shl edx, 2
    add r9d, edx
    mov [rbp-44], r8d
    mov rdi, rbx
    mov esi, r9d
    call cpu_mem_read32
    mov r8d, [rbp-44]
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.not_ls_imm:
    ; Load/store halfword immediate (1000)
    mov eax, [rbp-4]
    and eax, 0xF000
    cmp eax, 0x8000
    jne .not_ls_half_imm

    mov ecx, [rbp-4]
    shr ecx, 6
    and ecx, 0x1F
    shl ecx, 1
    mov [rbp-24], ecx        ; imm

    mov edx, [rbp-4]
    shr edx, 3
    and edx, 0x7
    mov [rbp-28], edx        ; rn

    mov eax, [rbp-4]
    and eax, 0x7
    mov [rbp-32], eax        ; rd

    mov ecx, [rbp-28]
    mov r8d, ecx
    mov esi, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    add esi, [rbp-24]
    mov [rbp-36], esi        ; addr

    test dword [rbp-4], (1 << 11)
    jnz .ls_half_ldr

    mov ecx, [rbp-32]
    mov r8d, ecx
    mov edx, [rbx + GBA_CORE.cpu_regs + r8 * 4]
    mov rdi, rbx
    mov esi, [rbp-36]
    call cpu_mem_write16
    jmp .done

.ls_half_ldr:
    mov rdi, rbx
    mov esi, [rbp-36]
    call cpu_mem_read16
    mov ecx, [rbp-32]
    mov r8d, ecx
    mov [rbx + GBA_CORE.cpu_regs + r8 * 4], eax
    jmp .done

.not_ls_half_imm:
    ; Fallback: unknown thumb opcode.
    add dword [rbx + GBA_CORE.cpu_unknown], 1

.done:
    leave
    ret

; -----------------------------------------------------------------------------
; Main stepping loop
; -----------------------------------------------------------------------------

; void gba_cpu_step_chunk(GbaCore *core, uint32_t cycles)
gba_cpu_step_chunk:
    push rbp
    mov rbp, rsp
    push rbx
    push r12
    push r13

    mov rbx, rdi
    mov r12d, esi

    test rbx, rbx
    jz .done
    cmp dword [rbx + GBA_CORE.magic], GBA_MAGIC
    jne .done

.loop:
    test r12d, r12d
    jz .sync

    ; Normal IRQ delivery path (not only HALT wakeups).
    call cpu_irq_pending
    test eax, eax
    jz .irq_checked

    mov eax, [rbx + GBA_CORE.cpu_cpsr]
    test eax, CPSR_I
    jnz .irq_checked
    mov ecx, eax
    and ecx, 0x1F
    cmp ecx, 0x12
    je .irq_checked
    call cpu_enter_irq

.irq_checked:
    cmp dword [rbx + GBA_CORE.halted], 0
    jne .tick_only

    mov eax, [rbx + GBA_CORE.cpu_cpsr]
    test eax, CPSR_T
    jz .arm

.thumb:
    mov rdi, rbx
    mov esi, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    call cpu_mem_read16
    mov edi, eax
    add dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 2
    call cpu_exec_thumb
    jmp .tick

.arm:
    mov rdi, rbx
    mov esi, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    call cpu_mem_read32
    mov edi, eax
    add dword [rbx + GBA_CORE.cpu_regs + (15 * 4)], 4
    call cpu_exec_arm
    jmp .tick

.tick_only:
    call cpu_irq_pending
    test eax, eax
    jz .tick

    mov eax, [rbx + GBA_CORE.cpu_cpsr]
    test eax, CPSR_I
    jnz .wake_halted_only
    call cpu_enter_irq
    jmp .tick

.wake_halted_only:
    mov dword [rbx + GBA_CORE.halted], 0

.tick:
    dec r12d
    jmp .loop

.sync:
    mov eax, [rbx + GBA_CORE.cpu_regs + (15 * 4)]
    mov [rbx + GBA_CORE.cpu_pc], eax

.done:
    pop r13
    pop r12
    pop rbx
    leave
    ret
