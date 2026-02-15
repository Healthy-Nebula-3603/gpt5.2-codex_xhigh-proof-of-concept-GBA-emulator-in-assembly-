default rel
bits 64

%include "gba_core.inc"

%define GBA_KEY_START 8
%define GBA_KEY_RIGHT 16
%define GBA_KEY_LEFT 32
%define GBA_KEY_UP 64
%define GBA_KEY_DOWN 128

extern gba_mem_read16
extern gba_mem_read32
extern gba_mem_write16
extern gba_mem_write32

section .text
global gba_sys_tick

; void gba_sys_tick(GbaCore *core, uint32_t cycles)
gba_sys_tick:
    push rbp
    mov rbp, rsp
    sub rsp, 192

    ; locals
    ; [rbp-8]   core
    ; [rbp-12]  keymask
    ; [rbp-16]  cycles
    ; [rbp-20]  vblank_event
    ; [rbp-24]  timer_i
    ; [rbp-40]  overflow[0]
    ; [rbp-36]  overflow[1]
    ; [rbp-32]  overflow[2]
    ; [rbp-28]  overflow[3]
    ; [rbp-44]  dma_i
    ; [rbp-48]  dma_timing
    ; [rbp-52]  dma_count
    ; [rbp-56]  dma_size
    ; [rbp-60]  dma_base
    ; [rbp-64]  dma_ctrl
    ; [rbp-68]  dma_src
    ; [rbp-72]  dma_dst
    ; [rbp-76]  dma_dst_orig
    ; [rbp-80]  timer_base
    ; [rbp-84]  timer_increments
    ; [rbp-88]  timer_prescale
    ; [rbp-92]  timer_count
    ; [rbp-96]  timer_reload
    ; [rbp-100] timer_overflows
    ; [rbp-104] timer_ctrl

    mov [rbp-8], rdi
    test rdi, rdi
    jz .done

    cmp dword [rdi + GBA_CORE.magic], GBA_MAGIC
    jne .done

    mov [rbp-16], esi

    movsxd rax, esi
    add qword [rdi + GBA_CORE.ticks], rax
    inc qword [rdi + GBA_CORE.frame_count]

    movzx eax, word [rdi + GBA_CORE.input_mask]
    mov [rbp-12], eax

    ; KEYINPUT is active-low. Bits 10..15 are unused and read back as 1.
    mov ecx, eax
    not ecx
    and ecx, 0x03FF
    or ecx, 0xFC00
    mov [rdi + GBA_CORE.io + IO_KEYINPUT], cx

    ; KEYCNT keypad IRQ: trigger on false->true condition edge.
    mov r11, [rbp-8]
    movzx ecx, word [r11 + GBA_CORE.io + IO_KEYCNT]
    test ecx, 0x4000
    jz .keyirq_disabled
    mov edx, ecx
    and edx, 0x03FF
    test edx, edx
    jz .keyirq_disabled

    movzx eax, word [r11 + GBA_CORE.io + IO_KEYINPUT]
    not eax
    and eax, 0x03FF

    test ecx, 0x8000
    jnz .keyirq_and_mode

    and eax, edx
    xor ecx, ecx
    test eax, eax
    setnz cl
    jmp .keyirq_cond_ready

.keyirq_and_mode:
    and eax, edx
    xor ecx, ecx
    cmp eax, edx
    sete cl

.keyirq_cond_ready:
    movzx edx, byte [r11 + GBA_CORE.keypad_irq_latched]
    test ecx, ecx
    jz .keyirq_store
    test edx, edx
    jnz .keyirq_store
    movzx eax, word [r11 + GBA_CORE.io + IO_IF]
    or eax, IRQ_KEYPAD
    mov [r11 + GBA_CORE.io + IO_IF], ax

.keyirq_store:
    mov [r11 + GBA_CORE.keypad_irq_latched], cl
    jmp .keyirq_done

.keyirq_disabled:
    mov byte [r11 + GBA_CORE.keypad_irq_latched], 0

.keyirq_done:

    ; Immediate DMA.
    mov dword [rbp-48], 0
    jmp .dma_process_start

.after_dma_immediate:
    ; Approximate VCOUNT stepping (1232 cycles per scanline).
    mov r11, [rbp-8]
    mov eax, [r11 + GBA_CORE.scanline_cycles]
    mov [rbp-108], eax
    add eax, [rbp-16]
    mov dword [rbp-20], 0

    ; Enter-HBlank edge for this tick (HDraw=960, HBlank=272).
    mov ecx, [rbp-108]
    cmp ecx, 960
    jae .scanline_loop
    cmp eax, 960
    jb .scanline_loop
    movzx edx, word [r11 + GBA_CORE.io + IO_DISPSTAT]
    test edx, 0x0010
    jz .scanline_loop
    movzx edx, word [r11 + GBA_CORE.io + IO_IF]
    or edx, IRQ_HBLANK
    mov [r11 + GBA_CORE.io + IO_IF], dx
    ; HBlank DMA.
    mov dword [rbp-48], 2
    jmp .dma_process_start

.after_dma_hblank:

.scanline_loop:
    cmp eax, 1232
    jb .scanline_done
    sub eax, 1232

    mov r11, [rbp-8]
    movzx ecx, word [r11 + GBA_CORE.io + IO_VCOUNT]
    inc ecx
    cmp ecx, 160
    jne .not_vblank_start

    movzx edx, word [r11 + GBA_CORE.io + IO_DISPSTAT]
    test edx, 0x0008
    jz .vblank_latch_update
    movzx edx, word [r11 + GBA_CORE.io + IO_IF]
    or edx, IRQ_VBLANK
    mov [r11 + GBA_CORE.io + IO_IF], dx

.vblank_latch_update:
    movzx edx, word [r11 + GBA_CORE.iwram + 0x1178]
    or edx, IRQ_VBLANK
    mov [r11 + GBA_CORE.iwram + 0x1178], dx
    movzx edx, word [r11 + GBA_CORE.iwram + 0x7FF8]
    or edx, IRQ_VBLANK
    mov [r11 + GBA_CORE.iwram + 0x7FF8], dx
.vblank_irq_done:
    mov dword [rbp-20], 1

.not_vblank_start:
    cmp ecx, 228
    jb .store_vcount
    xor ecx, ecx

.store_vcount:
    mov [r11 + GBA_CORE.io + IO_VCOUNT], cx
    mov [r11 + GBA_CORE.iwram + 0x7FF0], cl

    ; VCounter match IRQ on line transitions.
    movzx edx, word [r11 + GBA_CORE.io + IO_DISPSTAT]
    mov esi, edx
    shr esi, 8
    and esi, 0xFF
    cmp ecx, esi
    jne .scanline_loop
    test edx, 0x0020
    jz .scanline_loop
    movzx edx, word [r11 + GBA_CORE.io + IO_IF]
    or edx, IRQ_VCOUNT
    mov [r11 + GBA_CORE.io + IO_IF], dx
    jmp .scanline_loop

.scanline_done:
    mov r11, [rbp-8]
    mov [r11 + GBA_CORE.scanline_cycles], eax

    ; Update DISPSTAT read-only status bits (VBlank/HBlank/VCount-match).
    movzx ecx, word [r11 + GBA_CORE.io + IO_DISPSTAT]
    mov edx, ecx
    shr edx, 8
    and edx, 0xFF
    and ecx, 0xFFF8
    movzx esi, word [r11 + GBA_CORE.io + IO_VCOUNT]
    cmp esi, 160
    jb .dispstat_no_vblank
    or ecx, 0x0001
.dispstat_no_vblank:
    cmp eax, 960
    jb .dispstat_no_hblank
    or ecx, 0x0002
.dispstat_no_hblank:
    cmp esi, edx
    jne .dispstat_no_vcount
    or ecx, 0x0004
.dispstat_no_vcount:
    mov [r11 + GBA_CORE.io + IO_DISPSTAT], cx

    ; Timer state.
    mov dword [rbp-40], 0
    mov dword [rbp-36], 0
    mov dword [rbp-32], 0
    mov dword [rbp-28], 0
    mov dword [rbp-24], 0

.timer_loop:
    mov ecx, [rbp-24]
    cmp ecx, 4
    jae .timers_done

    mov eax, ecx
    shl eax, 2
    add eax, IO_TM0CNT_L
    mov [rbp-80], eax

    mov r11, [rbp-8]
    movzx edx, word [r11 + GBA_CORE.io + rax + 2]
    mov [rbp-104], edx

    test edx, 0x0080
    jnz .timer_enabled

    ; disabled
    mov ecx, [rbp-24]
    mov byte [r11 + GBA_CORE.timer_latched_en + rcx], 0
    mov qword [r11 + GBA_CORE.timer_accum + rcx * 8], 0
    mov dword [rbp-40 + rcx * 4], 0
    jmp .timer_next

.timer_enabled:
    mov ecx, [rbp-24]
    movzx eax, byte [r11 + GBA_CORE.timer_latched_en + rcx]
    test eax, eax
    jnz .timer_enabled_already

    mov byte [r11 + GBA_CORE.timer_latched_en + rcx], 1
    movzx eax, word [r11 + GBA_CORE.timer_reload + rcx * 2]
    mov esi, [rbp-80]
    mov [r11 + GBA_CORE.io + rsi], ax
    mov qword [r11 + GBA_CORE.timer_accum + rcx * 8], 0

.timer_enabled_already:
    mov byte [r11 + GBA_CORE.timer_latched_en + rcx], 1
    mov dword [rbp-84], 0

    cmp ecx, 0
    je .timer_prescaled

    mov eax, [rbp-104]
    test eax, 0x0004
    jz .timer_prescaled

    ; count-up mode
    mov eax, [rbp-40 + rcx * 4 - 4]
    mov [rbp-84], eax
    jmp .timer_increments_ready

.timer_prescaled:
    mov eax, 1
    mov esi, [rbp-104]
    and esi, 0x0003
    cmp esi, 0
    je .prescale_ready
    cmp esi, 1
    jne .not_div64
    mov eax, 64
    jmp .prescale_ready
.not_div64:
    cmp esi, 2
    jne .not_div256
    mov eax, 256
    jmp .prescale_ready
.not_div256:
    mov eax, 1024

.prescale_ready:
    mov [rbp-88], eax

    mov ecx, [rbp-24]
    movsxd r9, ecx
    mov rax, [r11 + GBA_CORE.timer_accum + r9 * 8]
    movsxd rdx, dword [rbp-16]
    add rax, rdx

    xor edx, edx
    mov ecx, [rbp-88]
    div ecx

    mov [rbp-84], eax

    mov ecx, [rbp-24]
    movsxd r9, ecx
    mov [r11 + GBA_CORE.timer_accum + r9 * 8], rdx

.timer_increments_ready:
    mov eax, [rbp-84]
    test eax, eax
    jz .timer_no_overflow

    mov ecx, [rbp-24]
    mov esi, [rbp-80]
    mov r11, [rbp-8]
    movzx r8d, word [r11 + GBA_CORE.io + rsi]
    movzx r9d, word [r11 + GBA_CORE.timer_reload + rcx * 2]

    mov [rbp-92], r8d
    mov [rbp-96], r9d

    mov r10d, r8d
    add r10d, eax
    xor r8d, r8d

.timer_overflow_loop:
    cmp r10d, 0x10000
    jb .timer_overflow_done
    sub r10d, 0x10000
    add r10d, [rbp-96]
    inc r8d
    jmp .timer_overflow_loop

.timer_overflow_done:
    mov [rbp-100], r8d

    mov ecx, [rbp-24]
    mov [rbp-40 + rcx * 4], r8d

    mov esi, [rbp-80]
    mov r11, [rbp-8]
    mov [r11 + GBA_CORE.io + rsi], r10w

    test r8d, r8d
    jz .timer_next

    mov eax, [rbp-104]
    test eax, 0x0040
    jz .timer_next

    movzx eax, word [r11 + GBA_CORE.io + IO_IF]
    mov ecx, [rbp-24]
    mov r9d, IRQ_TIMER0
    shl r9d, cl
    or eax, r9d
    mov [r11 + GBA_CORE.io + IO_IF], ax

    ; Minimal BIOS-like IRQ callback latch used by some ROM wait loops.
    cmp ecx, 2
    jne .timer_next
    mov byte [r11 + GBA_CORE.iwram + 0x0C4C], 1
    movzx eax, word [r11 + GBA_CORE.iwram + 0x1178]
    or eax, IRQ_TIMER2
    mov [r11 + GBA_CORE.iwram + 0x1178], ax
    movzx eax, word [r11 + GBA_CORE.iwram + 0x7FF8]
    or eax, IRQ_TIMER2
    mov [r11 + GBA_CORE.iwram + 0x7FF8], ax
    jmp .timer_next

.timer_no_overflow:
    mov ecx, [rbp-24]
    mov dword [rbp-40 + rcx * 4], 0

.timer_next:
    add dword [rbp-24], 1
    jmp .timer_loop

.timers_done:
    ; BIOS-style software IRQ latch:
    ; when any enabled interrupt is pending, reflect pending bits in IWRAM.
    mov r11, [rbp-8]
    movzx eax, word [r11 + GBA_CORE.io + IO_IE]
    movzx edx, word [r11 + GBA_CORE.io + IO_IF]
    and eax, edx
    test eax, eax
    jz .irq_latch_done
    movzx ecx, word [r11 + GBA_CORE.iwram + 0x1178]
    movzx edx, word [r11 + GBA_CORE.iwram + 0x7FF8]
    or ecx, eax
    or edx, eax
    mov [r11 + GBA_CORE.iwram + 0x1178], cx
    mov [r11 + GBA_CORE.iwram + 0x7FF8], dx

.irq_latch_done:
    cmp dword [rbp-20], 0
    jz .input_phase

    mov dword [rbp-48], 1
    jmp .dma_process_start

.after_dma_vblank:
.input_phase:
    mov eax, [rbp-12]

    test eax, GBA_KEY_RIGHT
    jz .no_right
    mov r11, [rbp-8]
    add dword [r11 + GBA_CORE.bg_phase], 2

.no_right:
    test eax, GBA_KEY_LEFT
    jz .no_left
    mov r11, [rbp-8]
    sub dword [r11 + GBA_CORE.bg_phase], 2

.no_left:
    test eax, GBA_KEY_UP
    jz .no_up
    mov r11, [rbp-8]
    add dword [r11 + GBA_CORE.bg_phase], 4

.no_up:
    test eax, GBA_KEY_DOWN
    jz .no_down
    mov r11, [rbp-8]
    sub dword [r11 + GBA_CORE.bg_phase], 4

.no_down:
    test eax, GBA_KEY_START
    jz .done

    mov r11, [rbp-8]
    mov dword [r11 + GBA_CORE.save_dirty], 1
    jmp .done

; DMA process for timing mode in [rbp-48].
.dma_process_start:
    mov dword [rbp-44], 0

.dma_channel_loop:
    mov ecx, [rbp-44]
    cmp ecx, 4
    jae .dma_process_done

    mov eax, ecx
    imul eax, IO_DMA_STRIDE
    add eax, IO_DMA0SAD
    mov [rbp-60], eax

    mov r11, [rbp-8]
    mov esi, [rbp-60]
    movzx edx, word [r11 + GBA_CORE.io + rsi + 10]
    mov [rbp-64], edx

    test edx, 0x8000
    jz .dma_next

    mov eax, edx
    shr eax, 12
    and eax, 0x3
    cmp eax, [rbp-48]
    jne .dma_next

    mov eax, [r11 + GBA_CORE.io + rsi]
    mov [rbp-68], eax
    mov eax, [r11 + GBA_CORE.io + rsi + 4]
    mov [rbp-72], eax
    mov [rbp-76], eax

    movzx eax, word [r11 + GBA_CORE.io + rsi + 8]
    test eax, eax
    jnz .dma_count_ready

    mov ecx, [rbp-44]
    cmp ecx, 3
    jne .dma_small_default
    mov eax, 0x10000
    jmp .dma_count_ready

.dma_small_default:
    mov eax, 0x4000

.dma_count_ready:
    mov [rbp-52], eax

    mov edx, [rbp-64]
    test edx, (1 << 10)
    jz .dma_size16
    mov dword [rbp-56], 4
    jmp .dma_size_done

.dma_size16:
    mov dword [rbp-56], 2

.dma_size_done:
.dma_transfer_loop:
    cmp dword [rbp-52], 0
    jz .dma_transfer_done

    mov edx, [rbp-64]
    test edx, (1 << 10)
    jz .dma_do_16

    mov rdi, [rbp-8]
    mov esi, [rbp-68]
    call gba_mem_read32
    mov edx, eax
    mov rdi, [rbp-8]
    mov esi, [rbp-72]
    call gba_mem_write32
    jmp .dma_adjust

.dma_do_16:
    mov rdi, [rbp-8]
    mov esi, [rbp-68]
    call gba_mem_read16
    mov edx, eax
    mov rdi, [rbp-8]
    mov esi, [rbp-72]
    call gba_mem_write16

.dma_adjust:
    ; src adjust
    mov edx, [rbp-64]
    shr edx, 7
    and edx, 0x3
    cmp edx, 0
    je .dma_src_inc
    cmp edx, 1
    je .dma_src_dec
    jmp .dma_dst_adjust

.dma_src_inc:
    mov eax, [rbp-56]
    add [rbp-68], eax
    jmp .dma_dst_adjust

.dma_src_dec:
    mov eax, [rbp-56]
    sub [rbp-68], eax

.dma_dst_adjust:
    mov edx, [rbp-64]
    shr edx, 5
    and edx, 0x3
    cmp edx, 0
    je .dma_dst_inc
    cmp edx, 1
    je .dma_dst_dec
    cmp edx, 2
    je .dma_count_dec
    ; mode 3 (reload): increment during transfer

.dma_dst_inc:
    mov eax, [rbp-56]
    add [rbp-72], eax
    jmp .dma_count_dec

.dma_dst_dec:
    mov eax, [rbp-56]
    sub [rbp-72], eax

.dma_count_dec:
    sub dword [rbp-52], 1
    jmp .dma_transfer_loop

.dma_transfer_done:
    mov r11, [rbp-8]
    mov esi, [rbp-60]
    mov eax, [rbp-68]
    mov [r11 + GBA_CORE.io + rsi], eax

    mov eax, [rbp-72]
    mov [r11 + GBA_CORE.io + rsi + 4], eax

    ; destination reload on repeat non-immediate mode.
    mov edx, [rbp-64]
    mov ecx, edx
    shr ecx, 5
    and ecx, 0x3
    cmp ecx, 3
    jne .dma_irq_eval
    test edx, (1 << 9)
    jz .dma_irq_eval
    cmp dword [rbp-48], 0
    je .dma_irq_eval
    mov eax, [rbp-76]
    mov [r11 + GBA_CORE.io + rsi + 4], eax

.dma_irq_eval:
    test edx, (1 << 14)
    jz .dma_enable_eval
    movzx eax, word [r11 + GBA_CORE.io + IO_IF]
    mov ecx, [rbp-44]
    mov r9d, IRQ_DMA0
    shl r9d, cl
    or eax, r9d
    mov [r11 + GBA_CORE.io + IO_IF], ax

.dma_enable_eval:
    ; disable immediate DMA or non-repeat transfers.
    cmp dword [rbp-48], 0
    je .dma_disable
    test edx, (1 << 9)
    jz .dma_disable
    jmp .dma_next

.dma_disable:
    and edx, 0x7FFF
    mov [r11 + GBA_CORE.io + rsi + 10], dx

.dma_next:
    add dword [rbp-44], 1
    jmp .dma_channel_loop

.dma_process_done:
    cmp dword [rbp-48], 0
    je .after_dma_immediate
    cmp dword [rbp-48], 2
    je .after_dma_hblank
    jmp .after_dma_vblank

.done:
    leave
    ret
