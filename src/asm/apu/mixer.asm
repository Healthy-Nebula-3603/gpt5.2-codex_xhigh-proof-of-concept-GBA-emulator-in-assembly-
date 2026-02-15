default rel
bits 64

%include "gba_core.inc"

%define GBA_KEY_A 1
%define GBA_KEY_B 2

section .text
global gba_apu_mix

; void gba_apu_mix(GbaCore *core)
gba_apu_mix:
    test rdi, rdi
    jz .done

    cmp dword [rdi + GBA_CORE.magic], GBA_MAGIC
    jne .done

    lea rsi, [rdi + GBA_CORE.audio]
    mov r8, [rdi + GBA_CORE.frame_count]
    shl r8, 10
    movzx r9d, word [rdi + GBA_CORE.input_mask]

    xor ecx, ecx

.sample_loop:
    cmp ecx, GBA_AUDIO_SAMPLES_PER_FRAME
    jae .done

    mov eax, r8d
    add eax, ecx
    and eax, 0xFF
    sub eax, 128
    imul eax, eax, 256

    test r9d, GBA_KEY_A
    jz .no_a
    add eax, 4096

.no_a:
    test r9d, GBA_KEY_B
    jz .no_b
    neg eax

.no_b:
    cmp eax, 32767
    jle .check_low
    mov eax, 32767

.check_low:
    cmp eax, -32768
    jge .store
    mov eax, -32768

.store:
    mov [rsi + rcx*4], ax

    mov edx, eax
    neg edx
    mov [rsi + rcx*4 + 2], dx

    inc ecx
    jmp .sample_loop

.done:
    ret
