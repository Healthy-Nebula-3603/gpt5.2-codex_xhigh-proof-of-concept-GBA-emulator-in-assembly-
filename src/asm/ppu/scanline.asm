default rel
bits 64

%include "gba_core.inc"

section .text
global gba_ppu_render

%define L_BACKDROP   0
%define L_PRIO       4
%define L_BG         8
%define L_BGCNT      12
%define L_CHARBASE   16
%define L_SCREENBASE 20
%define L_SIZE       24
%define L_COLOR256   28
%define L_HOFS       32
%define L_VOFS       36
%define L_WMASK      40
%define L_HMASK      44
%define L_Y          48
%define L_X          52
%define L_SY         56
%define L_TILEY      60
%define L_PY         64
%define L_ROWPTR     72
%define L_TMP0       80
%define L_OBJ_PRIO   84
%define L_OBJ_I      88
%define L_OBJ_ATTR0  92
%define L_OBJ_ATTR1  96
%define L_OBJ_ATTR2  100
%define L_OBJ_W      104
%define L_OBJ_H      108
%define L_OBJ_X      112
%define L_OBJ_Y      116
%define L_OBJ_PY     120
%define L_OBJ_PX     124
%define L_OBJ_SX     128
%define L_OBJ_SY     132
%define L_OBJ_TX     136
%define L_OBJ_TY     140
%define L_OBJ_TILE_BASE 144
%define L_OBJ_COLOR256 148
%define L_OBJ_HFLIP  152
%define L_OBJ_VFLIP  156
%define L_OBJ_PALBANK 160
%define L_OBJ_ROW_TILES 164
%define L_OBJ_TILE_INDEX 168
%define L_OBJ_ROWPTR 176
%define L_OBJ_MODE   184
%define L_OBJ_AFFINE_IDX 188
%define L_OBJ_DISP_W 192
%define L_OBJ_DISP_H 196
%define L_OBJ_PA     200
%define L_OBJ_PB     204
%define L_OBJ_PC     208
%define L_OBJ_PD     212
%define L_OBJ_DX     216
%define L_OBJ_DY     220
%define L_OBJ_GFXMODE 224
%define L_BLDALPHA  228
%define L_EVA       232
%define L_EVB       236
%define L_WIN0H     240
%define L_WIN1H     244
%define L_WIN0V     248
%define L_WIN1V     252
%define L_AFF_PA    256
%define L_AFF_PB    260
%define L_AFF_PC    264
%define L_AFF_PD    268
%define L_AFF_XREF  272
%define L_AFF_YREF  276
%define L_AFF_SIZE_PIX 280
%define L_AFF_WRAP  284
%define L_AFF_MAP_TILES 288
%define L_AFF_BASE_U 292
%define L_AFF_BASE_V 296
%define L_AFF_SX    300
%define L_AFF_SY    304

cpu_color16_to_argb:
    ; in:  ax = BGR555
    ; out: eax = ARGB8888
    movzx eax, ax

    mov ecx, eax
    and ecx, 0x1F              ; R5
    mov edx, ecx
    shl ecx, 3
    shr edx, 2
    or ecx, edx                ; R8 in ecx

    mov edx, eax
    shr edx, 5
    and edx, 0x1F              ; G5
    mov esi, edx
    shl edx, 3
    shr esi, 2
    or edx, esi                ; G8 in edx

    shr eax, 10
    and eax, 0x1F              ; B5 in eax
    mov esi, eax
    shl eax, 3
    shr esi, 2
    or eax, esi                ; B8 in eax

    shl ecx, 16
    shl edx, 8
    or ecx, edx
    or eax, ecx
    or eax, 0xFF000000
    ret

; void gba_ppu_render(GbaCore *core)
gba_ppu_render:
    test rdi, rdi
    jz .done

    cmp dword [rdi + GBA_CORE.magic], GBA_MAGIC
    jne .done

    push rbx
    push r12
    push r13
    push r14
    push r15
    sub rsp, 352

    mov r12, rdi
    lea rbx, [r12 + GBA_CORE.framebuffer]
    movzx r13d, word [r12 + GBA_CORE.io + IO_DISPCNT]

    mov eax, r13d
    and eax, 0x7               ; video mode

    cmp eax, 0
    je .mode0
    cmp eax, 1
    je .mode1
    cmp eax, 2
    je .mode2
    cmp eax, 3
    je .mode3
    cmp eax, 4
    je .mode4
    jmp .fallback

.mode0:
    movzx eax, word [r12 + GBA_CORE.io + IO_WIN0H]
    mov [rsp + L_WIN0H], eax
    movzx eax, word [r12 + GBA_CORE.io + IO_WIN1H]
    mov [rsp + L_WIN1H], eax
    movzx eax, word [r12 + GBA_CORE.io + IO_WIN0V]
    mov [rsp + L_WIN0V], eax
    movzx eax, word [r12 + GBA_CORE.io + IO_WIN1V]
    mov [rsp + L_WIN1V], eax

    ; Blend coefficients used by semi-transparent OBJ pixels.
    movzx eax, word [r12 + GBA_CORE.io + IO_BLDALPHA]
    mov [rsp + L_BLDALPHA], eax
    mov ecx, eax
    and ecx, 0x1F
    cmp ecx, 16
    jbe .mode0_eva_ok
    mov ecx, 16
.mode0_eva_ok:
    mov [rsp + L_EVA], ecx
    mov ecx, eax
    shr ecx, 8
    and ecx, 0x1F
    cmp ecx, 16
    jbe .mode0_evb_ok
    mov ecx, 16
.mode0_evb_ok:
    mov [rsp + L_EVB], ecx

    ; Backdrop color (palette index 0).
    movzx eax, word [r12 + GBA_CORE.pram]
    call cpu_color16_to_argb
    mov [rsp + L_BACKDROP], eax

    ; Initialize frame with backdrop color.
    mov eax, [rsp + L_BACKDROP]
    lea rdi, [rbx]
    mov ecx, GBA_FB_PIXELS
    rep stosd

    ; Draw text BGs by priority: low -> high so higher priority overwrites.
    ; For equal priority, BG0 should win, so draw BG3 -> BG0.
    mov dword [rsp + L_PRIO], 3

.mode0_prio_loop:
    mov eax, [rsp + L_PRIO]
    cmp eax, 0
    jl .render_done

    mov dword [rsp + L_BG], 3

.mode0_bg_loop:
    mov eax, [rsp + L_BG]
    cmp eax, 0
    jl .mode0_next_prio

    ; Mode-specific BG availability.
    mov eax, r13d
    and eax, 0x7
    cmp eax, 1
    je .mode1_bg_restrict
    cmp eax, 2
    jne .mode0_bg_present_check
    cmp dword [rsp + L_BG], 2
    jb .mode0_bg_next
    jmp .mode0_bg_present_check

.mode1_bg_restrict:
    cmp dword [rsp + L_BG], 3
    je .mode0_bg_next

.mode0_bg_present_check:

    ; DISPCNT enable bits: BG0..BG3 at bits 8..11.
    mov ecx, [rsp + L_BG]
    add ecx, 8
    mov edx, r13d
    bt edx, ecx
    jnc .mode0_bg_next

    ; BGxCNT
    mov eax, [rsp + L_BG]
    lea eax, [eax * 2 + IO_BG0CNT]
    movzx eax, word [r12 + GBA_CORE.io + rax]
    mov [rsp + L_BGCNT], eax

    ; Priority filter for this pass.
    mov ecx, eax
    and ecx, 0x3
    cmp ecx, [rsp + L_PRIO]
    jne .mode0_bg_next

    ; Character base block (16KB units).
    mov edx, eax
    shr edx, 2
    and edx, 0x3
    shl edx, 14
    mov [rsp + L_CHARBASE], edx

    ; Screen base block (2KB units).
    mov edx, eax
    shr edx, 8
    and edx, 0x1F
    shl edx, 11
    mov [rsp + L_SCREENBASE], edx

    ; Screen size code.
    mov edx, eax
    shr edx, 14
    and edx, 0x3
    mov [rsp + L_SIZE], edx

    ; 8bpp/4bpp select.
    mov edx, eax
    shr edx, 7
    and edx, 0x1
    mov [rsp + L_COLOR256], edx

    ; Affine BG selection for modes 1/2.
    mov eax, r13d
    and eax, 0x7
    cmp eax, 1
    je .mode1_affine_select
    cmp eax, 2
    je .mode2_affine_select
    jmp .mode0_text_bg_setup

.mode1_affine_select:
    cmp dword [rsp + L_BG], 2
    jne .mode0_text_bg_setup
    jmp .mode1_bg2_affine

.mode2_affine_select:
    cmp dword [rsp + L_BG], 2
    jb .mode0_bg_next
    jmp .mode1_bg2_affine

.mode0_text_bg_setup:

    ; BGxHOFS/BGxVOFS
    mov eax, [rsp + L_BG]
    lea eax, [eax * 4 + IO_BG0HOFS]
    movzx edx, word [r12 + GBA_CORE.io + rax]
    and edx, 0x1FF
    mov [rsp + L_HOFS], edx
    movzx edx, word [r12 + GBA_CORE.io + rax + 2]
    and edx, 0x1FF
    mov [rsp + L_VOFS], edx

    ; Width/height masks by size.
    mov eax, [rsp + L_SIZE]
    test eax, 1
    jz .mode0_w256
    mov dword [rsp + L_WMASK], 511
    jmp .mode0_w_done

.mode0_w256:
    mov dword [rsp + L_WMASK], 255

.mode0_w_done:
    test eax, 2
    jz .mode0_h256
    mov dword [rsp + L_HMASK], 511
    jmp .mode0_h_done

.mode0_h256:
    mov dword [rsp + L_HMASK], 255

.mode0_h_done:
    mov dword [rsp + L_Y], 0

.mode0_y_loop:
    mov eax, [rsp + L_Y]
    cmp eax, GBA_SCREEN_H
    jae .mode0_bg_next

    ; row pointer = framebuffer + y * 240 * 4
    mov edx, eax
    imul edx, (GBA_SCREEN_W * 4)
    lea rax, [rbx + rdx]
    mov [rsp + L_ROWPTR], rax

    ; sy and tile_y for this scanline.
    mov ecx, [rsp + L_Y]
    add ecx, [rsp + L_VOFS]
    and ecx, [rsp + L_HMASK]
    mov [rsp + L_SY], ecx

    mov edx, ecx
    and edx, 7
    mov [rsp + L_PY], edx

    shr ecx, 3
    mov [rsp + L_TILEY], ecx

    mov dword [rsp + L_X], 0

.mode0_x_loop:
    mov eax, [rsp + L_X]
    cmp eax, GBA_SCREEN_W
    jae .mode0_next_y

    ; sx and tile_x.
    mov ecx, [rsp + L_X]
    add ecx, [rsp + L_HOFS]
    and ecx, [rsp + L_WMASK]

    mov edx, ecx
    and edx, 7                ; px

    mov esi, ecx
    shr esi, 3                ; tile_x

    ; Generic text BG map selection over 32x32 screen blocks.
    mov edi, [rsp + L_TILEY]  ; tile_y

    mov eax, esi
    shr eax, 5                ; x_high

    mov r8d, edi
    shr r8d, 5                ; y_high

    and esi, 31               ; local_x
    and edi, 31               ; local_y

    xor r9d, r9d              ; block id
    mov ecx, [rsp + L_SIZE]
    test ecx, 1
    jz .mode0_no_block_x
    add r9d, eax

.mode0_no_block_x:
    test ecx, 2
    jz .mode0_no_block_y

    mov eax, 1
    test ecx, 1
    jz .mode0_block_y_mul
    mov eax, 2

.mode0_block_y_mul:
    imul eax, r8d
    add r9d, eax

.mode0_no_block_y:
    imul edi, 32
    add edi, esi
    shl edi, 1

    mov eax, r9d
    shl eax, 11
    add edi, eax
    add edi, [rsp + L_SCREENBASE]

    movzx eax, word [r12 + GBA_CORE.vram + rdi]
    mov [rsp + L_TMP0], eax

    ; Tile/pixel decode.
    mov ecx, eax
    and ecx, 0x03FF           ; tile id

    mov esi, edx              ; px
    mov edi, [rsp + L_PY]     ; py

    test eax, (1 << 10)
    jz .mode0_no_hflip
    mov esi, 7
    sub esi, edx

.mode0_no_hflip:
    test eax, (1 << 11)
    jz .mode0_no_vflip
    mov edi, 7
    sub edi, [rsp + L_PY]

.mode0_no_vflip:
    mov edx, [rsp + L_COLOR256]
    test edx, edx
    jz .mode0_pixel_4bpp

    ; 8bpp tile fetch.
    mov eax, ecx
    shl eax, 6
    add eax, [rsp + L_CHARBASE]
    lea eax, [eax + edi * 8]
    add eax, esi

    cmp eax, GBA_VRAM_SIZE
    jb .mode0_8bpp_addr_ok
    sub eax, GBA_VRAM_SIZE

.mode0_8bpp_addr_ok:
    movzx eax, byte [r12 + GBA_CORE.vram + rax]
    test eax, eax
    jz .mode0_pixel_next
    jmp .mode0_got_pal

.mode0_pixel_4bpp:
    ; 4bpp tile fetch.
    mov eax, ecx
    shl eax, 5
    add eax, [rsp + L_CHARBASE]
    lea eax, [eax + edi * 4]

    mov ecx, esi
    shr ecx, 1
    add eax, ecx

    cmp eax, GBA_VRAM_SIZE
    jb .mode0_4bpp_addr_ok
    sub eax, GBA_VRAM_SIZE

.mode0_4bpp_addr_ok:
    movzx eax, byte [r12 + GBA_CORE.vram + rax]

    test esi, 1
    jz .mode0_nibble_lo
    shr eax, 4
    jmp .mode0_nibble_done

.mode0_nibble_lo:
    and eax, 0x0F

.mode0_nibble_done:
    test eax, eax
    jz .mode0_pixel_next

    mov ecx, [rsp + L_TMP0]
    shr ecx, 12
    shl ecx, 4
    add eax, ecx

.mode0_got_pal:
    shl eax, 1
    movzx eax, word [r12 + GBA_CORE.pram + rax]
    call cpu_color16_to_argb

    mov rcx, [rsp + L_ROWPTR]
    mov edx, [rsp + L_X]
    mov [rcx + rdx * 4], eax

.mode0_pixel_next:
    add dword [rsp + L_X], 1
    jmp .mode0_x_loop

.mode0_next_y:
    add dword [rsp + L_Y], 1
    jmp .mode0_y_loop

.mode1_bg2_affine:
    ; Affine parameters (8.8 signed matrix, 27.8 signed reference points).
    cmp dword [rsp + L_BG], 3
    je .mode_affine_bg3

    ; BG2 affine registers.
    movsx eax, word [r12 + GBA_CORE.io + IO_BG2PA]
    mov [rsp + L_AFF_PA], eax
    movsx eax, word [r12 + GBA_CORE.io + IO_BG2PB]
    mov [rsp + L_AFF_PB], eax
    movsx eax, word [r12 + GBA_CORE.io + IO_BG2PC]
    mov [rsp + L_AFF_PC], eax
    movsx eax, word [r12 + GBA_CORE.io + IO_BG2PD]
    mov [rsp + L_AFF_PD], eax

    mov eax, [r12 + GBA_CORE.io + IO_BG2X]
    mov [rsp + L_AFF_XREF], eax
    mov eax, [r12 + GBA_CORE.io + IO_BG2Y]
    mov [rsp + L_AFF_YREF], eax
    jmp .mode_affine_regs_ready

.mode_affine_bg3:
    ; BG3 affine registers (mode 2).
    movsx eax, word [r12 + GBA_CORE.io + IO_BG3PA]
    mov [rsp + L_AFF_PA], eax
    movsx eax, word [r12 + GBA_CORE.io + IO_BG3PB]
    mov [rsp + L_AFF_PB], eax
    movsx eax, word [r12 + GBA_CORE.io + IO_BG3PC]
    mov [rsp + L_AFF_PC], eax
    movsx eax, word [r12 + GBA_CORE.io + IO_BG3PD]
    mov [rsp + L_AFF_PD], eax

    mov eax, [r12 + GBA_CORE.io + IO_BG3X]
    mov [rsp + L_AFF_XREF], eax
    mov eax, [r12 + GBA_CORE.io + IO_BG3Y]
    mov [rsp + L_AFF_YREF], eax

.mode_affine_regs_ready:

    ; Affine wrap flag (BG2CNT bit 13).
    mov eax, [rsp + L_BGCNT]
    shr eax, 13
    and eax, 1
    mov [rsp + L_AFF_WRAP], eax

    ; Affine size code: 128/256/512/1024 square pixels.
    mov eax, [rsp + L_SIZE]
    cmp eax, 0
    jne .mode1_aff_size_1
    mov dword [rsp + L_AFF_SIZE_PIX], 128
    mov dword [rsp + L_AFF_MAP_TILES], 16
    jmp .mode1_aff_size_done

.mode1_aff_size_1:
    cmp eax, 1
    jne .mode1_aff_size_2
    mov dword [rsp + L_AFF_SIZE_PIX], 256
    mov dword [rsp + L_AFF_MAP_TILES], 32
    jmp .mode1_aff_size_done

.mode1_aff_size_2:
    cmp eax, 2
    jne .mode1_aff_size_3
    mov dword [rsp + L_AFF_SIZE_PIX], 512
    mov dword [rsp + L_AFF_MAP_TILES], 64
    jmp .mode1_aff_size_done

.mode1_aff_size_3:
    mov dword [rsp + L_AFF_SIZE_PIX], 1024
    mov dword [rsp + L_AFF_MAP_TILES], 128

.mode1_aff_size_done:
    mov dword [rsp + L_Y], 0

.mode1_aff_y_loop:
    mov eax, [rsp + L_Y]
    cmp eax, GBA_SCREEN_H
    jae .mode0_bg_next

    ; row pointer = framebuffer + y * 240 * 4
    mov edx, eax
    imul edx, (GBA_SCREEN_W * 4)
    lea rax, [rbx + rdx]
    mov [rsp + L_ROWPTR], rax

    ; base_u = xref + pb*y
    mov eax, [rsp + L_AFF_PB]
    imul eax, [rsp + L_Y]
    add eax, [rsp + L_AFF_XREF]
    mov [rsp + L_AFF_BASE_U], eax

    ; base_v = yref + pd*y
    mov eax, [rsp + L_AFF_PD]
    imul eax, [rsp + L_Y]
    add eax, [rsp + L_AFF_YREF]
    mov [rsp + L_AFF_BASE_V], eax

    mov dword [rsp + L_X], 0

.mode1_aff_x_loop:
    mov eax, [rsp + L_X]
    cmp eax, GBA_SCREEN_W
    jae .mode1_aff_next_y

    ; sx = (base_u + pa*x) >> 8
    mov eax, [rsp + L_AFF_PA]
    imul eax, [rsp + L_X]
    add eax, [rsp + L_AFF_BASE_U]
    sar eax, 8
    mov [rsp + L_AFF_SX], eax

    ; sy = (base_v + pc*x) >> 8
    mov eax, [rsp + L_AFF_PC]
    imul eax, [rsp + L_X]
    add eax, [rsp + L_AFF_BASE_V]
    sar eax, 8
    mov [rsp + L_AFF_SY], eax

    cmp dword [rsp + L_AFF_WRAP], 0
    jne .mode1_aff_wrap_coords

    ; No wrap: out-of-range is transparent.
    mov eax, [rsp + L_AFF_SX]
    cmp eax, 0
    jl .mode1_aff_next_px
    cmp eax, [rsp + L_AFF_SIZE_PIX]
    jae .mode1_aff_next_px

    mov eax, [rsp + L_AFF_SY]
    cmp eax, 0
    jl .mode1_aff_next_px
    cmp eax, [rsp + L_AFF_SIZE_PIX]
    jae .mode1_aff_next_px
    jmp .mode1_aff_coords_ready

.mode1_aff_wrap_coords:
    mov ecx, [rsp + L_AFF_SIZE_PIX]
    dec ecx
    mov eax, [rsp + L_AFF_SX]
    and eax, ecx
    mov [rsp + L_AFF_SX], eax
    mov eax, [rsp + L_AFF_SY]
    and eax, ecx
    mov [rsp + L_AFF_SY], eax

.mode1_aff_coords_ready:
    ; Map entry is one byte tile id.
    mov esi, [rsp + L_AFF_SX]
    shr esi, 3
    mov edi, [rsp + L_AFF_SY]
    shr edi, 3
    imul edi, [rsp + L_AFF_MAP_TILES]
    add edi, esi

    mov eax, [rsp + L_SCREENBASE]
    add eax, edi
    and eax, 0x1FFFF
    cmp eax, 0x18000
    jb .mode1_aff_map_ok
    sub eax, 0x8000
.mode1_aff_map_ok:
    movzx ecx, byte [r12 + GBA_CORE.vram + rax]   ; tile id

    ; 8bpp tile fetch.
    mov eax, [rsp + L_AFF_SX]
    and eax, 7
    mov edx, [rsp + L_AFF_SY]
    and edx, 7
    mov esi, ecx
    shl esi, 6
    add esi, [rsp + L_CHARBASE]
    lea esi, [esi + edx * 8]
    add esi, eax
    and esi, 0x1FFFF
    cmp esi, 0x18000
    jb .mode1_aff_tile_ok
    sub esi, 0x8000
.mode1_aff_tile_ok:
    movzx eax, byte [r12 + GBA_CORE.vram + rsi]
    test eax, eax
    jz .mode1_aff_next_px

    shl eax, 1
    movzx eax, word [r12 + GBA_CORE.pram + rax]
    call cpu_color16_to_argb

    mov rcx, [rsp + L_ROWPTR]
    mov edx, [rsp + L_X]
    mov [rcx + rdx * 4], eax

.mode1_aff_next_px:
    add dword [rsp + L_X], 1
    jmp .mode1_aff_x_loop

.mode1_aff_next_y:
    add dword [rsp + L_Y], 1
    jmp .mode1_aff_y_loop

.mode0_bg_next:
    sub dword [rsp + L_BG], 1
    jmp .mode0_bg_loop

.mode0_next_prio:
    ; Composite OBJ at this same priority so BG/OBJ priority ordering is respected.
    test r13d, (1 << 12)
    jz .mode0_dec_prio
    mov eax, [rsp + L_PRIO]
    mov [rsp + L_OBJ_PRIO], eax
    mov dword [rsp + L_OBJ_I], 127
    jmp .mode0_obj_i_loop

.mode0_dec_prio:
    sub dword [rsp + L_PRIO], 1
    jmp .mode0_prio_loop

.mode0_obj_init:
.mode0_obj_prio_loop:
    jmp .mode0_dec_prio

.mode0_obj_i_loop:
    mov eax, [rsp + L_OBJ_I]
    cmp eax, 0
    jl .mode0_dec_prio

    mov edx, eax
    shl edx, 3
    movzx ecx, word [r12 + GBA_CORE.oam + rdx]
    mov [rsp + L_OBJ_ATTR0], ecx
    movzx ecx, word [r12 + GBA_CORE.oam + rdx + 2]
    mov [rsp + L_OBJ_ATTR1], ecx
    movzx ecx, word [r12 + GBA_CORE.oam + rdx + 4]
    mov [rsp + L_OBJ_ATTR2], ecx

    ; OBJ gfx mode (Attr0 bits 10-11):
    ; 0=normal, 1=semi-transparent, 2=OBJ-window, 3=prohibited.
    ; OBJ-window entries are masks and must not be drawn as visible pixels.
    mov eax, [rsp + L_OBJ_ATTR0]
    shr eax, 10
    and eax, 0x3
    mov [rsp + L_OBJ_GFXMODE], eax
    cmp eax, 2
    je .mode0_obj_next
    cmp eax, 3
    je .mode0_obj_next

    ; OBJ mode:
    ; 0 = regular, 1 = affine, 2 = hidden, 3 = affine double-size.
    mov eax, [rsp + L_OBJ_ATTR0]
    shr eax, 8
    and eax, 0x3
    mov [rsp + L_OBJ_MODE], eax
    cmp eax, 2
    je .mode0_obj_next

    ; Per-sprite priority pass.
    mov eax, [rsp + L_OBJ_ATTR2]
    shr eax, 10
    and eax, 0x3
    cmp eax, [rsp + L_OBJ_PRIO]
    jne .mode0_obj_next

    ; Decode shape/size -> width/height.
    mov eax, [rsp + L_OBJ_ATTR0]
    shr eax, 14
    and eax, 0x3
    cmp eax, 3
    je .mode0_obj_next

    mov ecx, [rsp + L_OBJ_ATTR1]
    shr ecx, 14
    and ecx, 0x3

    cmp eax, 0
    je .obj_shape_square
    cmp eax, 1
    je .obj_shape_horz
    jmp .obj_shape_vert

.obj_shape_square:
    cmp ecx, 0
    je .obj_size_8x8
    cmp ecx, 1
    je .obj_size_16x16
    cmp ecx, 2
    je .obj_size_32x32
    mov dword [rsp + L_OBJ_W], 64
    mov dword [rsp + L_OBJ_H], 64
    jmp .obj_size_done

.obj_shape_horz:
    cmp ecx, 0
    je .obj_size_16x8
    cmp ecx, 1
    je .obj_size_32x8
    cmp ecx, 2
    je .obj_size_32x16
    mov dword [rsp + L_OBJ_W], 64
    mov dword [rsp + L_OBJ_H], 32
    jmp .obj_size_done

.obj_shape_vert:
    cmp ecx, 0
    je .obj_size_8x16
    cmp ecx, 1
    je .obj_size_8x32
    cmp ecx, 2
    je .obj_size_16x32
    mov dword [rsp + L_OBJ_W], 32
    mov dword [rsp + L_OBJ_H], 64
    jmp .obj_size_done

.obj_size_8x8:
    mov dword [rsp + L_OBJ_W], 8
    mov dword [rsp + L_OBJ_H], 8
    jmp .obj_size_done

.obj_size_16x16:
    mov dword [rsp + L_OBJ_W], 16
    mov dword [rsp + L_OBJ_H], 16
    jmp .obj_size_done

.obj_size_32x32:
    mov dword [rsp + L_OBJ_W], 32
    mov dword [rsp + L_OBJ_H], 32
    jmp .obj_size_done

.obj_size_16x8:
    mov dword [rsp + L_OBJ_W], 16
    mov dword [rsp + L_OBJ_H], 8
    jmp .obj_size_done

.obj_size_32x8:
    mov dword [rsp + L_OBJ_W], 32
    mov dword [rsp + L_OBJ_H], 8
    jmp .obj_size_done

.obj_size_32x16:
    mov dword [rsp + L_OBJ_W], 32
    mov dword [rsp + L_OBJ_H], 16
    jmp .obj_size_done

.obj_size_8x16:
    mov dword [rsp + L_OBJ_W], 8
    mov dword [rsp + L_OBJ_H], 16
    jmp .obj_size_done

.obj_size_8x32:
    mov dword [rsp + L_OBJ_W], 8
    mov dword [rsp + L_OBJ_H], 32
    jmp .obj_size_done

.obj_size_16x32:
    mov dword [rsp + L_OBJ_W], 16
    mov dword [rsp + L_OBJ_H], 32

.obj_size_done:
    ; Display size equals object size except affine double-size mode.
    mov eax, [rsp + L_OBJ_W]
    mov [rsp + L_OBJ_DISP_W], eax
    mov eax, [rsp + L_OBJ_H]
    mov [rsp + L_OBJ_DISP_H], eax
    cmp dword [rsp + L_OBJ_MODE], 3
    jne .obj_disp_size_done
    shl dword [rsp + L_OBJ_DISP_W], 1
    shl dword [rsp + L_OBJ_DISP_H], 1
.obj_disp_size_done:

    ; X is 9-bit signed with wrap.
    mov eax, [rsp + L_OBJ_ATTR1]
    and eax, 0x1FF
    cmp eax, 256
    jb .obj_x_ready
    sub eax, 512
.obj_x_ready:
    mov [rsp + L_OBJ_X], eax

    ; Y is 8-bit wrapped.
    mov eax, [rsp + L_OBJ_ATTR0]
    and eax, 0xFF
    mov [rsp + L_OBJ_Y], eax

    mov eax, [rsp + L_OBJ_ATTR0]
    shr eax, 13
    and eax, 1
    mov [rsp + L_OBJ_COLOR256], eax

    ; H/V flip is only for regular OBJ.
    mov dword [rsp + L_OBJ_HFLIP], 0
    mov dword [rsp + L_OBJ_VFLIP], 0
    cmp dword [rsp + L_OBJ_MODE], 0
    jne .obj_affine_params

    mov eax, [rsp + L_OBJ_ATTR1]
    shr eax, 12
    and eax, 1
    mov [rsp + L_OBJ_HFLIP], eax

    mov eax, [rsp + L_OBJ_ATTR1]
    shr eax, 13
    and eax, 1
    mov [rsp + L_OBJ_VFLIP], eax
    jmp .obj_affine_done

.obj_affine_params:
    mov eax, [rsp + L_OBJ_ATTR1]
    shr eax, 9
    and eax, 0x1F
    mov [rsp + L_OBJ_AFFINE_IDX], eax

    mov ecx, eax
    shl ecx, 5

    movsx eax, word [r12 + GBA_CORE.oam + rcx + 6]
    mov [rsp + L_OBJ_PA], eax
    movsx eax, word [r12 + GBA_CORE.oam + rcx + 14]
    mov [rsp + L_OBJ_PB], eax
    movsx eax, word [r12 + GBA_CORE.oam + rcx + 22]
    mov [rsp + L_OBJ_PC], eax
    movsx eax, word [r12 + GBA_CORE.oam + rcx + 30]
    mov [rsp + L_OBJ_PD], eax

.obj_affine_done:

    mov eax, [rsp + L_OBJ_ATTR2]
    and eax, 0x3FF
    mov [rsp + L_OBJ_TILE_BASE], eax

    mov eax, [rsp + L_OBJ_ATTR2]
    shr eax, 12
    and eax, 0xF
    mov [rsp + L_OBJ_PALBANK], eax

    ; OBJ tile mapping mode: 1D when DISPCNT bit 6 set, else 2D.
    test r13d, (1 << 6)
    jz .obj_row_tiles_2d
    mov eax, [rsp + L_OBJ_W]
    shr eax, 3
.obj_row_tiles_1d_done:
    mov [rsp + L_OBJ_ROW_TILES], eax
    jmp .obj_row_tiles_done

.obj_row_tiles_2d:
    ; In 2D OBJ mapping, tile number rows are 32 entries in 4bpp units.
    ; For 8bpp, fetch path multiplies tile index by 2, so keep a 16-tile pitch here.
    cmp dword [rsp + L_OBJ_COLOR256], 0
    jne .obj_row_tiles_2d_8bpp
    mov dword [rsp + L_OBJ_ROW_TILES], 32
    jmp .obj_row_tiles_done

.obj_row_tiles_2d_8bpp:
    mov dword [rsp + L_OBJ_ROW_TILES], 16

.obj_row_tiles_done:
    mov dword [rsp + L_OBJ_PY], 0

.obj_py_loop:
    mov eax, [rsp + L_OBJ_PY]
    cmp eax, [rsp + L_OBJ_DISP_H]
    jae .mode0_obj_next

    ; sy = (obj_y + py) & 255
    mov ecx, [rsp + L_OBJ_Y]
    add ecx, eax
    and ecx, 0xFF
    mov [rsp + L_OBJ_SY], ecx
    cmp ecx, GBA_SCREEN_H
    jae .obj_next_py

    ; Texture Y with optional vflip.
    mov edx, eax
    cmp dword [rsp + L_OBJ_VFLIP], 0
    je .obj_ty_ready
    mov edx, [rsp + L_OBJ_H]
    dec edx
    sub edx, eax
.obj_ty_ready:
    mov [rsp + L_OBJ_TY], edx

    ; Row pointer for destination scanline.
    mov ecx, [rsp + L_OBJ_SY]
    imul ecx, (GBA_SCREEN_W * 4)
    lea rax, [rbx + rcx]
    mov [rsp + L_OBJ_ROWPTR], rax

    mov dword [rsp + L_OBJ_PX], 0

.obj_px_loop:
    mov eax, [rsp + L_OBJ_PX]
    cmp eax, [rsp + L_OBJ_DISP_W]
    jae .obj_next_py

    mov ecx, [rsp + L_OBJ_X]
    add ecx, eax
    mov [rsp + L_OBJ_SX], ecx
    cmp ecx, 0
    jl .obj_next_px
    cmp ecx, GBA_SCREEN_W
    jae .obj_next_px

    ; Texture coordinates.
    cmp dword [rsp + L_OBJ_MODE], 0
    jne .obj_affine_texcoord

    ; Regular OBJ: optional flips.
    mov edx, eax
    cmp dword [rsp + L_OBJ_HFLIP], 0
    je .obj_reg_tx_ready
    mov edx, [rsp + L_OBJ_W]
    dec edx
    sub edx, eax
.obj_reg_tx_ready:
    mov [rsp + L_OBJ_TX], edx

    mov edx, [rsp + L_OBJ_PY]
    cmp dword [rsp + L_OBJ_VFLIP], 0
    je .obj_reg_ty_ready
    mov edx, [rsp + L_OBJ_H]
    dec edx
    sub edx, [rsp + L_OBJ_PY]
.obj_reg_ty_ready:
    mov [rsp + L_OBJ_TY], edx
    jmp .obj_texcoord_ready

.obj_affine_texcoord:
    mov eax, [rsp + L_OBJ_PX]
    mov ecx, [rsp + L_OBJ_DISP_W]
    sar ecx, 1
    sub eax, ecx
    mov [rsp + L_OBJ_DX], eax

    mov eax, [rsp + L_OBJ_PY]
    mov ecx, [rsp + L_OBJ_DISP_H]
    sar ecx, 1
    sub eax, ecx
    mov [rsp + L_OBJ_DY], eax

    mov eax, [rsp + L_OBJ_PA]
    imul eax, [rsp + L_OBJ_DX]
    mov ecx, [rsp + L_OBJ_PB]
    imul ecx, [rsp + L_OBJ_DY]
    add eax, ecx
    sar eax, 8
    mov ecx, [rsp + L_OBJ_W]
    sar ecx, 1
    add eax, ecx
    mov [rsp + L_OBJ_TX], eax
    cmp eax, 0
    jl .obj_next_px
    cmp eax, [rsp + L_OBJ_W]
    jae .obj_next_px

    mov eax, [rsp + L_OBJ_PC]
    imul eax, [rsp + L_OBJ_DX]
    mov ecx, [rsp + L_OBJ_PD]
    imul ecx, [rsp + L_OBJ_DY]
    add eax, ecx
    sar eax, 8
    mov ecx, [rsp + L_OBJ_H]
    sar ecx, 1
    add eax, ecx
    mov [rsp + L_OBJ_TY], eax
    cmp eax, 0
    jl .obj_next_px
    cmp eax, [rsp + L_OBJ_H]
    jae .obj_next_px

    ; Tile index = tile_y * row_tiles + tile_x.
.obj_texcoord_ready:
    mov eax, [rsp + L_OBJ_TX]
    shr eax, 3
    mov ecx, [rsp + L_OBJ_TY]
    shr ecx, 3
    imul ecx, [rsp + L_OBJ_ROW_TILES]
    add ecx, eax
    mov [rsp + L_OBJ_TILE_INDEX], ecx

    mov eax, [rsp + L_OBJ_TX]
    and eax, 7
    mov edx, [rsp + L_OBJ_TY]
    and edx, 7

    cmp dword [rsp + L_OBJ_COLOR256], 0
    je .obj_fetch_4bpp

    ; 8bpp OBJ tile (attribute index uses 32-byte units; bit0 ignored).
    mov ecx, [rsp + L_OBJ_TILE_BASE]
    and ecx, 0x3FE
    mov esi, [rsp + L_OBJ_TILE_INDEX]
    lea ecx, [ecx + esi * 2]
    shl ecx, 5
    add ecx, 0x10000
    mov esi, edx
    shl esi, 3
    add ecx, esi
    add ecx, eax
    and ecx, 0x1FFFF
    cmp ecx, 0x18000
    jb .obj_8bpp_addr_ok
    sub ecx, 0x8000
.obj_8bpp_addr_ok:
    movzx eax, byte [r12 + GBA_CORE.vram + rcx]
    test eax, eax
    jz .obj_next_px
    jmp .obj_palette_ready

.obj_fetch_4bpp:
    ; 4bpp OBJ tile.
    mov ecx, [rsp + L_OBJ_TILE_BASE]
    add ecx, [rsp + L_OBJ_TILE_INDEX]
    shl ecx, 5
    add ecx, 0x10000
    mov esi, edx
    shl esi, 2
    add ecx, esi
    mov edx, eax
    shr edx, 1
    add ecx, edx
    and ecx, 0x1FFFF
    cmp ecx, 0x18000
    jb .obj_4bpp_addr_ok
    sub ecx, 0x8000
.obj_4bpp_addr_ok:
    movzx eax, byte [r12 + GBA_CORE.vram + rcx]
    test dword [rsp + L_OBJ_TX], 1
    jz .obj_nibble_lo
    shr eax, 4
    jmp .obj_nibble_done
.obj_nibble_lo:
    and eax, 0x0F
.obj_nibble_done:
    test eax, eax
    jz .obj_next_px
    mov ecx, [rsp + L_OBJ_PALBANK]
    shl ecx, 4
    add eax, ecx

.obj_palette_ready:
    ; OBJ palettes are at PRAM offset 0x200.
    shl eax, 1
    add eax, 0x200
    movzx eax, word [r12 + GBA_CORE.pram + rax]
    call cpu_color16_to_argb

    mov rcx, [rsp + L_OBJ_ROWPTR]
    mov edx, [rsp + L_OBJ_SX]
    jmp .obj_store_pixel

    ; Semi-transparent OBJ: blend source over current destination using BLDALPHA.
    mov r8d, eax
    mov r9d, [rcx + rdx * 4]
    mov esi, [rsp + L_EVA]
    mov edi, [rsp + L_EVB]

    ; Blue
    mov eax, r8d
    and eax, 0xFF
    imul eax, esi
    mov r10d, r9d
    and r10d, 0xFF
    imul r10d, edi
    add eax, r10d
    shr eax, 4
    cmp eax, 255
    jbe .obj_blend_b_ok
    mov eax, 255
.obj_blend_b_ok:
    mov r11d, eax

    ; Green
    mov eax, r8d
    shr eax, 8
    and eax, 0xFF
    imul eax, esi
    mov r10d, r9d
    shr r10d, 8
    and r10d, 0xFF
    imul r10d, edi
    add eax, r10d
    shr eax, 4
    cmp eax, 255
    jbe .obj_blend_g_ok
    mov eax, 255
.obj_blend_g_ok:
    shl eax, 8
    or r11d, eax

    ; Red
    mov eax, r8d
    shr eax, 16
    and eax, 0xFF
    imul eax, esi
    mov r10d, r9d
    shr r10d, 16
    and r10d, 0xFF
    imul r10d, edi
    add eax, r10d
    shr eax, 4
    cmp eax, 255
    jbe .obj_blend_r_ok
    mov eax, 255
.obj_blend_r_ok:
    shl eax, 16
    or r11d, eax
    or r11d, 0xFF000000
    mov eax, r11d

.obj_store_pixel:
    mov [rcx + rdx * 4], eax

.obj_next_px:
    add dword [rsp + L_OBJ_PX], 1
    jmp .obj_px_loop

.obj_next_py:
    add dword [rsp + L_OBJ_PY], 1
    jmp .obj_py_loop

.mode0_obj_next:
    sub dword [rsp + L_OBJ_I], 1
    jmp .mode0_obj_i_loop

.mode0_obj_next_prio:
    jmp .mode0_dec_prio

.mode1:
    ; Mode 1 currently reuses text/OBJ composition path.
    ; This avoids unsupported-mode fallback during transitions and menu scenes.
    jmp .mode0

.mode2:
    ; Mode 2 uses affine BG2/BG3 with the same OBJ composition path.
    jmp .mode0

.mode3:
    xor r14d, r14d

.mode3_loop:
    cmp r14d, GBA_FB_PIXELS
    jae .render_done

    mov eax, r14d
    shl eax, 1
    movzx eax, word [r12 + GBA_CORE.vram + rax]
    call cpu_color16_to_argb
    mov [rbx + r14 * 4], eax

    inc r14d
    jmp .mode3_loop

.mode4:
    ; DISPCNT bit 4 selects frame page in mode 4.
    xor r10d, r10d
    test r13d, (1 << 4)
    jz .mode4_page_ok
    mov r10d, 0xA000

.mode4_page_ok:
    xor r14d, r14d

.mode4_loop:
    cmp r14d, GBA_FB_PIXELS
    jae .render_done

    mov eax, r14d
    add eax, r10d
    movzx eax, byte [r12 + GBA_CORE.vram + rax]

    mov edx, eax
    shl edx, 1
    movzx eax, word [r12 + GBA_CORE.pram + rdx]
    call cpu_color16_to_argb
    mov [rbx + r14 * 4], eax

    inc r14d
    jmp .mode4_loop

.fallback:
    ; Unsupported modes: render deterministic diagnostic gradient.
    mov r10d, [r12 + GBA_CORE.bg_phase]
    xor ecx, ecx               ; idx
    xor edx, edx               ; x
    xor esi, esi               ; y

.fallback_loop:
    cmp ecx, GBA_FB_PIXELS
    jae .render_done

    mov eax, edx
    add eax, r10d
    and eax, 0xFF

    mov r11d, esi
    add r11d, r10d
    and r11d, 0xFF

    mov r14d, edx
    xor r14d, esi
    xor r14d, r10d
    and r14d, 0xFF

    shl r14d, 16
    shl r11d, 8
    or r14d, r11d
    or r14d, eax
    or r14d, 0xFF000000
    mov [rbx + rcx * 4], r14d

    inc ecx
    inc edx
    cmp edx, GBA_SCREEN_W
    jb .fallback_loop

    xor edx, edx
    inc esi
    jmp .fallback_loop

.render_done:
    add rsp, 352
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx

.done:
    ret
