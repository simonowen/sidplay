; SID Player v1.0, by Simon Owen
;
; WWW: http://simonowen.com/sam/sidplay/
;
; Emulates a 6510 CPU to play most C64 SID tunes in real time.
; Requires Quazar SID interface board (see www.quazar.clara.net)
;
; Load PSID file at 49152 (16K maximum) and call 32768 to play.
; POKE 32770,tune-number (default=0, for SID default)
; DPOKE 32771,pre-buffer-frames (default=50, for 1 second)
;
; Features:
;  - PAL timing (50Hz)
;  - all officially documented 6510 instructions
;  - supports polled and interrupt driven playback
;
; Not yet supported:
;  - 65xx decimal mode
;  - NTSC (60Hz) or 100Hz modes
;  - undocumented 6510 instructions
;  - mirrored SID addresses (D430-D7FF)
;
; Not possible to support:
;  - custom timer frequencies (including sample playback)
;  - banked memory
;  - RSID files

               AUTOEXEC

base:          EQU  &D000          ; Player based at 53248
dmp:           EQU  &8000          ; Load code at 32768
offset:        EQU  base-dmp       ; Relocation offset

psid_hdr:      EQU  &C000          ; PSID file loaded here
buffer_blocks: EQU  50             ; number of frames to buffer
buffer_low:    EQU  25             ; limit before screen disable

lmpr:          EQU  250            ; Low Memory Page Register
hmpr:          EQU  251            ; High Memory Page Register
rom0_off:      EQU  %00100000      ; LMPR bit to disable ROM0

low_page:      EQU  3              ; LMPR during emulation
high_page:     EQU  5              ; HMPR during emulation
buffer_page:   EQU  7              ; base page for SID buffering

ret_ok:        EQU  0              ; success
ret_prev:      EQU  1              ; previous song (up/left)
ret_next:      EQU  2              ; next song (down/right)
ret_badfile:   EQU  3              ; missing or invalid file
ret_invalid:   EQU  4              ; invalid opcode
ret_decimal:   EQU  5              ; decimal mode unsupported
ret_brk:       EQU  6              ; BRK unsupported

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

               ORG  base
               DUMP dmp

               JR   start

song:          DEFB 0              ; 0=play default song
pre_buffer:    DEFW 50             ; pre-buffer 1 second

start:         LD   HL,(psid_hdr)  ; 'PS' in 'PSID' signature
               LD   DE,&5350       ; 'PS'
               AND  A
               SBC  HL,DE          ; compare signature bytes
               LD   BC,ret_badfile
               RET  NZ             ; return if not found

               LD   HL,(psid_hdr+6); data offset
               LD   D,L            ; big->little endian
               LD   E,H
               LD   HL,psid_hdr    ; header address
               ADD  HL,DE          ; form data address
               LD   DE,(psid_hdr+8); load address (or zero)
               LD   A,D
               LD   D,E            ; big->little endian
               LD   E,A
               OR   D
               JR   NZ,got_load    ; jump if address valid
               LD   E,(HL)         ; take address from data
               INC  L              ; (already little endian)
               LD   D,(HL)
               INC  L

got_load:      EXX
               LD   HL,(psid_hdr+10) ; init addr (big endian)
               LD   A,H
               LD   H,L
               LD   L,A
               LD   (init_addr-offset),HL
               LD   HL,(psid_hdr+12) ; play addr (big endian)
               LD   A,H
               LD   H,L
               LD   L,A
               LD   (play_addr-offset),HL
               EXX

               LD   A,(song-offset) ; user requested song
               LD   C,A
               AND  A               ; zero?
               LD   A,(psid_hdr+17) ; default start song
               JR   Z,got_song      ; use default if zero
               LD   B,A
               LD   A,(psid_hdr+15) ; songs available
               CP   C               ; song out of range?
               LD   A,B
               JR   C,got_song      ; use default if so
               LD   A,C
got_song:      LD   (play_song-offset),A ; save song to play

; At this point we have:  HL=sid_data DE=load_addr

               PUSH HL
               EXX                 ; save load address
               POP  HL             ; sid data

               DI
               IN   A,(lmpr)
               EX   AF,AF'
               LD   A,low_page+rom0_off
               OUT  (lmpr),A

               LD   DE,&0200       ; copy to after 6510 stack
               LD   BC,&4000       ; 16K to copy
               LDIR                ; copy track into low memory

               LD   A,high_page+rom0_off
               OUT  (lmpr),A
               LD   HL,dmp
               LD   DE,base-&8000  ; target in low memory
               LD   BC,size
               LDIR                ; copy player code

               JP   low_entry-&8000 ; continue running there

low_entry:     IN   A,(hmpr)
               LD   C,A
               LD   A,high_page
               OUT  (hmpr),A
               JP   high_entry     ; jump to final code

high_entry:    LD   (old_stack),SP
               LD   SP,stack_top
               LD   A,C
               PUSH AF             ; save original HMPR
               EX   AF,AF'
               PUSH AF             ; save original LMPR

               LD   A,low_page+rom0_off
               OUT  (lmpr),A

               EXX                 ; restore DE=load_addr
               LD   A,D
               CP   &E0            ; loads at >= &E000?
               LD   HL,0           ; clip to end of memory
               JR   NC,got_end
               LD   H,base/256     ; clip to code base at &D000
got_end:       AND  A
               SBC  HL,DE          ; calculate maximum size
               LD   B,H
               LD   C,L
               LD   A,B
               CP   &40            ; 16K?
               JR   C,under_16k    ; < 16K
               LD   BC,&4000       ; cap at 16K
under_16k:     LD   HL,&0200
               ADD  HL,BC
               DEC  HL
               EX   DE,HL
               ADD  HL,BC
               DEC  HL
               EX   DE,HL
               LDDR                ; copy block into place

               CALL play_tune
               EX   AF,AF'
               CALL sid_reset

               DI
               LD   A,high_page+rom0_off
               OUT  (lmpr),A
               POP  AF             ; restore LMPR
               LD   C,A
               POP  AF             ; restore HMPR
               LD   SP,(old_stack)
               JP   low_exit-&8000

low_exit:      OUT  (hmpr),A
               JP   high_exit-offset

high_exit:     LD   A,C
               OUT  (lmpr),A
               XOR  A
               OUT  (254),A
               EX   AF,AF'
               LD   C,A
               LD   B,0
               IM   1
               EI
               RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Tune player

play_tune:     LD   HL,0
               LD   (&FFFA),HL     ; no 6510 interrupt handler
               LD   (blocks),HL    ; no buffered blocks
               LD   (head),HL      ; head/tail at buffer start
               LD   (tail),HL

               CALL reorder_decode ; optimise decode table

               LD   A,(play_song)  ; song to play
               DEC  A              ; player expects A=song-1
               LD   HL,(init_addr) ; tune init function
               CALL execute        ; initialise player
               AND  A
               RET  NZ             ; return any error

               CALL sid_reset      ; reset the SID
               CALL record_block   ; record initial SID state

               LD   HL,(play_addr) ; tune player poll address
               LD   A,H
               OR   L
               JR   NZ,buffer_loop ; non-zero means we have one
               LD   HL,(&FFFA)     ; use interrupt handler addr
               LD   (play_addr),HL ; store play address

buffer_loop:   LD   HL,(blocks)    ; current block count
               LD   DE,(pre_buffer); blocks to pre-buffer
               AND  A
               SBC  HL,DE
               JR   NC,buffer_done

               XOR  A
               LD   HL,(play_addr) ; poll or interrupt addr
               CALL execute
               AND  A
               RET  NZ             ; return any errors

               CALL record_block   ; record the state
               JR   buffer_loop    ; loop buffering more

buffer_done:   CALL enable_player  ; enable interrupt player

sleep_loop:    HALT                ; wait for a block to play

play_loop:     LD   A,&7F          ; bottom row
               IN   A,(254)        ; read keyboard
               RRA                 ; bit 0 is space
               LD   A,ret_ok
               RET  NC             ; exit if space pressed

               LD   A,&FF          ; cursor keys + cntrl
               IN   A,(254)
               CPL                 ; make pressed key bits set
               LD   C,A
               AND  %00001100      ; keep left/down bits
               LD   A,ret_prev
               RET  NZ             ; ret if left/down pressed
               LD   A,C
               AND  %00010010      ; keep right/up bits
               LD   A,ret_next
               RET  NZ             ; ret if right/up pressed

               LD   HL,(blocks)    ; check buffered blocks
               LD   DE,32768/32-1  ; maximum we can buffer
               AND  A
               SBC  HL,DE
               JR   NC,sleep_loop  ; sleep if buffer full

               XOR  A
               LD   HL,(play_addr)
               CALL execute        ; execute 1 frame
               AND  A              ; execution error?
               RET  NZ             ; return if so

               CALL record_block   ; record the new SID state
               JP   play_loop      ; generate more data

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Buffer management

record_block:  LD   DE,(head)
               LD   HL,&D400       ; record from live SID values
               LD   BC,25          ; 25 registers to copy

               IN   A,(lmpr)
               PUSH AF
               LD   A,buffer_page+rom0_off
               OUT  (lmpr),A
               LDIR
               LD   L,&24          ; changes for control 1
               LDI
               LD   L,&2B          ; changes for control 2
               LDI
               LD   L,&32          ; changes for control 3
               LDI
               LD   L,&24
               LD   (HL),0         ; clear control changes 1
               LD   L,&2B
               LD   (HL),0         ; clear control changes 2
               LD   L,&32
               LD   (HL),0         ; clear control changes 3
               INC  E
               INC  E
               INC  E
               INC  DE             ; top up to 32 byte block
               RES  7,D            ; wrap in 32K block
               LD   (head),DE
               POP  AF
               OUT  (lmpr),A

               LD   HL,(blocks)
               INC  HL             ; 1 more block available
               LD   (blocks),HL
               RET

play_block:    LD   HL,(blocks)
               LD   A,H
               OR   L
               RET  Z
               DEC  HL             ; 1 less block available
               LD   (blocks),HL
               LD   DE,buffer_low
               SBC  HL,DE
               JR   NC,buffer_ok   ; jump if we're not low
               LD   A,128          ; screen off for speed boost
               OUT  (254),A
buffer_ok:     IN   A,(lmpr)
               PUSH AF
               LD   A,buffer_page+rom0_off
               OUT  (lmpr),A
               LD   HL,(tail)
               CALL sid_update
               RES  7,H            ; wrap in 32K block
               LD   (tail),HL
               POP  AF
               OUT  (lmpr),A
               RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Interrupt handling

start_gap1:    DEFS &D200-$        ; error if previous code is
gap1:          EQU  $-start_gap1   ; too big for available gap!

im2_table:     DEFS 257            ; 256 overlapped WORDs

enable_player: LD   HL,im2_table
               LD   A,&DD          ; handler at &DDDD
im2_fill:      LD   (HL),A
               INC  L
               JR   NZ,im2_fill
               INC  H
               LD   (HL),A         ; complete the final entry
               LD   A,im2_table/256
               LD   I,A
               IM   2              ; set interrupt mode 2
               EI
               RET

im2_handler:   PUSH AF
               PUSH BC
               PUSH DE
               PUSH HL
               CALL play_block
               POP  HL
               POP  DE
               POP  BC
               POP  AF
               EI
               RETI
end_1:

start_gap2:    DEFS &D400-$        ; error if previous code is
gap2:          EQU  $-start_gap2   ; too big for available gap


               DEFS &D440-$        ; gap for C64 SID registers
                                   ; plus second changes set

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; SID interface functions

sid_reset:     LD   HL,last_regs
               LD   BC,&00D4       ; SID base port is &D4
               LD   D,B            ; write 0 to all registers
               LD   A,25           ; 25 registers to write
reset_loop:    OUT  (C),D          ; write to register
               LD   (HL),D         ; remember new value
               INC  HL
               SET  7,B
               OUT  (C),D          ; effectively strobe write
               RES  7,B
               INC  B
               CP   B
               JR   NZ,reset_loop  ; loop until all reset

               XOR  A
               LD   (&D424),A      ; control for voice 1
               LD   (&D42B),A      ; control for voice 2
               LD   (&D432),A      ; control for voice 3
               RET

sid_update:    EX   DE,HL          ; switch new values to DE
               LD   C,&D4          ; SID interface base port

               LD   HL,25          ; control 1 changes offset
               ADD  HL,DE
               LD   A,(HL)         ; fetch changes
               AND  A
               JR   Z,control2     ; skip if nothing changed
               LD   (HL),0         ; reset changes for next time
               LD   HL,&04         ; new register 4 offset
               ADD  HL,DE
               XOR  (HL)           ; toggle changed bits
               LD   B,&04          ; SID register 4
               OUT  (C),A          ; write intermediate value
               LD   (last_regs+4),A ; update last reg value
               SET  7,B
               OUT  (C),A          ; strobe

control2:      LD   HL,26          ; control 2 changes offset
               ADD  HL,DE
               LD   A,(HL)
               AND  A
               JR   Z,control3     ; skip if no changes
               LD   (HL),0
               LD   HL,&0B
               ADD  HL,DE
               XOR  (HL)
               LD   B,&0B
               OUT  (C),A
               LD   (last_regs+&0B),A
               SET  7,B
               OUT  (C),A

control3:      LD   HL,27          ; control 3 changes offset
               ADD  HL,DE
               LD   A,(HL)
               AND  A
               JR   Z,control_done ; skip if no changes
               LD   (HL),0
               LD   HL,&12
               ADD  HL,DE
               XOR  (HL)
               LD   B,&12
               OUT  (C),A
               LD   (last_regs+&12),A
               SET  7,B
               OUT  (C),A

control_done:  LD   HL,last_regs   ; previous register values
               LD   B,0            ; start with register 0
out_loop:      LD   A,(DE)         ; new register value
               CP   (HL)           ; compare with previous value
               JR   Z,sid_skip     ; skip if no change
               OUT  (C),A          ; write value
               LD   (HL),A         ; store new value
               SET  7,B
               OUT  (C),A          ; effectively strobe write
               RES  7,B
sid_skip:      INC  HL
               INC  DE
               INC  B              ; next register
               LD   A,B
               CP   25             ; 25 registers to write
               JR   NZ,out_loop    ; loop until all updated
               LD   HL,7
               ADD  HL,DE          ; make up to a block of 32
               RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; 6510 emulation

execute:       EX   DE,HL          ; PC stays in DE throughout
               LD   B,A            ; set A from Z80 accumulator
               XOR  A
               LD   IY,0           ; X=0 and Y=0
               EXX
               LD   HL,&01FF       ; 6502 stack pointer in HL'
               LD   D,%00000100    ; interrupts disabled
               LD   B,A            ; clear V
               LD   C,A            ; clear C
               EXX
               LD   C,A            ; set Z
               EX   AF,AF'         ; clear N

i_nop:
main_loop:     LD   A,(DE)         ; fetch opcode
               INC  DE             ; PC=PC+1
               LD   L,A
               LD   H,decode_table/256
               LD   A,(HL)         ; handler low
               INC  H
               LD   H,(HL)         ; handler high
               LD   L,A
               JP   (HL)           ; execute!


; 6510 addressing modes, shared by logical and arithmetic
; instructions, but inlined into the load and store.

a_indirect_x:  LD   A,(DE)         ; pre-indexed indirect with X
               INC  DE
               DEFB &FD
               ADD  A,H            ; add X
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               LD   A,(HL)
               JP   (IX)

a_zero_page:   LD   A,(DE)         ; zero-page
               INC  DE
               LD   H,0
               LD   L,A
               LD   A,(HL)
               JP   (IX)

a_absolute:    EX   DE,HL          ; absolute (2-bytes)
               LD   E,(HL)
               INC  HL
               LD   D,(HL)
               INC  HL
               EX   DE,HL
               LD   A,(HL)
               JP   (IX)

a_indirect_y:  LD   A,(DE)         ; post-indexed indirect
               INC  DE
               LD   L,A
               LD   H,0
               DEFB &FD
               LD   A,L            ; Y
               ADD  A,(HL)
               LD   C,A
               LD   A,H
               INC  HL
               ADC  A,(HL)
               LD   H,A
               LD   L,C
               LD   A,(HL)
               JP   (IX)

a_zero_page_x: LD   A,(DE)         ; zero-page indexed with X
               INC  DE
               DEFB &FD
               ADD  A,H            ; add X
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               LD   A,(HL)
               JP   (IX)

a_zero_page_y: LD   A,(DE)         ; zero-page indexed with Y
               INC  DE
               DEFB &FD
               ADD  A,L            ; add Y
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               LD   A,(HL)
               JP   (IX)

a_absolute_y:  EX   DE,HL          ; absolute indexed with Y
               DEFB &FD
               LD   A,L            ; Y
               ADD  A,(HL)
               LD   E,A
               INC  HL
               LD   A,0
               ADC  A,(HL)
               LD   D,A
               INC  HL
               EX   DE,HL
               LD   A,(HL)
               JP   (IX)

a_absolute_x:  EX   DE,HL          ; absolute indexed with X
               DEFB &FD
               LD   A,H            ; X
               ADD  A,(HL)
               LD   E,A
               INC  HL
               LD   A,0
               ADC  A,(HL)
               LD   D,A
               INC  HL
               EX   DE,HL
               LD   A,(HL)
               JP   (IX)


; Instruction implementations

i_unknown:     LD   A,ret_invalid  ; invalid opcode
               RET

i_clc:         EXX                 ; clear carry
               LD   C,0
               EXX
               JP   main_loop
i_sec:         EXX                 ; set carry
               LD   C,1
               EXX
               JP   main_loop
i_cli:         EXX                 ; clear interrupt disable
               RES  2,D
               EXX
               JP   main_loop
i_sei:         EXX                 ; set interrupt disable
               SET  2,D
               EXX
               JP   main_loop
i_clv:         EXX                 ; clear overflow
               LD   B,0
               EXX
               JP   main_loop
i_cld:         EXX                 ; clear decimal mode
               RES  3,D
               EXX
               JP   main_loop
i_sed:         LD   A,ret_decimal  ; set decimal mode
               RET                 ; not supported!


i_bpl:         LD   A,(DE)
               INC  DE
               EX   AF,AF'
               LD   L,A            ; copy N
               EX   AF,AF'
               BIT  7,L            ; test N
               JR   Z,i_branch     ; branch if plus
               JP   main_loop
i_bmi:         LD   A,(DE)
               INC  DE
               EX   AF,AF'
               LD   L,A            ; copy N
               EX   AF,AF'
               BIT  7,L            ; test N
               JR   NZ,i_branch    ; branch if minus
               JP   main_loop
i_bvc:         LD   A,(DE)         ; V in bit 6
               INC  DE             ; V set if non-zero
               EXX
               BIT  6,B
               EXX
               JR   Z,i_branch     ; branch if V clear
               JP   main_loop
i_bvs:         LD   A,(DE)         ; V in bit 6
               INC  DE
               EXX
               BIT  6,B
               EXX
               JR   NZ,i_branch    ; branch if V set
               JP   main_loop
i_bcc:         LD   A,(DE)         ; C in bit 1
               INC  DE
               EXX
               BIT  0,C
               EXX
               JR   Z,i_branch     ; branch if C clear
               JP   main_loop
i_bcs:         LD   A,(DE)
               INC  DE
               EXX
               BIT  0,C
               EXX
               JR   NZ,i_branch    ; branch if C set
               JP   main_loop
i_beq:         LD   A,(DE)
               INC  DE
               INC  C
               DEC  C              ; zero?
               JR   Z,i_branch     ; branch if zero
               JP   main_loop
i_bne:         LD   A,(DE)
               INC  DE
               INC  C
               DEC  C              ; zero?
               JP   Z,main_loop    ; no branch if not zero
i_branch:      LD   L,A            ; offset low
               RLA                 ; set carry with sign
               SBC  A,A            ; form high byte for offset
               LD   H,A
               ADD  HL,DE          ; PC=PC+e
               EX   DE,HL
               JP   main_loop

i_jmp_a:       EX   DE,HL          ; JMP nn
               LD   E,(HL)
               INC  HL
               LD   D,(HL)
               INC  HL
               JP   main_loop

i_jmp_i:       EX   DE,HL          ; JMP (nn)
               LD   E,(HL)
               INC  HL
               LD   D,(HL)
               INC  HL
               EX   DE,HL
               LD   E,(HL)
               INC  L              ; CPU bug wraps within page
               LD   D,(HL)
               JP   main_loop

i_jsr:         EX   DE,HL          ; JSR nn
               LD   E,(HL)         ; subroutine low
               INC  HL             ; only 1 inc - we push ret-1
               LD   D,(HL)         ; subroutine high
               EX   DE,HL
               DEFB &DD
               LD   L,E            ; store in IXl
               LD   A,D
               EXX
               LD   (HL),A         ; push ret-1 high byte
               DEC  L              ; S--
               DEFB &DD
               LD   A,L            ; restore from IXl
               LD   (HL),A         ; push ret-1 low byte
               DEC  L              ; S--
               EXX
               EX   DE,HL
               JP   main_loop

i_brk:         LD   A,ret_brk      ; BRK (unsupported)
               RET

i_rts:         EXX                 ; RTS
               INC  L              ; S++
               LD   A,ret_ok
               RET  Z              ; finish if stack empty
               LD   A,(HL)
               DEFB &DD
               LD   L,A            ; store in IXl
               INC  L              ; S++
               LD   A,(HL)
               EXX
               LD   D,A
               DEFB &DD
               LD   E,L            ; restore from IXl
               INC  DE             ; PC=PC+1 (odd but true)
               JP   main_loop

i_rti:         LD   A,ret_ok       ; not needed as we only ever
               RET                 ; call the handler directly

i_php:         CALL make_p         ; make P from status+flags
               OR   %00010000      ; B always pushed as 1
               EXX
               LD   (HL),A
               DEC  L              ; S--
               EXX
               JP   main_loop
i_plp:         EXX                 ; PLP
               INC  L              ; S++
               LD   A,(HL)         ; P
               EXX
               CALL split_p        ; split P into status+flags
               JP   main_loop
i_pha:         LD   A,B            ; PHA
               EXX
               LD   (HL),A
               DEC  L              ; S--
               EXX
               JP   main_loop
i_pla:         EXX                 ; PLA
               INC  L              ; S++
               LD   A,(HL)
               EXX
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_dex:         DEFB &FD            ; DEX
               DEC  H              ; X--
               DEFB &FD
               LD   A,H            ; X
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_dey:         DEFB &FD            ; DEY
               DEC  L              ; Y--
               DEFB &FD
               LD   A,L            ; Y
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_inx:         DEFB &FD            ; INX
               INC  H              ; X++
               DEFB &FD
               LD   A,H            ; X
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_iny:         DEFB &FD            ; INY
               INC  L              ; Y++
               DEFB &FD
               LD   A,L            ; Y
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_txa:         DEFB &FD            ; TXA
               LD   A,H            ; X
               LD   B,A            ; A=X
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_tya:         DEFB &FD            ; TYA
               LD   A,L            ; Y
               LD   B,A            ; A=Y
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_tax:         DEFB &FD            ; TAX
               LD   H,B            ; X=A
               LD   C,B            ; set Z
               LD   A,B
               EX   AF,AF'         ; set N
               JP   main_loop
i_tay:         DEFB &FD            ; TAY
               LD   L,B            ; Y=A
               LD   C,B            ; set Z
               LD   A,B
               EX   AF,AF'         ; set N
               JP   main_loop
i_txs:         DEFB &FD            ; TXS
               LD   A,H            ; X
               EXX
               LD   L,A            ; set S (no flags set)
               EXX
               JP   main_loop
i_tsx:         EXX                 ; TSX
               LD   A,L            ; fetch S
               EXX
               DEFB &FD
               LD   H,A            ; X=S
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop


; For speed, LDA/LDX/LDY instructions have addressing inlined

i_lda_ix:      LD   A,(DE)         ; LDA ($nn,X)
               INC  DE
               DEFB &FD
               ADD  A,H            ; add X
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               LD   A,(HL)
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_lda_z:       LD   A,(DE)         ; LDA $nn
               INC  DE
               LD   H,0
               LD   L,A
               LD   A,(HL)
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_lda_a:       EX   DE,HL          ; LDA $nnnn
               LD   E,(HL)
               INC  HL
               LD   D,(HL)
               INC  HL
               EX   DE,HL
               LD   A,(HL)
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_lda_iy:      LD   A,(DE)         ; LDA ($nn),Y
               INC  DE
               LD   L,A
               LD   H,0
               DEFB &FD
               LD   A,L            ; Y
               ADD  A,(HL)
               LD   C,A
               LD   A,H
               INC  HL
               ADC  A,(HL)
               LD   H,A
               LD   L,C
               LD   A,(HL)
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_lda_zx:      LD   A,(DE)         ; LDA $nn,X
               INC  DE
               DEFB &FD
               ADD  A,H            ; add X
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               LD   A,(HL)
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_lda_ay:      EX   DE,HL          ; LDA $nnnn,Y
               DEFB &FD
               LD   A,L            ; Y
               ADD  A,(HL)
               LD   E,A
               INC  HL
               LD   A,0
               ADC  A,(HL)
               LD   D,A
               INC  HL
               EX   DE,HL
               LD   A,(HL)
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_lda_ax:      EX   DE,HL          ; LDA $nnnn,X
               DEFB &FD
               LD   A,H            ; X
               ADD  A,(HL)
               LD   E,A
               INC  HL
               LD   A,0
               ADC  A,(HL)
               LD   D,A
               INC  HL
               EX   DE,HL
               LD   A,(HL)
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_lda_i:       LD   A,(DE)         ; LDA #$nn
               INC  DE
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_ldx_z:       LD   A,(DE)         ; LDX $nn
               INC  DE
               LD   H,0
               LD   L,A
               LD   A,(HL)         ; set NZ
               DEFB &FD
               LD   H,A            ; set X
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_ldx_a:       EX   DE,HL          ; LDX $nnnn
               LD   E,(HL)
               INC  HL
               LD   D,(HL)
               INC  HL
               EX   DE,HL
               LD   A,(HL)
               DEFB &FD
               LD   H,A            ; set X
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_ldx_zy:      LD   A,(DE)         ; LDX $nn,Y
               INC  DE
               DEFB &FD
               ADD  A,L            ; add Y
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               LD   A,(HL)
               DEFB &FD
               LD   H,A            ; set X
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_ldx_ay:      EX   DE,HL          ; LDX $nnnn,Y
               DEFB &FD
               LD   A,L            ; Y
               ADD  A,(HL)
               LD   E,A
               INC  HL
               LD   A,0
               ADC  A,(HL)
               LD   D,A
               INC  HL
               EX   DE,HL
               LD   A,(HL)
               DEFB &FD
               LD   H,A            ; set X
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_ldx_i:       LD   A,(DE)         ; LDX #$nn
               INC  DE
               DEFB &FD
               LD   H,A            ; set X
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_ldy_z:       LD   A,(DE)         ; LDY $nn
               INC  DE
               LD   H,0
               LD   L,A
               LD   A,(HL)
               DEFB &FD
               LD   L,A            ; set Y
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_ldy_a:       EX   DE,HL          ; LDY $nnnn
               LD   E,(HL)
               INC  HL
               LD   D,(HL)
               INC  HL
               EX   DE,HL
               LD   A,(HL)
               DEFB &FD
               LD   L,A            ; set Y
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_ldy_zx:      LD   A,(DE)         ; LDY $nn,X
               INC  DE
               DEFB &FD
               ADD  A,H            ; add X
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               LD   A,(HL)
               DEFB &FD
               LD   L,A            ; set Y
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_ldy_ax:      EX   DE,HL          ; LDY $nnnn,X
               DEFB &FD
               LD   A,H            ; X
               ADD  A,(HL)
               LD   E,A
               INC  HL
               LD   A,0
               ADC  A,(HL)
               LD   D,A
               INC  HL
               EX   DE,HL
               LD   A,(HL)
               DEFB &FD
               LD   L,A            ; set Y
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_ldy_i:       LD   A,(DE)         ; LDY #$nn
               INC  DE
               DEFB &FD
               LD   L,A            ; set Y
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop


; For speed, STA/STX/STY instructions have addressing inlined

i_sta_ix:      LD   A,(DE)         ; STA ($xx,X)
               INC  DE
               DEFB &FD
               ADD  A,H            ; add X
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               LD   (HL),B
               JP   main_loop
i_sta_z:       LD   A,(DE)         ; STA $nn
               INC  DE
               LD   H,0
               LD   L,A
               LD   (HL),B
               JP   main_loop
i_sta_iy:      LD   A,(DE)
               INC  DE
               LD   L,A
               LD   H,0
               DEFB &FD
               LD   A,L            ; Y
               ADD  A,(HL)
               DEFB &DD
               LD   L,A            ; save in IXl
               LD   A,H
               INC  HL
               ADC  A,(HL)
               LD   H,A
               DEFB &DD
               LD   A,L            ; restore low byte from IXl
               LD   L,A
               LD   (HL),B
               JP   main_loop
i_sta_zx:      LD   A,(DE)
               INC  DE
               DEFB &FD
               ADD  A,H            ; add X
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               LD   (HL),B
               JP   main_loop
i_sta_ay:      EX   DE,HL
               DEFB &FD
               LD   A,L            ; Y
               ADD  A,(HL)
               LD   E,A
               INC  HL
               LD   A,0
               ADC  A,(HL)
               LD   D,A
               INC  HL
               EX   DE,HL
               LD   A,H
               CP   &D4
               JR   Z,sid_write_b  ; needed for Uridium
               LD   (HL),B
               JP   main_loop
i_sta_ax:      EX   DE,HL
               DEFB &FD
               LD   A,H            ; X
               ADD  A,(HL)
               LD   E,A
               INC  HL
               LD   A,0
               ADC  A,(HL)
               LD   D,A
               INC  HL
               EX   DE,HL
               LD   A,H
               CP   &D4
               JR   Z,sid_write_b  ; needed for Parallax
               LD   (HL),B
               JP   main_loop
i_sta_a:       EX   DE,HL
               LD   E,(HL)
               INC  HL
               LD   D,(HL)
               INC  HL
               EX   DE,HL
               LD   A,H
               CP   &D4
               JR   Z,sid_write_b  ; needed for Parallax
               LD   (HL),B
               JP   main_loop
i_stx_z:       LD   A,(DE)
               INC  DE
               LD   H,0
               LD   L,A
               DEFB &FD
               LD   A,H            ; X
               LD   (HL),A
               JP   main_loop
i_stx_zy:      LD   A,(DE)
               INC  DE
               DEFB &FD
               ADD  A,L            ; add Y
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               DEFB &FD
               LD   A,H            ; X
               LD   (HL),A
               JP   main_loop
i_stx_a:       EX   DE,HL
               LD   E,(HL)
               INC  HL
               LD   D,(HL)
               INC  HL
               EX   DE,HL
               LD   A,H
               CP   &D4
               DEFB &FD
               LD   A,H            ; X
               JR   Z,sid_write    ; needed for Glider Rider
               LD   (HL),A
               JP   main_loop

i_sty_z:       LD   A,(DE)
               INC  DE
               LD   H,0
               LD   L,A
               DEFB &FD
               LD   A,L            ; Y
               LD   (HL),A
               JP   main_loop
i_sty_zx:      LD   A,(DE)
               INC  DE
               DEFB &FD
               ADD  A,H            ; add X
               LD   L,A            ; (may wrap in zero page)
               LD   H,0
               DEFB &FD
               LD   A,L            ; Y
               LD   (HL),A
               JP   main_loop
i_sty_a:       EX   DE,HL
               LD   E,(HL)
               INC  HL
               LD   D,(HL)
               INC  HL
               EX   DE,HL
               DEFB &FD
               LD   A,L            ; Y
               LD   (HL),A
               JP   main_loop

sid_write_b:   LD   A,B
sid_write:     EXX
               LD   E,A            ; save new value
               EXX
               XOR  (HL)           ; determine changed bits
               SET  5,L            ; switch to changes set
               OR   (HL)           ; merge previous changes
               LD   (HL),A         ; update changes
               RES  5,L            ; back to original set
               EXX
               LD   A,E            ; retrieve new value
               EXX
               LD   (HL),A         ; update SID register
               JP   main_loop


i_adc_ix:      LD   IX,i_adc
               JP   a_indirect_x
i_adc_z:       LD   IX,i_adc
               JP   a_zero_page
i_adc_a:       LD   IX,i_adc
               JP   a_absolute
i_adc_iy:      LD   IX,i_adc
               JP   a_indirect_y
i_adc_zx:      LD   IX,i_adc
               JP   a_zero_page_x
i_adc_ay:      LD   IX,i_adc
               JP   a_absolute_y
i_adc_ax:      LD   IX,i_adc
               JP   a_absolute_x
i_adc_i:       LD   A,(DE)
               INC  DE
i_adc:         LD   L,A
               EXX
               LD   A,C            ; C
               EXX
               RRA                 ; set up carry
               LD   A,B            ; A
               ADC  A,L            ; A+M+C
               LD   B,A            ; set A
               JP   set_nvzc

i_sbc_ix:      LD   IX,i_sbc
               JP   a_indirect_x
i_sbc_z:       LD   IX,i_sbc
               JP   a_zero_page
i_sbc_a:       LD   IX,i_sbc
               JP   a_absolute
i_sbc_iy:      LD   IX,i_sbc
               JP   a_indirect_y
i_sbc_zx:      LD   IX,i_sbc
               JP   a_zero_page_x
i_sbc_ay:      LD   IX,i_sbc
               JP   a_absolute_y
i_sbc_ax:      LD   IX,i_sbc
               JP   a_absolute_x
i_sbc_i:       LD   A,(DE)
               INC  DE
i_sbc:         LD   L,A
               EXX
               LD   A,C            ; C
               EXX
               RRA                 ; set up carry
               LD   A,B            ; A
               CCF                 ; uses inverted carry
               SBC  A,L            ; A-M-(1-C)
               CCF                 ; no carry for overflow
               LD   B,A            ; set A
               JP   set_nvzc

i_and_ix:      LD   IX,i_and
               JP   a_indirect_x
i_and_z:       LD   IX,i_and
               JP   a_zero_page
i_and_a:       LD   IX,i_and
               JP   a_absolute
i_and_iy:      LD   IX,i_and
               JP   a_indirect_y
i_and_zx:      LD   IX,i_and
               JP   a_zero_page_x
i_and_ay:      LD   IX,i_and
               JP   a_absolute_y
i_and_ax:      LD   IX,i_and
               JP   a_absolute_x
i_and_i:       LD   A,(DE)
               INC  DE
i_and:         AND  B              ; A&x
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_eor_ix:      LD   IX,i_eor
               JP   a_indirect_x
i_eor_z:       LD   IX,i_eor
               JP   a_zero_page
i_eor_a:       LD   IX,i_eor
               JP   a_absolute
i_eor_iy:      LD   IX,i_eor
               JP   a_indirect_y
i_eor_zx:      LD   IX,i_eor
               JP   a_zero_page_x
i_eor_ay:      LD   IX,i_eor
               JP   a_absolute_y
i_eor_ax:      LD   IX,i_eor
               JP   a_absolute_x
i_eor_i:       LD   A,(DE)
               INC  DE
i_eor:         XOR  B              ; A^x
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_ora_ix:      LD   IX,i_ora
               JP   a_indirect_x
i_ora_z:       LD   IX,i_ora
               JP   a_zero_page
i_ora_a:       LD   IX,i_ora
               JP   a_absolute
i_ora_iy:      LD   IX,i_ora
               JP   a_indirect_y
i_ora_zx:      LD   IX,i_ora
               JP   a_zero_page_x
i_ora_ay:      LD   IX,i_ora
               JP   a_absolute_y
i_ora_ax:      LD   IX,i_ora
               JP   a_absolute_x
i_ora_i:       LD   A,(DE)
               INC  DE
i_ora:         OR   B              ; A|x
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_cmp_ix:      LD   IX,i_cmp
               JP   a_indirect_x
i_cmp_z:       LD   IX,i_cmp
               JP   a_zero_page
i_cmp_a:       LD   IX,i_cmp
               JP   a_absolute
i_cmp_iy:      LD   IX,i_cmp
               JP   a_indirect_y
i_cmp_zx:      LD   IX,i_cmp
               JP   a_zero_page_x
i_cmp_ay:      LD   IX,i_cmp
               JP   a_absolute_y
i_cmp_ax:      LD   IX,i_cmp
               JP   a_absolute_x
i_cmp_i:       LD   A,(DE)
               INC  DE
i_cmp:         LD   L,A            ; save operand
               LD   A,B            ; A
               SUB  L              ; SUB needed for end result
               CCF
               EXX
               RL   C              ; retrieve carry
               EXX
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_cpx_z:       LD   IX,i_cpx
               JP   a_zero_page
i_cpx_a:       LD   IX,i_cpx
               JP   a_absolute
i_cpx_i:       LD   A,(DE)
               INC  DE
i_cpx:         LD   L,A            ; save operand
               DEFB &FD
               LD   A,H            ; X
               SUB  L              ; SUB needed for end result
               CCF
               EXX
               RL   C              ; retrieve carry
               EXX
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_cpy_z:       LD   IX,i_cpy
               JP   a_zero_page
i_cpy_a:       LD   IX,i_cpy
               JP   a_absolute
i_cpy_i:       LD   A,(DE)
               INC  DE
i_cpy:         LD   L,A            ; save operand
               DEFB &FD
               LD   A,L            ; Y
               SUB  L              ; SUB needed for end result
               CCF
               EXX
               RL   C              ; retrieve carry
               EXX
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop


i_dec_z:       LD   IX,i_dec
               JP   a_zero_page
i_dec_zx:      LD   IX,i_dec
               JP   a_zero_page_x
i_dec_a:       LD   IX,i_dec
               JP   a_absolute
i_dec_ax:      LD   IX,i_dec
               JP   a_absolute_x
i_dec:         DEC  A
               LD   (HL),A         ; set memory
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_inc_z:       LD   IX,i_inc
               JP   a_zero_page
i_inc_zx:      LD   IX,i_inc
               JP   a_zero_page_x
i_inc_a:       LD   IX,i_inc
               JP   a_absolute
i_inc_ax:      LD   IX,i_inc
               JP   a_absolute_x
i_inc:         INC  A
               LD   (HL),A         ; set memory
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop


i_asl_z:       LD   IX,i_asl
               JP   a_zero_page
i_asl_zx:      LD   IX,i_asl
               JP   a_zero_page_x
i_asl_a:       LD   IX,i_asl
               JP   a_absolute
i_asl_ax:      LD   IX,i_asl
               JP   a_absolute_x
i_asl_acc:     SLA  B              ; A << 1
               EXX
               RL   C              ; retrieve carry
               EXX
               LD   C,B            ; set Z
               LD   A,B
               EX   AF,AF'         ; set N
               JP   main_loop
i_asl:         ADD  A,A            ; x << 1
               LD   (HL),A         ; set memory
               EXX
               RL   C              ; retrieve carry
               EXX
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_lsr_z:       LD   IX,i_lsr
               JP   a_zero_page
i_lsr_zx:      LD   IX,i_lsr
               JP   a_zero_page_x
i_lsr_a:       LD   IX,i_lsr
               JP   a_absolute
i_lsr_ax:      LD   IX,i_lsr
               JP   a_absolute_x
i_lsr_acc:     SRL  B              ; A >> 1
               EXX
               RL   C              ; retrieve carry
               EXX
               LD   C,B            ; set Z
               LD   A,B
               EX   AF,AF'         ; set N
               JP   main_loop
i_lsr:         SRL  A              ; x >> 1
               LD   (HL),A         ; set memory
               EXX
               RL   C              ; retrieve carry
               EXX
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_rol_z:       LD   IX,i_rol
               JP   a_zero_page
i_rol_zx:      LD   IX,i_rol
               JP   a_zero_page_x
i_rol_a:       LD   IX,i_rol
               JP   a_absolute
i_rol_ax:      LD   IX,i_rol
               JP   a_absolute_x
i_rol_acc:     LD   A,B
               EXX
               RR   C              ; set up carry
               RLA                 ; A << 1
               RL   C              ; retrieve carry
               EXX
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_rol:         EXX
               RR   C              ; set up carry
               RLA                 ; x << 1
               RL   C              ; retrieve carry
               EXX
               LD   (HL),A         ; set memory
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop

i_ror_z:       LD   IX,i_ror
               JP   a_zero_page
i_ror_zx:      LD   IX,i_ror
               JP   a_zero_page_x
i_ror_a:       LD   IX,i_ror
               JP   a_absolute
i_ror_ax:      LD   IX,i_ror
               JP   a_absolute_x
i_ror_acc:     LD   A,B
               EXX
               RR   C              ; set up carry
               RRA                 ; A >> 1
               RL   C              ; retrieve carry
               EXX
               LD   B,A            ; set A
               LD   C,B            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop
i_ror:         EXX
               RR   C              ; set up carry
               RRA                 ; x >> 1
               RL   C              ; retrieve carry
               EXX
               LD   (HL),A         ; set memory
               LD   C,A            ; set Z
               EX   AF,AF'         ; set N
               JP   main_loop


i_bit_z:       LD   IX,i_bit
               JP   a_zero_page
i_bit_a:       LD   IX,i_bit
               JP   a_absolute
i_bit:         LD   L,A            ; keep memory value
               EX   AF,AF'         ; N flag set from bit 7
               LD   A,L
               AND  %01000000      ; V flag set from bit 6
               EXX
               LD   B,A            ; set V
               EXX
               LD   A,B            ; A
               AND  L              ; perform BIT test
               LD   C,A            ; set Z
               JP   main_loop


set_nvzc:      LD   C,A            ; set Z
               RLA                 ; C in bit 0, no effect on V
               EXX
               LD   C,A            ; set C
               JP   PE,set_v
               LD   B,%00000000    ; V clear
               EXX
               LD   A,C
               EX   AF,AF'         ; set N
               JP   main_loop
set_v:         LD   B,%01000000    ; V set
               EXX
               LD   A,C
               EX   AF,AF'         ; set N
               JP   main_loop

make_p:        EX   AF,AF'
               AND  %10000000      ; keep N
               LD   L,A            ; N
               EX   AF,AF'
               LD   A,C            ; Z
               SUB  1              ; set carry if zero
               RLA
               RLA
               AND  %00000010      ; keep 6510 Z bit
               OR   L              ; N+Z
               EXX
               OR   B              ; N+V+Z
               LD   E,A
               LD   A,C
               AND  %00000001      ; keep C
               OR   E              ; N+V+Z+C
               EXX
               RET

split_p:       EXX
               LD   E,A            ; save P
               AND  %00111100      ; keep CPU bits
               LD   D,A            ; set status
               LD   A,E
               EX   AF,AF'         ; set N
               LD   A,E
               AND  %01000000      ; keep V
               LD   B,A            ; set V
               LD   A,E
               AND  %00000001      ; keep C
               LD   C,A            ; set C
               LD   A,E
               CPL
               AND  %00000010      ; Z=0 NZ=2
               EXX
               LD   C,A            ; set NZ
               RET

; Reordering the decode table to group low and high bytes means
; we avoid any 16-bit arithmetic for the decode stage, saving
; 12T on the old method (cool tip from Dave Laundon)
reorder_decode:XOR  A
               LD   BC,0           ; use zero-page
               LD   HL,256         ; use 6510 stack area
               LD   DE,decode_table
reorder_loop:  EX   AF,AF'
               LD   A,(DE)
               LD   (BC),A         ; low byte
               INC  E
               INC  C
               LD   A,(DE)
               LD   (HL),A
               INC  DE
               INC  L
               EX   AF,AF'
               DEC  A
               JR   NZ,reorder_loop
               LD   H,2            ; HL=512
               LD   B,H            ; BC=512
               LDDR
               LD   H,L            ; HL=0
clear_loop:    LD   (HL),H         ; tidy zero-page
               INC  L
               DJNZ clear_loop
               RET


start_gap3:    DEFS &DC00-$        ; error if previous code is
gap3:          EQU  $-start_gap3   ; too big for available gap!


               DEFS &DD20-$

old_stack:     DEFW 0
stack:         DEFS 64             ; small private stack
stack_top:     EQU  $

blocks:        DEFW 0              ; buffered block count
head:          DEFW 0              ; head for recorded data
tail:          DEFW 0              ; tail for playing data

init_addr:     DEFW 0
play_addr:     DEFW 0
play_song:     DEFB 0

last_regs:     DEFS 32             ; last values written to SID


start_gap4:    DEFS &DDDD-$        ; error if previous block is
gap4:          EQU  $-start_gap4   ; too big for available gap!
               JP   im2_handler    ; interrupt mode 2 handler


               DEFS &DE00-$        ; must be 512-byte aligned

decode_table:  DEFW i_brk,i_ora_ix,i_unknown,i_unknown     ; 00
               DEFW i_unknown,i_ora_z,i_asl_z,i_unknown    ; 04
               DEFW i_php,i_ora_i,i_asl_acc,i_unknown      ; 08
               DEFW i_unknown,i_ora_a,i_asl_a,i_unknown    ; 0C

               DEFW i_bpl,i_ora_iy,i_unknown,i_unknown     ; 10
               DEFW i_unknown,i_ora_zx,i_asl_zx,i_unknown  ; 14
               DEFW i_clc,i_ora_ay,i_unknown,i_unknown     ; 18
               DEFW i_unknown,i_ora_ax,i_asl_ax,i_unknown  ; 1C

               DEFW i_jsr,i_and_ix,i_unknown,i_unknown     ; 20
               DEFW i_bit_z,i_and_z,i_rol_z,i_unknown      ; 24
               DEFW i_plp,i_and_i,i_rol_acc,i_unknown      ; 28
               DEFW i_bit_a,i_and_a,i_rol_a,i_unknown      ; 2C

               DEFW i_bmi,i_and_iy,i_unknown,i_unknown     ; 30
               DEFW i_unknown,i_and_zx,i_rol_zx,i_unknown  ; 34
               DEFW i_sec,i_and_ay,i_unknown,i_unknown     ; 38
               DEFW i_unknown,i_and_ax,i_rol_ax,i_unknown  ; 3C

               DEFW i_rti,i_eor_ix,i_unknown,i_unknown     ; 40
               DEFW i_unknown,i_eor_z,i_lsr_z,i_unknown    ; 44
               DEFW i_pha,i_eor_i,i_lsr_acc,i_unknown      ; 48
               DEFW i_jmp_a,i_eor_a,i_lsr_a,i_unknown      ; 4C

               DEFW i_bvc,i_eor_iy,i_unknown,i_unknown     ; 50
               DEFW i_unknown,i_eor_zx,i_lsr_zx,i_unknown  ; 54
               DEFW i_cli,i_eor_ay,i_unknown,i_unknown     ; 58
               DEFW i_unknown,i_eor_ax,i_lsr_ax,i_unknown  ; 5C

               DEFW i_rts,i_adc_ix,i_unknown,i_unknown     ; 60
               DEFW i_unknown,i_adc_z,i_ror_z,i_unknown    ; 64
               DEFW i_pla,i_adc_i,i_ror_acc,i_unknown      ; 68
               DEFW i_jmp_i,i_adc_a,i_ror_a,i_unknown      ; 6C

               DEFW i_bvs,i_adc_iy,i_unknown,i_unknown     ; 70
               DEFW i_unknown,i_adc_zx,i_ror_zx,i_unknown  ; 74
               DEFW i_sei,i_adc_ay,i_unknown,i_unknown     ; 78
               DEFW i_unknown,i_adc_ax,i_ror_ax,i_unknown  ; 7C

               DEFW i_unknown,i_sta_ix,i_unknown,i_unknown ; 80
               DEFW i_sty_z,i_sta_z,i_stx_z,i_unknown      ; 84
               DEFW i_dey,i_unknown,i_txa,i_unknown        ; 88
               DEFW i_sty_a,i_sta_a,i_stx_a,i_unknown      ; 8C

               DEFW i_bcc,i_sta_iy,i_unknown,i_unknown     ; 90
               DEFW i_sty_zx,i_sta_zx,i_stx_zy,i_unknown   ; 94
               DEFW i_tya,i_sta_ay,i_txs,i_unknown         ; 98
               DEFW i_unknown,i_sta_ax,i_unknown,i_unknown ; 9C

               DEFW i_ldy_i,i_lda_ix,i_ldx_i,i_unknown     ; A0
               DEFW i_ldy_z,i_lda_z,i_ldx_z,i_unknown      ; A4
               DEFW i_tay,i_lda_i,i_tax,i_unknown          ; A8
               DEFW i_ldy_a,i_lda_a,i_ldx_a,i_unknown      ; AC

               DEFW i_bcs,i_lda_iy,i_unknown,i_unknown     ; B0
               DEFW i_ldy_zx,i_lda_zx,i_ldx_zy,i_unknown   ; B4
               DEFW i_clv,i_lda_ay,i_tsx,i_unknown         ; B8
               DEFW i_ldy_ax,i_lda_ax,i_ldx_ay,i_unknown   ; BC

               DEFW i_cpy_i,i_cmp_ix,i_unknown,i_unknown   ; C0
               DEFW i_cpy_z,i_cmp_z,i_dec_z,i_unknown      ; C4
               DEFW i_iny,i_cmp_i,i_dex,i_unknown          ; C8
               DEFW i_cpy_a,i_cmp_a,i_dec_a,i_unknown      ; CC

               DEFW i_bne,i_cmp_iy,i_unknown,i_unknown     ; D0
               DEFW i_unknown,i_cmp_zx,i_dec_zx,i_unknown  ; D4
               DEFW i_cld,i_cmp_ay,i_unknown,i_unknown     ; D8
               DEFW i_unknown,i_cmp_ax,i_dec_ax,i_unknown  ; DC

               DEFW i_cpx_i,i_sbc_ix,i_unknown,i_unknown   ; E0
               DEFW i_cpx_z,i_sbc_z,i_inc_z,i_unknown      ; E4
               DEFW i_inx,i_sbc_i,i_nop,i_unknown          ; E8
               DEFW i_cpx_a,i_sbc_a,i_inc_a,i_unknown      ; EC

               DEFW i_beq,i_sbc_iy,i_unknown,i_unknown     ; F0
               DEFW i_unknown,i_sbc_zx,i_inc_zx,i_unknown  ; F4
               DEFW i_sed,i_sbc_ay,i_unknown,i_unknown     ; F8
               DEFW i_unknown,i_sbc_ax,i_inc_ax,i_unknown  ; FC

end:           EQU  $
size:          EQU  end-base
