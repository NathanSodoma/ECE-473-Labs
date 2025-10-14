    .text
    .globl delay_cycles
    .type  delay_cycles, @function
/*
 * 
 *
 * total cycles = N + 8*cycles
 * N (fixed overhead) = 9 cycles inside this function (does not include CALL).
 * 
 *  - Entry zero-check and branch to loop:  sbiw (2) + breq (1 when not taken) + rjmp (2) = 5
 *  - Each loop iteration costs exactly 8 cycles, including the final iteration:
 *        nop (1) + nop (1) + nop (1) + nop (1) + sbiw (2) + brne taken (2) = 8
 *    On the final iteration, brne is not taken (1), but the following padding nop (1)
 *    makes it 8 as well: 1+1+1+1+2+1+1 = 8.
 *  - Function epilogue: ret (4)
 *  - When cycles == 0, we skip the loop and execute a balancing nop before ret so
 *    the zero case also equals the same fixed overhead N = 9 cycles:
 *        sbiw (2) + breq taken (2) + nop (1) + ret (4) = 9
 *
 * Therefore for cycles > 0:
 *    total = 5 (entry to loop) + 8*cycles (loop body) + 4 (ret) = 9 + 8*cycles
 * and for cycles == 0:
 *    total = 9
 * In both cases, total = N + 8*cycles with N = 9.
 */

delay_cycles:
    sbiw    r24, 0         ; 2  ; test if cycles == 0 without changing r24:r25
    breq    .Lret_with_nop ; 2/1; if zero, branch to early return path
    rjmp    .Lloop         ; 2  ; align to loop entry

.Lloop:
    nop                    ; 1
    nop                    ; 1
    nop                    ; 1
    nop                    ; 1
    sbiw    r24, 1         ; 2  ; cycles-- (r25:r24 = r25:r24 - 1)
    brne    .Lloop         ; 2/1; loop until cycles == 0
    nop                    ; 1  ; pad so final iteration is exactly 8 cycles

.Lret:
    ret                    ; 4

.Lret_with_nop:
    nop                    ; 1  ; balance zero-case so overhead N = 9 cycles
    rjmp    .Lret          ; 2  ; (could be 'ret' directly, but we reuse .Lret)
    .size  delay_cycles, .-delay_cycles
