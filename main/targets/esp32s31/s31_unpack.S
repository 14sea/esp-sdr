/* S31 SIMD core 1: native IQ10 (Q low, I high) to interleaved windowed IQ16.
 * void s31_unpack_iq10_win(src, dst, duplicated_window, groups8)
 * Source is word-aligned; destination and window are 16-byte aligned.
 * Reads up to one extra vector: callers must guard bank ends and wraps.
 * ((w >> 4) & 0x0000ffc0) | ((w << 22) & 0xffc00000), then Q15
 * multiplication gives exactly (signed_sample * window) >> 9. Mask Q before
 * shifting: the S31 left-shift instruction saturates overflowing words.
 */
    .text
    .align  4
    .global s31_unpack_iq10_win
    .type   s31_unpack_iq10_win, @function
s31_unpack_iq10_win:
    addi sp,sp,-16
    mv t3,a0
    mv t4,a1
    mv t5,a2
    li a4,0x0000ffc0
    sw a4,0(sp)
    li a4,0x000003ff
    sw a4,4(sp)
    mv a4,sp
    esp.vldbc.32.ip q6,a4,4
    esp.vldbc.32.ip q7,a4,0
    esp.ld.128.usar.ip q0, t3, 16
    esp.lp.setup 0,a3,.Lunpack_end
    /* words 0..3 */
    esp.ld.128.usar.ip q1, t3, 16
    esp.src.q    q2, q0, q1
    li a4,4
    esp.movx.w.sar a4
    esp.vsr.u32 q3,q2
    li a4,22
    esp.movx.w.sar a4
    esp.andq q4,q2,q7
    esp.vsl.32   q4, q4
    esp.andq     q3, q3, q6
    esp.orq      q3, q3, q4
    esp.vld.128.ip q5, t5, 16
    li a4,15
    esp.movx.w.sar a4
    esp.vmul.s16 q3, q3, q5
    esp.vst.128.ip q3, t4, 16
    /* words 4..7 */
    esp.ld.128.usar.ip q0, t3, 16
    esp.src.q    q2, q1, q0
    li a4,4
    esp.movx.w.sar a4
    esp.vsr.u32 q3,q2
    li a4,22
    esp.movx.w.sar a4
    esp.andq q4,q2,q7
    esp.vsl.32   q4, q4
    esp.andq     q3, q3, q6
    esp.orq      q3, q3, q4
    esp.vld.128.ip q5, t5, 16
    li a4,15
    esp.movx.w.sar a4
    esp.vmul.s16 q3, q3, q5
.Lunpack_end: esp.vst.128.ip q3,t4,16
    addi sp,sp,16
    ret
    .size   s31_unpack_iq10_win, . - s31_unpack_iq10_win
