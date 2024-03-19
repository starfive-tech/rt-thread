/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018/10/28     Bernard      The unify RISC-V porting implementation
 * 2018/12/27     Jesven       Add SMP support
 * 2021/02/02     lizhirui     Add userspace support
 * 2022/10/22     WangXiaoyao  Support User mode RVV;
 *                             Trimming process switch context
 */

#include "cpuport.h"
#include "stackframe.h"

/*
 * void rt_hw_context_switch_to(rt_ubase_t to);
 *
 * a0 --> to SP pointer
 */
.globl rt_hw_context_switch_to
rt_hw_context_switch_to:
    LOAD sp, (a0)

    la s0, rt_current_thread
    LOAD s1, (s0)

    #ifdef RT_USING_SMART
        mv a0, s1
        jal lwp_aspace_switch
    #endif

    RESTORE_ALL
    sret

/*
 * void rt_hw_context_switch(rt_ubase_t from, rt_ubase_t to);
 *
 * a0 --> from SP pointer
 * a1 --> to SP pointer
 *
 * It should only be used on local interrupt disable
 */
.globl rt_hw_context_switch
rt_hw_context_switch:
    mv t2, sp
    li t0, 0x120//set SPIE and SPP = 1
    csrs sstatus, t0//if enter here,caller must be in system thread
    csrw sepc, ra//return address
    //saved from thread context
    SAVE_ALL

    STORE t2, 32 * REGBYTES(sp)//save user_sp


    STORE sp, (a0)

    // restore to thread SP
    LOAD sp, (a1)

    // restore Address Space
    la s0, rt_current_thread
    LOAD s1, (s0)

    #ifdef RT_USING_SMART
        mv a0, s1
        jal lwp_aspace_switch
    #endif

    RESTORE_ALL
    sret

#ifdef ENABLE_VECTOR
/**
 * @param a0 pointer to frame bottom
 */
.global rt_hw_vector_ctx_save
rt_hw_vector_ctx_save:
    SAVE_VECTOR a0
    ret

/**
 * @param a0 pointer to frame bottom
 */
.global rt_hw_vector_ctx_restore
rt_hw_vector_ctx_restore:
    RESTORE_VECTOR a0
    ret

.global rt_hw_disable_vector
rt_hw_disable_vector:
    li t0, SSTATUS_VS
    csrc sstatus, t0
    ret

.global rt_hw_enable_vector
rt_hw_enable_vector:
    li t0, SSTATUS_VS
    csrs sstatus, t0
    ret
#endif /* ENABLE_VECTOR */
