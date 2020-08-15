/*
 * Copyright (c) 2009 Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "../include/hax.h"
#include "include/compiler.h"
#include "include/ia32_defs.h"
#include "include/vcpu.h"

//#define assertmsg RTL_SOFT_ASSERTMSG
#define assertmsg ASSERTMSG

static void rmode_segment_valid(segment_desc_t *var, char *msg)
{
    assertmsg(msg, var->ar == 0xf3);
    assertmsg(msg, var->base == (var->selector << 4));
    assertmsg(msg, var->limit == 0xffff);
}

static void ar_valid(segment_desc_t *var, char *msg)
{
    assertmsg(msg, var->present);
    assertmsg(msg, (var->ar & 0xF00) == 0);
    if (var->limit & 0xFFF != 0xFFF)
        assertmsg(msg, var->granularity == 0);
    if ((var->limit >> 20) & 0xFFF)
        assertmsg(msg, var->granularity == 1);
    assertmsg(msg, (var->ar & 0xFFFE0000) == 0);
}

static void code_segment_valid(struct vcpu_t *vcpu)
{
    uint32_t scpu_ctls = vmx(vcpu, scpu_ctls_base);
    segment_desc_t *cs = &vcpu->state->_cs, *ss = &vcpu->state->_ss;
    char *msg = "cs:";

    assertmsg(msg, !cs->null);
    if (scpu_ctls & UNRESTRICTED_GUEST)
        assertmsg(msg, (cs->type == 3 || cs->type & 9));
    else
        assertmsg(msg, cs->type & 9);
    assertmsg(msg, cs->desc);
    if (cs->type == 3)
        assertmsg(msg, cs->dpl == 0);
    if (cs->type & 4)
        assertmsg(msg, cs->dpl <= ss->dpl);
    else
        assertmsg(msg, cs->dpl == ss->dpl);
    ar_valid(cs, msg);
}

static void stack_segment_valid(struct vcpu_t *vcpu)
{
    uint32_t scpu_ctls = vmx(vcpu, scpu_ctls_base);
    segment_desc_t *cs = &vcpu->state->_cs, *ss = &vcpu->state->_ss;
    char *msg = "ss:";

    if (!ss->null)
    {
        assertmsg(msg, (ss->type == 3 || ss->type == 7));
        assertmsg(msg, ss->desc);
        ar_valid(ss, msg);
    }
    if ((scpu_ctls & UNRESTRICTED_GUEST) == 0)
    {
        assertmsg(msg, ss->dpl == (ss->selector & 3));
        assertmsg(msg, (ss->selector & 3) == (cs->selector & 3));
    }
    if (cs->type == 3 || !(vcpu->state->_cr0 & CR0_PE))
    assertmsg(msg, ss->dpl == 0);
}

static void data_segment_valid(struct vcpu_t *vcpu, segment_desc_t *var, char *msg)
{
    uint32_t scpu_ctls = vmx(vcpu, scpu_ctls_base);

    if (!var->null)
    {
        assertmsg(msg, var->type & 1);
        if (var->type & 8) assertmsg(msg, var->type & 2);
        assertmsg(msg, var->desc);
        if ((scpu_ctls & UNRESTRICTED_GUEST) == 0 && var->type <= 11)
            assertmsg(msg, var->dpl >= (var->selector & 3));
        ar_valid(var, msg);
    }
}

static void tr_valid(struct vcpu_t *vcpu)
{
    segment_desc_t *tr = &vcpu->state->_tr;
    char *msg = "tr:";

    assertmsg(msg, tr->type == 3 || tr->type == 11);
    assertmsg(msg, tr->desc == 0);
    assertmsg(msg, !tr->null);
    ar_valid(tr, msg);
    assertmsg(msg, (tr->selector & 4) == 0);
}

static void ldtr_valid(struct vcpu_t *vcpu)
{
    segment_desc_t *ldtr = &vcpu->state->_ldt;
    char *msg = "ldtr:";

    if (!ldtr->null)
    {
        assertmsg(msg, ldtr->type == 2);
        assertmsg(msg, ldtr->desc == 0);
        ar_valid(ldtr, msg);
    }
}

void guest_state_valid(struct vcpu_t *vcpu)
{
    /* real mode guest state checks */
    if (!(vcpu->state->_cr0 & CR0_PE) || (vcpu->state->_rflags & RFLAGS_VM))
    {
        rmode_segment_valid(&vcpu->state->_cs, "cs:");
        rmode_segment_valid(&vcpu->state->_ss, "ss:");
        rmode_segment_valid(&vcpu->state->_ds, "ds:");
        rmode_segment_valid(&vcpu->state->_es, "es:");
        rmode_segment_valid(&vcpu->state->_fs, "fs:");
        rmode_segment_valid(&vcpu->state->_gs, "gs:");
    } else {
        /* protected mode guest state checks */
        code_segment_valid(vcpu);
        stack_segment_valid(vcpu);
        data_segment_valid(vcpu, &vcpu->state->_ds, "ds:");
        data_segment_valid(vcpu, &vcpu->state->_es, "es:");
        data_segment_valid(vcpu, &vcpu->state->_fs, "fs:");
        data_segment_valid(vcpu, &vcpu->state->_gs, "gs:");
        tr_valid(vcpu);
        ldtr_valid(vcpu);
        assertmsg("gdtr:", (vcpu->state->_gdt.limit & 0xFFFF0000) == 0);
        assertmsg("idtr:", (vcpu->state->_idt.limit & 0xFFFF0000) == 0);
    }
    /* TODO:
     * - Add checks on RIP
     * - Add checks on RFLAGS
     */
}
