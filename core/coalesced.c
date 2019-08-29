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

/* We have to following assumption on coalesced writes:
 *
 * The usermode application is responsible for checking of REP prefix on
 * HAX_EXIT_FAST_MMIO and handle it accordingly. The result of such an
 * instruction will NOT be added to coalesced MMIO queue, as usermode
 * application can handle it faster by knowing counter in ECX and ESI->EDI
 * directly.
 * Usermode application must update ESI, EDI and ECX accordingly when
 * handling it!
 *
 * When consuming the coalesced queue, caller has to reset count to 0!
 */

#include "include/vm.h"
#include "include/hax_driver.h"
#include "include/vcpu.h"
#include "../include/hax.h"
#include "include/coalesced.h"
#include "include/paging.h"

struct coalesced_t
{
    struct hax_coalesced_mmio *mmiobuf;
    struct hax_vcpu_mem mmiobuf_vcpumem;
    hax_gpa_space gpa_space;
    hax_memslot *last_slot;
    uint32_t exit_status_save;
    uint64_t io_buf_save;
    int restore;
};

static int coalesced_write_setup(struct coalesced_t *ct);
static void coalesced_write_destroy(struct coalesced_t *ct);
static void _coalesced_status(struct vcpu_t *vcpu);

int coalesced_write_init(struct vm_t *vm)
{
    if (vm->clsc) return 0;

    if (vm->clsc = hax_vmalloc(sizeof(struct hax_coalesced_mmio), 0))
        return coalesced_write_setup(vm->clsc);
    return -ENOMEM;
}

void coalesced_write_exit(struct vm_t *vm)
{
    if (!vm->clsc) return;
    coalesced_write_destroy(vm->clsc);
    hax_vfree(vm->clsc, sizeof(struct hax_coalesced_mmio));

    vm->clsc = NULL;
}


static int coalesced_write_setup(struct coalesced_t *ct)
{
    int ret;

    ret = hax_setup_vcpumem(&ct->mmiobuf_vcpumem, 0, sizeof(struct hax_coalesced_mmio), 0);
    if (ret >= 0) {
        ct->mmiobuf = (struct hax_coalesced_mmio *)ct->mmiobuf_vcpumem.kva;
        if (gpa_space_init(&ct->gpa_space) == 0)
            return ret;
    }
    coalesced_write_destroy(ct);

    return -ENOMEM;
}

static void coalesced_write_destroy(struct coalesced_t *ct)
{
    if (ct->mmiobuf)
    {
        if (ct->mmiobuf_vcpumem.uva)
            hax_clear_vcpumem(&ct->mmiobuf_vcpumem);

        gpa_space_free(&ct->gpa_space);
        ct->mmiobuf = NULL;
    }
}

/* Support for coalesced write of emulator */
em_status_t coalesced_write(struct em_context_t *ctxt)
{
    struct vcpu_t *vcpu = ctxt->vcpu;
    const struct em_opcode_t *opcode = &ctxt->opcode;
    struct coalesced_t *clsc = vcpu->vm->clsc;
    struct hax_fastmmio *hft = (struct hax_fastmmio *)vcpu->io_buf;
    hax_memslot *memslot;
    uint64_t gfn;

    /* We are not initialized, no coalesced pages to handle */
    if (!clsc) return EM_EXIT_MMIO;

    /* Determine if this is for one of our handled coalesced write memory areas
     * memslot_find doesn't have cache, so we cache */
    gfn = hft->gpa >> PG_ORDER_4K;
    if ((clsc->last_slot && gfn >= clsc->last_slot->base_gfn &&
        gfn <= clsc->last_slot->base_gfn + clsc->last_slot->npages) ||
        (clsc->last_slot = memslot_find(&clsc->gpa_space, gfn)))
    {
        if (clsc->mmiobuf->size >= sizeof(clsc->mmiobuf->mmio) / sizeof(clsc->mmiobuf->mmio[0])) {
            vcpu_set_panic(vcpu);
            hax_log(HAX_LOGPANIC, "%s: Nobody handled HAX_EXIT_COALESCED_MMIO, "
                "queue is full now: vcpu_id=%u\n",
                __func__, vcpu->vcpu_id);
        }

        /* Add current event to list */
        clsc->mmiobuf->mmio[clsc->mmiobuf->size++] = *hft;

        /* Set our marker, so that callers detect that we did a coalesced MMIO */
        _coalesced_status(vcpu);

        /* If we are full now, flush it immediately, nothing to redo here after flush,
         * therefore just overwrite HAX_EXIT_MMIO with coalesced one */
        if (clsc->mmiobuf->size >= sizeof(clsc->mmiobuf->mmio) / sizeof(clsc->mmiobuf->mmio[0]))
            return EM_EXIT_MMIO;

        /* Data just added to buffer, continue emulator. */
        return EM_CONTINUE;
    }

    /* We were on EXIT_MMIO and there is nothing to handle for us, so do MMIO exit */
    return EM_EXIT_MMIO;
}

int coalesced_set_mapping(struct vm_t *vm, uint64_t start_gfn,    uint64_t npages)
{
    int ret;

    if (!vm->clsc)
    {
        /* No buffer yet, but we need one now, allocate */
        if ((ret = coalesced_write_init(vm)) < 0) return ret;
    }

    /* Insert into list, do not check VA as it is always 0 for MMIO */
    vm->clsc->last_slot = NULL;
    return memslot_set_mapping(&vm->clsc->gpa_space, start_gfn, npages, 0, HAX_MEMSLOT_INVALIDUVA);
}

static void _coalesced_status(struct vcpu_t *vcpu)
{
    vcpu->tunnel->_exit_status = HAX_EXIT_COALESCED_MMIO;
    *(uint64_t*)vcpu->io_buf = vcpu->vm->clsc->mmiobuf_vcpumem.uva;
}

int coalesced_flush(struct vcpu_t *vcpu)
{
    struct coalesced_t *clsc = vcpu->vm->clsc;

    if (clsc && clsc->mmiobuf->size)
    {
        /* Save original call for later */
        clsc->exit_status_save = vcpu->tunnel->_exit_status;
        clsc->io_buf_save = *(uint64_t*)vcpu->io_buf;
        clsc->restore = 1;
        _coalesced_status(vcpu);
        return HAX_EXIT;
    }
    return HAX_RESUME;
}

int coalesced_handle(struct vcpu_t *vcpu)
{
    struct coalesced_t *clsc = vcpu->vm->clsc;

    /* Restore the original call for handling */
    if (clsc->restore)
    {
        vcpu->tunnel->_exit_status = clsc->exit_status_save;
        *(uint64_t*)vcpu->io_buf = clsc->io_buf_save;
        clsc->restore = 0;
        return HAX_EXIT;
    }
    return HAX_RESUME;
}
