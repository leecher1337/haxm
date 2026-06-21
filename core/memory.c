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
#include "include/vcpu.h"
#include "include/vm.h"
#include "include/hax_driver.h"
#include "include/hax_core_interface.h"
#include "include/ept.h"
#ifdef CONFIG_HAX_EPT2
#include "include/paging.h"
#endif  // CONFIG_HAX_EPT2

#ifdef CONFIG_HAX_EPT2
static int handle_alloc_ram(struct vm_t *vm, uint64_t start_uva, uint64_t size)
{
    int ret;
    hax_ramblock *block;

    if (!start_uva) {
        hax_error("%s: start_uva == 0\n", __func__);
        return -EINVAL;
    }
    if (!size) {
        hax_error("%s: size == 0\n", __func__);
        return -EINVAL;
    }

    hax_assert(vm != NULL);
    ret = ramblock_add(&vm->gpa_space.ramblock_list, start_uva, size, NULL,
                       &block);
    if (ret) {
        hax_error("%s: ramblock_add() failed: ret=%d, start_uva=0x%llx,"
                  " size=0x%llx\n", __func__, ret, start_uva, size);
        return ret;
    }
    return 0;
}
#endif  // CONFIG_HAX_EPT2

int hax_vm_add_ramblock(struct vm_t *vm, uint64_t start_uva, uint64_t size)
{
#ifdef CONFIG_HAX_EPT2
    return handle_alloc_ram(vm, start_uva, size);
#else  // !CONFIG_HAX_EPT2
    uint64_t gva = start_uva;
    uint32_t leftsize;
    struct hax_vcpu_mem *mem = NULL, *curmem, *smem;
    int entry_num = 0, i, ret;
    uint32_t cursize;

    /* A valid size is needed */
    if (0 == size) {
        hax_error("hax_vm_alloc_ram: the size is 0, invalid!\n");
        return -EINVAL;
    }

    if (!gva || gva & 0xfff) {
        hax_error("Invalid gva %llx for allocating memory.\n", gva);
        return -EINVAL;
    }

    hax_info("hax_vm_alloc_ram: size 0x%x\n", size);
    if (!hax_test_bit(VM_STATE_FLAGS_MEM_ALLOC, &vm->flags)) {
        hax_info("!VM_STATE_FLAGS_MEM_ALLOC\n");
        hax_mutex_lock(hax->hax_lock);
        if (hax->mem_limit && (size > hax->mem_quota)) {
            hax_error("HAX is out of memory quota.\n");
            hax_mutex_unlock(hax->hax_lock);
            return -EINVAL;
        }
        hax_mutex_unlock(hax->hax_lock);
        hax_info("Memory allocation, va:%llx, size:%x\n", *va, size);
    } else {
        hax_info("spare alloc: mem_limit 0x%llx, size 0x%x, spare_ram 0x%llx\n",
                 hax->mem_limit, size, vm->spare_ramsize);
        if (hax->mem_limit && (size > vm->spare_ramsize)) {
            hax_error("HAX is out of memory quota, because application"
                      " requests another %x bytes\n", size);
            return -EINVAL;
        }
    }

    entry_num = (size - 1) / HAX_RAM_ENTRY_SIZE + 1;

    mem = (struct hax_vcpu_mem *)hax_vmalloc(
            sizeof(struct hax_vcpu_mem) * (entry_num + vm->ram_entry_num), 0);
    if (!mem)
        return -ENOMEM;
    memset(mem, 0,
           sizeof(struct hax_vcpu_mem) * (entry_num + vm->ram_entry_num));
    memcpy_s(mem, sizeof(struct hax_vcpu_mem) * (entry_num + vm->ram_entry_num),
             vm->ram_entry, sizeof(struct hax_vcpu_mem) * vm->ram_entry_num);

    smem = curmem = mem + vm->ram_entry_num;

    leftsize = size;
    while (leftsize > 0) {
        cursize = leftsize > HAX_RAM_ENTRY_SIZE ? HAX_RAM_ENTRY_SIZE : leftsize;
        hax_debug("Memory allocation, gva:%llx, cur_size:%x\n", gva, cursize);

        ret = hax_setup_vcpumem(curmem, gva, cursize, HAX_VCPUMEM_VALIDVA);
        if (ret < 0)
            goto fail;

        hax_debug("Alloc ram %x kva is %p uva %llx\n", cursize, curmem->kva,
                  curmem->uva);

        leftsize -= cursize;
        curmem++;
        gva += cursize;
    }

    if (vm->ram_entry) {
        hax_vfree(vm->ram_entry,
                  sizeof(struct hax_vcpu_mem) * vm->ram_entry_num);
    }

    vm->ram_entry = mem;
    vm->ram_entry_num += entry_num;
    if (!hax_test_bit(VM_STATE_FLAGS_MEM_ALLOC, &vm->flags)) {
        hax_mutex_lock(hax->hax_lock);
        if (hax->mem_limit) {
            hax->mem_quota -= size;
        }
        hax_mutex_unlock(hax->hax_lock);
        hax_test_and_set_bit(VM_STATE_FLAGS_MEM_ALLOC, &vm->flags);
        vm->spare_ramsize = VM_SPARE_RAMSIZE;
        hax_info("!VM_STATE_FLAGS_MEM_ALLOC: spare_ram 0x%llx\n",
                 vm->spare_ramsize);
    } else {
        if (hax->mem_limit) {
            vm->spare_ramsize -= size;
            hax_info("VM_STATE_FLAGS_MEM_ALLOC: spare_ram 0x%llx\n",
                     vm->spare_ramsize);
        }
    }
    hax_debug("Memory allocationg done!\n");
    return 0;

fail:
    curmem = smem;
    for (i = 0; i < entry_num; i++) {
        hax_clear_vcpumem(curmem);
        curmem++;
    }

    hax_vfree(mem,
              sizeof(struct hax_vcpu_mem) * (entry_num + vm->ram_entry_num));
    return -EINVAL;
#endif  // CONFIG_HAX_EPT2
}

int hax_vm_free_all_ram(struct vm_t *vm)
{
    int i;
    uint64_t tsize = 0;
    struct hax_vcpu_mem *mem;
    uint64_t spare = 0;

    mem = vm->ram_entry;
    for (i = 0; i < vm->ram_entry_num; i++) {
        tsize += mem->size;
        hax_clear_vcpumem(mem);
        mem++;
    }
    if (vm->ram_entry) {
        hax_vfree(vm->ram_entry,
                  sizeof(struct hax_vcpu_mem) * vm->ram_entry_num);
        vm->ram_entry_num = 0;
        vm->ram_entry = NULL;
    }

    hax_mutex_lock(hax->hax_lock);
    if (hax->mem_limit) {
        if (hax_test_bit(VM_STATE_FLAGS_MEM_ALLOC, &vm->flags)) {
            spare = VM_SPARE_RAMSIZE - vm->spare_ramsize;
        }
        hax->mem_quota += tsize - spare;
    }
    hax_mutex_unlock(hax->hax_lock);
    return 0;
}

int in_pmem_range(struct hax_vcpu_mem *pmem, uint64_t va)
{
    return (va >= pmem->uva) && (va < pmem->uva + pmem->size);
}

static struct hax_vcpu_mem *get_pmem_range(struct vm_t *vm, uint64_t va)
{
    int i;
    struct hax_vcpu_mem *mem;

    mem = vm->ram_entry;
    for (i = 0; i < vm->ram_entry_num; i++) {
        if (!mem->hinfo)
            continue;
        if (!in_pmem_range(mem, va)) {
            mem++;
            continue;
        }
        return mem;
    }
    return NULL;
}

#ifdef CONFIG_HAX_EPT2
static int handle_set_ram(struct vm_t *vm, uint64_t start_gpa, uint64_t size,
                          uint64_t start_uva, uint32_t flags)
{
    bool unmap = flags & HAX_RAM_INFO_INVALID;
    hax_gpa_space *gpa_space;
    uint64_t start_gfn, npages;
    int ret;
    hax_ept_tree *ept_tree;

    // HAX_RAM_INFO_INVALID indicates that guest physical address range
    // [start_gpa, start_gpa + size) should be unmapped
    if (unmap && ((flags & ~HAX_RAM_INFO_COALESCED) != HAX_RAM_INFO_INVALID || start_uva)) {
        hax_error("%s: Invalid start_uva=0x%llx or flags=0x%x for unmapping\n",
                  __func__, start_uva, flags);
        return -EINVAL;
    }
    if (!unmap && !start_uva) {
        hax_error("%s: Cannot map to an invalid UVA\n", __func__);
        return -EINVAL;
    }
    if (!size) {
        hax_error("%s: size == 0\n", __func__);
        return -EINVAL;
    }

    hax_assert(vm != NULL);
    gpa_space = &vm->gpa_space;
    start_gfn = start_gpa >> PG_ORDER_4K;
    npages = size >> PG_ORDER_4K;

    ret = gpa_space_adjust_prot_bitmap(gpa_space, start_gfn + npages);
    if (ret) {
        hax_error("%s: Failed to resize prot bitmap: ret=%d, start_gfn=0x%llx,"
                  " npages=0x%llx, start_uva=0x%llx, flags=0x%x\n", __func__,
                  ret, start_gfn, npages, start_uva, flags);
        return ret;
    }

    /*
     * Translate the user-space HAX_RAM_INFO_* flags to the internal
     * HAX_MEMSLOT_* flags before handing them to the memslot layer.  ROM
     * (0x01) and INVALID (0x80) happen to share the same bit values, but
     * FAULTISMMIO is 0x10 in the API and 0x20 internally -- AND the API's
     * 0x10 collides with HAX_MEMSLOT_INVALIDUVA (0x10).  Passing the raw API
     * flags here (the historical behaviour) made memslot_set_mapping() read a
     * FAULTISMMIO request as INVALIDUVA: it skipped ramblock_find(), left
     * memslot->block == NULL (-> NULL-deref crash in memslot_dump_list), and
     * never set HAX_MEMSLOT_FAULTISMMIO so ept2 never trapped the page.  The
     * coalesced.c caller already passes internal flags; do the same here.
     * (COALESCED is handled separately below and is not a memslot flag.)
     */
    {
        uint32_t memslot_flags = 0;
        if (flags & HAX_RAM_INFO_ROM)         memslot_flags |= HAX_MEMSLOT_READONLY;
        if (flags & HAX_RAM_INFO_INVALID)     memslot_flags |= HAX_MEMSLOT_INVALID;
        if (flags & HAX_RAM_INFO_FAULTISMMIO) memslot_flags |= HAX_MEMSLOT_FAULTISMMIO;
        ret = memslot_set_mapping(gpa_space, start_gfn, npages, start_uva,
                                  memslot_flags);
    }
    if (ret) {
        hax_error("%s: memslot_set_mapping() failed: ret=%d, start_gfn=0x%llx,"
                  " npages=0x%llx, start_uva=0x%llx, flags=0x%x\n", __func__,
                  ret, start_gfn, npages, start_uva, flags);
        return ret;
    }
    memslot_dump_list(gpa_space);

    if (flags & HAX_RAM_INFO_COALESCED) {
        ret = coalesced_set_mapping(vm, start_gfn, npages);
        if (ret) {
            hax_error("%s: coalesced_set_mapping() failed: ret=%d, start_gfn=0x%llx,"
                " npages=0x%llx\n", __func__, ret, start_gfn, npages);
        }
    }
    ept_tree = &vm->ept_tree;
    if (!hax_test_and_clear_bit(0, (uint64_t *)&ept_tree->invept_pending)) {
        // INVEPT pending flag was set
        hax_info("%s: Invoking INVEPT for VM #%d\n", __func__, vm->vm_id);
        invept(vm, EPT_INVEPT_SINGLE_CONTEXT);
    }
    return 0;
}
#endif  // CONFIG_HAX_EPT2

int hax_vm_set_ram(struct vm_t *vm, struct hax_set_ram_info *info)
{
#ifdef CONFIG_HAX_EPT2
    return handle_set_ram(vm, info->pa_start, info->size, info->va,
                          info->flags);
#else  // !CONFIG_HAX_EPT2
    int num = info->size >> HAX_PAGE_SHIFT;
    uint64_t gpfn = info->pa_start >> HAX_PAGE_SHIFT;
    uint64_t cur_va = info->va;
    bool is_unmap = info->flags & HAX_RAM_INFO_INVALID;
    bool is_readonly = info->flags & HAX_RAM_INFO_ROM;
    uint emt = is_unmap ? EMT_NONE : (is_readonly ? EMT_UC : EMT_WB);
    uint perm = is_unmap ? EPT_TYPE_NONE
                : (is_readonly ? EPT_TYPE_ROM : EPT_TYPE_MEM);
    bool ept_modified = false;

    // HAX_RAM_INFO_INVALID indicates that guest physical address range
    // [pa_start, pa_start + size) should be unmapped
    if (is_unmap && (info->flags != HAX_RAM_INFO_INVALID || info->va)) {
        hax_error("HAX_VM_IOCTL_SET_RAM called with invalid parameter(s):"
                  " flags=0x%x, va=0x%llx\n", info->flags, info->va);
        return -EINVAL;
    }

    while (num > 0) {
        uint64_t hpfn;
        uint64_t hva;
        bool epte_modified;

        if (is_unmap) {
            hpfn = 0;
            hva = 0;
        } else {
            struct hax_vcpu_mem *pmem = get_pmem_range(vm, cur_va);
            if (!pmem) {
                hax_error("Can't find pmem for va %llx", cur_va);
                return -ENOMEM;
            }
            hpfn = get_hpfn_from_pmem(pmem, cur_va);

            if (hpfn <= 0) {
                hax_error("Can't get host address for va %llx", cur_va);
                /*
                 * Shall we revert the already setup one? Assume not since the
                 * QEMU should exit on such situation, although it does not.
                 */
                return -ENOMEM;
            }
#if defined(HAX_PLATFORM_DARWIN)
#ifdef HAX_ARCH_X86_64
            hva = (uint64_t)pmem->kva + (cur_va - pmem->uva);
#else
            hva = (uint64_t)(uint32_t)pmem->kva + (cur_va - pmem->uva);
#endif
#else   // !HAX_PLATFORM_DARWIN
#ifdef HAX_ARCH_X86_64
            hva = (uint64_t)pmem->kva + (cur_va - pmem->uva);
#else
            hva = 0;
#endif
#endif
            cur_va += HAX_PAGE_SIZE;
        }

        if (!hax_core_set_p2m(vm, gpfn, hpfn, hva, info->flags)) {
            return -ENOMEM;
        }
        if (!ept_set_pte(vm, gpfn << HAX_PAGE_SHIFT, hpfn << HAX_PAGE_SHIFT, emt, perm,
                         &epte_modified)) {
            hax_error("ept_set_pte() failed at gpfn 0x%llx hpfn 0x%llx\n", gpfn,
                      hpfn);
            return -ENOMEM;
        }
        ept_modified = ept_modified || epte_modified;

        gpfn++;
        num--;
    }
    if (ept_modified) {
        /* Invalidate EPT cache (see IASDM Vol. 3C 28.3.3.4) */
        hax_info("Calling INVEPT after EPT update (pa_start=0x%llx, size=0x%x,"
                 " flags=0x%x)\n", info->pa_start, info->size, info->flags);
        invept(vm, EPT_INVEPT_SINGLE_CONTEXT);
    }
    return 0;
#endif  // CONFIG_HAX_EPT2
}

#ifdef CONFIG_HAX_EPT2
int hax_vm_set_ram2(struct vm_t *vm, struct hax_set_ram_info2 *info)
{
    return handle_set_ram(vm, info->pa_start, info->size, info->va,
                          info->flags);
}

int hax_vm_protect_ram(struct vm_t *vm, struct hax_protect_ram_info *info)
{
    return gpa_space_protect_range(&vm->gpa_space, info->pa_start, info->size,
                                   info->flags);
}

/* Read-and-clear the EPT dirty bitmap for a guest-physical page range (used by
 * the userland VRAM dirty-page video sync).  Returns the number of dirty pages
 * found, or -EINVAL if the host CPU can't track EPT accessed/dirty flags (the
 * caller then falls back to a full scan).  Re-arms tracking with INVEPT whenever
 * anything was dirty.  Kept here (not in the platform ioctl handler) because
 * struct vm_t / ept_tree are private to the core. */
int hax_vm_query_clear_dirty(struct vm_t *vm, uint64_t base_gfn,
                             uint32_t npages, uint8_t *bitmap)
{
    int found;

    if (ept_cap_ad_supported()) {
        /* Fast path: hardware EPT accessed/dirty flags (Haswell+). */
        found = ept_tree_query_clear_dirty(&vm->ept_tree, base_gfn, npages,
                                           bitmap);
        if (found > 0)
            invept(vm, EPT_INVEPT_SINGLE_CONTEXT);
        return found;
    }

    /* Fallback (no A/D, e.g. Ivy Bridge): access-tracking via PTE invalidation.
     * Lazy-arm on the first call / range change -- arming invalidates the range,
     * so nothing is marked yet; report the whole range dirty once so the caller
     * establishes its baseline with a full scan.  Real per-frame dirty sets
     * follow on subsequent queries. */
    if (!vm->dlog.active || vm->dlog.base_gfn != base_gfn ||
        vm->dlog.npages != npages) {
        int r = hax_vm_dirty_log_arm(vm, base_gfn, npages);
        if (r < 0)
            return r;
        memset(bitmap, 0xFF, (npages + 7) >> 3);
        return (int)npages;
    }

    found = hax_vm_dirty_log_query(vm, base_gfn, npages, bitmap);
    if (found < 0)
        return found;
    /* Re-arm: invalidate the pages accessed since the last query (they now hold
     * PTEs) so their next write re-faults and re-marks.  Pages still invalidated
     * are no-ops, so this only touches what changed. */
    if (ept_tree_invalidate_entries(&vm->ept_tree, base_gfn, npages) > 0)
        invept(vm, EPT_INVEPT_SINGLE_CONTEXT);
    return found;
}

/* --- Write-protect dirty-page logging (EPT-A/D fallback) ---------------------
/* hax_test_and_set_bit() accesses the 64-bit word holding the bit, so the
 * bitmap must be allocated in whole qwords (not just ceil-bytes) or a bit in the
 * last partial qword would touch unallocated memory. */
#define DLOG_ALLOC_BYTES(npages)  ((uint32_t)(((npages) + 63) >> 6) << 3)

void hax_vm_dirty_log_disarm(struct vm_t *vm)
{
    if (vm->dlog.bitmap) {
        hax_vfree(vm->dlog.bitmap, DLOG_ALLOC_BYTES(vm->dlog.npages));
        vm->dlog.bitmap = NULL;
    }
    vm->dlog.active   = 0;
    vm->dlog.npages   = 0;
    vm->dlog.base_gfn = 0;
}

/* Arm dirty-logging over [base_gfn, base_gfn + npages).  Allocates a fresh
 * all-clean bitmap.  Re-arming the exact same range is a cheap no-op; a
 * different range reallocates.  Returns 0, or -errno. */
int hax_vm_dirty_log_arm(struct vm_t *vm, uint64_t base_gfn, uint32_t npages)
{
    uint32_t bytes;

    if (!npages || npages > (1u << 20))
        return -EINVAL;
    if (vm->dlog.active && vm->dlog.base_gfn == base_gfn &&
        vm->dlog.npages == npages)
        return 0;
    hax_vm_dirty_log_disarm(vm);
    bytes = DLOG_ALLOC_BYTES(npages);
    vm->dlog.bitmap = hax_vmalloc(bytes, 0);
    if (!vm->dlog.bitmap)
        return -ENOMEM;
    memset(vm->dlog.bitmap, 0, bytes);
    vm->dlog.base_gfn = base_gfn;
    vm->dlog.npages   = npages;
    vm->dlog.active   = 1;
    /* Invalidate the range's EPT PTEs so the guest's next access to each page
     * re-faults -> the exit_ept_violation hook marks it.  Must INVEPT ourselves:
     * invept_pending is only auto-flushed by handle_set_ram (memory.c). */
    if (ept_tree_invalidate_entries(&vm->ept_tree, base_gfn, npages) > 0)
        invept(vm, EPT_INVEPT_SINGLE_CONTEXT);
    return 0;
}

/* Record a guest page as dirtied.  Called from the EPT-violation handler
 * (stage 2).  No-op when disarmed or out of range. */
void hax_vm_dirty_log_mark(struct vm_t *vm, uint64_t gfn)
{
    uint64_t idx;

    if (!vm->dlog.active || !vm->dlog.bitmap || gfn < vm->dlog.base_gfn)
        return;
    idx = gfn - vm->dlog.base_gfn;
    if (idx >= vm->dlog.npages)
        return;
    hax_test_and_set_bit((int)idx, (uint64_t *)vm->dlog.bitmap);
}

/* Copy the dirty bitmap for [base_gfn, base_gfn + npages) into `out`, then clear
 * it.  Returns the number of dirty pages, or -EINVAL if not armed for this exact
 * range.  Used by QUERY_DIRTY's write-protect branch (stage 3). */
int hax_vm_dirty_log_query(struct vm_t *vm, uint64_t base_gfn, uint32_t npages,
                           uint8_t *out)
{
    uint32_t bytes, i;
    int found = 0;

    if (!vm->dlog.active || !vm->dlog.bitmap)
        return -EINVAL;
    if (base_gfn != vm->dlog.base_gfn || npages != vm->dlog.npages)
        return -EINVAL;
    bytes = (npages + 7) >> 3;                 /* meaningful bytes for the caller */
    memcpy_s(out, bytes, vm->dlog.bitmap, bytes);
    memset(vm->dlog.bitmap, 0, DLOG_ALLOC_BYTES(npages));   /* incl qword padding */
    for (i = 0; i < bytes; i++) {
        uint8_t b = out[i];
        while (b) { found += b & 1; b >>= 1; }
    }
    return found;
}
#endif  // CONFIG_HAX_EPT2

int hax_vcpu_setup_hax_tunnel(struct vcpu_t *cv, struct hax_tunnel_info *info)
{
    int ret = -ENOMEM;

    if (!cv || !info)
        return -EINVAL;

    // The tunnel and iobuf are always set together.
    if (cv->tunnel && cv->iobuf_vcpumem) {
        hax_info("setup hax tunnel request for already setup one\n");
        info->size = HAX_PAGE_SIZE;
        info->va = cv->tunnel_vcpumem->uva;
        info->io_va = cv->iobuf_vcpumem->uva;
        return 0;
    }

    cv->tunnel_vcpumem = hax_vmalloc(sizeof(struct hax_vcpu_mem), 0);
    if (!cv->tunnel_vcpumem)
        goto error;

    cv->iobuf_vcpumem = hax_vmalloc(sizeof(struct hax_vcpu_mem), 0);
    if (!cv->iobuf_vcpumem)
        goto error;

    ret = hax_setup_vcpumem(cv->tunnel_vcpumem, 0, HAX_PAGE_SIZE, 0);
    if (ret < 0)
        goto error;

    ret = hax_setup_vcpumem(cv->iobuf_vcpumem, 0, IOS_MAX_BUFFER, 0);
    if (ret < 0)
        goto error;

    info->va = cv->tunnel_vcpumem->uva;
    info->io_va = cv->iobuf_vcpumem->uva;
    info->size = HAX_PAGE_SIZE;
    set_vcpu_tunnel(cv, (struct hax_tunnel *)cv->tunnel_vcpumem->kva,
                    (uint8_t *)cv->iobuf_vcpumem->kva);
    return 0;
error:
    if (cv->tunnel_vcpumem) {
        if (cv->tunnel_vcpumem->uva) {
            hax_clear_vcpumem(cv->tunnel_vcpumem);
        }
        hax_vfree(cv->tunnel_vcpumem, sizeof(struct hax_vcpu_mem));
        cv->tunnel_vcpumem = NULL;
    }
    if (cv->iobuf_vcpumem) {
        if (cv->iobuf_vcpumem->uva) {
            hax_clear_vcpumem(cv->iobuf_vcpumem);
        }
        hax_vfree(cv->iobuf_vcpumem, sizeof(struct hax_vcpu_mem));
        cv->iobuf_vcpumem = NULL;
    }
    return ret;
}

int hax_vcpu_destroy_hax_tunnel(struct vcpu_t *cv)
{
    if (!cv)
        return -EINVAL;
    if (!cv->tunnel_vcpumem && !cv->iobuf_vcpumem)
        return 0;
    set_vcpu_tunnel(cv, NULL, NULL);

    if (cv->tunnel_vcpumem) {
        hax_assert(cv->tunnel_vcpumem->uva);
        hax_clear_vcpumem(cv->tunnel_vcpumem);
        hax_vfree(cv->tunnel_vcpumem, sizeof(struct hax_vcpu_mem));
        cv->tunnel_vcpumem = NULL;
    }
    if (cv->iobuf_vcpumem) {
        hax_assert(cv->iobuf_vcpumem->uva);
        hax_clear_vcpumem(cv->iobuf_vcpumem);
        hax_vfree(cv->iobuf_vcpumem, sizeof(struct hax_vcpu_mem));
        cv->iobuf_vcpumem = NULL;
    }
    return 0;
}
