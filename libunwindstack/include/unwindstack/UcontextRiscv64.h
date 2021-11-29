/*
 * Copyright (C) 2016 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _LIBUNWINDSTACK_UCONTEXT_RISCV64_H
#define _LIBUNWINDSTACK_UCONTEXT_RISCV64_H

#include <stdint.h>

#include <unwindstack/MachineRiscv64.h>

namespace unwindstack {

struct __riscv_mc_f_ext_state {
  unsigned int __f[32];
  unsigned int __fcsr;
};

struct __riscv_mc_d_ext_state {
  unsigned long long int __f[32];
  unsigned int __fcsr;
};

struct __riscv_mc_q_ext_state {
  unsigned long long int __f[64] __attribute__ ((__aligned__ (16)));
  unsigned int __fcsr;
  /* Reserved for expansion of sigcontext structure.  Currently zeroed
     upon signal, and must be zero upon sigreturn.  */
  unsigned int __glibc_reserved[3];
};

union __riscv_mc_fp_state {
  struct __riscv_mc_f_ext_state __f;
  struct __riscv_mc_d_ext_state __d;
  struct __riscv_mc_q_ext_state __q;
};

struct riscv64_stack_t {
  uint64_t ss_sp;    // void __user*
  int32_t ss_flags;  // int
  uint64_t ss_size;  // size_t
};

struct riscv64_sigset_t {
  uint64_t sig;  // unsigned long
};

struct riscv64_mcontext_t {
  uint64_t regs[RISCV64_REG_MAX];  // __u64
  union  __riscv_mc_fp_state __fpregs;
};

struct riscv64_ucontext_t {
  uint64_t uc_flags;  // unsigned long
  uint64_t uc_link;   // struct ucontext*
  riscv64_stack_t uc_stack;
  riscv64_sigset_t uc_sigmask;
  // The kernel adds extra padding after uc_sigmask to match glibc sigset_t on RISCV64.
  char __glibc_reserved[1024 / 8 - sizeof (sigset_t)];
  // The full structure requires 16 byte alignment, but our partial structure
  // doesn't, so force the alignment.
  riscv64_mcontext_t uc_mcontext __attribute__((aligned(16)));
};

}  // namespace unwindstack

#endif  // _LIBUNWINDSTACK_UCONTEXT_RISCV64_H
