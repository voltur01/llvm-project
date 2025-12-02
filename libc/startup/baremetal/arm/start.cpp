//===-- Implementation of crt for arm -------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "hdr/stdint_proxy.h"
#include "src/__support/macros/config.h"
#include "src/stdlib/atexit.h"
#include "src/stdlib/exit.h"
#include "src/string/memcpy.h"
#include "src/string/memset.h"
#include "startup/baremetal/fini.h"
#include "startup/baremetal/init.h"
#include "semihost.h"

#include <arm_acle.h> // For __arm_wsr
#include <stddef.h>
#include <time.h>

extern "C" {
int main(int argc, char **argv);
void _start();

// Semihosting library initialisation. Required for printf, etc.
extern "C" void _platform_init(void);

// These symbols are provided by the linker. The exact names are not defined by
// a standard.
extern uintptr_t __stack;
extern uintptr_t __data_source[];
extern uintptr_t __data_start[];
extern uintptr_t __data_size[];
extern uintptr_t __bss_start[];
extern uintptr_t __bss_size[];
} // extern "C"

namespace {
#if __ARM_ARCH_PROFILE == 'M'
// Based on
// https://developer.arm.com/documentation/107565/0101/Use-case-examples/Generic-Information/What-is-inside-a-program-image-/Vector-table
void NMI_Handler() {}
void HardFault_Handler() { LIBC_NAMESPACE::exit(1); }
void MemManage_Handler() { LIBC_NAMESPACE::exit(1); }
void BusFault_Handler() { LIBC_NAMESPACE::exit(1); }
void UsageFault_Handler() { LIBC_NAMESPACE::exit(1); }
void SVC_Handler() {}
void DebugMon_Handler() {}
void PendSV_Handler() {}
void SysTick_Handler() {}

// Architecturally the bottom 7 bits of VTOR are zero, meaning the vector table
// has to be 128-byte aligned, however an implementation can require more bits
// to be zero and Cortex-M23 can require up to 10, so 1024-byte align the vector
// table.
using HandlerType = void (*)(void);
[[gnu::section(".vectors"), gnu::aligned(1024), gnu::used]]
const HandlerType vector_table[] = {
    reinterpret_cast<HandlerType>(&__stack), // SP
    _start,                                  // Reset
    NMI_Handler,                             // NMI Handler
    HardFault_Handler,                       // Hard Fault Handler
    MemManage_Handler,                       // MPU Fault Handler
    BusFault_Handler,                        // Bus Fault Handler
    UsageFault_Handler,                      // Usage Fault Handler
    0,                                       // Reserved
    0,                                       // Reserved
    0,                                       // Reserved
    0,                                       // Reserved
    SVC_Handler,                             // SVC Handler
    DebugMon_Handler,                        // Debug Monitor Handler
    0,                                       // Reserved
    PendSV_Handler,                          // PendSV Handler
    SysTick_Handler,                         // SysTick Handler
                                             // Unused
};
#else
// Based on
// https://developer.arm.com/documentation/den0013/0400/Boot-Code/Booting-a-bare-metal-system
void Reset_Handler() { LIBC_NAMESPACE::exit(1); }
void Undefined_Handler() { LIBC_NAMESPACE::exit(1); }
void SWI_Handler() { LIBC_NAMESPACE::exit(1); }
void PrefetchAbort_Handler() { LIBC_NAMESPACE::exit(1); }
void DataAbort_Handler() { LIBC_NAMESPACE::exit(1); }
void IRQ_Handler() { LIBC_NAMESPACE::exit(1); }
void FIQ_Handler() { LIBC_NAMESPACE::exit(1); }

// The AArch32 exception vector table has 8 entries, each of which is 4
// bytes long, and contains code. The whole table must be 32-byte aligned.
// The table may also be relocated, so we make it position-independent by
// having a table of handler addresses and loading the address to pc.
[[gnu::section(".vectors"), gnu::aligned(32), gnu::used, gnu::naked,
  gnu::target("arm")]]
void vector_table() {
  asm("LDR pc, [pc, #24]");
  asm("LDR pc, [pc, #24]");
  asm("LDR pc, [pc, #24]");
  asm("LDR pc, [pc, #24]");
  asm("LDR pc, [pc, #24]");
  asm("LDR pc, [pc, #24]");
  asm("LDR pc, [pc, #24]");
  asm("LDR pc, [pc, #24]");
  asm(".word %c0" : : "X"(Reset_Handler));
  asm(".word %c0" : : "X"(Undefined_Handler));
  asm(".word %c0" : : "X"(SWI_Handler));
  asm(".word %c0" : : "X"(PrefetchAbort_Handler));
  asm(".word %c0" : : "X"(DataAbort_Handler));
  asm(".word %c0" : : "X"(0));
  asm(".word %c0" : : "X"(IRQ_Handler));
  asm(".word %c0" : : "X"(FIQ_Handler));
}
#endif
} // namespace

namespace LIBC_NAMESPACE_DECL {
[[noreturn]] void do_start() {
  // FIXME: set up the QEMU test environment

#if __ARM_ARCH_PROFILE == 'A' || __ARM_ARCH_PROFILE == 'R'
  // Set up registers to be used in exception handling
  // Copy the current sp value to each of the banked copies of sp.
  __arm_wsr("CPSR_c", 0x11); // FIQ
  asm volatile("mov sp, %0" : : "r"(__builtin_frame_address(0)));
  __arm_wsr("CPSR_c", 0x12); // IRQ
  asm volatile("mov sp, %0" : : "r"(__builtin_frame_address(0)));
  __arm_wsr("CPSR_c", 0x17); // ABT
  asm volatile("mov sp, %0" : : "r"(__builtin_frame_address(0)));
  __arm_wsr("CPSR_c", 0x1B); // UND
  asm volatile("mov sp, %0" : : "r"(__builtin_frame_address(0)));
  __arm_wsr("CPSR_c", 0x1F); // SYS
  asm volatile("mov sp, %0" : : "r"(__builtin_frame_address(0)));
  __arm_wsr("CPSR_c", 0x13); // SVC
#endif

#if __ARM_ARCH_PROFILE == 'M' &&                                               \
    (defined(__ARM_FP) || defined(__ARM_FEATURE_MVE))
  // Enable FPU and MVE. They can't be enabled independently: the two are
  // governed by the same bits in CPACR.
  // Based on
  // https://developer.arm.com/documentation/dui0646/c/Cortex-M7-Peripherals/Floating-Point-Unit/Enabling-the-FPU
  // Set CPACR cp10 and cp11.
  auto cpacr = reinterpret_cast<volatile uint32_t *const>(0xE000ED88);
  *cpacr |= (0xF << 20);
  __dsb(0xF);
  __isb(0xF);
#if defined(__ARM_FEATURE_MVE)
  // Initialize low-overhead-loop tail predication to its neutral state
  uint32_t fpscr;
  __asm__ __volatile__("vmrs %0, FPSCR" : "=r"(fpscr) : :);
  fpscr |= (0x4 << 16);
  __asm__ __volatile__("vmsr FPSCR, %0" : : "r"(fpscr) :);
#endif
#elif (__ARM_ARCH_PROFILE == 'A' || __ARM_ARCH_PROFILE == 'R') &&              \
    defined(__ARM_FP)
  // Enable FPU.
  // Based on
  // https://developer.arm.com/documentation/dui0472/m/Compiler-Coding-Practices/Enabling-NEON-and-FPU-for-bare-metal
  // Set CPACR cp10 and cp11.
  uint32_t cpacr = __arm_rsr("p15:0:c1:c0:2");
  cpacr |= (0xF << 20);
  __arm_wsr("p15:0:c1:c0:2", cpacr);
  __isb(0xF);
  // Set FPEXC.EN
  uint32_t fpexc;
  __asm__ __volatile__("vmrs %0, FPEXC" : "=r"(fpexc) : :);
  fpexc |= (0x1 << 30);
  __asm__ __volatile__("vmsr FPEXC, %0" : : "r"(fpexc) :);
#endif

  // Perform the equivalent of scatterloading
  LIBC_NAMESPACE::memcpy(__data_start, __data_source,
                         reinterpret_cast<uintptr_t>(__data_size));
  LIBC_NAMESPACE::memset(__bss_start, '\0',
                         reinterpret_cast<uintptr_t>(__bss_size));
  __libc_init_array();

  _platform_init();
  LIBC_NAMESPACE::atexit(&__libc_fini_array);
  LIBC_NAMESPACE::exit(main(0, 0));
}
} // namespace LIBC_NAMESPACE_DECL

extern "C" {
#ifdef __ARM_ARCH_ISA_ARM
// If ARM state is supported, it must be used (instead of Thumb)
[[gnu::naked, gnu::target("arm")]]
#endif
void _start() {
  asm volatile("mov sp, %0" : : "r"(&__stack));
  asm volatile("bl %0" : : "X"(LIBC_NAMESPACE::do_start));
}
} // extern "C"

// IO and time retargetting to sehimosting for libc testing

namespace {

void stdio_open(struct __llvm_libc_stdio_cookie *cookie, size_t mode) {
  size_t args[3];
  args[0] = reinterpret_cast<size_t>(":tt");
  args[1] = mode;
  args[2] = static_cast<size_t>(3); /* name length */
  cookie->handle = semihosting_call(SYS_OPEN, args);
}
} // namespace

extern "C" {

void __llvm_libc_exit(int status) {

#if defined(__ARM_64BIT_STATE) && __ARM_64BIT_STATE
  size_t block[2];
  block[0] = ADP_Stopped_ApplicationExit;
  block[1] = status;
  semihosting_call(SYS_EXIT, block);
#else
  if (status == 0) {
    semihosting_call(
        SYS_EXIT, reinterpret_cast<const void *>(ADP_Stopped_ApplicationExit));
  } else {
    semihosting_call(SYS_EXIT, reinterpret_cast<const void *>(
                                   ADP_Stopped_RunTimeErrorUnknown));
  }
#endif

  __builtin_unreachable(); /* semihosting call doesn't return */
}

ssize_t __llvm_libc_stdio_read(struct __llvm_libc_stdio_cookie *cookie,
                               const char *buf, size_t size) {
  size_t args[4];
  args[0] = static_cast<size_t>(cookie->handle);
  args[1] = reinterpret_cast<size_t>(buf);
  args[2] = size;
  args[3] = 0;
  ssize_t retval = semihosting_call(SYS_READ, args);
  if (retval >= 0)
    retval = size - retval;
  return retval;
}

ssize_t __llvm_libc_stdio_write(struct __llvm_libc_stdio_cookie *cookie,
                                const char *buf, size_t size) {
  size_t args[4];
  args[0] = static_cast<size_t>(cookie->handle);
  args[1] = reinterpret_cast<size_t>(buf);
  args[2] = size;
  ssize_t retval = semihosting_call(SYS_WRITE, args);
  if (retval >= 0)
    retval = size - retval;
  return retval;
}

struct __llvm_libc_stdio_cookie __llvm_libc_stdin_cookie;
struct __llvm_libc_stdio_cookie __llvm_libc_stdout_cookie;
struct __llvm_libc_stdio_cookie __llvm_libc_stderr_cookie;

bool __llvm_libc_timespec_get_active(struct timespec *ts) {
  long retval = semihosting_call(SYS_CLOCK, 0);
  if (retval == -1)
    return false;

  // Semihosting uses centiseconds
  ts->tv_sec = (retval / 100);
  ts->tv_nsec = (retval % 100) * (1'000'000'000 / 100);
  return true;
}

bool __llvm_libc_timespec_get_utc(struct timespec *ts) {
  long retval = semihosting_call(SYS_TIME, 0);

  // Semihosting uses seconds
  ts->tv_sec = retval;
  ts->tv_nsec = 0;
  return true;
}

// Entry point
void _platform_init(void) {
  stdio_open(&__llvm_libc_stdin_cookie, OPENMODE_R);
  stdio_open(&__llvm_libc_stdout_cookie, OPENMODE_W);
  stdio_open(&__llvm_libc_stderr_cookie, OPENMODE_W);
}
} // extern "C"

