; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc --mtriple=loongarch64 -mattr=+d --code-model=small < %s | \
; RUN:    FileCheck --check-prefix=SMALL %s
; RUN: llc --mtriple=loongarch64 -mattr=+d --code-model=medium < %s | \
; RUN:    FileCheck --check-prefix=MEDIUM %s
; RUN: llc --mtriple=loongarch64 -mattr=+d --code-model=large < %s | \
; RUN:    FileCheck --check-prefix=LARGE %s

declare void @llvm.memset.p0.i64(ptr, i8, i64, i1)
declare i32 @callee(i32)

define i32 @call_globaladdress(i32 %a) nounwind {
; SMALL-LABEL: call_globaladdress:
; SMALL:       # %bb.0:
; SMALL-NEXT:    addi.d $sp, $sp, -16
; SMALL-NEXT:    st.d $ra, $sp, 8 # 8-byte Folded Spill
; SMALL-NEXT:    bl callee
; SMALL-NEXT:    ld.d $ra, $sp, 8 # 8-byte Folded Reload
; SMALL-NEXT:    addi.d $sp, $sp, 16
; SMALL-NEXT:    ret
;
; MEDIUM-LABEL: call_globaladdress:
; MEDIUM:       # %bb.0:
; MEDIUM-NEXT:    addi.d $sp, $sp, -16
; MEDIUM-NEXT:    st.d $ra, $sp, 8 # 8-byte Folded Spill
; MEDIUM-NEXT:    pcaddu18i $ra, %call36(callee)
; MEDIUM-NEXT:    jirl $ra, $ra, 0
; MEDIUM-NEXT:    ld.d $ra, $sp, 8 # 8-byte Folded Reload
; MEDIUM-NEXT:    addi.d $sp, $sp, 16
; MEDIUM-NEXT:    ret
;
; LARGE-LABEL: call_globaladdress:
; LARGE:       # %bb.0:
; LARGE-NEXT:    addi.d $sp, $sp, -16
; LARGE-NEXT:    st.d $ra, $sp, 8 # 8-byte Folded Spill
; LARGE-NEXT:    pcalau12i $a1, %got_pc_hi20(callee)
; LARGE-NEXT:    addi.d $ra, $zero, %got_pc_lo12(callee)
; LARGE-NEXT:    lu32i.d $ra, %got64_pc_lo20(callee)
; LARGE-NEXT:    lu52i.d $ra, $ra, %got64_pc_hi12(callee)
; LARGE-NEXT:    ldx.d $ra, $ra, $a1
; LARGE-NEXT:    jirl $ra, $ra, 0
; LARGE-NEXT:    ld.d $ra, $sp, 8 # 8-byte Folded Reload
; LARGE-NEXT:    addi.d $sp, $sp, 16
; LARGE-NEXT:    ret
  %1 = call i32 @callee(i32 %a)
  ret i32 %1
}

define void @call_external_sym(ptr %dst) {
; SMALL-LABEL: call_external_sym:
; SMALL:       # %bb.0: # %entry
; SMALL-NEXT:    addi.d $sp, $sp, -16
; SMALL-NEXT:    .cfi_def_cfa_offset 16
; SMALL-NEXT:    st.d $ra, $sp, 8 # 8-byte Folded Spill
; SMALL-NEXT:    .cfi_offset 1, -8
; SMALL-NEXT:    ori $a2, $zero, 1000
; SMALL-NEXT:    move $a1, $zero
; SMALL-NEXT:    bl memset
; SMALL-NEXT:    ld.d $ra, $sp, 8 # 8-byte Folded Reload
; SMALL-NEXT:    addi.d $sp, $sp, 16
; SMALL-NEXT:    ret
;
; MEDIUM-LABEL: call_external_sym:
; MEDIUM:       # %bb.0: # %entry
; MEDIUM-NEXT:    addi.d $sp, $sp, -16
; MEDIUM-NEXT:    .cfi_def_cfa_offset 16
; MEDIUM-NEXT:    st.d $ra, $sp, 8 # 8-byte Folded Spill
; MEDIUM-NEXT:    .cfi_offset 1, -8
; MEDIUM-NEXT:    ori $a2, $zero, 1000
; MEDIUM-NEXT:    move $a1, $zero
; MEDIUM-NEXT:    pcaddu18i $ra, %call36(memset)
; MEDIUM-NEXT:    jirl $ra, $ra, 0
; MEDIUM-NEXT:    ld.d $ra, $sp, 8 # 8-byte Folded Reload
; MEDIUM-NEXT:    addi.d $sp, $sp, 16
; MEDIUM-NEXT:    ret
;
; LARGE-LABEL: call_external_sym:
; LARGE:       # %bb.0: # %entry
; LARGE-NEXT:    addi.d $sp, $sp, -16
; LARGE-NEXT:    .cfi_def_cfa_offset 16
; LARGE-NEXT:    st.d $ra, $sp, 8 # 8-byte Folded Spill
; LARGE-NEXT:    .cfi_offset 1, -8
; LARGE-NEXT:    ori $a2, $zero, 1000
; LARGE-NEXT:    move $a1, $zero
; LARGE-NEXT:    pcalau12i $a3, %got_pc_hi20(memset)
; LARGE-NEXT:    addi.d $ra, $zero, %got_pc_lo12(memset)
; LARGE-NEXT:    lu32i.d $ra, %got64_pc_lo20(memset)
; LARGE-NEXT:    lu52i.d $ra, $ra, %got64_pc_hi12(memset)
; LARGE-NEXT:    ldx.d $ra, $ra, $a3
; LARGE-NEXT:    jirl $ra, $ra, 0
; LARGE-NEXT:    ld.d $ra, $sp, 8 # 8-byte Folded Reload
; LARGE-NEXT:    addi.d $sp, $sp, 16
; LARGE-NEXT:    ret
entry:
  call void @llvm.memset.p0.i64(ptr %dst, i8 0, i64 1000, i1 false)
  ret void
}

;; Tail call with different codemodel.
declare i32 @callee_tail(i32 %i)
define i32 @caller_tail(i32 %i) nounwind {
; SMALL-LABEL: caller_tail:
; SMALL:       # %bb.0: # %entry
; SMALL-NEXT:    b callee_tail
;
; MEDIUM-LABEL: caller_tail:
; MEDIUM:       # %bb.0: # %entry
; MEDIUM-NEXT:    pcaddu18i $t8, %call36(callee_tail)
; MEDIUM-NEXT:    jr $t8
;
; LARGE-LABEL: caller_tail:
; LARGE:       # %bb.0: # %entry
; LARGE-NEXT:    pcalau12i $a1, %got_pc_hi20(callee_tail)
; LARGE-NEXT:    addi.d $a2, $zero, %got_pc_lo12(callee_tail)
; LARGE-NEXT:    lu32i.d $a2, %got64_pc_lo20(callee_tail)
; LARGE-NEXT:    lu52i.d $a2, $a2, %got64_pc_hi12(callee_tail)
; LARGE-NEXT:    ldx.d $a1, $a2, $a1
; LARGE-NEXT:    jr $a1
entry:
  %r = tail call i32 @callee_tail(i32 %i)
  ret i32 %r
}
