default rel                         ; symbols are addressed relative to rip (position-independent executable)

global c_SphereContactHandrolled

section .text

; u32 c_SphereContactHandrolled(struct c_Manifold *out, struct ds_ConvexCache *cache, const struct c_Shape *s0,
;                               const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
;
; System V x86-64: out rdi, cache rsi, s0 rdx, t0 rcx, s1 r8, t1 r9; the count returns in eax.
c_SphereContactHandrolled:
    xor     eax, eax
    ret

section .note.GNU-stack noalloc noexec nowrite progbits     ; tells the linker the stack need not be executable
