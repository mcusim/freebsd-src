/* #define	__memcpy_aarch64_simd	memcpy */
/* #define	__memmove_aarch64_simd	memmove */
#define __memmove_aarch64 memmove
#define __memcpy_aarch64 memcpy
#include "aarch64/memcpy.S"
