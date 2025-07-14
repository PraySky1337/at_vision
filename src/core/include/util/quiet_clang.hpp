#pragma once
#ifdef __clang__
# define BEGIN_CLANG_DIAGNOSTIC_QUIET                                               \
     _Pragma("clang diagnostic push") _Pragma("clang diagnostic ignored \"-Wall\"") \
         _Pragma("clang diagnostic ignored \"-Wextra\"")                            \
             _Pragma("clang diagnostic ignored \"-Wpedantic\"")

# define END_CLANG_DIAGNOSTIC_QUIET _Pragma("clang diagnostic pop")
#else
# define BEGIN_CLANG_DIAGNOSTIC_QUIET
# define END_CLANG_DIAGNOSTIC_QUIET
#endif