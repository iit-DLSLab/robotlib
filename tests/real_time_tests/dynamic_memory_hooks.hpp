/*!
 * @brief Implementation of hook functions to check dynamic memory allocation/deallocation/reallocation for Eigen and non-Eigen structures.
 *         This implementation has some limitations:
 *           * __builtin_return_address is only avaiable for gcc compiler.
 *           * __libc_malloc only works with glibc.
 *         In order to test dynamic memory allocation/deallocation/reallocation you have to:
 *           * reset de variable checking dynamic memory allocation/deallocation/reallocation 
 *           * activate the hook
 *           * add the code you want to test
 *           * deactivate the hook
 *           * test if any of the allocation/deallocation/reallocation operation has happened
 * @example
 *          [...]
 *          reset_variables_checking_use_of_dynamic_memory(); 
 *          activate_hooks();
 *          <code you want to test>
 *          deactivate_hooks();
 *          is_dynamic_memory_used(true/false, true/false, true/false, true/false) # These boolean values are related to what you want to test.
*/

#ifndef _ROBOTLIB_DYNAMIC_MEMORY_HOOKS_HPP_
#define _ROBOTLIB_DYNAMIC_MEMORY_HOOKS_HPP_
#include <gtest/gtest.h>

// ** GNU C library functions **
extern "C" void *__libc_malloc(size_t size);
extern "C" void *__libc_calloc(size_t size_1, size_t size_2);
extern "C" void *__libc_realloc(void* ptr, size_t size);
extern "C" void __libc_free(void* ptr);

// ** Redefinition of malloc, calloc, realloc, free functions ** 
inline void* malloc (size_t size);
inline void* calloc (size_t size_1, size_t size_2);
inline void* realloc (void* ptr, size_t size);
inline void free (void* ptr);

// ** Hook functions **
inline void* malloc_hook (size_t size);
inline void* calloc_hook (size_t size_1, size_t size_2);
inline void* realloc_hook (void* ptr, size_t size);
inline void free_hook (void* ptr);

// ** Utility functions **
inline void activate_hooks();
inline void deactivate_hooks();
inline void reset_variables_checking_use_of_dynamic_memory();
inline void is_dynamic_memory_used(const bool malloc_value, const bool calloc_value, const bool realloc_value, const bool free_value);

// ** Variables to activate/deactivate hook functions **
inline bool malloc_hook_active { false };
inline bool calloc_hook_active { false };
inline bool realloc_hook_active { false };
inline bool free_hook_active { false };

// ** Variables to check dynamic memory allocation/deallocation/reallocation **
inline bool malloc_is_used { false };
inline bool calloc_is_used { false };
inline bool realloc_is_used { false };
inline bool free_is_used { false };

// ** Implementation **
void* malloc (size_t size)
{
  // void *caller = __builtin_return_address(0);
  if (malloc_hook_active)
    return malloc_hook(size);
  return __libc_malloc(size);
}
void* calloc (size_t size_1, size_t size_2)
{
  // void *caller = __builtin_return_address(0);
  if (calloc_hook_active)
    return calloc_hook(size_1, size_2);
  return __libc_calloc(size_1, size_2);
}
void* realloc (void* ptr, size_t size)
{
  // void *caller = __builtin_return_address(0);
  if (realloc_hook_active)
    return realloc_hook(ptr, size);
  return __libc_realloc(ptr, size);
}
void free (void* ptr)
{
  // void *caller = __builtin_return_address(0);
  if (free_hook_active)
    return free_hook(ptr);
  return __libc_free(ptr);
}

void* malloc_hook (size_t size)
{
  void *result;

  // deactivate hooks for logging
  malloc_hook_active = false;

  result = malloc(size);

  // do logging
  malloc_is_used = true;

  // Debug print
  // printf("malloc\n");
  
  // reactivate hooks
  malloc_hook_active = true;

  return result;
}
void* calloc_hook (size_t size_1, size_t size_2)
{
  void *result;

  // deactivate hooks for logging
  calloc_hook_active = false;

  result = calloc(size_1, size_2);

  // do logging
  calloc_is_used = true;

  // reactivate hooks
  calloc_hook_active = true;

  return result;
}
void* realloc_hook (void* ptr, size_t size)
{
  void *result;

  // deactivate hooks for logging
  realloc_hook_active = false;

  result = realloc(ptr, size);

  // do logging
  realloc_is_used = true;

  // reactivate hooks
  realloc_hook_active = true;

  return result;
}
void free_hook (void* ptr)
{
  // deactivate hooks for logging
  free_hook_active = false;

  free(ptr);

  // do logging
  free_is_used = true;

  // Debug print
  // printf("free\n");

  // reactivate hooks
  free_hook_active = true;

}

void activate_hooks()
{
    malloc_hook_active = true;
    calloc_hook_active = true;
    realloc_hook_active = true;
    free_hook_active = true;
}
void deactivate_hooks()
{
    malloc_hook_active = false;
    calloc_hook_active = false;
    realloc_hook_active = false;
    free_hook_active = false;
}
void reset_variables_checking_use_of_dynamic_memory()
{
    malloc_is_used = false;
    calloc_is_used = false;
    realloc_is_used = false;
    free_is_used = false;
}
void is_dynamic_memory_used(const bool malloc_value, const bool calloc_value, const bool realloc_value, const bool free_value)
{
  EXPECT_EQ(malloc_is_used, malloc_value);
  EXPECT_EQ(calloc_is_used, calloc_value);
  EXPECT_EQ(realloc_is_used, realloc_value);
  EXPECT_EQ(free_is_used, free_value);
}

#endif // _ROBOTLIB_DYNAMIC_MEMORY_HOOKS_HPP_