#ifndef _ROBOTLIB_DYNAMIC_MEMORY_HOOKS_HPP_
#define _ROBOTLIB_DYNAMIC_MEMORY_HOOKS_HPP_

// Implementation of hook functions to check dynamic memory allocation for Eigen and non-Eigen structures.

// This implementation has some limitations:
// * __builtin_return_address is only avaiable for gcc compiler.
// * __libc_malloc only works with glibc.

extern "C" void *__libc_malloc(size_t size);

void* malloc (size_t size);

// Hook functions
void* malloc_hook (size_t size, void *caller);

// Utility functions
void activate_hooks();
void deactivate_hooks();
void reset_variables_checking_use_of_dynamic_memory();

// Variables to activate/deactivate hook functions
bool malloc_hook_active { false };

// Variables to check dynamic memory allocation
bool malloc_is_used { false };

void* malloc (size_t size)
{
  void *caller = __builtin_return_address(0);
  if (malloc_hook_active)
    return malloc_hook(size, caller);
  return __libc_malloc(size);
}

void* malloc_hook (size_t size, void *caller)
{
  void *result;

  // deactivate hooks for logging
  malloc_hook_active = false;

  result = malloc(size);

  // do logging
  malloc_is_used = true;

  // reactivate hooks
  malloc_hook_active = true;

  return result;
}

void activate_hooks()
{
    malloc_hook_active = true;
}

void deactivate_hooks()
{
    malloc_hook_active = false;
}

void reset_variables_checking_use_of_dynamic_memory()
{
    malloc_is_used = false;
}

void is_dynamic_memory_used(const bool malloc_value)
{
  EXPECT_EQ(malloc_is_used, malloc_value);
}

#endif // _ROBOTLIB_DYNAMIC_MEMORY_HOOKS_HPP_