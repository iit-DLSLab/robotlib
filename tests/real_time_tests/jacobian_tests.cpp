// #define EIGEN_RUNTIME_NO_MALLOC

#include "robot_base.hpp"
// TODO: Change this include
#include "../src/robots/dummy_quadruped.cpp"
#include <gtest/gtest.h>
/* Prototypes for __malloc_hook, __free_hook */
#include <malloc.h>


// Hook functions for checking dynamic memory allocation for Eigen and non-Eigen structures
// ***********************************************************

bool is_malloc { false };
bool is_free_malloc { false };

/* Prototypes for our hooks.  */
static void init_hook_functions (void);
static void reset_hook_functions (void);
static void *malloc_hook (size_t, const void *);
static void free_hook (void*, const void *);

/* Variables to save original hooks */
static void *(*old_malloc_hook)(size_t, const void *);
static void (*old_free_hook)(void*, const void *);

static void
init_hook_functions (void)
{
  old_malloc_hook = __malloc_hook;
  old_free_hook = __free_hook;
  __malloc_hook = malloc_hook;
  __free_hook = free_hook;
}

static void
reset_hook_functions (void)
{
  __malloc_hook = old_malloc_hook;
  __free_hook = old_free_hook;
  is_malloc = false;
  is_free_malloc = false;
}
static void *
malloc_hook (size_t size, const void *caller)
{
  void *result;
  /* Restore all old hooks */
  __malloc_hook = old_malloc_hook;
  __free_hook = old_free_hook;
  /* Call recursively */
  result = malloc (size);
  /* Save underlying hooks */
  old_malloc_hook = __malloc_hook;
  old_free_hook = __free_hook;
  /* printf might call malloc, so protect it too. */
  is_malloc = true;
//   printf ("malloc (%u) returns %p\n", (unsigned int) size, result);
  /* Restore our own hooks */
  __malloc_hook = malloc_hook;
  __free_hook = free_hook;
  return result;
}

static void
free_hook (void *ptr, const void *caller)
{
  /* Restore all old hooks */
  __malloc_hook = old_malloc_hook;
  __free_hook = old_free_hook;
  /* Call recursively */
  free (ptr);
  /* Save underlying hooks */
  old_malloc_hook = __malloc_hook;
  old_free_hook = __free_hook;
  /* printf might call free, so protect it too. */
  is_free_malloc = true;
//   printf ("freed pointer %p\n", ptr);
  /* Restore our own hooks */
  __malloc_hook = malloc_hook;
  __free_hook = free_hook;
}
// *****************************************************

TEST(RealTimeTest, eigen_multiplication)
{
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>A,B,C;
    A.setOnes(500,500);
    B.setOnes(500,500);
    C.setOnes(500,500);
   
    init_hook_functions();
    
    // Eigen::internal::set_is_malloc_allowed(false);
    // C = A*B;
    // C.noalias() = A * B;
    (C.noalias()=A.lazyProduct(B));
    // Eigen::internal::set_is_malloc_allowed(true);
    
    EXPECT_EQ(is_malloc, false);
    EXPECT_EQ(is_free_malloc, false);
    reset_hook_functions();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}