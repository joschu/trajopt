#include <gtest/gtest.h>

int
main(int argc, char** argv)
{
#ifdef HAVE_GUROBI
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
#else
  printf("don't have gurobi: not running sco tests\n");
#endif
}
