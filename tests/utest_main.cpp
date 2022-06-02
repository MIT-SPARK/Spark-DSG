#include <glog/logging.h>
#include <gtest/gtest.h>

auto main(int argc, char **argv) -> int {
  ::testing::InitGoogleTest(&argc, argv);

  FLAGS_minloglevel = 3;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);

  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
