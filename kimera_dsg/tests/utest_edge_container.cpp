#include <gtest/gtest.h>
#include <kimera_dsg/edge_container.h>

namespace kimera {

TEST(EdgeContainerTests, NewEdgesCorrect) {
  EdgeContainer container;
  container.insert(0, 1, nullptr);
  container.insert(1, 2, nullptr);
  container.insert(0, 3, nullptr);

  {
    std::vector<EdgeKey> expected{{0, 1}, {0, 3}, {1, 2}};

    std::vector<EdgeKey> result;
    container.getNew(result, false);
    EXPECT_EQ(expected, result);
  }

  {
    std::vector<EdgeKey> expected{{0, 1}, {0, 3}, {1, 2}};

    std::vector<EdgeKey> result;
    container.getNew(result, true);
    EXPECT_EQ(expected, result);
  }

  {
    std::vector<EdgeKey> expected;

    std::vector<EdgeKey> result;
    container.getNew(result, true);
    EXPECT_EQ(expected, result);
  }
}

TEST(EdgeContainerTests, RemovedEdgesCorrect) {
  EdgeContainer container;
  container.insert(0, 1, nullptr);
  container.insert(1, 2, nullptr);
  container.insert(0, 3, nullptr);

  {
    std::vector<EdgeKey> expected;

    std::vector<EdgeKey> result;
    container.getRemoved(result, false);
    EXPECT_EQ(expected, result);
  }

  container.remove(1, 0);

  {
    std::vector<EdgeKey> expected{{0, 1}};

    std::vector<EdgeKey> result;
    container.getRemoved(result, false);
    EXPECT_EQ(expected, result);
  }

  {
    std::vector<EdgeKey> expected{{0, 1}};

    std::vector<EdgeKey> result;
    container.getRemoved(result, true);
    EXPECT_EQ(expected, result);
  }

  {
    std::vector<EdgeKey> expected;

    std::vector<EdgeKey> result;
    container.getRemoved(result, false);
    EXPECT_EQ(expected, result);
  }
}

TEST(EdgeContainerTests, RemovedWithAddCorrect) {
  EdgeContainer container;
  container.insert(0, 1, nullptr);

  {
    std::vector<EdgeKey> expected;

    std::vector<EdgeKey> result;
    container.getRemoved(result, false);
    EXPECT_EQ(expected, result);
  }

  container.remove(1, 0);

  {
    std::vector<EdgeKey> expected{{0, 1}};

    std::vector<EdgeKey> result;
    container.getRemoved(result, false);
    EXPECT_EQ(expected, result);
  }

  container.insert(0, 1, nullptr);

  {
    std::vector<EdgeKey> expected;

    std::vector<EdgeKey> result;
    container.getRemoved(result, false);
    EXPECT_EQ(expected, result);
  }
}

}  // namespace kimera
