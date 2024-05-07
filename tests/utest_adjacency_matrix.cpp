/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include <gtest/gtest.h>
#include <spark_dsg/adjacency_matrix.h>

#include "spark_dsg/logging.h"

namespace spark_dsg {

struct AdjacencyMatrixTestConfig {
  std::map<NodeId, size_t> ordering;
  bool weighted;
  std::vector<double> expected_values_lt;  // lower triangular values
};

struct AdjacencyMatrixFixture
    : public testing::TestWithParam<AdjacencyMatrixTestConfig> {
  AdjacencyMatrixFixture() : layer(1) {}

  virtual ~AdjacencyMatrixFixture() = default;

  void SetUp() override {
    for (size_t i = 0; i < 4; ++i) {
      layer.emplaceNode(i, std::make_unique<NodeAttributes>());
    }

    // small upper left clique
    layer.insertEdge(0, 1, std::make_unique<EdgeAttributes>(0.1));
    layer.insertEdge(0, 2, std::make_unique<EdgeAttributes>(0.2));
    layer.insertEdge(1, 2, std::make_unique<EdgeAttributes>(0.3));
    // other edges
    layer.insertEdge(1, 3, std::make_unique<EdgeAttributes>(0.4));
  }

  IsolatedSceneGraphLayer layer;
};

TEST_P(AdjacencyMatrixFixture, AdjacencyMatrixCorrect) {
  AdjacencyMatrixTestConfig config = GetParam();

  Eigen::MatrixXd A;
  if (config.weighted) {
    A = getAdjacencyMatrix(
        layer, config.ordering, [&](const NodeId source, const NodeId target) {
          return layer.getEdge(source, target).info->weight;
        });
  } else {
    A = getAdjacencyMatrix(layer, config.ordering);
  }

  ASSERT_EQ(config.ordering.size(), static_cast<size_t>(A.rows()));
  ASSERT_EQ(config.ordering.size(), static_cast<size_t>(A.cols()));

  size_t index = 0;
  for (int r = 0; r < A.rows(); ++r) {
    for (int c = 0; c < r + 1; ++c) {
      if (r == c) {
        EXPECT_EQ(0.0, A(r, c));
      } else {
        ASSERT_LT(index, config.expected_values_lt.size())
            << "test config invalid: " << index
            << " >= " << config.expected_values_lt.size();
        EXPECT_EQ(config.expected_values_lt[index], A(r, c))
            << "invalid value @ " << r << ", " << c << " -> " << std::endl
            << A;
        EXPECT_EQ(config.expected_values_lt[index], A(c, r))
            << "invalid value @ " << r << ", " << c << " -> " << std::endl
            << A;
      }
      index++;
    }
  }

  SparseMatrixXd sparse_A;
  if (config.weighted) {
    sparse_A = getSparseAdjacencyMatrix(
        layer, config.ordering, [&](const NodeId source, const NodeId target) {
          return layer.getEdge(source, target).info->weight;
        });
  } else {
    sparse_A = getSparseAdjacencyMatrix(layer, config.ordering);
  }

  Eigen::MatrixXd dense_A(sparse_A);
  EXPECT_EQ(A, dense_A) << "orig: " << std::endl
                        << A << std::endl
                        << " sparse-to-dense: " << std::endl
                        << dense_A;
}

TEST_P(AdjacencyMatrixFixture, LaplacianCorrect) {
  AdjacencyMatrixTestConfig config = GetParam();

  Eigen::MatrixXd L;
  if (config.weighted) {
    L = getLaplacian(
        layer, config.ordering, [&](const NodeId source, const NodeId target) {
          return layer.getEdge(source, target).info->weight;
        });
  } else {
    L = getLaplacian(layer, config.ordering);
  }

  ASSERT_EQ(config.ordering.size(), static_cast<size_t>(L.rows()));
  ASSERT_EQ(config.ordering.size(), static_cast<size_t>(L.cols()));

  size_t index = 0;
  for (int r = 0; r < L.rows(); ++r) {
    for (int c = 0; c < r + 1; ++c) {
      if (r == c) {
        EXPECT_NEAR(config.expected_values_lt[index], L(r, c), 1.0e-7)
            << "invalid value @ " << r << ", " << c << " -> " << std::endl
            << L;
      } else {
        ASSERT_LT(index, config.expected_values_lt.size())
            << "test config invalid: " << index
            << " >= " << config.expected_values_lt.size();
        EXPECT_NEAR(-config.expected_values_lt[index], L(r, c), 1.0e-7)
            << "invalid value @ " << r << ", " << c << " -> " << std::endl
            << L;
        EXPECT_NEAR(-config.expected_values_lt[index], L(c, r), 1.0e-7)
            << "invalid value @ " << r << ", " << c << " -> " << std::endl
            << L;
      }
      index++;
    }
  }

  SparseMatrixXd sparse_L;
  if (config.weighted) {
    sparse_L = getSparseLaplacian(
        layer, config.ordering, [&](const NodeId source, const NodeId target) {
          return layer.getEdge(source, target).info->weight;
        });
  } else {
    sparse_L = getSparseLaplacian(layer, config.ordering);
  }

  Eigen::MatrixXd dense_L(sparse_L);
  EXPECT_EQ(L, dense_L);
}

// note that the diagonal entries are the degrees (so the same test can cover the
// laplacians)
const AdjacencyMatrixTestConfig adjacency_test_cases[] = {
    {{{0, 0}, {1, 1}, {2, 2}}, false, {2.0, 1.0, 2.0, 1.0, 1.0, 2.0}},
    {{{0, 0}, {2, 2}, {1, 1}}, false, {2.0, 1.0, 2.0, 1.0, 1.0, 2.0}},
    {{{0, 1}, {1, 0}, {2, 2}}, false, {2.0, 1.0, 2.0, 1.0, 1.0, 2.0}},
    {{{0, 1}, {1, 2}, {2, 0}}, false, {2.0, 1.0, 2.0, 1.0, 1.0, 2.0}},
    {{{0, 1}, {1, 2}, {2, 0}}, true, {0.5, 0.2, 0.3, 0.3, 0.1, 0.4}},
    {{{0, 0}, {1, 1}, {2, 2}, {3, 3}},
     true,
     {0.3, 0.1, 0.8, 0.2, 0.3, 0.5, 0.0, 0.4, 0.0, 0.4}},
};

INSTANTIATE_TEST_SUITE_P(MatrixMethodsMatchExpected,
                         AdjacencyMatrixFixture,
                         testing::ValuesIn(adjacency_test_cases));

}  // namespace spark_dsg
