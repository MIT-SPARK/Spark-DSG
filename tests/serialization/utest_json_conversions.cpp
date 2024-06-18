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

#include "spark_dsg/serialization/json_conversions.h"
#include "spark_dsg_tests/type_comparisons.h"

namespace spark_dsg {

using nlohmann::json;

TEST(JsonConversions, EigenVectorJson) {
  {  // double vector
    Eigen::Vector3d expected;
    expected << 1.0, 2.0, 3.0;

    json output = expected;

    auto result = output.get<Eigen::Vector3d>();
    EXPECT_EQ(expected, result);
  }

  {  // uint8_t vector
    Color expected(1, 2, 3);

    json output = expected;

    auto result = output.get<Color>();
    EXPECT_EQ(expected, result);
  }

  {  // dynamic float vector
    Eigen::VectorXf expected(5, 1);
    expected << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;

    json output = expected;

    auto result = output.get<Eigen::VectorXf>();
    EXPECT_EQ(expected, result);
  }

  {  // dynamic int vector
    Eigen::VectorXi expected(5, 1);
    expected << 1, 2, 3, 4, 5;

    json output = expected;

    auto result = output.get<Eigen::VectorXi>();
    EXPECT_EQ(expected, result);
  }
}

TEST(JsonConversions, EigenMatrixJson) {
  {  // double fixed-size matrix
    Eigen::Matrix2d expected;
    expected << 1.0, 2.0, 3.0, 4.0;

    json output = expected;

    auto result = output.get<Eigen::Matrix2d>();
    EXPECT_EQ(expected, result);
  }

  {  // mixed matrix size
    Eigen::Matrix<float, 3, Eigen::Dynamic> expected(3, 4);
    expected << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f,
        12.0f;

    json output = expected;

    auto result = output.get<Eigen::Matrix<float, 3, Eigen::Dynamic>>();
    EXPECT_EQ(expected, result);
  }

  {  // dynamic matrix
    Eigen::MatrixXd expected(2, 3);
    expected << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    json output = expected;

    auto result = output.get<Eigen::MatrixXd>();
    EXPECT_EQ(expected, result);
  }
}

TEST(JsonConversions, EigenQuaternionJson) {
  std::stringstream ss;

  {  // single-precision
    Eigen::Quaternionf expected(0.0, 0.0, 1.0, 0.0);
    json output = expected;
    auto result = output.get<Eigen::Quaternionf>();
    ASSERT_TRUE(::spark_dsg::test::quaternionsEqual(expected, result));
  }

  {  // double-precision
    Eigen::Quaterniond expected(0.0, 0.0, 0.0, 1.0);
    json output = expected;
    auto result = output.get<Eigen::Quaterniond>();
    ASSERT_TRUE(::spark_dsg::test::quaternionsEqual(expected, result));
  }
}

TEST(JsonConversions, BoundingBoxJson) {
  {  // invalid type
    BoundingBox expected;
    json output = expected;
    BoundingBox result = output.get<BoundingBox>();
    EXPECT_EQ(expected, result);
  }

  {  // ABB
    Eigen::Vector3f expected_pos{1.0f, 2.0f, 3.0f};
    Eigen::Vector3f expected_dim{4.0f, 5.0f, 6.0f};
    BoundingBox expected(expected_pos, expected_dim);
    json output = expected;
    BoundingBox result = output.get<BoundingBox>();
    EXPECT_EQ(expected, result);
  }

  {  // OBB
    Eigen::Vector3f expected_pos{1.0f, 2.0f, 3.0f};
    Eigen::Vector3f expected_dim{4.0f, 5.0f, 6.0f};
    Eigen::Quaternionf expected_rot(0.0, 0.0, 1.0, 0.0);
    BoundingBox expected(expected_pos, expected_dim, expected_rot);
    json output = expected;
    BoundingBox result = output.get<BoundingBox>();
    EXPECT_EQ(expected, result);
  }
}

}  // namespace spark_dsg
