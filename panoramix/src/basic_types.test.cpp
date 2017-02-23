#include "basic_types.hpp"
#include "utility.hpp"
#include "../panoramix.unittest.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <random>

using namespace pano;

TEST(BasicType, Vec) {
  {
    core::Vec3 v1(0, 0, 0), v2(1, 1, 1);
    core::Vec3 v12 = v1 + v2 * 3.0;
    ASSERT_TRUE(v12 == core::Vec3(3, 3, 3));
  }
  {
    core::Vec4 v1(0, 0, 0, 0), v2(1, 1, 1, 1);
    core::Vec4 v12 = v1 + v2 * 3.0;
    ASSERT_TRUE(v12 == core::Vec4(3, 3, 3, 3));
  }
}

TEST(BasicType, HPoint) {
  for (int i = 0; i < 1000; i++) {
    core::Vec4 v4;
    std::generate(v4.val, v4.val + 4, std::rand);
    auto hp = core::HPointFromVector(v4);
    auto p = VectorFromHPoint(hp);
    ASSERT_LT(core::norm(p - v4), 1e-5);
    core::HPoint<double, 4> hp5 = v4;
    auto p5 = hp5.value();
    ASSERT_LT(core::norm(p5 - v4), 1e-5);
  }
}

TEST(BasicType, Line) {
  std::vector<core::Line2> lines = {
      {core::Point2(1, 2), core::Point2(3, 4)},
      {core::Point2(5, 6), core::Point2(7, 8)},
      {core::Point2(9, 10), core::Point2(11, 12)},
      {core::Point2(13, 14), core::Point2(15, 16)},
      {core::Point2(17, 18), core::Point2(19, 20)},
      {core::Point2(21, 22), core::Point2(23, 24)}};

  Eigen::Map<Eigen::Matrix<double, 4, Eigen::Dynamic>> linesData(
      (double *)(lines.data()), 4, lines.size());
  std::cout << linesData << std::endl;
}

TEST(BasicType, VecCast) {
  core::Imaged im(100, 100, 0.0);
  auto imi = core::ecast<int>(im);

  for (auto &i : imi) {
    ASSERT_EQ(i, 0);
  }

  core::Image3d im3(100, 100, core::Vec3(1, 2, 3));
  auto imi3 = core::ecast<int>(im3);
  auto imd3 = core::ecast<double>(im3);

  for (auto &i : imi3) {
    ASSERT_TRUE(i == core::Vec3i(1, 2, 3));
  }
  for (auto &i : imd3) {
    ASSERT_TRUE(i == core::Vec3(1, 2, 3));
  }
}
