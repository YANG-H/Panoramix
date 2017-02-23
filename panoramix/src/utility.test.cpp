#include <list>
#include <random>
#include <vector>

#include "utility.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano;

inline double randf() { return (std::rand() % 100000) / 100000.0; }

TEST(UtilityTest, HasValue) {
  std::vector<std::pair<core::Line2, double>> hlines = {
      {{{1.0, 2.0}, {4.0, 5.0}}, 0.0},
      {{{1.0, 2.0}, {NAN, 5.0}}, 0.0},
      {{{1.0, 2.0}, {0.0, 5.0}}, 0.0},
      {{{1.0, 2.0}, {4.0, 5.0}}, 0.0},
      {{{1.0, 2.0}, {4.0, 5.0}}, 0.0}};

  ASSERT_TRUE(core::HasValue(hlines, [](auto e) { return std::isnan(e); }));
  ASSERT_FALSE(core::HasValue(hlines, [](auto e) { return std::isinf(e); }));
}

TEST(UtilityTest, Distance) {
  using namespace core;

  auto d = Distance(std::complex<double>(1, 2), std::complex<double>(3, 4));
  ASSERT_DOUBLE_EQ(2 * sqrt(2), d);
}

TEST(UtilityTest, BoundingBox) {
  using namespace core;
  Line3 l1(Point3(0.5, 0.1, 1), Point3(1, 0.4, 0.7));
  Line3 l2(Point3(0.6, 1, 0.9), Point3(0.2, -1, 0.5));

  Line3 lines[] = {l1, l2};
  auto box = BoundingBoxOfContainer(lines);

  ASSERT_EQ(0.2, box.minCorner[0]);
  ASSERT_EQ(1, box.maxCorner[0]);

  ASSERT_EQ(-1, box.minCorner[1]);
  ASSERT_EQ(1, box.maxCorner[1]);

  ASSERT_EQ(0.5, box.minCorner[2]);
  ASSERT_EQ(1, box.maxCorner[2]);

  Enabled<Line3> lines2[] = {EnableAs(l1, true), EnableAs(l2, false)};
  ASSERT_TRUE(BoundingBoxOfContainer(lines2) == BoundingBox(l1));
}

TEST(UtilityTest, BoundBetween) {
  for (int i = 0; i < 10000; i++) {
    double x = double(rand()) / rand() + rand();
    double a = double(rand()) / rand() + rand();
    double b = a + abs(double(rand()) / rand());
    if (a == b || std::isnan(x) || std::isnan(a) || std::isnan(b) ||
        std::isinf(x) || std::isinf(a) || std::isinf(b))
      continue;
    double xx = core::BoundBetween(x, a, b);
    ASSERT_LE(a, xx);
    ASSERT_LE(xx, b);
    ASSERT_TRUE(xx == x || xx == a || xx == b);
  }
}

TEST(UtilityTest, WrapBetween) {
  for (int i = 0; i < 10000; i++) {
    double x = double(rand()) / rand() + rand();
    double a = double(rand()) / rand() + rand();
    double b = a + abs(double(rand()) / rand());
    if (a == b || std::isnan(x) || std::isnan(a) || std::isnan(b) ||
        std::isinf(x) || std::isinf(a) || std::isinf(b))
      continue;
    double xx = core::WrapBetween(x, a, b);
    double rem = (xx - x) / (b - a) - std::round((xx - x) / (b - a));
    if (std::isnan(rem)) {
      assert(0);
    }
    EXPECT_NEAR(0, rem, 1e-5);
    EXPECT_LE(a, xx);
    EXPECT_LT(xx, b);
  }

  // int test
  int x = core::WrapBetween(0, 1, 2);
  for (int i = 0; i < 10000; i++) {
    int x = rand();
    int a = rand();
    int b = a + abs(rand());
    if (a == b)
      continue;

    int xx = core::WrapBetween(x, a, b);
    int rem = (xx - x) % (b - a);

    EXPECT_EQ(0, rem);
    EXPECT_LE(a, xx);
    EXPECT_LT(xx, b);
  }
}

TEST(UtilityTest, SubscriptAndIndex) {
  auto i = core::EncodeSubscriptToIndex(core::Point<int, 2>(1, 2),
                                        core::Vec<int, 2>(2, 3));
  ASSERT_EQ(5, i);

  {
    int trueIndex = 0;
    for (int a = 0; a < 10; a++) {
      for (int b = 0; b < 20; b++) {
        for (int c = 0; c < 15; c++) {
          for (int d = 0; d < 9; d++) {
            int index =
                core::EncodeSubscriptToIndex(core::Point<int, 4>(a, b, c, d),
                                             core::Vec<int, 4>(10, 20, 15, 9));
            ASSERT_EQ(trueIndex, index);

            auto subs = core::DecodeIndexToSubscript(
                index, core::Vec<int, 4>(10, 20, 15, 9));
            ASSERT_EQ(0, core::norm(core::Point<int, 4>(a, b, c, d) - subs));

            trueIndex++;
          }
        }
      }
    }
  }

  {
    float trueIndex = 0;
    for (int a = 0; a < 10; a++) {
      for (int b = 0; b < 20; b++) {
        for (int c = 0; c < 15; c++) {
          for (int d = 0; d < 9; d++) {
            float index = core::EncodeSubscriptToIndex(
                core::Point<float, 4>(a, b, c, d),
                core::Vec<float, 4>(10, 20, 15, 9));
            ASSERT_FLOAT_EQ(trueIndex, index);

            auto subs = core::DecodeIndexToSubscript(
                index, core::Vec<float, 4>(10, 20, 15, 9));
            ASSERT_FLOAT_EQ(
                0, core::norm(core::Point<float, 4>(a, b, c, d) - subs));

            trueIndex++;
          }
        }
      }
    }
  }
}

TEST(UtilityTest, AngleBetweenDirected) {
  core::Vec2 v1(1, 0), v2(1, 1);
  ASSERT_DOUBLE_EQ(M_PI_4, core::AngleBetweenDirected(v1, v2));
  ASSERT_DOUBLE_EQ(M_PI_4, core::SignedAngle(v1, v2, false));
  ASSERT_DOUBLE_EQ(-M_PI_4, core::SignedAngle(v1, v2, true));
  ASSERT_DOUBLE_EQ(-M_PI_4, core::SignedAngle(v1, v2));
  core::Vec2 v3(-1, -1);
  ASSERT_DOUBLE_EQ(-M_PI_4 * 3, core::SignedAngle(v1, v3, false));
  ASSERT_DOUBLE_EQ(M_PI_4 * 3, core::SignedAngle(v1, v3, true));
  ASSERT_FLOAT_EQ(M_PI, core::AngleBetweenDirected(v2, v3));
  ASSERT_DOUBLE_EQ(
      0.0, core::AngleBetweenDirected(core::Vec3(1.0, 1.9, 0.1),
                                      core::Vec3(1.0, 1.9, 0.1000000001)));
}

TEST(UtilityTest, DistanceFromPointToLine) {
  core::Line3 l;
  l.first = {1, 0, 0};
  l.second = {-1, 0, 0};
  auto infLine = l.ray();
  for (double x = -3; x <= 3; x += 0.5) {
    core::Point3 p(x, 1, 0);
    if (x < -1) {
      ASSERT_DOUBLE_EQ(core::norm(l.second - p), core::Distance(p, l));
    } else if (x > 1) {
      ASSERT_DOUBLE_EQ(core::norm(l.first - p), core::Distance(p, l));
    } else {
      ASSERT_DOUBLE_EQ(1, core::Distance(p, l));
    }
    ASSERT_DOUBLE_EQ(
        1, core::norm(core::ProjectionOfPointOnLine(p, l).position - p));
    ASSERT_DOUBLE_EQ(
        1, core::norm(core::DistanceFromPointToLine(p, l.ray()).second - p));
  }
}

TEST(UtilityTest, PlaneIntersection) {
  int testednum = 0;
  for (int i = 0; i < 10000; i++) {
    core::Ray3 ray(core::Point3(randf(), randf(), randf()),
                   core::Vec3(randf(), randf(), randf()));
    core::Point3 p1(randf(), randf(), randf()), p2(randf(), randf(), randf());
    core::Plane3 plane1(ray.anchor, ray.direction.cross(p1 - ray.anchor));
    core::Plane3 plane2(ray.anchor, ray.direction.cross(p2 - ray.anchor));
    if (core::IsFuzzyParallel(plane1.normal, plane2.normal)) {
      continue;
    }
    testednum++;
    core::Ray3 r = core::IntersectionOfPlaneAndPlane(plane1, plane2).unwrap();
    ASSERT_TRUE(core::IsFuzzyParallel(ray.anchor - r.anchor, r.direction));
    ASSERT_TRUE(core::IsFuzzyParallel(ray.direction, r.direction));
  }
  std::cout << testednum << "/10000 are tested" << std::endl;
}

TEST(UtilityTest, DistanceBetweenTwoLines) {
  core::Line3 l1 = {{1, 0, 0}, {-1, 0, 0}};
  core::Line3 l2 = {{0, 1, 1}, {0, -1, 1}};
  core::Line3 l3 = {{0, 2, 1}, {0, 3, 1}};
  ASSERT_DOUBLE_EQ(1, core::Distance(l1, l2));
  ASSERT_DOUBLE_EQ(1, core::Distance(l1, l2.reversed()));
  ASSERT_DOUBLE_EQ(1, core::Distance(l1.reversed(), l2));
  ASSERT_DOUBLE_EQ(core::norm(l3.first - l1.center()), core::Distance(l1, l3));
  ASSERT_DOUBLE_EQ(core::norm(l3.first - l1.center()),
                   core::Distance(l1.reversed(), l3));
  ASSERT_DOUBLE_EQ(core::norm(l3.first - l1.center()),
                   core::Distance(l1, l3.reversed()));

  double dd = 1;
  ASSERT_DOUBLE_EQ(
      dd,
      core::Distance(core::Line3{{0, 0, 0}, {1, 0, 0}},
                     core::Line3{{0, 0, dd}, {1, 0, dd + 0.01}}.reversed()));
  ASSERT_DOUBLE_EQ(
      dd, core::Distance(core::Line3{{0, 0, 0}, {1, 0, 0}},
                         core::Line3{{0, 0, dd}, {1, 0, dd}}.reversed()));

  core::Line3 l4 = {{1, 1, 0}, {-1, 1, 0}};
  ASSERT_DOUBLE_EQ(1, core::Distance(l1, l4));
  ASSERT_DOUBLE_EQ(1, core::Distance(l1, l4.reversed()));
  ASSERT_DOUBLE_EQ(1, core::Distance(l1.reversed(), l4));

  core::Line3 l5 = {{2, 1, 0}, {3, 1, 0}};
  ASSERT_DOUBLE_EQ(core::norm(l1.first, l5.first), core::Distance(l1, l5));
  ASSERT_DOUBLE_EQ(core::norm(l1.first, l5.first),
                   core::Distance(l1.reversed(), l5));
  ASSERT_DOUBLE_EQ(core::norm(l1.first, l5.first),
                   core::Distance(l1, l5.reversed()));

  core::Line3 l6 = {{2, 1, 0}, {-2, 1, 0}};
  ASSERT_DOUBLE_EQ(1, core::Distance(l1, l6));
  ASSERT_DOUBLE_EQ(1, core::Distance(l1, l6.reversed()));
  ASSERT_DOUBLE_EQ(1, core::Distance(l1.reversed(), l6));

  core::Line3 l7 = {{0, 0, 1}, {1, 0, 1}};
  core::Line3 l8 = {{0, 1, 0}, {0, 2, 0}};
  ASSERT_DOUBLE_EQ(sqrt(2.0), core::Distance(l7, l8));

  core::Line3 l9 = {{1, 0, 1}, {0, 0, 1}};
  core::Line3 l10 = {{0, 2, 0}, {0, 1, 0}};
  ASSERT_DOUBLE_EQ(sqrt(2.0), core::Distance(l7, l10));
  ASSERT_DOUBLE_EQ(sqrt(2.0), core::Distance(l9, l8));
  ASSERT_DOUBLE_EQ(sqrt(2.0), core::Distance(l9, l10));

  ASSERT_DOUBLE_EQ(0, core::Distance(l7, l7));
  ASSERT_DOUBLE_EQ(0, core::Distance(l10, l10));

  ASSERT_DOUBLE_EQ(0, core::Distance(l7, l9));
  ASSERT_DOUBLE_EQ(0, core::Distance(l8, l10));

  {
    core::Line3 a = {
        {0.32060601460883287, 0.92543477139591090, -0.20194619899378968},
        {0.24497237141965944, 0.95024535429166723, -0.19241180807874725}};
    core::Line3 b = {
        {0.15085395484832950, 0.90385564866472523, -0.40035990144304773},
        {0.096251150572768340, 0.90140252138592014, -0.42214832754912596}};
    auto pab = core::DistanceBetweenTwoLines(a, b);

    ASSERT_LE(pab.first, core::Distance(a.first, b.first));
    ASSERT_LE(pab.first, core::Distance(a.first, b.second));
    ASSERT_LE(pab.first, core::Distance(a.second, b.first));
    ASSERT_LE(pab.first, core::Distance(a.second, b.second));
  }
  {
    core::Line3 a = {{0.98184, -0.120335, 0.146665},
                     {0.65886, 0.72241, -0.209827}};
    core::Line3 b = {{0.493696, 0.844419, 0.207651},
                     {0.245523, 0.952812, 0.178513}};
    auto pab = core::DistanceBetweenTwoLines(a, b);

    ASSERT_LE(pab.first, core::Distance(a.first, b.first));
    ASSERT_LE(pab.first, core::Distance(a.first, b.second));
    ASSERT_LE(pab.first, core::Distance(a.second, b.first));
    ASSERT_LE(pab.first, core::Distance(a.second, b.second));
  }

  for (int i = 0; i < 1000; i++) {
    core::Line3 aa = {{randf(), randf(), randf()}, {randf(), randf(), randf()}};
    core::Line3 bb = {{randf(), randf(), randf()}, {randf(), randf(), randf()}};
    auto p = core::DistanceBetweenTwoLines(aa, bb);
    core::Ray3 iaa = {{randf(), randf(), randf()}, {randf(), randf(), randf()}};
    core::Ray3 ibb = {{randf(), randf(), randf()}, {randf(), randf(), randf()}};
    auto ip = core::DistanceBetweenTwoLines(iaa, ibb);

    EXPECT_LE(p.first - 0.1, core::Distance(aa.first, bb.first));
    EXPECT_LE(p.first - 0.1, core::Distance(aa.first, bb.second));
    EXPECT_LE(p.first - 0.1, core::Distance(aa.second, bb.first));
    EXPECT_LE(p.first - 0.1, core::Distance(aa.second, bb.second));

    EXPECT_LE(ip.first - 0.1, core::Distance(iaa.anchor, ibb.anchor));
  }
}

TEST(UtilityTest, BarycentricCoordinatesOfLineAndPlaneUnitIntersection) {
  core::Point3 pts[] = {core::Point3(1, 0, 0), core::Point3(0, 1, 0),
                        core::Point3(0, 0, 1)};

  auto bc1 = core::BarycentricCoordinatesOfLineAndPlaneUnitIntersection(
      core::Ray3(core::Point3(0, 0, 0), core::Point3(1, 1, 1)), pts);

  EXPECT_FLOAT_EQ(bc1[0], 1.0 / 3.0);
  EXPECT_FLOAT_EQ(bc1[1], 1.0 / 3.0);
  EXPECT_FLOAT_EQ(bc1[2], 1.0 / 3.0);

  for (int i = 0; i < 3; i++) {
    auto bc2 = core::BarycentricCoordinatesOfLineAndPlaneUnitIntersection(
        core::Ray3(core::Point3(0, 0, 0), pts[i]), pts);
    for (int k = 0; k < 3; k++) {
      EXPECT_FLOAT_EQ(bc2[k], i == k ? 1.0 : 0.0);
    }
  }

  for (int i = 0; i < 3; i++) {
    core::Vec3 dir(1, 1, 1);
    dir(i) = -.5;
    auto bc3 = core::BarycentricCoordinatesOfLineAndPlaneUnitIntersection(
        core::Ray3(core::Point3(0, 0, 0), dir), pts);
    for (int k = 0; k < 3; k++) {
      EXPECT_FLOAT_EQ(bc3[k], k == i ? -1.0 / 3.0 : 2.0 / 3.0);
    }
  }
}

TEST(UtilityTest, EigenVectorsAndValues) {
  {
    std::vector<core::Point3> pts;
    std::generate_n(std::back_inserter(pts), 10000, []() {
      auto a = randf();
      auto b = randf();
      auto c = randf();
      return core::Point3(a, b / 1000.0, c / 1000.0);
    });
    std::generate_n(std::back_inserter(pts), 1, []() {
      auto a = randf();
      auto b = randf();
      auto c = randf();
      return core::Point3(a, b / 1000.0 + 1.0, c / 1000.0);
    });
    auto result = core::EigenVectorAndValuesFromPoints(pts);
    for (auto &r : result) {
      std::cout << "eigen vector: " << r.component << "  value: " << r.score
                << std::endl;
    }

    std::sort(result.begin(), result.end());
    std::cout << "planarity = " << (result[1].score / result[2].score *
                                    result[1].score / result[0].score)
              << std::endl;
  }

  core::Plane3 plane(core::Point3(1, 2, 3), core::Vec3(-2, -5, -7));
  core::Vec3 x, y;
  std::tie(x, y) = core::ProposeXYDirectionsFromZDirection(plane.normal);
  std::vector<core::Point3> pts;
  std::generate_n(std::back_inserter(pts), 10000, [&]() {
    auto a = randf();
    auto b = randf();
    auto c = randf();
    return a * x + b * y + c / 1000.0 * plane.normal + plane.anchor;
  });
  auto result = core::EigenVectorAndValuesFromPoints(pts);
  for (auto &r : result) {
    std::cout << "eigen vector: " << core::normalize(r.component)
              << "  value: " << r.score << std::endl;
  }
  std::sort(result.begin(), result.end());
  std::cout << "planarity = " << (result[1].score / result[2].score *
                                  result[1].score / result[0].score)
            << std::endl;
}
