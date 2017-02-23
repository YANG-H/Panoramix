#include "basic_types.hpp"
#include "forest.hpp"

#include <random>

#include "../panoramix.unittest.hpp"

using namespace pano;


inline double randf() { return (std::rand() % 100000) / 100000.0; }

TEST(Serialization, BasicTypes) {
  std::vector<core::Classified<std::pair<core::Line3, core::HPoint2>>> data1(
      1000);
  std::vector<core::Classified<std::pair<core::Line3, core::HPoint2>>> data1c;
  for (auto &d : data1) {
    d.claz = std::rand();
    d.component.first = core::Line3(core::Point3(randf(), randf(), randf()),
                                    core::Point3(randf(), randf(), randf()));
    d.component.second = core::HPoint2(core::Point2(randf(), randf()), randf());
  }
  std::map<std::string, std::pair<core::Box2, core::GeoCoord>> data2;
  std::map<std::string, std::pair<core::Box2, core::GeoCoord>> data2c;
  for (int i = 0; i < 1000; i++) {
    data2[std::to_string(i)] =
        std::make_pair(core::Box2(core::Point2(randf(), randf()),
                                  core::Point2(randf(), randf())),
                       core::GeoCoord(randf(), randf()));
  }

  {
    std::ofstream os(PANORAMIX_TEST_DATA_DIR_STR "/data1data2.cereal",
                     std::ios::binary);
    cereal::BinaryOutputArchive archive(os);
    archive(data1, data2);
  }
  {
    std::ifstream is(PANORAMIX_TEST_DATA_DIR_STR "/data1data2.cereal",
                     std::ios::binary);
    cereal::BinaryInputArchive archive(is);
    archive(data1c, data2c);
  }

  EXPECT_EQ(data1.size(), data1c.size());
  EXPECT_EQ(data2.size(), data2c.size());

  for (int i = 0; i < data1.size(); i++) {
    EXPECT_TRUE(data1[i] == data1c[i]);
  }

  for (int i = 0; i < 1000; i++) {
    EXPECT_TRUE(data2[std::to_string(i)] == data2c[std::to_string(i)]);
  }
}

TEST(Serialization, FileTime) {
  core::Line3 testEntity;
  {
    std::ofstream os(PANORAMIX_TEST_DATA_DIR_STR "/line.cereal",
                     std::ios::binary);
    cereal::BinaryOutputArchive archive(os);
    archive(testEntity);
  }

  auto t1 =
      core::LastModifiedTimeOfFile(PANORAMIX_TEST_DATA_DIR_STR "/gm.cereal");
  auto t2 =
      core::LastModifiedTimeOfFile(PANORAMIX_TEST_DATA_DIR_STR "/line.cereal");
  auto t3 = core::CurrentTime();

  ASSERT_LE(t1, t2);
  ASSERT_LE(t2, t3);
}