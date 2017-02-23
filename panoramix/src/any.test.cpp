#include <vector>
#include <list>
#include <random>

#include "any.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano;

TEST(MiscTest, Any) {
  using namespace core;

  Any aInt = 1;
  Any aFloat = 2.0f;
  Any aVector = std::vector<std::array<int, 3>>(5);

  ASSERT_TRUE(aInt.is<int>());
  ASSERT_TRUE(aFloat.is<float>());
  ASSERT_TRUE((aVector.is<std::vector<std::array<int, 3>>>()));
  ASSERT_FALSE((aVector.is<std::vector<std::array<int, 2>>>()));

  int vInt = aInt;
  ASSERT_EQ(vInt, 1);
  float vFloat = aFloat;
  ASSERT_EQ(vFloat, 2.0f);
  std::vector<std::array<int, 3>> vVector = aVector;
  ASSERT_TRUE((vVector == std::vector<std::array<int, 3>>(5)));

  std::swap(aInt, aFloat);
  ASSERT_EQ((float)aInt, 2.0f);
  ASSERT_EQ((int)aFloat, 1);

  ASSERT_FALSE(aVector.null());
  aVector = nullptr;
  ASSERT_TRUE(aVector.null());

  Any something;
  ASSERT_TRUE(something.null());
  something = std::list<int>{1, 2, 3, 4};
  ASSERT_FALSE(something.null());
  ASSERT_TRUE(something.is<std::list<int>>());
  something = std::set<double>{1.0, 2.0, 3.0};
  ASSERT_FALSE(something.is<std::list<int>>());
  ASSERT_TRUE(something.is<std::set<double>>());

  Any mess =
      std::list<Any>{std::make_tuple(1, 3.0, "123"),
                     std::vector<Any>{std::array<int, 3>{{1, 2, 3}},
                                      std::pair<double, double>{1.5, 7.4}}};

  ASSERT_TRUE(
      (mess.cast<std::list<Any>>()
           .back()
           .cast<std::vector<Any>>()
           .front()
           .cast<std::array<int, 3>>() == std::array<int, 3>{{1, 2, 3}}));
  ASSERT_TRUE((mess.cast<std::list<Any>>()
                   .back()
                   .cast<std::vector<Any>>()
                   .back()
                   .cast<std::pair<double, double>>() ==
               std::pair<double, double>{1.5, 7.4}));
}