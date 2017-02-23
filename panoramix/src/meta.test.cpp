#include "meta.hpp"

#include "../panoramix.unittest.hpp"

#include <iostream>
#include <random>

using namespace pano;

TEST(Tuples, Invoke) {
  static_assert(
      core::SequenceElement<2, core::Sequence<1, 2, 3, 4>>::value == 3, "");
  static_assert(
      core::SequenceContainsElement<4, core::Sequence<4, 3, 2>>::value, "");
  static_assert(
      core::TupleContainsType<float, std::tuple<bool, float, int>>::value, "");
  static_assert(core::TypeFirstLocationInTuple<
                    float, std::tuple<int, float, double>>::value == 1,
                "");

  auto args = std::make_tuple(1, 2, 3, 4, 5.0);
  auto fun = [](int a, int b, int c, int d, double e) {
    return (a + b + c + d + e) * 2.0;
  };

  auto sumDoubled = core::Invoke(fun, args);
  ASSERT_EQ(30, sumDoubled);

  auto fun2 = [](int a, int b, int c, int d, double e) {
    std::cout << a << b << c << d << e << std::endl;
  };

  core::Invoke(fun2, args);

  core::Invoke([](int a, double b) { std::cout << a << b << std::endl; },
               std::make_pair(5, 5.0));
}

TEST(Meta, WhatOf) {
  std::vector<std::vector<std::string>> strs = {
      {"a", "b"}, {"c"}, {"d", "e", "fg"}};
  ASSERT_EQ(core::CountOf<std::string>(strs), 6);
  ASSERT_EQ(core::CountOf<char>(strs), 7);
  ASSERT_TRUE((core::AllOf<std::string>(strs, [](auto & str){return str != "0";})));
  ASSERT_TRUE((!core::AllOf<std::string>(strs, [](auto & str){return str == "a";})));
}
