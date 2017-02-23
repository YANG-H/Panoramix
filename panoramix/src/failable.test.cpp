#include <vector>
#include <list>
#include <random>

#include "failable.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano;

TEST(FailableTest, Failable) {
  core::Failable<std::vector<int>> opt;
  ASSERT_TRUE(opt.null());
  opt = core::AsResult(std::vector<int>{1, 2, 3, 4});
  ASSERT_TRUE(!opt.null());

  auto data = opt.unwrap();
  ASSERT_TRUE((data == std::vector<int>{1, 2, 3, 4}));
  ASSERT_TRUE(opt.null());

  auto opt2 = core::AsResult(std::move(data));
  ASSERT_TRUE(!opt2.null());

  opt = std::move(opt2);
  ASSERT_TRUE(!opt.null());
}