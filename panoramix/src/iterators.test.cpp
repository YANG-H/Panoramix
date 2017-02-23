#include "iterators.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano::core;

TEST(IteratorsTest, TransformAndConcated) {
  double a[] = {1, 2, 3, 4, 5};
  const std::vector<double> b = {6, 7, 8, 9, 10};

  double test = 1.0;
  int count = 0;
  for (double e : MakeTransformRange(MakeConcatedRange(a, b),
                                     [](double e) { return e * e; })) {
    ASSERT_EQ(e, test * test);
    test += 1.0;
	count ++;
  }
  ASSERT_EQ(count, 10);
}

TEST(IteratorsTest, Iota) {
  std::vector<int> es;
  for (auto e : MakeConcatedRange(MakeIotaRange(5), MakeIotaRange(5, 8))) {
    es.push_back(e);
  }
  ASSERT_TRUE(es.size() == 8);
  for (int i = 0; i < es.size(); i++) {
    ASSERT_EQ(es[i], i);
  }
}