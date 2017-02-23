#include <vector>
#include <list>
#include <random>

#include "containers.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano;

inline double randf() { return (std::rand() % 100000) / 100000.0; }

TEST(ContainerTest, RTreeWrapperLargeData) {
  std::list<core::Line2> lines;
  std::generate_n(std::back_inserter(lines), 100000, []() {
    return core::Line2{core::Point2(std::rand(), std::rand()),
                       core::Point2(std::rand(), std::rand())};
  });
  core::RTreeWrapper<core::Line2> rtree(lines.begin(), lines.end());
  EXPECT_EQ(lines.size(), rtree.size());
}

TEST(ContainerTest, MaxHeap) {
  std::vector<double> data(50000);
  std::generate(data.begin(), data.end(), randf);
  std::vector<int> ids(data.size());
  std::iota(ids.begin(), ids.end(), 0);

  std::vector<core::Scored<int>> qd(data.size());
  for (int i = 0; i < data.size(); i++)
    qd[i] = core::ScoreAs(i, data[i]);
  std::priority_queue<core::Scored<int>> Q(qd.begin(), qd.end());
  core::MaxHeap<int> H(ids.begin(), ids.end(),
                       [&data](int id) { return data[id]; });

  ASSERT_EQ(Q.size(), H.size());

  int count = ids.size() + 1;
  while (!Q.empty()) {
    ASSERT_EQ(Q.size(), H.size());
    ASSERT_EQ(Q.top().score, H.topScore());
    Q.pop();
    H.pop();

    if (count % 2 == 0) {
      double v = randf();
      Q.push(core::ScoreAs(count, v));
      H.set(count, v);
    }

    count++;
  }

  core::MaxHeap<int> HH;
  int N = 5000;
  for (int i = 0; i < N; i++) {
    HH.set(i, randf());
  }
  for (int i = 0; i < N * 3; i++) {
    int key = i % N;
    HH.set(key, randf());
    int topKey = HH.top();
    // assert topKey has the highest score
    for (int j = 0; j < N; j++) {
      ASSERT_LE(HH.at(j), HH.at(topKey));
    }
  }
}

TEST(ContainerTest, Dictionary) {
  core::Dictionary<std::string> dict({3, 5, 4, 3});
  dict.insert({1, 2, 3, 1}, "1231");
  dict.insert({1, 3, 2, 1}, "1321");
  dict.insert({2, 1, 3, 2}, "2132");

  ASSERT_TRUE(dict.at({1, 2, 3, 1}) == "1231");
  ASSERT_TRUE(dict.at({1, 3, 2, 1}) == "1321");
  ASSERT_TRUE(dict.at({2, 1, 3, 2}) == "2132");

  ASSERT_TRUE(!dict.contains({1, 1, 1, 1}));

  auto dict2 = std::move(dict);

  ASSERT_TRUE(dict2.at({1, 2, 3, 1}) == "1231");
  ASSERT_TRUE(dict2.at({1, 3, 2, 1}) == "1321");
  ASSERT_TRUE(dict2.at({2, 1, 3, 2}) == "2132");

  dict2.insert({2, 1, 3, 2}, "xxxx");
  ASSERT_TRUE(dict2.at({2, 1, 3, 2}) == "xxxx");

  ASSERT_TRUE(!dict2.contains({1, 1, 1, 1}));

  core::SaveToDisk("./dict.cereal", dict2);

  core::Dictionary<std::string> dict3;
  core::LoadFromDisk("./dict.cereal", dict3);

  ASSERT_TRUE(dict3.at({2, 1, 3, 2}) == "xxxx");

  ASSERT_TRUE(dict3.at({1, 2, 3, 1}) == "1231");
  ASSERT_TRUE(dict3.at({1, 3, 2, 1}) == "1321");
}
