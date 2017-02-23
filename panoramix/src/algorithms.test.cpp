#include <list>
#include <random>
#include <vector>

#include "algorithms.hpp"
#include "containers.hpp"

#include "../panoramix.unittest.hpp"

using namespace pano;

TEST(AlgorithmsTest, ForeachCompatible) {
  std::list<double> data = {1.0, 2.0, 3.0, 5.0, 6.0, 7.0};
  std::list<double> selected;
  core::FilterBy(data.begin(), data.end(), std::back_inserter(selected),
                 [](double a, double b) { return abs(a - b) >= 1.5; });
  std::list<double> groundTruth = {1.0, 3.0, 5.0, 7.0};
  ASSERT_TRUE(selected == groundTruth);
}

TEST(AlgorithmsTest, MergeNearNaiveCorrectness) {
  std::list<double> arr1;
  arr1.resize(1000);
  std::generate(arr1.begin(), arr1.end(), std::rand);
  std::vector<double> arr2(arr1.begin(), arr1.end());

  double thres = 100;
  std::vector<std::list<double>::iterator> gBegins1;
  core::MergeNearNaive(std::begin(arr1), std::end(arr1),
                       std::back_inserter(gBegins1), std::false_type(), thres);
  std::vector<std::vector<double>::iterator> gBegins2;
  core::MergeNearNaive(std::begin(arr2), std::end(arr2),
                       std::back_inserter(gBegins2), std::true_type(), thres);
  ASSERT_EQ(gBegins1.size(), gBegins2.size());
  auto i = gBegins1.begin();
  auto j = gBegins2.begin();
  for (; i != gBegins1.end(); ++i, ++j) {
    EXPECT_EQ(**i, **j);
  }
  for (auto i = gBegins2.begin(); i != gBegins2.end(); ++i) {
    auto inext = std::next(i);
    auto begin = *i;
    auto end = inext == gBegins2.end() ? std::end(arr2) : *inext;
    auto beginVal = *begin;
    for (auto j = begin; j != end; ++j) {
      EXPECT_NEAR(*j, beginVal, thres);
    }
  }
}

TEST(AlgorithmsTest, MergeNearRTreeCorrectness) {
  std::list<double> arr1;
  arr1.resize(1000);
  std::generate(arr1.begin(), arr1.end(), std::rand);
  std::vector<double> arr2(arr1.begin(), arr1.end());

  double thres = 100;
  std::vector<std::list<double>::iterator> gBegins1;
  core::MergeNearRTree(std::begin(arr1), std::end(arr1),
                       std::back_inserter(gBegins1), std::false_type(), thres);
  std::vector<std::vector<double>::iterator> gBegins2;
  core::MergeNearNaive(std::begin(arr2), std::end(arr2),
                       std::back_inserter(gBegins2), std::false_type(), thres);
  ASSERT_EQ(gBegins1.size(), gBegins2.size());
  auto i = gBegins1.begin();
  auto j = gBegins2.begin();
  for (; i != gBegins1.end(); ++i, ++j) {
    auto iv = **i;
    auto jv = **j;
    EXPECT_DOUBLE_EQ(0, core::Distance(**i, **j));
  }

  core::RTreeWrapper<core::Line2> lines;
  lines.insert({{1, 2}, {3, 4}});
  lines.insert({{2, 3}, {4, 5}});
}

TEST(AlgorithmsTest, MergeNearRTreeEfficiency) {
  std::list<core::Vec4> arr1;
  std::srand(0);
  arr1.resize(5000);
  std::generate(arr1.begin(), arr1.end(), []() {
    return core::Vec4(std::rand() % 100, std::rand() % 100, std::rand() % 100,
                      std::rand() % 100);
  });
  double thres = 2;
  std::vector<decltype(arr1.begin())> gBegins;
  gBegins.reserve(arr1.size() / 2);
  core::MergeNearRTree(std::begin(arr1), std::end(arr1),
                       std::back_inserter(gBegins), std::false_type(), thres);
  std::cout << gBegins.size() << std::endl;
}

TEST(AlgorithmsTest, MergeNearNaiveEfficiency) {
  std::list<core::Vec4> arr1;
  std::srand(0);
  arr1.resize(5000);
  std::generate(arr1.begin(), arr1.end(), []() {
    return core::Vec4(std::rand() % 100, std::rand() % 100, std::rand() % 100,
                      std::rand() % 100);
  });
  double thres = 2;
  std::vector<decltype(arr1.begin())> gBegins;
  gBegins.reserve(arr1.size() / 2);
  core::MergeNearNaive(std::begin(arr1), std::end(arr1),
                       std::back_inserter(gBegins), std::false_type(), thres);
  std::cout << gBegins.size() << std::endl;
}

TEST(AlgorithmsTest, MinimumSpanningTree) {
  std::vector<int> verts = {0, 1, 2, 3, 4, 5};
  std::vector<int> edges = {0, 1, 2, 3, 4, 5, 6, 7};
  struct EdgeProperty {
    int fromv, tov;
    double w;
  };
  std::vector<EdgeProperty> edgeProperties = {
      {0, 1, 0.1}, {1, 2, 0.2}, {0, 2, 0.5}, {0, 5, 0.2},
      {5, 4, 0.7}, {2, 4, 0.3}, {3, 4, 0.1}, {2, 3, 0.5}};

  std::vector<int> MST;
  MST.reserve(5);
  core::MinimumSpanningTree(
      verts.begin(), verts.end(), edges.begin(), edges.end(),
      std::back_inserter(MST),
      [&edgeProperties](int e) { // get vertices of edge
        return std::make_pair(edgeProperties[e].fromv, edgeProperties[e].tov);
      },
      [&edgeProperties](int e1, int e2) { // compare weights of edges
        return edgeProperties[e1].w < edgeProperties[e2].w;
      });

  std::vector<int> correctMST = {0, 1, 3, 5, 6};
  EXPECT_TRUE(std::is_permutation(MST.begin(), MST.end(), correctMST.begin()));
}

TEST(AlgorithmsTest, MinimumSpanningTree2) {
  std::vector<int> verts = {0, 1, 2, 3, 4, 5, 6};
  std::vector<int> edges = {0, 1, 2, 3, 4, 5, 6, 7};
  struct EdgeProperty {
    int fromv, tov;
    double w;
  };
  std::vector<EdgeProperty> edgeProperties = {
      {0, 1, 0.1}, {1, 2, 0.5}, {2, 3, 0.2}, {0, 3, 0.6}, {0, 2, 0.2},

      {4, 5, 0.3}, {4, 6, 0.8}, {5, 6, 0.2}};

  std::vector<int> MST;
  MST.reserve(5);
  core::MinimumSpanningTree(
      verts.begin(), verts.end(), edges.begin(), edges.end(),
      std::back_inserter(MST),
      [&edgeProperties](int e) { // get vertices of edge
        return std::make_pair(edgeProperties[e].fromv, edgeProperties[e].tov);
      },
      [&edgeProperties](int e1, int e2) { // compare weights of edges
        return edgeProperties[e1].w < edgeProperties[e2].w;
      });

  std::vector<int> correctMST = {0, 4, 2, 5, 7};
  EXPECT_TRUE(std::is_permutation(MST.begin(), MST.end(), correctMST.begin()));
}

TEST(AlgorithmsTest, DFS_CC) {
  std::vector<int> verts = {0, 1, 2, 3, 4, 5, 6};
  struct EdgeProperty {
    int fromv, tov;
    double w;
  };
  std::vector<EdgeProperty> edgeProperties = {
      {0, 1, 0.1}, {1, 2, 0.5}, {2, 4, 0.2}, {0, 4, 0.6}, {0, 2, 0.2},

      {3, 5, 0.3}, {3, 6, 0.8}, {5, 6, 0.2}};

  struct Data {
    std::shared_ptr<std::vector<int>> vntable;
    int vid;
  };
  auto compData = [](const Data &a, const Data &b) {
    return a.vntable == b.vntable && a.vid == b.vid;
  };
  auto getValue = [](const Data &cdata) -> int {
    return (*(cdata.vntable))[cdata.vid];
  };
  auto setToNext = [](Data &cdata) { cdata.vid++; };

  auto vNeighborsContainerGetter = [&verts, &edgeProperties, &compData,
                                    &getValue, &setToNext](int v) {
    std::vector<int> vns;
    for (auto &edge : edgeProperties) {
      if (edge.fromv == v)
        vns.push_back(edge.tov);
      if (edge.tov == v)
        vns.push_back(edge.fromv);
    }
    return vns;
  };

  std::vector<int> visitedVids;
  core::DepthFirstSearch(verts.begin(), verts.end(), vNeighborsContainerGetter,
                         [&visitedVids](int vid) {
                           visitedVids.push_back(vid);
                           return true;
                         });
  std::vector<int> correctVisitedVids = {0, 1, 2, 4, 3, 5, 6};
  EXPECT_TRUE(std::equal(visitedVids.begin(), visitedVids.end(),
                         correctVisitedVids.begin()));

  std::vector<int> ccids;
  int ccnum = core::ConnectedComponents(
      verts.begin(), verts.end(), vNeighborsContainerGetter,
      [&ccids](int vid, int cid) { ccids.push_back(cid); });
  std::vector<int> correctCCids = {0, 0, 0, 0, 1, 1, 1};

  EXPECT_EQ(2, ccnum);
  EXPECT_TRUE(std::equal(ccids.begin(), ccids.end(), correctCCids.begin()));
}

TEST(AlgorithmsTest, TopologicalSort) {
  {
    std::vector<int> verts = {0, 1, 2, 3, 4, 5, 6};
    std::random_shuffle(verts.begin(), verts.end());
    struct Edge {
      int from, to;
    };
    std::vector<Edge> edges = {{0, 1}, {0, 2}, {1, 3}, {2, 4}, {1, 6}, {4, 5}};
    std::vector<int> sortedVerts;
    core::TopologicalSort(verts.begin(), verts.end(),
                          std::back_inserter(sortedVerts), [&edges](int vert) {
                            std::vector<int> predecessors;
                            for (auto &e : edges) {
                              if (e.to == vert)
                                predecessors.push_back(e.from);
                            }
                            return predecessors;
                          });
    for (auto &e : edges) {
      auto fromPos = std::find(sortedVerts.begin(), sortedVerts.end(), e.from) -
                     sortedVerts.begin();
      auto toPos = std::find(sortedVerts.begin(), sortedVerts.end(), e.to) -
                   sortedVerts.begin();
      ASSERT_TRUE(fromPos <= toPos);
    }
  }

  {
    std::vector<int> verts(1000);
    std::iota(verts.begin(), verts.end(), 0);
    std::random_shuffle(verts.begin(), verts.end());
    struct Edge {
      int from, to;
    };
    std::vector<Edge> edges(1000);
    std::generate(edges.begin(), edges.end(), [&verts]() {
      int v1 = rand() % verts.size();
      int v2 = rand() % verts.size();
      return v1 < v2 ? Edge{v1, v2} : Edge{v2, v1};
    });
    std::vector<int> sortedVerts;
    core::TopologicalSort(verts.begin(), verts.end(),
                          std::back_inserter(sortedVerts), [&edges](int vert) {
                            std::vector<int> predecessors;
                            for (auto &e : edges) {
                              if (e.to == vert)
                                predecessors.push_back(e.from);
                            }
                            return predecessors;
                          });
    for (auto &e : edges) {
      auto fromPos = std::find(sortedVerts.begin(), sortedVerts.end(), e.from) -
                     sortedVerts.begin();
      auto toPos = std::find(sortedVerts.begin(), sortedVerts.end(), e.to) -
                   sortedVerts.begin();

      ASSERT_TRUE(fromPos <= toPos);
    }
  }
}