#include "../panoramix.unittest.hpp"
#include "optimization.hpp"

using namespace pano;
using namespace pano::experimental;

TEST(OptimizationTest, SimulatedAnnealing) {
  double solution = 0.0;
  std::default_random_engine rng;
  int niters =
      SimulatedAnnealing(solution, [](double s) { return (s - 1) * (s - 1); },
                         [](int iter) { return 1.0 / (iter + 1); },
                         [](double s, int iter, auto &&forEachNeighbor) {
                           if (iter < 100000) {
                             forEachNeighbor(s + 0.1 / (iter + 1));
                             forEachNeighbor(s - 0.1 / (iter + 1));
                           }
                         },
                         rng);
  ASSERT_TRUE(abs(solution - 1) < 0.1);
}