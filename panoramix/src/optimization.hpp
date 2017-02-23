#pragma once

#include "basic_types.hpp"
#include "containers.hpp"
#include "eigen.hpp"

namespace pano {
namespace experimental {

using namespace ::pano::core;

// SimulatedAnnealing
// - EnergyFunT: (StateT)->Scalar
// - TemperatureFunT: (int iter)->Scalar
// - NeighborsFunT: (StateT, int iter, ((StateT)->void) callback )
template <class StateT, class EnergyFunT, class TemperatureFunT,
          class NeighborsFunT, class RNG>
int SimulatedAnnealing(StateT &initialState, EnergyFunT energyFun,
                       TemperatureFunT temperatureFun,
                       NeighborsFunT neighborsFun, RNG &&rng,
                       double stopWhenEnergyIsLowerThan = 1e-5);

template <class EnergyFunT>
std::vector<bool> BeamSearch(size_t nconfigs, EnergyFunT energy_fun,
                             size_t beam_width);
}
}

////////////////////////////////////////////////
//// implementations
////////////////////////////////////////////////
namespace pano {
namespace experimental {
// SimulatedAnnealing
// - EnergyFunT: (StateT)->Scalar
// - TemperatureFunT: (int iter)->Scalar
// - NeighborsFunT: (StateT, int iter, ((StateT)->void) forEachNeighbor )
template <class StateT, class EnergyFunT, class TemperatureFunT,
          class NeighborsFunT, class RNG>
int SimulatedAnnealing(StateT &initialState, EnergyFunT energyFun,
                       TemperatureFunT temperatureFun,
                       NeighborsFunT neighborsFun, RNG &&rng,
                       double stopWhenEnergyIsLowerThan) {

  StateT &finalState = initialState;
  double finalEnergy = energyFun(initialState);

  std::uniform_real_distribution<double> dist(0.0, 1.0);
  StateT state = initialState;
  double energy = finalEnergy;
  int i = 0;

  while (true) {
    double temperature = temperatureFun(i);

    StateT newStateWithLowestEnergy;
    double lowestEnergy = std::numeric_limits<double>::infinity();
    bool hasNewState = false;
    neighborsFun(state, i, [&newStateWithLowestEnergy, &lowestEnergy,
                            &hasNewState, &energyFun](auto &&newState) {
      double newEnergy = energyFun(newState);
      if (newEnergy < lowestEnergy) {
        newStateWithLowestEnergy = newState;
        lowestEnergy = newEnergy;
        hasNewState = true;
      }
    });

    if (!hasNewState) {
      break;
    }

    double prob = 1.0;
    if (energy <= lowestEnergy) {
      prob = exp(-(lowestEnergy - energy) / temperature);
    }
    if (prob >= dist(rng)) {
      state = newStateWithLowestEnergy;
      energy = lowestEnergy;

      if (energy < finalEnergy) {
        finalState = state;
        finalEnergy = energy;

        if (finalEnergy < stopWhenEnergyIsLowerThan) {
          break;
        }
      }
    }

    ++i;
  }
  std::cout << "final energy: " << finalEnergy << '\n';
  return i;
}

template <class EnergyFunT>
std::vector<bool> BeamSearch(size_t nconfigs, EnergyFunT energy_fun,
                             size_t beam_width) {

  double cur_lowest_energy = std::numeric_limits<double>::infinity();
  std::vector<bool> cur_best_config(nconfigs, false);
  using MinHeap = core::MaxHeap<std::vector<bool>, double, std::greater<double>,
                                std::map<std::vector<bool>, int>>;
  MinHeap min_heap;
  min_heap.set(cur_best_config, energy_fun(cur_best_config));

  while (!min_heap.empty()) {
    if (min_heap.topScore() < cur_lowest_energy) {
      cur_lowest_energy = min_heap.topScore();
      cur_best_config = min_heap.top();
    }

    // collect next generation configs
    std::set<std::vector<bool>> next_generation;

    // only consider the childeren of current top <= beam_width nodes
    for (int i = 0; i < beam_width && !min_heap.empty(); i++) {
      const std::vector<bool> &cur_config = min_heap.top();
      assert(cur_config.size() == nconfigs);
      for (int k = 0; k < nconfigs; k++) {
        if (!cur_config[k]) {
          auto new_config = cur_config;
          new_config[k] = true;
          next_generation.insert(new_config);
        }
      }
      min_heap.pop();
    }

    // now we put the next generation configs into the min heap
    min_heap.clear();
    for (auto &config : next_generation) {
      double energy = energy_fun(config);
      if (energy > cur_lowest_energy) {
        continue;
      }
      min_heap.set(config, energy);
    }
  }

  return cur_best_config;
}
}
}