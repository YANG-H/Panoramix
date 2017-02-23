#pragma once

#include "basic_types.hpp"

namespace pano {
namespace core {
struct FactorCategory {
  using CostFunction = std::function<double(const std::vector<int> &var_labels,
                                            void *additional_data)>;
  CostFunction cost;
  double c_alpha;
};
struct VarCategory {
  size_t nlabels;
  double c_i;
};

// factor graph
class FactorGraph {
public:
  void reserveFactorCategories(size_t cap);
  void reserveVarCategories(size_t cap);

  // returns var_cat
  int addVarCategory(size_t nlabels, double c_i);
  // returns factor_cat
  int addFactorCategory(FactorCategory::CostFunction cost, double c_alpha);
  template <class CostFunT>
  auto addFactorCategory(CostFunT cost, double c_alpha)
      -> decltype(cost(std::declval<std::vector<int>>()), (int)0) {
    return addFactorCategory(
        [cost](const std::vector<int> &var_labels, void *) -> double {
          return cost(var_labels);
        },
        c_alpha);
  }

  void reserveFactors(size_t cap);
  void reserveVars(size_t cap);

  // returns var
  int addVar(int var_cat);
  // returns factor
  int addFactor(int factor_cat, const std::vector<int> &vars);
  int addFactor(int factor_cat, std::vector<int> &&vars);
  int addFactor(int factor_cat, std::initializer_list<int> vars);
  template <class VarIterT>
  int addFactor(int factor_cat, VarIterT vars_begin, VarIterT vars_end) {
    return addFactor(factor_cat, std::vector<int>(vars_begin, vars_end));
  }

  inline size_t nvars() const { return _var2cat.size(); }
  inline size_t nfactors() const { return _factor2cat.size(); }

  void clear();
  bool valid() const;

  double cost(const std::vector<int> &var_labels,
              void *additional_data = nullptr) const;

  // convex belief propagation
  std::vector<int>
  solve(int max_epoch, int inner_loop_num = 10,
        std::function<bool(int epoch, double energy, double denergy,
                           const std::vector<int> &cur_best_var_labels)>
            callback = nullptr,
        void *additional_data = nullptr) const;

  template <class CallbackFunT>
  auto solve(int max_epoch, int inner_loop_num, CallbackFunT callback,
             void *additional_data = nullptr) const
      -> decltype(callback(0, 0.0), std::vector<int>()) {
    return solve(
        max_epoch, inner_loop_num,
        [callback](int epoch, double energy, double denergy,
                   const std::vector<int> &cur_best_var_labels) -> bool {
          return callback(epoch, energy);
        },
        additional_data);
  }

private:
  std::vector<FactorCategory> _factorCats;
  std::vector<VarCategory> _varCats;

  std::vector<int> _var2cat;
  std::vector<std::set<int>> _var2factors;

  std::vector<int> _factor2cat;
  std::vector<std::vector<int>> _factor2vars;
};
}
}
