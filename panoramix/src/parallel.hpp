#pragma once

#include <thread>

namespace pano {
namespace core {
template <class FunT> void ParallelRun(int n, int concurrency_num, FunT &&fun);
template <class FunT>
void ParallelRun(int n, int concurrency_num, int batch_num, FunT &&fun);
}
}

////////////////////////////////////////////////
//// implementations
////////////////////////////////////////////////
namespace pano {
namespace core {
template <class FunT> void ParallelRun(int n, int concurrency_num, FunT &&fun) {
  std::vector<std::thread> threads;
  threads.reserve(concurrency_num);
  for (int i = 0; i < n; i++) {
    threads.emplace_back(std::forward<FunT>(fun), i);
    if (threads.size() >= concurrency_num || i == n - 1) {
      for (auto &t : threads) {
        t.join();
      }
      threads.clear();
    }
  }
}

template <class FunT>
void ParallelRun(int n, int concurrency_num, int batch_num, FunT &&fun) {
  std::vector<std::thread> threads;
  threads.reserve(concurrency_num);
  for (int bid = 0; bid < n / batch_num + 1; bid++) {
    int bfirst = bid * batch_num;
    int blast = std::min(n, (bid + 1) * batch_num) - 1;
    std::cout << "processing " << bfirst << "-" << blast << "  with total " << n
              << std::endl;
    threads.emplace_back(
        [fun](int first, int last) {
          for (int i = first; i <= last; i++) {
            fun(i);
          }
        },
        bfirst, blast);
    if (threads.size() >= concurrency_num || bid == n / batch_num) {
      for (auto &t : threads) {
        t.join();
      }
      threads.clear();
    }
  }
}
}
}