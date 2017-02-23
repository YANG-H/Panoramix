#include "panorama_reconstruction.hpp"

int main_label(int argc, char **argv);
int main_run(int argc, char **argv);

int main(int argc, char **argv) {
  auto routine = &main_run;
  return routine(argc, argv);
}