#include "gtest/gtest.h"

#include "../src/ui.hpp"
#include "../src/file.hpp"

#include "panoramix.unittest.hpp"

int main(int argc, char *argv[], char *envp[]) {
  pano::gui::UI::SetCmdArgs(argc, argv, envp);
  pano::misc::MakeDir(PANORAMIX_TEST_DATA_DIR_STR);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
