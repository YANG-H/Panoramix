#pragma once

#if (defined _MSC_VER)
#pragma warning(disable : 4503)
#pragma warning(disable : 4800)
#pragma warning(disable : 4819)
#elif (defined __GNUG__)
#pragma GCC diagnostic ignored "-Wenum-compare"
#endif

namespace pano {
namespace core {

#define ASSERT_OR_PANIC(cond)                                                  \
  if (!cond) {                                                                 \
    std::cerr << #cond << std::endl;                                           \
    std::exit(0);                                                              \
  }

#define CONCAT_IMPL(x, y) x##y
#define MACRO_CONCAT(x, y) CONCAT_IMPL(x, y)

#define FORCEINLINE __forceinline

// macros describing function implementations
#define FUNCTION_LINE_FILE_STRING                                              \
  ("[" __FUNCTION__ "][line: " + std::to_string(__LINE__) + "]["__FILE__       \
                                                            "]")

// not implemented error
#define NOT_IMPLEMENTED_YET(...)                                               \
  throw std::runtime_error(FUNCTION_LINE_FILE_STRING +                         \
                           " This feature has not yet been implemented! \n")

// not tested warning
#define NOT_TESTED_YET(...)                                                    \
  std::cout << (FUNCTION_LINE_FILE_STRING +                                    \
                "This feature has not yet been tested! \n")                    \
            << std::endl

// improvable here
#define IMPROVABLE_HERE(...)

// there are bugs here
#define THERE_ARE_BUGS_HERE(...)                                               \
  std::cout << (FUNCTION_LINE_FILE_STRING +                                    \
                "This feature may have BUGS, Check it: \""__VA_ARGS__          \
                "\"\n")                                                        \
            << std::endl

// there are bottlenecks here
#define THERE_ARE_BOTTLENECKS_HERE(...)                                        \
  std::cout << (FUNCTION_LINE_FILE_STRING +                                    \
                "This feature has improvable BOTTLENECKS! \n")                 \
            << std::endl

// should never be called error
#define SHOULD_NEVER_BE_CALLED(...)                                            \
  throw std::runtime_error(FUNCTION_LINE_FILE_STRING +                         \
                           "This feature should never be called! \n")

// should never be instanciated error
#define SHOULD_NEVER_BE_INSTANCIATED(...)                                      \
  static_assert(                                                               \
      pano::core::AlwaysFalse<__VA_ARGS__>::value,                             \
      FUNCTION_LINE_FILE_STRING +                                              \
          "This feature should never be instanciated by compiler! \n")

#define LOG(...)                                                               \
  std::cout << "[Log] #######" << (std::string("  ") + __VA_ARGS__)            \
            << " #######" << std::endl
#define WARNNING(...)                                                          \
  std::cout << "[WARNNING!] #######" << (std::string("  ") + __VA_ARGS__)      \
            << " #######" << std::endl
}
}
