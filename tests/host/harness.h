// Micro test harness for the va_core host tests. No external dependencies:
// TEST() registers a function, CHECK/CHECK_EQ record failures with file:line,
// run_all_tests() prints a per-test verdict and returns non-zero on failure.
#pragma once

#include <cstdio>
#include <vector>

struct TestCase {
  const char *name;
  void (*fn)();
};

inline std::vector<TestCase> &test_registry() {
  static std::vector<TestCase> t;
  return t;
}

inline int &failure_count() {
  static int f = 0;
  return f;
}

#define TEST(name)                                                                     \
  static void name();                                                                  \
  [[maybe_unused]] static const bool name##_registered =                               \
      (test_registry().push_back({#name, &name}), true);                               \
  static void name()

#define CHECK(expr)                                                                  \
  do {                                                                               \
    if (!(expr)) {                                                                   \
      std::printf("    FAIL %s:%d: CHECK(%s)\n", __FILE__, __LINE__, #expr);         \
      failure_count()++;                                                             \
    }                                                                                \
  } while (0)

template <typename A, typename B>
inline void check_eq_impl(const A &a, const B &b, const char *as, const char *bs,
                          const char *file, int line) {
  if (!(a == b)) {
    std::printf("    FAIL %s:%d: CHECK_EQ(%s, %s) — values differ\n", file, line, as, bs);
    failure_count()++;
  }
}

#define CHECK_EQ(a, b) check_eq_impl((a), (b), #a, #b, __FILE__, __LINE__)

inline int run_all_tests() {
  int failed_tests = 0;
  for (const auto &t : test_registry()) {
    const int before = failure_count();
    t.fn();
    const bool ok = (failure_count() == before);
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", t.name);
    if (!ok)
      failed_tests++;
  }
  std::printf("%zu tests, %d failed (%d failed checks)\n", test_registry().size(),
              failed_tests, failure_count());
  return failure_count() == 0 ? 0 : 1;
}
