#include "slog.h"

__attribute__((noinline))void foo() {
  LOG_INFO("log2");
}
  uint64_t now() {
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000'000'000 + ts.tv_nsec;
  }

int main(int, char**) {
  slog::initLogging();
  auto t1 = now();
  int cnt = 10;
  for (int i = 0; i < cnt; ++i) {
    LOG_INFO("x {}", "xx");
  }
  auto t2 = now();
  std::cout << (t2 -t1)/(double)cnt << std::endl;
}
