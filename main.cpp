#include "slog.h"

__attribute__((noinline))void foo() {
  LOG_IMPL("log2");
}

int main(int, char**) {
  strade::initLogging();
  LOG_IMPL("log2 {0}, {1}", 12, 13);

  strade::g_log.poll();
}
