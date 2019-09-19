#include "slog.h"

__attribute__((noinline))void foo() {
  LOG_INFO("log2");
}

int main(int, char**) {
  strade::initLogging();
  strade::g_log.runLoggingThread();
  LOG_INFO("log1");
  LOG_ERROR("log2");
}
