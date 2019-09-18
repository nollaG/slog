#include "slog.h"

namespace strade {
  StmtStore g_logStmtStore;
  Log g_log;

  __thread  LogQueue* g_logQueue;

  void initLogging() {
    g_logQueue = new LogQueue();
    g_log.registerLogQueue(g_logQueue);
  }
}
