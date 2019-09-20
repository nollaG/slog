#pragma once

#include <fmt/format.h>
#include <fmt/chrono.h>

#include <sstream>
#include <vector>
#include <string_view>
#include <string>
#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>
#include <type_traits>

#include <time.h>
#include <unistd.h>

#include <sys/syscall.h>


namespace slog {

constexpr size_t LogMsgBufferSize = 64;
constexpr size_t LogMsgQueueSize  = 102400;

namespace {
  inline uint64_t rdtsc() {
    unsigned int lo, hi;
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    return ((uint64_t)hi<<32) | lo;
  }
}

[[gnu::cold,gnu::noinline,gnu::weak]] int gettid() {
  return syscall(SYS_gettid);
}

// basic compile-time strings
template <size_t N>
  constexpr char at(size_t i, const char (&s)[N]) {
    return (i < N) ? s[i] : '\0';
  }
template <size_t N>
  constexpr size_t at8S(size_t i, size_t k, const char (&s)[N]) {
    return (k==8) ? 0 : ((static_cast<size_t>(at(i+k,s))<<(8*k))|at8S(i,k+1,s));
  }
template <size_t N>
  constexpr size_t at8(size_t i, const char (&s)[N]) {
    return at8S(i, 0, s);
  }
template <size_t... pcs>
  struct strpack {
    static const char* str() {
      constexpr size_t msg[] = {pcs...};
      static_assert(msg[(sizeof(msg)/sizeof(msg[0]))-1] == 0, "compile-time string larger than internal max limit (this limit can be bumped in storage.H)");

      static const size_t smsg[] = {pcs...};
      return reinterpret_cast<const char*>(smsg);
    }
  };
#define SLOG_PRIV_HPPF_TSTR32(i,s)   ::slog::at8(i+(8*0),s),::slog::at8(i+(8*1),s),::slog::at8(i+(8*2),s),::slog::at8(i+(8*3),s)
#define SLOG_PRIV_HPPF_TSTR128(i,s)  SLOG_PRIV_HPPF_TSTR32(i+(32*0),s),SLOG_PRIV_HPPF_TSTR32(i+(32*1),s),SLOG_PRIV_HPPF_TSTR32(i+(32*2),s),SLOG_PRIV_HPPF_TSTR32(i+(32*3),s)
#define SLOG_PRIV_HPPF_TSTR512(i,s)  SLOG_PRIV_HPPF_TSTR128(i+(128*0),s),SLOG_PRIV_HPPF_TSTR128(i+(128*1),s),SLOG_PRIV_HPPF_TSTR128(i+(128*2),s),SLOG_PRIV_HPPF_TSTR128(i+(128*3),s)
#define SLOG_PRIV_HPPF_TSTR1024(i,s) SLOG_PRIV_HPPF_TSTR512(i+(512*0),s),SLOG_PRIV_HPPF_TSTR512(i+(512*1),s)
#define SLOG_PRIV_HPPF_TSTR(s)       ::slog::strpack<SLOG_PRIV_HPPF_TSTR1024(0,s)>


struct LogMsg {
  char     buffer[LogMsgBufferSize];
};

enum LogLevel {
  DEBUG = 1,
  INFO  = 2,
  WARN  = 3,
  ERROR = 4
};

[[gnu::cold, gnu::noinline, gnu::weak]] std::string_view to_string(LogLevel level) {
  switch (level) {
    case LogLevel::DEBUG: return "DEBUG";
    case LogLevel::INFO:  return "INFO";
    case LogLevel::WARN:  return "WARN";
    case LogLevel::ERROR: return "ERROR";
    default: return "UK";
  }
}

struct LogClosure {
  virtual ~LogClosure() {}

  virtual std::string apply() const = 0;
  virtual const std::string& filename() const = 0;
  virtual LogLevel level() const = 0;
  virtual uint32_t line( ) const = 0;
  virtual uint64_t timestamp() const = 0;
};

template <LogLevel Level, typename File, uint32_t Line, typename FormatStr, typename ArgsTuple>
struct LogClosureT;

//LogClosure close over:
//Compile time:
//  level, filanem, lineno, format
//Runtime:
//  timestamp, args payload
template <LogLevel Level, typename File, uint32_t Line, typename FormatStr, typename ... Args>
class LogClosureT<Level, File, Line, FormatStr, std::tuple<Args...>> : public LogClosure {
  public:
    [[gnu::hot]] inline LogClosureT(uint64_t ts, Args&& ... args)
      : _ts(ts), _data(std::forward<Args>(args)...)
    {
    }

    std::string apply() const final {
      static std::string fmt = FormatStr::str();
      return std::apply([=](const auto&... args)->std::string {
          return fmt::format(fmt, args...); }, _data);
    }

    const std::string& filename() const final {
      static std::string x = std::string(File::str()).substr(std::string(File::str()).find_last_of("/\\") + 1);
      return x;
    }

    LogLevel level()     const final { return Level; }
    uint32_t line( )     const final { return Line;  } 
    uint64_t timestamp() const final { return _ts;  }

  private:
    uint64_t _ts;
    std::tuple<std::decay_t<Args>...> _data;
};

class LogQueue {
  public:
    [[gnu::cold, gnu::noinline]] LogQueue() {
      for (size_t i = 0; i < LogMsgQueueSize; ++i) {
        memset(&_buffer[i], 1, sizeof(_buffer[i]));
      }
      _tid = gettid();
    }

    int tid() const { return _tid; }

    [[gnu::hot]] LogMsg* getWrite() {
      while(true) {
        if (_widx - _ridx < LogMsgQueueSize) {
          return &_buffer[_widx % LogMsgQueueSize];
        }
      }
    }
    
    [[gnu::hot]] void commitWrite() {
      asm volatile ("" : : : "memory");
      ++_widx;
    }

    [[gnu::cold, gnu::noinline]] LogMsg* getRead() {
      if (_ridx < _widx) {
        return &_buffer[_ridx % LogMsgQueueSize];
      }
      return nullptr;
    }

    [[gnu::cold, gnu::noinline]] void commitRead() {
      asm volatile ("" : : : "memory");
      ++_ridx;
    }

  private:
    int _tid;
    volatile size_t _widx { 0 };
    char padding1[64];
    volatile size_t _ridx { 0 };
    char padding2[64];
    LogMsg _buffer[LogMsgQueueSize];
};

class Log {
  public:
    [[gnu::cold, gnu::noinline]] Log() {
      calibrate();
      runLoggingThread();
    }

    [[gnu::cold, gnu::noinline]] ~Log() {
      stop();
      if (_thread.joinable()) {
        _thread.join();
      }
      pollOnce();
    }

    [[gnu::cold, gnu::noinline]] void stop() {
      _stop = true;
    }

    std::string format(LogClosure* closure, int tid) {
      using namespace fmt::literals;
      static int pid = getpid();
      auto timestamp = _baseTime + (closure->timestamp() - _baseTick) * _nanos / _ticks;
      auto epoch = std::chrono::nanoseconds(timestamp);
      auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::time_point(epoch));
      auto ns = timestamp % 1000'000'000;
      auto tm = fmt::localtime(t);
      return fmt::format("{level:<5} [{time:%F %H:%M:%S}.{nano:0>9}] [{pid}:{tid}] {text} ({file}:{line})",
          "level"_a = to_string(closure->level()),
          "time"_a  = tm,
          "nano"_a  = ns,
          "pid"_a   = pid,
          "tid"_a   = tid,
          "text"_a  = closure->apply(),
          "file"_a  = closure->filename(),
          "line"_a  = closure->line()
      );
    }

    [[gnu::cold, gnu::noinline]] void calibrate() {
      auto r1 = now();
      auto t1 = rdtsc();
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      auto r2 = now();
      auto t2 = rdtsc();

      _nanos = r2 - r1;
      _ticks = t2 - t1;
      _baseTime = r2;
      _baseTick = t2;
    }

    void pollOnce() {
      std::lock_guard<std::mutex> guard(_lock);
      for (auto q : _qs) {
        while(auto msg = q->getRead()) {
          auto closure = (LogClosure*)msg->buffer;
          std::cout << format(closure, q->tid()) << std::endl;
          closure->~LogClosure();
          q->commitRead();
        }
      }
    }

    void registerLogQueue(LogQueue* q) {
      std::lock_guard<std::mutex> guard(_lock);
      _qs.push_back(q);
    }
  private:
    void runLoggingThread() {
      _thread = std::thread([&] {
        while(!_stop) {
          pollOnce();
        }
      });
    }

    uint64_t now() {
      timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      return ts.tv_sec * 1000'000'000 + ts.tv_nsec;
    }

  private:
    uint64_t _nanos, _ticks, _baseTime, _baseTick;
    std::mutex             _lock;
    volatile bool          _stop { false };
    std::thread            _thread;
    std::vector<LogQueue*> _qs;
};

inline Log g_log;
inline __thread  LogQueue* g_logQueue = nullptr;

template <LogLevel Level, typename File, uint32_t Line, typename FormatStr, typename ... Args>
[[gnu::hot]] inline void logImpl(Args&& ... args) {
  LogMsg* msg = (LogMsg*)g_logQueue->getWrite();
  using Closure = LogClosureT<Level, File, Line, FormatStr, std::tuple<Args...>>;
  static_assert(sizeof(LogClosureT<Level, File, Line, FormatStr, std::tuple<Args...>>) <= LogMsgBufferSize, "Logging args is too large");
  new (msg->buffer) Closure(rdtsc(), std::forward<Args>(args)...);
  g_logQueue->commitWrite();
}

#define LOG_IMPL(LEVEL, FMT, ARGS...) ::slog::logImpl<LEVEL, SLOG_PRIV_HPPF_TSTR(__FILE__),__LINE__,SLOG_PRIV_HPPF_TSTR(FMT)>(ARGS)

#define LOG_INFO(FMT, ARGS...)  LOG_IMPL(::slog::LogLevel::INFO,  FMT, ##ARGS)
#define LOG_DEBUG(FMT, ARGS...) LOG_IMPL(::slog::LogLevel::DEBUG, FMT, ##ARGS)
#define LOG_ERROR(FMT, ARGS...) LOG_IMPL(::slog::LogLevel::ERROR, FMT, ##ARGS)

[[gnu::cold, gnu::noinline, gnu::weak]] void initLogging() {
  g_logQueue = new LogQueue();
  g_log.registerLogQueue(g_logQueue);
}

}
