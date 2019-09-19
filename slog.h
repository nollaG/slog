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


namespace strade {

constexpr size_t LogMsgBufferSize = 64;
constexpr size_t LogMsgQueueSize  = 102400;

inline uint64_t rdtsc() {
  unsigned int lo, hi;
  __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
  return ((uint64_t)hi<<32) | lo;
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
#define SLOG_PRIV_HPPF_TSTR32(i,s)   ::strade::at8(i+(8*0),s),::strade::at8(i+(8*1),s),::strade::at8(i+(8*2),s),::strade::at8(i+(8*3),s)
#define SLOG_PRIV_HPPF_TSTR128(i,s)  SLOG_PRIV_HPPF_TSTR32(i+(32*0),s),SLOG_PRIV_HPPF_TSTR32(i+(32*1),s),SLOG_PRIV_HPPF_TSTR32(i+(32*2),s),SLOG_PRIV_HPPF_TSTR32(i+(32*3),s)
#define SLOG_PRIV_HPPF_TSTR512(i,s)  SLOG_PRIV_HPPF_TSTR128(i+(128*0),s),SLOG_PRIV_HPPF_TSTR128(i+(128*1),s),SLOG_PRIV_HPPF_TSTR128(i+(128*2),s),SLOG_PRIV_HPPF_TSTR128(i+(128*3),s)
#define SLOG_PRIV_HPPF_TSTR1024(i,s) SLOG_PRIV_HPPF_TSTR512(i+(512*0),s),SLOG_PRIV_HPPF_TSTR512(i+(512*1),s)
#define SLOG_PRIV_HPPF_TSTR(s)       ::strade::strpack<SLOG_PRIV_HPPF_TSTR1024(0,s)>

template <typename T>
struct ArgPacker;

template <typename T>
struct PrimPacker {
  using OutT = T;
  constexpr static size_t size(const T&) { return sizeof(T); }
  static void write(char*& buffer, T x)  { *(T*)buffer = x; buffer += sizeof(T); }
  static OutT read(char*& buffer)        { T x = *(T*)buffer; buffer += sizeof(T); return x; }
};


template <typename ... Ts>
struct ArgsPacker;

#define DEFINE_PRIM_PACKER(T) template<> struct ArgPacker<T> : public PrimPacker<T> {};

DEFINE_PRIM_PACKER(bool);
DEFINE_PRIM_PACKER(char);
DEFINE_PRIM_PACKER(double);
DEFINE_PRIM_PACKER(float);
DEFINE_PRIM_PACKER(int8_t);
DEFINE_PRIM_PACKER(int16_t);
DEFINE_PRIM_PACKER(int32_t);
DEFINE_PRIM_PACKER(int64_t);
DEFINE_PRIM_PACKER(uint8_t);
DEFINE_PRIM_PACKER(uint16_t);
DEFINE_PRIM_PACKER(uint32_t);
DEFINE_PRIM_PACKER(uint64_t);
DEFINE_PRIM_PACKER(std::string_view);

template <>
struct ArgPacker<std::string> {
  using OutT = std::string;
  static size_t size(const std::string& x) {
    return ArgPacker<size_t>::size(1) + x.size();
  }
  static void write(char*& buffer, const std::string& x) {
    ArgPacker<size_t>::write(buffer, x.size());
    memcpy(buffer, x.data(), x.size());
    buffer += x.size();
  }
  static std::string read(char*& buffer) {
    size_t size = ArgPacker<size_t>::read(buffer);
    std::string x(buffer, size);
    buffer += size;
    return x;
  }
};

template <>
struct ArgsPacker<> {
  constexpr static size_t size(){ return 0; }
  static void  write(char*& buffer) {}
};

template <typename T, typename ... Ts>
struct ArgsPacker<T, Ts...> {
  constexpr static size_t size(T&& arg, Ts&& ... args) { return ArgPacker<std::decay_t<T>>::size(std::forward<T>(arg)) + ArgsPacker<Ts...>::size(std::forward<Ts>(args)...); }
  static void write(char*& buffer, T&& arg, Ts&& ... args) { ArgPacker<std::decay_t<T>>::write(buffer, std::forward<T>(arg)); ArgsPacker<Ts...>::write(buffer, std::forward<Ts>(args)...); }
};

template <typename Result, int I, typename ... Ts>
struct ArgsDecoderImpl;

template <typename Result, int I, typename T, typename ... Ts>
struct ArgsDecoderImpl<Result, I, T, Ts...> {
  [[gnu::cold]] static void prepare(Result& result, char*& buffer) {
    std::get<I>(result) = ArgPacker<std::decay_t<T>>::read(buffer);
    ArgsDecoderImpl<Result, I+1, Ts...>::prepare(result, buffer);
  }
};

template <typename Result, int I>
struct ArgsDecoderImpl<Result, I> {
  static void prepare(Result& result, char*& buffer) {}
};

template <typename ... Ts>
struct ArgsDecoder {
  using Result = std::tuple<typename ArgPacker<Ts>::OutT...>;
  static std::tuple<Ts...> prepare(char*& buffer) {
    Result result;
    ArgsDecoderImpl<Result, 0, Ts...>::prepare(result, buffer);
    return result;
  }
};


template <typename ... Ts>
[[gnu::cold]] std::string formatLogMsg(const std::string& fmt, char* buffer) {
  const auto& params = ArgsDecoder<Ts...>::prepare(buffer);
  return std::apply([=](const auto&... args)->std::string {
      return fmt::format(fmt, args...); }, params);
}

template <typename ... Args>
[[gnu::hot]] void writeArgs(char* buffer, Args&& ... args) {
  if ((ArgPacker<std::decay_t<Args>>::size(args) + ...) > LogMsgBufferSize) {
    throw std::runtime_error("exceeded maximum log buffer size.");
  }
  ArgsPacker<Args...>::write(buffer, std::forward<Args>(args)...);
}

template <>
constexpr void writeArgs(char* buffer) { }

struct LogMsg {
  uint32_t id;
  uint64_t timestamp;
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

struct StmtStore {
  struct Stmt {
    std::string filename;
    uint32_t    lineno;
    LogLevel    level;
    std::function<std::string(char*)> formatter;

    std::string apply(LogMsg* msg, int tid) {
      static int pid = getpid();
      auto epoch = std::chrono::nanoseconds(msg->timestamp);
      auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::time_point(epoch));
      auto ns = msg->timestamp % 1000'000'000;
      return fmt::format("{:<5} [{:%F %H:%M:%S}.{:0>9}] [{}:{}] {} ({}:{})", to_string(level), fmt::localtime(t), ns, pid, tid, formatter(msg->buffer), filename, lineno);
    }
  };

  template <typename ... Args>
  uint32_t addStmt(LogLevel level, const std::string& filename, uint32_t lineno, const std::string& fmt) {
    Stmt s {
      .filename = filename.substr(filename.find_last_of("/\\") + 1),
      .lineno   = lineno,
      .level    = level,
      .formatter = [fmt](char* buffer) {
        return formatLogMsg<Args...>(fmt, buffer);
      }
    };
    _stmts.push_back(s);
    return _stmts.size() - 1;
  }

  std::vector<Stmt> _stmts;

};

inline StmtStore g_logStmtStore;

template <typename ... Args>
std::tuple<std::decay_t<Args>...> makeLogArgsTypes(Args&& ... args);

template <LogLevel Level, typename File, uint32_t Line, typename FormatStr, typename ArgsType>
struct LogStatement;

template <LogLevel Level, typename File, uint32_t Line, typename FormatStr, typename ... Args>
struct LogStatement<Level, File, Line, FormatStr, std::tuple<Args...>> {
  static uint32_t id;
};

template <LogLevel Level, typename File, uint32_t Line, typename FormatStr, typename ... Args>
uint32_t LogStatement<Level, File, Line, FormatStr, std::tuple<Args...>>::id = g_logStmtStore.addStmt<Args...>(Level, File::str(), Line, FormatStr::str());

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
          msg->timestamp = _baseTime + (msg->timestamp - _baseTick)*_nanos/_ticks;
          std::cout << g_logStmtStore._stmts[msg->id].apply(msg, q->tid()) << std::endl;
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

template <typename ... Args>
[[gnu::hot]] inline void logImpl(uint32_t id, Args&& ... args) {
  LogMsg* msg = (LogMsg*)g_logQueue->getWrite();
  msg->id = id;
  msg->timestamp = rdtsc();
  writeArgs(msg->buffer, std::forward<Args>(args)...);
  g_logQueue->commitWrite();
}

#define LOG_IMPL(LEVEL, FMT, ARGS...) ::strade::logImpl(::strade::LogStatement<LEVEL, SLOG_PRIV_HPPF_TSTR(__FILE__),__LINE__,SLOG_PRIV_HPPF_TSTR(FMT),decltype(::strade::makeLogArgsTypes(ARGS))>::id, ##ARGS)

#define LOG_INFO(FMT, ARGS...)  LOG_IMPL(::strade::LogLevel::INFO,  FMT, ##ARGS)
#define LOG_DEBUG(FMT, ARGS...) LOG_IMPL(::strade::LogLevel::DEBUG, FMT, ##ARGS)
#define LOG_ERROR(FMT, ARGS...) LOG_IMPL(::strade::LogLevel::ERROR, FMT, ##ARGS)

[[gnu::cold, gnu::noinline, gnu::weak]] void initLogging() {
  g_logQueue = new LogQueue();
  g_log.registerLogQueue(g_logQueue);
}

}
