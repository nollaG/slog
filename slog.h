#pragma once

#include <fmt/format.h>

#include <sstream>
#include <vector>
#include <string_view>
#include <string>
#include <atomic>
#include <iostream>

#include <type_traits>

namespace strade {

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
#define PRIV_HPPF_TSTR32(i,s)   ::strade::at8(i+(8*0),s),::strade::at8(i+(8*1),s),::strade::at8(i+(8*2),s),::strade::at8(i+(8*3),s)
#define PRIV_HPPF_TSTR128(i,s)  PRIV_HPPF_TSTR32(i+(32*0),s),PRIV_HPPF_TSTR32(i+(32*1),s),PRIV_HPPF_TSTR32(i+(32*2),s),PRIV_HPPF_TSTR32(i+(32*3),s)
#define PRIV_HPPF_TSTR512(i,s)  PRIV_HPPF_TSTR128(i+(128*0),s),PRIV_HPPF_TSTR128(i+(128*1),s),PRIV_HPPF_TSTR128(i+(128*2),s),PRIV_HPPF_TSTR128(i+(128*3),s)
#define PRIV_HPPF_TSTR1024(i,s) PRIV_HPPF_TSTR512(i+(512*0),s),PRIV_HPPF_TSTR512(i+(512*1),s)
#define PRIV_HPPF_TSTR(s)       ::strade::strpack<PRIV_HPPF_TSTR1024(0,s)>

template <typename T>
struct ArgPacker;

template <typename T>
struct PrimPacker {
  constexpr static size_t size(T)       { return sizeof(T); }
  static void write(char*& buffer, T x) { *(T*)buffer = x; buffer += sizeof(T); }
  static T    read(char*& buffer)       { T x = *(T*)buffer; buffer += sizeof(T); return x; }
};


template <typename ... Ts>
struct ArgsPacker;

#define DEFINE_PRIM_PACKER(T) template<> struct ArgPacker<T> : public PrimPacker<T> {};

DEFINE_PRIM_PACKER(bool);
DEFINE_PRIM_PACKER(char);
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
struct ArgsPacker<> {
  constexpr static size_t size(){ return 0; }
  static void  write(char*& buffer) {}
};

template <typename T, typename ... Ts>
struct ArgsPacker<T, Ts...> {
  static size_t size(T&& arg, Ts&& ... args) { return ArgPacker<std::decay_t<T>>::size(std::forward<T>(arg)) + ArgsPacker<Ts...>::size(std::forward<Ts>(args)...); }
  static void write(char*& buffer, T&& arg, Ts&& ... args) { ArgPacker<std::decay_t<T>>::write(buffer, std::forward<T>(arg)); ArgsPacker<Ts...>::write(buffer, std::forward<Ts>(args)...); }
};

template <typename Result, int I, typename ... Ts>
struct ArgsDecoderImpl;

template <typename Result, int I, typename T, typename ... Ts>
struct ArgsDecoderImpl<Result, I, T, Ts...> {
  static void prepare(Result& result, char*& buffer) {
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
  static std::tuple<Ts...> prepare(char*& buffer) {
    using Result = std::tuple<Ts...>;
    Result result;
    ArgsDecoderImpl<Result, 0, Ts...>::prepare(result, buffer);
    return result;
  }
};


template <typename ... Ts>
std::string formatLogMsg(const std::string& fmt, char* buffer) {
  const auto& params = ArgsDecoder<Ts...>::prepare(buffer);
  return std::apply([=](const auto&... args)->std::string {
      return fmt::format(fmt, args...); }, params);
}

template <typename ... Args>
void writeArgs(char* buffer, Args&& ... args) {
  ArgsPacker<Args...>::write(buffer, std::forward<Args>(args)...);
}

struct LogMsg {
  uint32_t id;
  uint64_t timestamp;
  char     buffer[512];
};

struct StmtStore {
  struct Stmt {
    std::string filename;
    uint32_t    lineno;
    std::function<std::string(char*)> formatter;

    std::string apply(LogMsg* msg) {
      std::ostringstream os;
      os << '[' << filename << ':' << lineno << "] " << formatter(msg->buffer);
      return os.str();
    }
  };


  template <typename ... Args>
  uint32_t addStmt(const std::string& filename, uint32_t lineno, const std::string& fmt) {
    Stmt s {
      .filename = filename,
      .lineno   = lineno,
      .formatter = [fmt](char* buffer) {
        return formatLogMsg<Args...>(fmt, buffer);
      }
    };
    _stmts.push_back(s);
    return _stmts.size() - 1;
  }

  std::vector<Stmt> _stmts;

};

extern StmtStore g_logStmtStore;

template <typename ... Args>
std::tuple<Args...> makeLogArgsTypes(Args&& ... args);

template <typename File, uint32_t Line, typename FormatStr, typename ArgsType>
struct LogStatement;

template <typename File, uint32_t Line, typename FormatStr, typename ... Args>
struct LogStatement<File, Line, FormatStr, std::tuple<Args...>> {
  static uint32_t id;
};

template <typename File, uint32_t Line, typename FormatStr, typename ... Args>
uint32_t LogStatement<File, Line, FormatStr, std::tuple<Args...>>::id = g_logStmtStore.addStmt<Args...>(File::str(), Line, FormatStr::str());

class LogQueue {
  public:
    LogMsg* getWrite() {
      while(true) {
        if (_widx.load(std::memory_order_relaxed) - _ridx.load(std::memory_order_acquire) < _capacity) {
          return &_buffer[_widx.load(std::memory_order_relaxed) % _capacity];
        }
      }
    }
    
    void commitWrite() {
      _widx.fetch_add(1, std::memory_order_release);;
    }

    LogMsg* getRead() {
      if (_ridx.load(std::memory_order_relaxed) < _widx.load(std::memory_order_acquire)) {
        return &_buffer[_ridx.load(std::memory_order_relaxed) % _capacity];
      }
      return nullptr;
    }

    void commitRead() {
      _ridx.fetch_add(1, std::memory_order_release);;
    }

  private:
    size_t _capacity { 1000 };
    std::atomic<size_t> _widx { 0 };
    std::atomic<size_t> _ridx { 0 };
    LogMsg _buffer[1000];
};

class Log {
  public:
    void poll() {
      for (auto q : _qs) {
        while(auto msg = q->getRead()) {
          std::cout << g_logStmtStore._stmts[msg->id].apply(msg) << std::endl;
          q->commitRead();
        }
      }
    }

    void registerLogQueue(LogQueue* q) {
      _qs.push_back(q);
    }

  private:
    std::vector<LogQueue*> _qs;
};

extern Log g_log;
extern __thread  LogQueue* g_logQueue;

template <typename ... Args>
[[gnu::hot]] inline void logImpl(uint32_t id, Args&& ... args) {
  LogMsg* msg = (LogMsg*)g_logQueue->getWrite();
  msg->id = id;
  writeArgs(msg->buffer, std::forward<Args>(args)...);
  g_logQueue->commitWrite();
}

#define LOG_IMPL(FMT, ARGS...) ::strade::logImpl(::strade::LogStatement<PRIV_HPPF_TSTR(__FILE__),__LINE__,PRIV_HPPF_TSTR(FMT),decltype(::strade::makeLogArgsTypes(ARGS))>::id __VA_OPT__(,) ARGS)


void initLogging();


}

