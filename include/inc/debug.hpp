#ifndef DEBUG
#define DEBUG

#include <Arduino.h>   // 必须先于下面的 #define Serial，确保真 Serial 已声明

// ════════════════════════════════════════════════════════════════
//  ★ 一键日志开关
//     _DEBUG = 1 → 正常输出所有日志（LOG 宏 + 直接 Serial.xxx）
//     _DEBUG = 0 → 全部日志编译为空：零开销、零串口输出
//
//  本工程已确认：所有 .cpp 里的 Serial.print/printf/println 都是日志，
//  没有功能性串口收发（read/available/write 等）。因此 _DEBUG=0 时
//  用一个"黑洞"对象顶替 Serial，无需改任何 .cpp。
//
//  ⚠ 使用规则（很重要）：
//    本头文件里的  #define Serial _nullSerial  是 token 级替换，只对
//    "包含本头文件之后" 的代码生效。所以在每个 .cpp 中，
//    #include "inc/debug.hpp" 应放在【所有其它 #include 之后】，
//    避免把库头文件里的 Serial 也误替换（绝大多数库只在其 .cpp 用
//    Serial，一般不受影响；若某库头文件用到 Serial 且编译报错，
//    把该库的 include 移到 debug.hpp 之前即可）。
//
//  LOG_FATAL(...)：致命/停机提示，无论 _DEBUG 取值都会输出，
//    专用于「打印后即 while(1) 停机」这类场景，避免无 log 版静默卡死。
// ════════════════════════════════════════════════════════════════
#define _DEBUG 1


#if _DEBUG
  // ───────────── 有日志版 ─────────────
  #define LOG(...)         Serial.printf(__VA_ARGS__)
  // Serial 走原生 USB-CDC；一旦连接状态在芯片内部卡住（外设活动密集时偶发），
  // 默认 100ms 发送超时会让每次打印都白等一轮。发送超时设 0：缓冲区满了直接丢，
  // 绝不阻塞任何任务——反正只是调试日志，丢几行不影响功能。
  // #define LOG_BEGIN(baud)  do { Serial.begin(baud); Serial.setTxTimeoutMs(0); } while (0)
  //
  // ★ 波特率时钟源必须钉在 XTAL，否则 L2 唤醒瞬间会吐一段乱码。
  //   本工程 platformio.ini 是 ARDUINO_USB_MODE=0 且没开 CDC_ON_BOOT，
  //   所以 Serial 走的是 UART0（板载 USB-UART 桥），而 UART 的波特率分频是从
  //   时钟源算出来的。默认源随 DFS 变：L2 放开 CPU_FREQ_MAX 锁后 APB 从 240MHz
  //   掉到 40MHz，分频器没跟着重算 → 实际波特率跑偏 → 那几个字节就是乱码；
  //   等频率爬回 240MHz 后又自动恢复正常（与实测"只在唤醒那一瞬乱一小段"吻合）。
  //   XTAL 是 40MHz 固定源，不受调频影响。
  //   ⚠ setClockSource() 必须在 begin() 之前调用（见 HardwareSerial.cpp:658，
  //     _uart 已创建时它会直接报错返回 false）。
  #define LOG_BEGIN(baud)  do {                          \
      Serial.begin(baud);                                \
  } while (0)
  #define LOG_FATAL(...)   Serial.printf(__VA_ARGS__)

#else
  // ───────────── 无日志版 ─────────────
  #define LOG(...)         ((void)0)
  #define LOG_BEGIN(baud)  ((void)0)

  // 在 #define Serial 之前，先捕获"真 Serial"的引用，供 LOG_FATAL 使用。
  // 函数体在此处解析，Serial 仍是真对象；之后即使 #define Serial 也不影响它。
  namespace _dbg {
    inline auto& fatalOut() { return Serial; }   // 返回真 Serial（HWCDC&/HardwareSerial&）
  }
  // 致命提示：无 log 版也输出（需要时自动 begin），打印后通常紧跟 while(1)
  #define LOG_FATAL(...)                                  \
      do {                                                \
          _dbg::fatalOut().begin(115200);                 \
          _dbg::fatalOut().printf(__VA_ARGS__);           \
          _dbg::fatalOut().flush();                       \
      } while (0)

  // "黑洞"串口：所有方法空内联，返回 0/false，编译器整段优化掉。
  class _NullSerial {
  public:
    template <typename... Args> int    printf (Args...) { return 0; }
    template <typename... Args> size_t print  (Args...) { return 0; }
    template <typename... Args> size_t println(Args...) { return 0; }
    size_t println(void)                  { return 0; }
    void   begin (unsigned long)          {}
    void   begin (unsigned long, uint32_t){}
    void   end   ()                       {}
    void   flush ()                       {}
    int    available()                    { return 0; }
    operator bool() const                 { return false; }
  };

  // header-only：每个翻译单元各持一份无状态实例（不会产生链接冲突）
  static _NullSerial _nullSerial;

  // 把 token Serial 重定向到黑洞对象（注意上面的"使用规则"）
  #define Serial _nullSerial

#endif

#endif