#ifndef _MSG
#define _MSG

#include <stdint.h>

// main→web 队列传递的短命令串长度（"TAP" / "CFG_UPDATE" / "NEW_VOICE" 等）。
// V1_1.cpp 建队列、web.cpp 收、mic.cpp 发都用它，避免多处魔法数字 16 不一致越界。
#define WEB_MSG_LEN 16

typedef enum {
  NONE,
  MESSAGE_LCD,
  MESSAGE_WEB,
  MESSAGE_IMU,
} CommandIdToMain_t;

typedef struct {
  CommandIdToMain_t cmd;
  int         intVal;
  char        strVal[128];
} MessageToMain_t;

typedef enum {
  LCDMSG_STR,
  LCDMSG_PLAY,
  LCDMSG_PLAY_NONINT,

  LCDMSG_CFG_UPDATE

} CommandIdToLCD_t;

typedef enum {
  FREE_MOTION,
  MOTION_IDLE,
  MOTION_SHAKE,
  MOTION_TAP,
  MOTION_VIDEOSHOW,
  MOTION_SHOWUP,      // 开机
  MOTION_WINK,        // 拍一拍送达成功
  MOTION_STARTSLEEP,  // 进 L1（播完接 sleeping 循环）
  MOTION_SLEEPING,

} MOTION_t;

typedef struct {
  CommandIdToLCD_t cmd;
  MOTION_t motion;
  int intVal;
  char  strVal[128];
  int playTimes;
  bool interruptAble;
} MessageToLCD_t;

typedef struct {
  MOTION_t motion;
  char  strVal[128];

} MessageToWeb_t;

typedef enum {
  AUD_PLAY,
  AUD_PLAY_NOSYNC,
  AUD_STOP,
  AUD_GAIN,

  AUD_CFG_UPDATE,
} AUDCMD_t;

typedef struct {
  AUDCMD_t cmd;
  float newGain;

} MessageToAud_t;

typedef enum {
  MIC_START,
  MIC_STOP,
} MICCMD_t;

typedef struct {
  MICCMD_t cmd;

} MessageToMic_t;

typedef struct {
    int16_t roll_cd;    // 这里复用为 ax (mg)
    int16_t pitch_cd;   // 复用为 ay (mg)
    int16_t yaw_cd;     // 复用为 az (mg)
} PosSample_t;


#endif


