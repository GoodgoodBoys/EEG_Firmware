#ifndef _FS_HPP
#define _FS_HPP

#include <LittleFS.h>

// LittleFS 初始化（替换原 sdInit）
// formatOnFail=true: 首次或损坏时自动格式化
bool fsInit();

#endif
