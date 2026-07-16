#include "inc/fs.hpp"
#include "inc/debug.hpp"

bool fsInit()
{
    LOG("[FS] 挂载 LittleFS...\n");

    if (!LittleFS.begin(true)) {    // true = 挂载失败时格式化
        LOG("[FS] LittleFS 挂载失败！\n");
        return false;
    }

    LOG("[FS] LittleFS 已挂载: 总=%u KB  已用=%u KB  可用=%u KB\n",
        LittleFS.totalBytes() / 1024,
        LittleFS.usedBytes()  / 1024,
        (LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024);

    // 确保必要目录存在
    if (!LittleFS.exists("/def"))    LittleFS.mkdir("/def");
    if (!LittleFS.exists("/Config")) LittleFS.mkdir("/Config");

    return true;
}
