#ifndef _WEB
#define _WEB

#include <WiFi.h>

#define TEST_SSID "HUAWEI-R1G96U"
#define TEST_PASSWORD "HW13924654101"

#define DEF_SERVER_IP       "j515e510.ala.cn-shenzhen.emqxsl.cn"
#define DEF_SERVER_PORT     8883
#define USER_NAME           "USER001"
#define USER_PASSWORD       "USER001"
#define TOPIC_SUB_HEADER    "dev"
#define TOPIC_PUB_HEADER    "term"

#define TOPIC_SUB_CMD       "cmd"
#define TOPIC_SUB_VIDEO     "video"



void webTask(void *webParameter);


#endif
