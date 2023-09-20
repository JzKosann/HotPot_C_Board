////
//// Created by ShiF on 2023/9/14.
////
//
//#include "log.h"
//#include "stdarg.h"
//#include "string.h"
//#include "stdio.h"
//#include "log_port.h"
//
//typedef enum {
//    LOG_FALSE,
//    LOG_TRUE
//} LOG_STATUS_E;
//
//#define LOG_USE_LEVEL  1
//#define LOG_FILE_NAME  0
//#define LOG_FUNC_NAME  0
//#define LOG_LINE_NUM   0
//#define LOG_SHOW_TIME  0
//#define LOG_USE_COLOR  1
//
//#define LOG_ONCE_ONLY  0
//#define LOG_LEVEL_SINGLE 0
//
//static LOG_LEVEL_E LOG_OUTPUT_LEV = LOG_LEV_E;
//static LOG_PORT_E logUsePort = LOG_SERIAL;
////static LOG_PORT_E logUsePort = LOG_SDCARD;
//
////一些ANSI颜色控制码
//#define LOG_COLOR_END           "\033"
//#define LOG_COLOR_BLACK         "\033[30m"
//#define LOG_COLOR_DEEP_RED      "\033[31m"
//#define LOG_COLOR_LIGHT_GREEN   "\033[32m"
//#define LOG_COLOR_BROWN         "\033[33m"
//#define LOG_COLOR_LIGHT_BLUE    "\033[34m"
//#define LOG_COLOR_PURPLE        "\033[35m"
//#define LOG_COLOR_SKY_BLUE      "\033[36m"
//#define LOG_COLOR_GREY          "\033[37m"
//#define LOG_COLOR_LIGHT_BLACK   "\033[1;30m"
//#define LOG_COLOR_RED           "\033[1;31m"
//#define LOG_COLOR_GREEN         "\033[1;32m"
//#define LOG_COLOR_YELLOW        "\033[1;33m"
//#define LOG_COLOR_BLUE          "\033[1;34m"
//#define LOG_COLOR_PINK          "\033[1;35m"
//#define LOG_COLOR_CYAN          "\033[1;36m"
//#define LOG_COLOR_WHITE         "\033[1;37m"
//
////static char* LOG_COLOR_LIST[5] =
////    {LOG_COLOR_BLACK,
////     LOG_COLOR_RED,
////     LOG_COLOR_PINK,
////     LOG_COLOR_CYAN,
////     LOG_COLOR_BLUE
////    };
//
//static char *LOG_COLOR_LIST[5] =
//        {LOG_COLOR_BROWN,
//         LOG_COLOR_DEEP_RED,
//         LOG_COLOR_GREEN,
//         LOG_COLOR_LIGHT_BLACK,
//         LOG_COLOR_SKY_BLUE
//        };
//
//static char LOG_STR_BUF[LOG_BUF_MAX_LENGTH] = {0};
//static int LOG_STR_BUF_P = 0;
//static LOG_STATUS_E LOG_UPDATE_FLAG = LOG_FALSE;
//
//static void LOG_AddTime(uint64_t logTime) {
//    int logSec, logMin, logHour = 0;
//    logSec = (int) (logTime % 60);
//    logMin = (int) (logTime / 60);
//    logHour = (int) (logMin / 60);
//    logMin = (int) (logMin % 60);
//    LOG_STR_BUF_P = (int) (strlen(LOG_STR_BUF));
//    snprintf(LOG_STR_BUF + LOG_STR_BUF_P, LOG_BUF_MAX_LENGTH - LOG_STR_BUF_P,
//             "*****LOG_TIME:<%d:%d:%d> *****\n", logHour, logMin, logSec);
//
//}
//
//static LOG_STATUS_E LOG_CheckTimes(uint64_t logAddress) {
//    static uint32_t logLines[LOG_USED_MAX_TIMES] = {0};
//    static uint32_t logCount = 0;
//    if (LOG_UPDATE_FLAG == LOG_TRUE) {
//        LOG_UPDATE_FLAG = LOG_FALSE;
//        memset(logLines, 0, LOG_USED_MAX_TIMES);
//        logCount = 0;
//    }
//    if (logCount >= LOG_USED_MAX_TIMES) {
//        return LOG_FALSE;
//    }
//
//    int numStart = 0;
//    uint32_t logNowAddress = (logAddress & 0x0000FFFF);
//
//    while (numStart <= logCount) {
//        if (logNowAddress == logLines[numStart]) {
//            return LOG_FALSE;
//        }
//        numStart++;
//    }
//
//    logLines[logCount] = logNowAddress;
//    logCount++;
//    return LOG_TRUE;
//
//
//}
//
//static void LOG_PortSend(void) {
//    switch (logUsePort) {
//        case LOG_SERIAL:
//            PORT_UartSend(LOG_STR_BUF, strlen(LOG_STR_BUF) + 1);
//            break;
//        case LOG_SDCARD:
//            Port_SdcWrite(LOG_STR_BUF, strlen(LOG_STR_BUF) + 1);
//            break;
//        case LOG_PC:
//            puts(LOG_STR_BUF);
//            break;
//        default:
//            break;
//    }
//}
//
//
//extern void
//LOG_Msg(LOG_LEVEL_E logLevel, const char *fileName, const char *funcName, int lineNum, const char *logStr, ...) {
//#if (LOG_USE_LEVEL)
//#if LOG_LEVEL_SINGLE
//    if(logLevel != LOG_OUTPUT_LEV)
//            return;
//#else
//    if (logLevel > LOG_OUTPUT_LEV || logLevel < LOG_LEV_A)
//        return;
//#endif
//
//#endif
//
//#if LOG_ONCE_ONLY
//    uint64_t logStrAddress = (uint64_t) logStr;
//    if(LOG_CheckTimes(logStrAddress) == LOG_FALSE)
//        return;
//
//#endif
//
//#if LOG_FILE_NAME
//    LOG_STR_BUF_P = (int)(strlen(LOG_STR_BUF));
//    snprintf(LOG_STR_BUF + LOG_STR_BUF_P, LOG_BUF_MAX_LENGTH - LOG_STR_BUF_P, "%s ", fileName);
//#endif
//
//#if LOG_FUNC_NAME
//    LOG_STR_BUF_P = (int)(strlen(LOG_STR_BUF));
//    snprintf(LOG_STR_BUF + LOG_STR_BUF_P, LOG_BUF_MAX_LENGTH - LOG_STR_BUF_P, "%s ", funcName);
//#endif
//
//#if LOG_LINE_NUM
//    LOG_STR_BUF_P = (int)(strlen(LOG_STR_BUF));
//    snprintf(LOG_STR_BUF + LOG_STR_BUF_P, LOG_BUF_MAX_LENGTH - LOG_STR_BUF_P, "line%d >> ", lineNum);
//#endif
//
//#if LOG_USE_COLOR
//    if (logUsePort != LOG_SDCARD) {
//        LOG_STR_BUF_P = (int) (strlen(LOG_STR_BUF));
//        snprintf(LOG_STR_BUF + LOG_STR_BUF_P, LOG_BUF_MAX_LENGTH - LOG_STR_BUF_P, "%s", LOG_COLOR_LIST[logLevel - 1]);
//    }
//#endif
//
//    LOG_STR_BUF_P = (int) (strlen(LOG_STR_BUF));
//    va_list logData;
//    va_start(logData, logStr);
//    vsnprintf(LOG_STR_BUF + LOG_STR_BUF_P, LOG_BUF_MAX_LENGTH - LOG_STR_BUF_P, logStr, logData);
//    va_end(logData);
//
//#if LOG_USE_COLOR
//    if (logUsePort != LOG_SDCARD) {
//        LOG_STR_BUF_P = (int) (strlen(LOG_STR_BUF));
//        snprintf(LOG_STR_BUF + LOG_STR_BUF_P, LOG_BUF_MAX_LENGTH - LOG_STR_BUF_P, "%s", LOG_COLOR_END);
//    }
//#endif
//
//}
//
//extern void LOG_Update(void) {
//    LOG_PortSend();
//    memset(LOG_STR_BUF, 0, LOG_BUF_MAX_LENGTH);
//    LOG_UPDATE_FLAG = LOG_TRUE;
//
//}
//
//extern void LOG_UpdateWithTime(uint64_t logTime) {
//
//    if (LOG_OUTPUT_LEV == LOG_Banned)
//        return;
//#if (LOG_SHOW_TIME)
//    LOG_AddTime(logTime);
//#endif
//    LOG_PortSend();
//    memset(LOG_STR_BUF, 0, LOG_BUF_MAX_LENGTH);
//    LOG_UPDATE_FLAG = LOG_TRUE;
//
//}
//
//extern void LOG_SetLevel(LOG_LEVEL_E logNowLevel) {
//    LOG_OUTPUT_LEV = logNowLevel;
//}
//
//extern LOG_PORT_E LOG_GetPort(void) {
//    return logUsePort;
//}
//
//extern void LOG_SetPort(LOG_PORT_E logNowPort) {
//    logUsePort = logNowPort;
//}
//
