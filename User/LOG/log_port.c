////
//// Created by ShiF on 2023/9/14.
////
//
//#include "log_port.h"
//#include "log.h"
//#include "stdio.h"
//#include "string.h"
//
//#include "usart.h"
//#include "bsp_driver_sd.h"
//#include "fatfs.h"
//#include "sdio.h"
//
//#define LOG_UART huart8
//
//static char PORT_STR_BUF[LOG_BUF_MAX_LENGTH] = {0};
//
//static FATFS sdcFatfs;
//static uint8_t sdcBuffer[_MAX_SS];
//static uint32_t sdcWriteNum;
//static char *sdcFileName = "log.txt";
//static uint8_t sdcTestText[] = "***Good morning, good afternoon and good night ! ***\n";
//static HAL_SD_CardInfoTypeDef sdcInfo;
//static uint8_t sdcInitState = 0;
//
//
//static void PORT_SdcPrintInfo(void) {
//    uint64_t CardCap;        //SD������
//    HAL_SD_CardCIDTypeDef SDCard_CID;
//    HAL_SD_GetCardCID(&hsd, &SDCard_CID);    //��ȡCID
//    HAL_SD_GetCardInfo(&hsd, &sdcInfo);                    //��ȡSD����Ϣ
//    CardCap = (uint64_t) (sdcInfo.LogBlockNbr) * (uint64_t) (sdcInfo.LogBlockSize);    //����SD������
//    switch (sdcInfo.CardType) {
//        case CARD_SDSC: {
//            if (sdcInfo.CardVersion == CARD_V1_X)
//                LOG_Printf("Card Type:SDSC V1\r\n");
//            else if (sdcInfo.CardVersion == CARD_V2_X)
//                LOG_Printf("Card Type:SDSC V2\r\n");
//        }
//            break;
//        case CARD_SDHC_SDXC:
//            LOG_Printf("Card Type:SDHC\r\n");
//            break;
//        default:
//            break;
//    }
//    LOG_Printf("Card ManufacturerID: %d \r\n", SDCard_CID.ManufacturerID);                //������ID
//    LOG_Printf("CardVersion:         %d \r\n", (uint32_t) (sdcInfo.CardVersion));        //���汾��
//    LOG_Printf("Class:               %d \r\n", (uint32_t) (sdcInfo.Class));            //
//    LOG_Printf("Card RCA(RelCardAdd):%d \r\n", sdcInfo.RelCardAdd);                    //����Ե�ַ
//    LOG_Printf("Card BlockNbr:       %d \r\n", sdcInfo.BlockNbr);                        //������
//    LOG_Printf("Card BlockSize:      %d \r\n", sdcInfo.BlockSize);                    //���С
//    LOG_Printf("LogBlockNbr:         %d \r\n", (uint32_t) (sdcInfo.LogBlockNbr));        //�߼�������
//    LOG_Printf("LogBlockSize:        %d \r\n", (uint32_t) (sdcInfo.LogBlockSize));        //�߼����С
//    LOG_Printf("Card Capacity:       %d MB\r\n", (uint32_t) (CardCap >> 20));                //������
//}
//
//extern void PORT_SdcInit(void) {
//    if (LOG_GetPort() != LOG_SDCARD) {
//        return;
//    }
//    BSP_SD_Init();
//    FRESULT sdcFlag;
//    sdcFlag = f_mount(&sdcFatfs, (TCHAR const *) "", 0);
////	LOG_Printf("mount:%d\n", sdcFlag);
//    sdcFlag = f_mkfs((TCHAR const *) SDPath, FM_ANY, 0, sdcBuffer, sizeof(sdcBuffer));
////	LOG_Printf("mfs:%d\n", sdcFlag);
////	PORT_SdcPrintInfo();
//    if (sdcFlag == FR_OK) {
//        sdcFlag = f_open(&SDFile, sdcFileName, FA_CREATE_ALWAYS | FA_WRITE);
////		LOG_Printf("open:%d\n", sdcFlag);
//        if (sdcFlag == FR_OK) {
//            f_write(&SDFile, sdcTestText, sizeof(sdcTestText), (void *) &sdcWriteNum);
//            f_close(&SDFile);
//            sdcInitState = 1;
//        }
//    }
//}
//
//extern void Port_SdcWrite(char *dataStr, int dataLength) {
//    memset(PORT_STR_BUF, 0, LOG_BUF_MAX_LENGTH);
//    snprintf(PORT_STR_BUF, dataLength, "%s", (const char *) dataStr);
//
//    if (LOG_GetPort() != LOG_SDCARD || sdcInitState == 0) {
//        return;
//    }
//
//    FRESULT sdcFlag = f_open(&SDFile, sdcFileName, FA_OPEN_APPEND | FA_WRITE);
//    if (sdcFlag == FR_OK) {
//        f_write(&SDFile, PORT_STR_BUF, dataLength, (void *) &sdcWriteNum);
//        f_close(&SDFile);
//    }
//
//}
//
//
//extern void PORT_UartSend(char *sendStr, int sendLength) {
//    memset(PORT_STR_BUF, 0, LOG_BUF_MAX_LENGTH);
//    snprintf(PORT_STR_BUF, sendLength, "%s", (const char *) sendStr);
//    __HAL_DMA_DISABLE(LOG_UART.hdmatx);
//    HAL_UART_Transmit_DMA(&LOG_UART, (uint8_t *) PORT_STR_BUF, sendLength);
//}
//
