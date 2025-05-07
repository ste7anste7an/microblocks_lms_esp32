#include "espVersion.h"
#include <string.h>

#include <arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
//#include <esp_system.h> 

/*
char* getChipModel(void) {
    //#if CONFIG_IDF_TARGET_ESP32
      uint32_t chip_ver = REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG, EFUSE_RD_CHIP_PACKAGE);
      uint32_t pkg_ver = chip_ver & 0x7;
      switch (pkg_ver) {
        case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ6:
          if ((getChipRevision() / 100) == 3) {
            return "ESP32-D0WDQ6-V3";
          } else {
            return "ESP32-D0WDQ6";
          }
        case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ5:
          if ((getChipRevision() / 100) == 3) {
            return "ESP32-D0WD-V3";
          } else {
            return "ESP32-D0WD";
          }
        case EFUSE_RD_CHIP_VER_PKG_ESP32D2WDQ5:   return "ESP32-D2WD";
        case EFUSE_RD_CHIP_VER_PKG_ESP32U4WDH:    return "ESP32-U4WDH";
        case EFUSE_RD_CHIP_VER_PKG_ESP32PICOD4:   return "ESP32-PICO-D4";
        case EFUSE_RD_CHIP_VER_PKG_ESP32PICOV302: return "ESP32-PICO-V3-02";
        case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDR2V3: return "ESP32-D0WDR2-V3";
        default:                                  return "Unknown";
      }
    }
*/
// returns 2 for ESP32-PICO-V3-02 , 1 for standard esp32
int getESPVersion(void) {
    int version=0;
    if (strcmp(ESP.getChipModel(), "ESP32-PICO-V3-02") == 0) {
        version = 2;
    } else {
        version=1;
    }
    return version;
}