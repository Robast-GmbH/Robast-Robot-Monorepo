#ifndef HARDWARE_NODES__NFC_BRIDGE__ELATEC_API_H_
#define HARDWARE_NODES__NFC_GRIDGE__ELATEC_API_H_

#define DEVICE_STATE "010A"
#define BLOCK_SIZE   15

#define SET_SERIAL_TO_ASCII "000515A7"
#define SEARCH_TAG          "050010"

#define NFC_LOGIN_MC(KEY, KEY_TYPE, SECTOR) \
  std::string("0B00") + std::string(KEY) + std::string(KEY_TYPE) + std::string(SECTOR)
#define NFC_LOGIN_MC_STANDARD(SECTOR) std::string("0B00FFFFFFFFFFFF00") + std::string(SECTOR)
#define NFC_READ_MC(BLOCK)            std::string("0B01") + std::string(BLOCK)
#define NFC_WRITE_MC(BLOCK, DATA) \
  std::string("0B02") + std::string(BLOCK) + std::string(DATA)   // Data ist als HEX string of 16Byte

#define TOP_LEDS_INIT(COLOR) std::string("0410") + std::string(COLOR)
#define TOP_LEDS_ON(COLOR)   std::string("0411") + std::string(COLOR)
#define TOP_LED_OFF(COLOR)   std::string("0412") + std::string(COLOR)

#define LED_RED        "05"
#define LED_GREEN      "06"
#define LED_YELLOW     "07"
#define BOTTOM_LED_ON  "0408"
#define BOTTOM_LED_OFF "0409"

#define BEEP(VOLUME, FREQUENCY, ON_TIME, OFF_TIME) \
  std::string("0407") + std::string(VOLUME) + std::string(FREQUENCY) + std::string(ON_TIME) + std::string(OFF_TIME)
#define BEEP_STANDART "0407166009F401F401"

#define RESPONCE_ERROR                   "0000"
#define RESPONCE_SUCCESS                 "0001"
#define RESPONCE_DEVICE_STATE_CONFIGURED "0003"
#endif   // HARDWARE_NODES__NFC_BRIDGE__ELATEC_API_H_