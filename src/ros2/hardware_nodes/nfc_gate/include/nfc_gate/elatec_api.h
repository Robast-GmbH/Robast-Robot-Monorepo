#ifndef NFC_GATE__ELATEC_API_HPP_
#define NFC_GATE__ELATEC_API_HPP_

#define SET_SERIAL_TO_ASCII "000515A7"
#define SEARCH_TAG "050010"



#define TOP_LEDS_INIT(COLOR) string("0410") + string(COLOR)
#define TOP_LEDS_ON(COLOR) string("0411") + string(COLOR)
#define TOP_LED_OFF(COLOR) string("0412") + string(COLOR)

#define BEEP(VOLUME,FREQUENCY,ON_TIME, OFF_TIME)  string("0407") + string(VOLUME)+ string(FREQUENCY)+ string(ON_TIME)+ string(OFF_TIME)
#define BEEP_STANDART "0407166009F401F401"

#define NFC_LOGIN_MC(KEY, KEY_TYPE, SECTOR)  string("0B00") + string(KEY) + string(KEY_TYPE) + string(SECTOR)
#define NFC_LOGIN_MC_STANDART(SECTOR) string("0B00FFFFFFFFFFFF00") + string(SECTOR)
#define NFC_READ_MC(BLOCK) string("0B01") + string(BLOCK)
#define NFC_WRITE_MC(BLOCK,DATA)  string("0B02") + string(BLOCK)+ string(DATA)//Data ist als HEX string of 16Byte

#define LED_RED "05"
#define LED_GREEN "06"
#define LED_YELLOW "07"
#define BOTTOM_LED_ON "0408"
#define BOTTOM_LED_OFF "0409"

#define BOOL_SUCCESS "0001"

#endif  // NFC_GATE__NFC_GATE_HPP_