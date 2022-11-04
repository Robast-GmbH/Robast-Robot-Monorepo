#include "nfc_gate/serial_helper_mock.hpp"

namespace serial_helper
{           
    
    MockSerialHelper::MockSerialHelper(string key)
    {
        key_code= key; 
    }    
    
    MockSerialHelper::~MockSerialHelper()
    {

    }
    
    string MockSerialHelper::open_serial()
    {
        return "0001";
    }

    void MockSerialHelper::close_serial()
    {
       
    }

    uint16_t MockSerialHelper::read_serial(string* result, uint16_t max_num_bytes)
    {
        *result= "0001";
        return 4;
    }

    string MockSerialHelper::write_serial(string msg) 
    {
         if(msg==NFC_READ_MC("02"))
        {

          return key_code;
        }
        return "0001";
    }

    string MockSerialHelper::send_ascii_cmd(string cmd)
    {
        if(cmd==NFC_READ_MC("02"))
        {

          return key_code;
        }
        return "0001";
    }

    string MockSerialHelper::ascii_interaction(string cmd, string* responce, uint16_t responce_size )
    {
        (void) responce_size;
        if(cmd == NFC_READ_MC("02"))
        {
            *responce =key_code;
        } 
        else if(cmd == DEVICE_STATE)
        {
            *responce = RESPONCE_DEVICE_STATE_CONFIGURED;
        }
        else
        {
            *responce ="0001";
        }

        return *responce; 
    }
    
}