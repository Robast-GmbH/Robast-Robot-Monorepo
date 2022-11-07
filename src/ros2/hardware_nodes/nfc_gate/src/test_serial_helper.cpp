#include "nfc_gate/test_serial_helper.hpp"

namespace serial_helper
{           
    
    TestSerialHelper::TestSerialHelper(string key)
    {
        key_code= key; 
    }    
    
    TestSerialHelper::~TestSerialHelper()
    {

    }
    
    string TestSerialHelper::open_serial()
    {
        return "0001";
    }

    void TestSerialHelper::close_serial()
    {
       
    }

    uint16_t TestSerialHelper::read_serial(string* result, uint16_t max_num_bytes)
    {
        *result= "0001";
        return 4;
    }

    string TestSerialHelper::write_serial(string msg) 
    {
         if(msg==NFC_READ_MC("02"))
        {

          return key_code;
        }
        return "0001";
    }

    string TestSerialHelper::send_ascii_cmd(string cmd)
    {
        if(cmd==NFC_READ_MC("02"))
        {

          return key_code;
        }
        return "0001";
    }

    string TestSerialHelper::ascii_interaction(string cmd, string* responce, uint16_t responce_size )
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