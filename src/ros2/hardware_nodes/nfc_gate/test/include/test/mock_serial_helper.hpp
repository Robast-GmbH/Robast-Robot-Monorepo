
#include "serial_helper/i_serial_helper.h" 
#include "nfc_gate/elatec_api.h"

namespace serial_helper
{

    class MockSerialHelper :public ISerialHelper
    {
    public:
        MockSerialHelper(string key);
        ~MockSerialHelper();

        string open_serial();
        void close_serial();
        uint16_t read_serial(string* result, uint16_t max_num_bytes);
        string write_serial(string msg);
        string send_ascii_cmd(string cmd);
        string ascii_interaction(string cmd, string* responce, uint16_t responce_size);

    private:
        string key_code;
    };
}