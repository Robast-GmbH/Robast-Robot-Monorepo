
#include "serial_helper/i_serial_helper.h" 
#include "nfc_gate/elatec_api.h"

namespace serial_helper
{

    class MockSerialHelper : public ISerialHelper
    {
    public:
        MockSerialHelper(std::string key);
        MockSerialHelper();
        ~MockSerialHelper();

        std::string open_serial();
        void close_serial();
        uint16_t read_serial(std::string* result, uint16_t max_num_bytes);
        std::string write_serial(std::string msg);
        std::string send_ascii_cmd(std::string cmd);
        std::string ascii_interaction(std::string cmd, std::string* response, uint16_t response_size);

    private:
        std::string key_code_;
    };
}