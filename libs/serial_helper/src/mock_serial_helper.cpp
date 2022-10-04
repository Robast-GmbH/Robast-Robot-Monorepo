  #include "../include/mock_serial_helper.h"

namespace serial_helper
{
            MockSerialHelper(map<string,string > responces );
            {
                this->responces = responces;
            }
          

            string MockSerialHelper::open_serial()
            {

            }

            void MockSerialHelper::close_serial()
            {

            }

            uint16_t MockSerialHelper::read_serial(string* result, uint16_t max_num_bytes)
            {
                if(!responceBuffer.empty())
                {
                  result = &responceBuffer.front();
                  responceBuffer.pop();
                  return result->size();
                }
                return 0;
            }

            string MockSerialHelper::write_serial(string msg)
            {
              if( this->responces.find(msg)!= responces.end()) 
              {
                responceBuffer.push(responces[msg]);
               return ""; 
              }
              return "Error no Message";
                
            }

            string MockSerialHelper::send_ascii_cmd(string cmd)
            {
                 return this->write_serial(cmd + "\r");
            }
}