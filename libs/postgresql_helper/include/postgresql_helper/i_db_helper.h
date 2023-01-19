#ifndef I_DB_HELPER_HPP_
#define I_DB_HELPER_HPP_

#include <cstring>
#include <string>


namespace DB
{
    class IDBHelper
    {
        public:
            virtual std::string open_connection() = 0;
            virtual void close_connection() = 0;
        
            virtual bool perform_querry(std::string* SqlStatments, std::string** result) = 0;
            virtual int perform_transaction(std::string* SqlStatements) = 0; 
    };
}

#endif /* I_DB_HELPER_HPP_ */