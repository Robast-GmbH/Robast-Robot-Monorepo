#ifndef I_DB_HELPER_HPP_
#define I_DB_HELPER_HPP_

#include <cstring>
#include <string>


namespace db_helper
{
    class IDBHelper
    {
        public:
            virtual std::string open_connection() = 0;
            virtual void close_connection() = 0;

            virtual bool perform_query(std::string sqlStatment, std::vector< std::vector<std::string> >*result_data, std::vector<std::string>* result_header) = 0;
            virtual int perform_transaction(std::string SqlStatement) = 0;
            
            virtual bool checkUserTag(std::string tag, std::vector<std::string> lookup_scope, std::string* name) = 0;
    };
}

#endif /* I_DB_HELPER_HPP_ */