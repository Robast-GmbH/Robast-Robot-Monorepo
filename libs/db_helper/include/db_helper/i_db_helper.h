#ifndef I_DB_HELPER_HPP_
#define I_DB_HELPER_HPP_

#include <cstring>
#include <string>
#include <memory>


namespace db_helper
{
    class IDBHelper
    {
        public:

            virtual bool perform_query(std::string sqlStatment, std::unique_ptr< std::vector< std::vector<std::string> >>result_data, std::unique_ptr<std::vector<std::string>> result_header) = 0;
            virtual int perform_transaction(std::string SqlStatement) = 0;
            
            virtual bool checkUserTag(std::string tag, std::vector<std::string> lookup_scope, std::shared_ptr< std::string> name) = 0;
    };
}

#endif /* I_DB_HELPER_HPP_ */