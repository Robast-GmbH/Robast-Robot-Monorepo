#ifndef HARDWARE_NODES__I_DB_HELPER_H_
#define HARDWARE_NODES__I_DB_HELPER_H_

#include <cstring>
#include <string>
#include <memory>
#include <vector>


namespace db_helper
{
    class IDBHelper
    {
        public:

            virtual bool perform_query(std::string sql_statment, std::unique_ptr< std::vector< std::vector<std::string> >>result_data, std::unique_ptr<std::vector<std::string>> result_header) = 0;
            virtual int perform_transaction(std::string sql_statement) = 0;
            
            virtual bool checkUserTag(std::string tag, std::vector<std::string> lookup_scope, std::shared_ptr<std::string> user_name) = 0;
    };
}

#endif /* HARDWARE_NODES__I_DB_HELPER_H_ */