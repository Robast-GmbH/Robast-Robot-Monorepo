#ifndef HARDWARE_NODES__MOCK_POSTGRESQL_HELPER_HPP_
#define HARDWARE_NODES__MOCK_POSTGRESQL_HELPER_HPP_
#include <vector>
#include <cstring>
#include <string>
#include <map>
#include "db_helper/i_db_helper.h"


namespace db_helper
{
    class MockPostgreSqlHelper:public IDBHelper
    {
  

        public:
            MockPostgreSqlHelper( std::map< std::string, std::string> validUserList);
            ~MockPostgreSqlHelper();

            std::string open_connection();
            void close_connection();
            bool perform_query(std::string sqlStatment, std::unique_ptr<std::vector< std::vector<std::string> >>result_data, std::unique_ptr< std::vector<std::string>> result_header);
            int perform_transaction(std::string SqlStatement);

            bool checkUserTag(std::string tag, std::vector<std::string> lookup_scope, std::shared_ptr<std::string> user_name);

        private:
            std::map< std::string, std::string> user_list_;

    };
}

#endif /* HARDWARE_NODES__MOCK_POSTGRESQL_HELPER_HPP_ */