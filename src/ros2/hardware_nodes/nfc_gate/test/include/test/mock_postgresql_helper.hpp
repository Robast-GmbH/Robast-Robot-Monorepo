#ifndef MOCK_POSTGRESQL_HELPER_HPP_
#define MOCK_POSTGRESQL_HELPER_HPP_
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <string>
#include <map>
#include "db_helper/i_db_helper.h"


namespace db
{
    class MockPostgreSqlHelper:public IDBHelper
    {
  

        public:
            MockPostgreSqlHelper( std::map< std::string, std::string> validUserList);
            ~MockPostgreSqlHelper();

            std::string open_connection();
            void close_connection();
            bool perform_query(std::string sqlStatment, std::vector< std::vector<std::string> >*result_data, std::vector<std::string>* result_header);
            int perform_transaction(std::string SqlStatement);

            bool checkUserTag(std::string tag, std::vector<std::string> Lookup_scope, std::string* result_data);

        private:
            std::map< std::string, std::string> user_list_;

    };
}

#endif /* MOCK_POSTGRESQL_HELPER_HPP_ */