#ifndef HARDWARE_NODES__POSTGRESQL_CONNECTOR_HPP_
#define HARDWARE_NODES__POSTGRESQL_CONNECTOR_HPP_

#include <iostream>
#include <fstream>

#include <cstring>
#include <string>

#include <pqxx/pqxx>
#include "i_db_helper.h"


namespace db_helper
{
    class PostgreSqlHelper:public IDBHelper
    {
        private:
            const std::string connection_parameter;

        public:
            PostgreSqlHelper(std::string username, std::string password, std::string host, std::string dbname );
            ~PostgreSqlHelper();
            bool perform_query(std::string sqlStatment, std::unique_ptr<std::vector< std::vector<std::string> >> result_data, std::unique_ptr< std::vector<std::string>> result_header);
            int perform_transaction(std::string SqlStatement);
            bool checkUserTag(std::string tag, std::vector<std::string> lookup_scope, std::shared_ptr< std::string> name);
    };
}

#endif /* HARDWARE_NODES__POSTGRESQL_CONNECTOR_HPP_ */