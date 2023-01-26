#ifndef POSTGRESQL_CONNECTOR_HPP_
#define POSTGRESQL_CONNECTOR_HPP_

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
            pqxx::connection* connection_handle;
            std::string connection_string;

        public:
            PostgreSqlHelper(std::string username, std::string password, std::string host, std::string dbname );
            ~PostgreSqlHelper();

            std::string open_connection();
            void close_connection();
            bool perform_query(std::string sqlStatment, std::vector< std::vector<std::string> >* result_data, std::vector<std::string>* result_header);
            int perform_transaction(std::string SqlStatement);
            bool checkUserTag(std::string tag, std::vector<std::string> lookup_scope, std::string* name);
    


    };
}

#endif /* POSTGRESQL_CONNECTOR_HPP_ */