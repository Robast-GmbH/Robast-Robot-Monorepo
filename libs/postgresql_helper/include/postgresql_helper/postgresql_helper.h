#ifndef POSTGRESQL_HELPER_HPP_
#define POSTGRESQL_HELPER_HPP_

#include <iostream>
#include <fstream>

#include <cstring>
#include <string>
#include <iostream>
#include <pqxx/pqxx>
#include "i_db_helper.h"


namespace DB
{
    class PostgreSqlHelper:public IDBHelper
    {
        private:
            pqxx::connection connection_handle;
            std::string connection_string;

        public:
            PostgreSqlHelper(std::string username, std::string password, std::string host, std::string dbname );
            ~PostgreSqlHelper();

            std::string open_connection();
            void close_connection();
            bool perform_querry(std::string SqlStatment, std::string** result);
            int perform_transaction(std::string SqlStatement); 
            

    };
}

#endif /* POSTGRESQL_HELPER_HPP_ */