#ifndef POSTGRESQL_HELPER_HPP_
#define POSTGRESQL_HELPER_HPP_

#include <iostream>
#include <fstream>

#include <cstring>
#include <string>
#include <iostream>
#include <pqxx/pqxx>
#include "i_postgresql_helper.h"

using namespace std;

namespace postgresql_helper
{
    class PostgreSqlHelper:public IPostgreSqlHelper
    {
        private:
            pqxx::connection connection_handle;
            string connection_string;

        public:
            PostgreSqlHelper(string username, string password, string host, string dbname );
            ~PostgreSqlHelper();

            string open_connection();
            void close_connection();
            bool perform_querry(*string SqlStatments, **string result );
            int perform_transaction(*string SqlStatements); 
            

    };
}

#endif /* POSTGRESQL_HELPER_HPP_ */