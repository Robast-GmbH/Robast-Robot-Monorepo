#ifndef HARDWARE_NODES__POSTGRESQL_CONNECTOR_HPP_
#define HARDWARE_NODES__POSTGRESQL_CONNECTOR_HPP_

#include <time.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <pqxx/pqxx>
#include <string>

#include "i_db_helper.h"

namespace db_helper
{
  class PostgreSqlHelper : public IDBHelper
  {
   private:
    const std::string connection_parameter;
    std::vector<std::vector<std::string>> perform_query(std::string sql_statment);

    int perform_transaction(std::string sql_statement);
    std::vector<std::vector<std::string>> perform_transaction_with_return(std::string sql_statement);

   public:
    PostgreSqlHelper(std::string username, std::string password, std::string host, int port, std::string db_name);
    ~PostgreSqlHelper();

    std::string test_connection();

    std::string createUser(std::string first_name, std::string last_name);
    int createNfcCode(std::string user_id, int max_ID);
    bool checkUserTag(std::string tag,
                      std::vector<std::string> lookup_scope,
                      std::shared_ptr<std::string> user_name,
                      std::shared_ptr<int> id);
    bool checkUser(std::string id, std::string first_name, std::string last_name);
  };
}   // namespace db_helper
#endif /* HARDWARE_NODES__POSTGRESQL_CONNECTOR_HPP_ */