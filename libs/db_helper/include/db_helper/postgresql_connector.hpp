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
    std::vector<std::vector<std::string>> perform_query(std::string sql_statment, std::shared_ptr<std::string> error_message);

    int perform_transaction(std::string sql_statement, std::shared_ptr<std::string> error_message);
    std::vector<std::vector<std::string>> perform_transaction_with_return(std::string sql_statement,std::shared_ptr<std::string> error_message);

   public:
    PostgreSqlHelper(std::string username, std::string password, std::string host, int port, std::string db_name);
    ~PostgreSqlHelper();

    std::string test_connection();

    int createUser(std::string first_name, std::string last_name);
    bool createNfcCode(std::string user_id, std::string card_uid);
    bool checkUserTag(std::string tag,
                      std::shared_ptr<std::string> user_name,
                      std::shared_ptr<int> id,
                      std::shared_ptr<std::string> error_msg,
                      std::vector<std::string> lookup_scope= std::vector<std::string>());
    bool checkUser(int id, std::shared_ptr<std::string> error_msg);
  };
}   // namespace db_helper
#endif /* HARDWARE_NODES__POSTGRESQL_CONNECTOR_HPP_ */