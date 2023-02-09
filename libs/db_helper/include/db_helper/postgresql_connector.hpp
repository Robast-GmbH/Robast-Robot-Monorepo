#ifndef HARDWARE_NODES__POSTGRESQL_CONNECTOR_HPP_
#define HARDWARE_NODES__POSTGRESQL_CONNECTOR_HPP_

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

   public:
    PostgreSqlHelper(std::string username, std::string password, std::string host, std::string db_name);
    ~PostgreSqlHelper();
    bool perform_query(std::string sql_statment, std::unique_ptr<std::vector<std::vector<std::string>>> result_data,
                       std::unique_ptr<std::vector<std::string>> result_header);
    int perform_transaction(std::string sql_statement);
    bool checkUserTag(std::string tag, std::vector<std::string> lookup_scope, std::shared_ptr<std::string> user_name);
  };
}   // namespace db_helper
#endif /* HARDWARE_NODES__POSTGRESQL_CONNECTOR_HPP_ */