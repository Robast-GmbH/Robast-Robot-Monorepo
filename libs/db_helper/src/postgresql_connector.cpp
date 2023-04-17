#include "db_helper/postgresql_connector.hpp"

namespace db_helper
{
  PostgreSqlHelper::PostgreSqlHelper(std::string username, std::string password, std::string host, std::string db_name)
      : connection_parameter("user=" + username + " password=" + password + " host=" + host +
                             " port=5432 dbname=" + db_name + " target_session_attrs=read-write")
  {
  }

  PostgreSqlHelper::~PostgreSqlHelper()
  {
  }
  std::string PostgreSqlHelper::test_connection()
  {
    try
    {
      // make the Query from the DB
      pqxx::connection connection_handle = pqxx::connection("");
      pqxx::work session{connection_handle};

     
      connection_handle.disconnect();
    }
    catch (const pqxx::pqxx_exception& e)
    {
      return "error";
    }
    return "";
  }
  bool PostgreSqlHelper::perform_query(std::string sql_statment,
                                       std::unique_ptr<std::vector<std::vector<std::string>>> result_data,
                                       std::unique_ptr<std::vector<std::string>> result_header)
  {
    pqxx::result raw_db_feedback;
    try
    {
      // make the Query from the DB
      pqxx::connection connection_handle = pqxx::connection("");
      pqxx::work session{connection_handle};

      raw_db_feedback = session.exec(sql_statment);
      connection_handle.disconnect();
    }
    catch (const pqxx::pqxx_exception& e)
    {
      return false;
    }

    // fill the colum name list
    for (int i = 0; i < raw_db_feedback.columns(); i++)
    {
      result_header->push_back(raw_db_feedback.column_name(i));
    }
    // fill the tabel Body
    for (pqxx::const_result_iterator::reference raw_row : raw_db_feedback)
    {
      std::vector<std::string> temp_row;
      for (pqxx::const_row_iterator::reference raw_data : raw_row)
      {
        temp_row.push_back(raw_data.c_str());
      }
      result_data->push_back(temp_row);
    }
    return true;
  }

  int PostgreSqlHelper::perform_transaction(std::string sql_statement)
  {
    pqxx::connection connection_handle = pqxx::connection(connection_parameter);
    pqxx::work query_handle = pqxx::work(connection_handle);
    pqxx::result result_handle{query_handle.exec(sql_statement)};
    int affected_Rows = result_handle.affected_rows();
    if(affected_Rows>0)
    {
       query_handle.commit();
    }
   else 
   {
       query_handle.abort();
   }
    connection_handle.disconnect();
    return affected_Rows;
  }

  int PostgreSqlHelper::perform_transaction_with_return(std::string sql_statement, std::unique_ptr<std::vector<std::vector<std::string>>> result_data)
  {
    pqxx::connection connection_handle = pqxx::connection(connection_parameter);
    pqxx::work query_handle = pqxx::work(connection_handle);
    pqxx::result result_handle{query_handle.exec(sql_statement)};

    int affected_Rows = result_handle.affected_rows();
    if(affected_Rows>0)
    {
       query_handle.commit();
    }
   else 
   {
       query_handle.abort();
   }

   // fill the tabel Body
    for (pqxx::const_result_iterator::reference raw_row : result_handle)
    {
      std::vector<std::string> temp_row;
      for (pqxx::const_row_iterator::reference raw_data : raw_row)
      {
        temp_row.push_back(raw_data.c_str());
      }
      result_data->push_back(temp_row);
    }
    
    connection_handle.disconnect();
    return affected_Rows;
  }


  bool PostgreSqlHelper::checkUserTag(std::string tag, std::vector<std::string> lookup_scope,
                                      std::shared_ptr<std::string> user_name, std::shared_ptr<int> id )
  {
    std::vector<std::vector<std::string>> data = std::vector<std::vector<std::string>>();
    std::vector<std::string> header = std::vector<std::string>();

    std::string target_users = "";
    for (int i = 0; i < lookup_scope.size(); i++)
    {
      if (target_users != "")
      {
        target_users += ", ";
      }
      target_users += lookup_scope[i];
    }

    perform_query("SELECT concat(first_name, ' ', last_name,) AS \"name\", id FROM public.\"USER\" WHERE key =" + tag +
                      " AND id in (" + target_users + ");",
                  std::make_unique<std::vector<std::vector<std::string>>>(data),
                  std::make_unique<std::vector<std::string>>(header));
    if (data.size() == 1)
    {
      *user_name = data[0][0];
      *id = stoi(data[1][0]);
      return true;
    }
    return false;
  }

   bool PostgreSqlHelper::addUser(std::string first_name, std::string last_name)
  {
    std::string query = "";
    return perform_transaction(query);
   
    return false;
  }


}   // namespace db_helper