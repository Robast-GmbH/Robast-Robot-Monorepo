#include "db_helper/postgresql_connector.hpp"

namespace db_helper
{
  PostgreSqlHelper::PostgreSqlHelper(
      std::string username, std::string password, std::string host, int port, std::string db_name)
      : connection_parameter("user=" + username + " password=" + password + " host=" + host + " port=" +
                             std::to_string(port) + " dbname=" + db_name + " target_session_attrs=read-write")
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
      pqxx::connection connection_handle = pqxx::connection(connection_parameter);
      pqxx::work session{connection_handle};

      connection_handle.disconnect();
    }
    catch (const pqxx::pqxx_exception& e)
    {
      return "error";
    }
    return "successfull";
  }

  std::vector<std::vector<std::string>> PostgreSqlHelper::perform_query(std::string sql_statment,
                                                                        std::shared_ptr<std::string> error_message)
  {
    std::vector<std::vector<std::string>> result_data;
    pqxx::result raw_db_feedback;
    try
    {
      // make the Query from the DB
      pqxx::connection connection_handle = pqxx::connection(connection_parameter);
      pqxx::work session{connection_handle};

      raw_db_feedback = session.exec(sql_statment);
      connection_handle.disconnect();
    }
    catch (const pqxx::pqxx_exception& e)
    {
      *error_message = e.base().what();
      return result_data;
    }

    // fill the table Body
    for (pqxx::const_result_iterator::reference raw_row : raw_db_feedback)
    {
      std::vector<std::string> temp_row;
      for (pqxx::const_row_iterator::reference raw_data : raw_row)
      {
        temp_row.push_back(raw_data.c_str());
      }
      result_data.push_back(temp_row);
    }
    return result_data;
  }

  int PostgreSqlHelper::perform_transaction(std::string sql_statement, std::shared_ptr<std::string> error_message)
  {
    pqxx::connection connection_handle = pqxx::connection(connection_parameter);
    pqxx::work query_handle = pqxx::work(connection_handle);
    pqxx::result result_handle{query_handle.exec(sql_statement)};
    int affected_Rows = result_handle.affected_rows();
    if (affected_Rows > 0)
    {
      query_handle.commit();
    }
    else
    {
      *error_message = "query had no effect on the DB data";
      query_handle.abort();
    }
    connection_handle.disconnect();
    return affected_Rows;
  }

  std::vector<std::vector<std::string>> PostgreSqlHelper::perform_transaction_with_return(
      std::string sql_statement, std::shared_ptr<std::string> error_message)
  {
    pqxx::connection connection_handle = pqxx::connection(connection_parameter);
    pqxx::work query_handle = pqxx::work(connection_handle);
    pqxx::result result_handle{query_handle.exec(sql_statement)};
    std::vector<std::vector<std::string>> result_data = std::vector<std::vector<std::string>>();

    int affected_Rows = result_handle.affected_rows();
    if (affected_Rows > 0)
    {
      query_handle.commit();
    }
    else
    {
      *error_message = "query had no effect on the DB data";
      query_handle.abort();
    }

    // fill the table Body
    for (pqxx::result::const_iterator row = result_handle.begin(); row != result_handle.end(); ++row)
    {
      std::vector<std::string> temp_row;
      for (pqxx::row::const_iterator field = row.begin(); field != row.end(); ++field)
      {
        temp_row.push_back(field->c_str());
      }
      result_data.push_back(temp_row);
    }

    connection_handle.disconnect();
    return result_data;
  }

  bool PostgreSqlHelper::checkUserTag(std::string tag,
                                      std::shared_ptr<std::string> user_name,
                                      std::shared_ptr<int> id,
                                      std::shared_ptr<std::string> error_msg,
                                      std::vector<std::string> lookup_scope )
  {
    std::vector<std::vector<std::string>> data = std::vector<std::vector<std::string>>();

    std::string target_users = "";
    for (int i = 0; i < lookup_scope.size(); i++)
    {
      if (target_users != "")
      {
        target_users += ", ";
      }
      target_users += lookup_scope[i];
    }
    data = perform_query(
        "SELECT id, name FROM public.\"user\" WHERE id= (SELECT user_id FROM public.authentication WHERE nfc_code = " +
            tag + ")",
        error_msg);

    if (*error_msg == "" && data.size() == 1)
    {
      *user_name = data[0][1];
      *id = stoi(data[0][0]);
      return true;
    }

    *error_msg = "The tag is not assigned to a user!";
    return false;
  }

  bool PostgreSqlHelper::checkUser(int id, std::shared_ptr<std::string> error_msg)
  {
    std::vector<std::vector<std::string>> data = std::vector<std::vector<std::string>>();
    data = perform_query("SELECT count(*) FROM public.user WHERE id =" + std::to_string(id) + ";", error_msg);
    if (*error_msg != "")
    {
      return false;
    }
    return data[0][0] == "1";
  }

  int PostgreSqlHelper::createUser(std::string first_name, std::string last_name)
  {
    std::vector<std::vector<std::string>> result;
    std::string add_user_query = "INSERT INTO public.\"account\" (user_id, first_name, last_name) VALUES ( DEFAULT, '" +
                                 first_name + "', '" + last_name + "') RETURNING user_id;";
    std::string error_msg = "";
    result = perform_transaction_with_return(add_user_query, std::make_shared<std::string>(error_msg));
    if (error_msg != "")
    {
      return -1;
    }
    return stoi(result[0][0]);
  }

  bool PostgreSqlHelper::createNfcCode(std::string user_id, std::string card_uid)
  {
    int changed_rows;
    std::string create_nfc_query =
        "INSERT INTO public.authentication (user_id, token_uid) VALUES(" + user_id + ", '" + card_uid + "');";

    std::string error_msg = "";
    changed_rows = perform_transaction(create_nfc_query, std::make_shared<std::string>(error_msg));
    if (error_msg != "")
    {
      return false;
    }
    return changed_rows == 1;
  }

}   // namespace db_helper