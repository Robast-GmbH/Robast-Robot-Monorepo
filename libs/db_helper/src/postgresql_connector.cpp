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
  std::vector<std::vector<std::string>> PostgreSqlHelper::perform_query(std::string sql_statment)
  {
    std::vector<std::vector<std::string>> result_data;
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
      return std::vector<std::vector<std::string>>();
    }

    // fill the tabel Body
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

  int PostgreSqlHelper::perform_transaction(std::string sql_statement)
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
      query_handle.abort();
    }
    connection_handle.disconnect();
    return affected_Rows;
  }

  std::vector<std::vector<std::string>> PostgreSqlHelper::perform_transaction_with_return(std::string sql_statement)
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
                                      std::vector<std::string> lookup_scope,
                                      std::shared_ptr<std::string> user_name,
                                      std::shared_ptr<int> id)
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
        "SELECT concat(first_name, ' ', last_name,) AS \"name\", user_id FROM public.\"account\" WHERE key =" + tag +
        " AND user_id in (" + target_users + ");");
    if (data.size() == 1)
    {
      if (user_name.get() != nullptr)
      {
        user_name.reset(new std::string(data[1][0]));
      }

      if (id.get() != nullptr)
      {
        id.reset(new int(stoi(data[1][1])));
      }

      return true;
    }
    return false;
  }

  bool PostgreSqlHelper::checkUser(const std::string id, const std::string first_name, const std::string last_name)
  {
    std::vector<std::vector<std::string>> data = std::vector<std::vector<std::string>>();
    data = perform_query("SELECT first_name, last_name FROM public.account WHERE user_id =" + id + ";");

    return (data[0][0] == first_name && data[0][1] == last_name);
  }

  std::string PostgreSqlHelper::createUser(std::string first_name, std::string last_name)
  {
    std::vector<std::vector<std::string>> result;
    std::string add_user_query = "INSERT INTO public.\"account\" (user_id, first_name, last_name) VALUES ( DEFAULT, '" +
                                 first_name + "', '" + last_name + "') RETURNING user_id;";
    result = perform_transaction_with_return(add_user_query);
    return result[0][0];
  }

  int PostgreSqlHelper::createNfcCode(std::string user_id, int max_id)
  {
    std::vector<std::vector<std::string>> result;
    std::string create_nfc_query = "INSERT INTO public.user_nfc_codes (user_id, card_token) VALUES(" + user_id +
                                   ", floor(random()* (" + std::to_string(max_id / 2) +
                                   " + 1))*2) RETURNING card_token; ";
    result = perform_transaction_with_return(create_nfc_query);

    return std::stoi(result[0][0]);
  }

}   // namespace db_helper