#include "db_helper/postgresql_connector.hpp"


namespace db_helper
{
    PostgreSqlHelper::PostgreSqlHelper(std::string username, std::string password, std::string host, std::string dbname):
        connection_parameter("user=" + username + " password=" + password + " host=" + host + " port=5432 dbname=" + dbname + " target_session_attrs=read-write")
    {
    }

     PostgreSqlHelper::~PostgreSqlHelper()
    {
    }

    bool PostgreSqlHelper::perform_query(std::string sqlStatment, std::unique_ptr<std::vector< std::vector<std::string> >> result_data, std::unique_ptr<std::vector<std::string>> result_header)
    {
        pqxx::result raw_db_feedback;
        try
        {
            // make the Query from the DB
            pqxx::connection connection_handle = pqxx::connection("");
            pqxx::work session{ connection_handle };
       
            raw_db_feedback= session.exec(sqlStatment);
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
        //fill the tabel Body
        for (pqxx::const_result_iterator::reference raw_row : raw_db_feedback)
        {
            std::vector< std::string> temp_row;
            for (pqxx::const_row_iterator::reference raw_data : raw_row)
            {
                temp_row.push_back(raw_data.c_str());
            }
            result_data->push_back(temp_row);
        }
        return true;
    }
    
   bool PostgreSqlHelper::checkUserTag(std::string tag, std::vector<std::string> lookup_scope, std::shared_ptr<std::string> name)
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
    
        perform_query(  "SELECT concat(first_name, ' ', last_name) AS \"name\",FROM public.\"USER\" WHERE key =" + tag + " AND id in (" + target_users + ");",
                        std::make_unique< std::vector<std::vector<std::string>>>(data),
                        std::make_unique<std::vector<std::string>>(header));
        if (data.size() == 1)
        {
            *name = data[0][0];
            return true;
        }
        return false;
    }

    int PostgreSqlHelper::perform_transaction(std::string SqlStatement)
    {
        pqxx::connection connection_handle = pqxx::connection(connection_parameter);
        pqxx::work query_handle= pqxx::work( connection_handle);
        pqxx::result result_handle{ query_handle.exec(SqlStatement)};
        int affected_Rows = result_handle.affected_rows();
        query_handle.commit();
        connection_handle.disconnect();
        return affected_Rows;
    }
}