#include "../include/db_helper/postgresql_connector.h"


namespace db_helper
{
    PostgreSqlHelper::PostgreSqlHelper(std::string username, std::string password, std::string host, std::string dbname )
    {
        connection_string_ = "user=" + username + " password=" + password + " host=" + host + " port=5432 dbname=" + dbname + " target_session_attrs=read-write";
    }

    PostgreSqlHelper::~PostgreSqlHelper()
    {
        close_connection();
    }

    std::string PostgreSqlHelper::open_connection()
    {
        connection_handle_ =  new pqxx::connection(connection_string_);
    }

    void PostgreSqlHelper::close_connection()
    {
        connection_handle_->disconnect();
    }

    bool PostgreSqlHelper::perform_query(std::string sqlStatment, std::vector< std::vector<std::string> >*result_data, std::vector<std::string>* result_header )
    {
        // make the Query from the DB
        pqxx::work session{ *connection_handle_ };
        open_connection();
        pqxx::result raw_db_feedback{ session.exec(sqlStatment) };
        close_connection();

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
    }
    
   bool PostgreSqlHelper::checkUserTag(std::string tag, std::vector<std::string> lookup_scope, std::string* name)
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
    
        perform_query("SELECT concat(first_name, ' ', last_name) AS \"name\",FROM public.\"USER\" WHERE key =" + tag + " AND id in (" + target_users + ");", &data, &header);
        if (data.size() == 1)
        {
            *name = data[0][0];
            return true;
        }
        return false;
    }

    int PostgreSqlHelper::perform_transaction(std::string SqlStatement)
    {
        open_connection();
        pqxx::work query_handle{ *connection_handle_ };
        pqxx::result result_handle{ query_handle.exec(SqlStatement)};
        int affected_Rows = result_handle.affected_rows();
        query_handle.commit();
        close_connection();
        return affected_Rows;
    }
}