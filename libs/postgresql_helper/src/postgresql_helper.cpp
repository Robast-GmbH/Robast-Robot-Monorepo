#include "../include/postgresql_helper/postgresql_helper.h"

namespace DB
{
    PostgreSqlHelper::PostgreSqlHelper(std::string username, std::string password, std::string host, std::string dbname )
    {
        connection_string = "user=" + username + " password=" + password + " host=" + host + " port=5432 dbname=" + dbname + " target_session_attrs=read-write";
    }

    PostgreSqlHelper::~PostgreSqlHelper()
    {
        close_connection();
    }

    std::string PostgreSqlHelper::open_connection()
    {
       pqxx::connection connection_handle{connection_string.c_str() };
    }

    void PostgreSqlHelper::close_connection()
    {
        connection_handle.disconnect();
    }

    bool PostgreSqlHelper::perform_querry(std::string sqlStatment, std::string** result)
    {
        if(!connection_handle.is_open())
        {
            open_connection();
        }
         pqxx::nontransaction query_handle(connection_handle);

         result_handle result_handle{ query_handle.exec(sqlStatement) };
        resultHeader= new std::string[result.colums];

        // for (int i= 0; i<=result_handle.colums; i++)
        // {
        //     resultHeader[i]= column_name(i);
        // }
        // result = new std::string[result_handle.columns][result_handle.size()];
        // int y=0;
        // // for (int c = result_handle.begin(); c != result_handle.end(); ++c)
        // // {

        // //     for (int x = 0; x <= result_handle.colums; x++)
        // //     {
        // //         result[y][x] = c[x];
        // //     }
        // //     y++;
        // // }
    }
    
    int PostgreSqlHelper::perform_transaction(std::string SqlStatement)
    {
        // if(!connection_handle.is_open())
        // {
        //     open_connection();
        // }
        // pqxx::work query_handle(connection_handle);
        // pqxx::result_handle result_handle( query_handle.exec( SqlStatements));
        // query_handle.commit();
        // return result_handle.affected_rows();
    }
}