#include "../include/postgresql_helper/postgresql_helper.h"

namespace postgresql_helper
{
    PostgreSqlHelper(string username, string password, string host, string dbname  )
    {
      connection_string ="user="+username+" password="+password+" host="+host+" port=5432 dbname="+dbname+" target_session_attrs=read-write"
    }

    ~PostgreSqlHelper()
    {
        close_connection();
    }

    string open_connection()
    {
        try {
                connection_handle(connection_string);
            } catch (const exception &e) {
                cerr << e.what() << endl;
            }
    }

    void close_connection()
    {
        connection_handle.disconnect ();
    }

    bool perform_querry(*string sqlStatment, **string result, *string resultHeader)
    {
        if(!connection_handle.is_open())
        {
            open_connection();
        }
        pqxx::nontransaction query_handle(C);

        pqxx::result_handle result_handle( query_handle.exec(sqlStatement));
        resultHeader= new string[result.colums];

        for (int i= 0; i<=result_handle.colums; i++)
        {
            resultHeader[i]= column_name(i);
        }
        result= new string[result_handle.columns][result_handle.size()]
        int y=0;
        for (result::const_iterator c = result_handle.begin(); c != result_handle.end(); ++c)
        {

            for(int x =0; x<=result_handle.colums)
            {
                result[y][x]=c[x]
            } 
            y++;
        }
    }
    
    int perform_transaction(*string SqlStatements)
    {
        if(!connection_handle.is_open())
        {
            open_connection();
        }
        work query_handle(C);
        pqxx::result_handle result_handle( query_handle.exec( SqlStatements));
        query_handle.commit();
        return result_handle.affected_rows();
    }
}