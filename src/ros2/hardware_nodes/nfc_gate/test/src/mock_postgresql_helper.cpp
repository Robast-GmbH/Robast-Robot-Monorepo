#include "test/mock_postgresql_helper.hpp"

namespace db
{
    MockPostgreSqlHelper::MockPostgreSqlHelper(std::map< std::string, std::string> validUserList)
    {
        user_list_ = validUserList;
    }

    MockPostgreSqlHelper::~MockPostgreSqlHelper()
    {
        
    }

    std::string MockPostgreSqlHelper::open_connection()
    {
      
    }

    void MockPostgreSqlHelper::close_connection()
    {
       
    }

    
    bool MockPostgreSqlHelper::perform_query(std::string sqlStatment, std::vector< std::vector<std::string> >* result_data, std::vector<std::string>* result_header)
    {
        return true;
    }
    
    bool MockPostgreSqlHelper::checkUserTag(std::string tag, std::vector<std::string> Lookup_scope, std::string* result_data)
    {
        if (user_list_.count(tag))
        {
            *result_data = user_list_[tag];
                return true;
        }
        return false;
    }

    int MockPostgreSqlHelper::perform_transaction(std::string SqlStatement)
    {
        return 0;
    }
    
}