#include "test/mock_postgresql_helper.hpp"

namespace db_helper
{
    MockPostgreSqlHelper::MockPostgreSqlHelper(std::map< std::string, std::string> validUserList)
    {
        user_list_ = validUserList;
    }

    bool MockPostgreSqlHelper::perform_query(std::string sqlStatment __attribute__((unused)), std::unique_ptr< std::vector< std::vector<std::string> >> result_data __attribute__((unused)), std::unique_ptr<std::vector<std::string>> result_header __attribute__((unused)))
    {
        return true;
    }
    
    bool MockPostgreSqlHelper::checkUserTag(std::string scanned_key, std::vector<std::string> lookup_scope __attribute__((unused)), std::shared_ptr<std::string> related_username )
    {
        if (user_list_.count(scanned_key))
        {
            *related_username = user_list_[scanned_key];
            return true;
        }
        return false;
    }

    int MockPostgreSqlHelper::perform_transaction(std::string SqlStatement __attribute__((unused)))
    {
        return 0;
    }
    
}