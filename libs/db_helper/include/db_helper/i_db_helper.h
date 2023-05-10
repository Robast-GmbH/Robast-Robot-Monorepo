#ifndef HARDWARE_NODES__I_DB_HELPER_H_
#define HARDWARE_NODES__I_DB_HELPER_H_

#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace db_helper
{
  class IDBHelper
  {
   public:
    virtual std::string test_connection() = 0;

    virtual std::string createUser(std::string first_name, std::string last_name) = 0;
    virtual int createNfcCode(std::string user_id, int max_id) = 0;
    virtual bool lookupUserTag(std::string tag,
                              std::vector<std::string> lookup_scope,
                              std::shared_ptr<std::string> user_name,
                              std::shared_ptr<int> id) = 0;
    virtual bool checkUser(const std::string id, const std::string first_name, const std::string last_name) = 0;
  };
}   // namespace db_helper

#endif /* HARDWARE_NODES__I_DB_HELPER_H_ */