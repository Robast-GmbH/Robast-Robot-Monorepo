#ifndef ROBOT_CLIENT__TASK_ID_HPP_
#define ROBOT_CLIENT__TASK_ID_HPP_

#include <algorithm>
#include <string>

namespace rmf_robot_client
{
  struct TaskId
  {
    int id;
    int phase;

    // Constructor for convenience
    TaskId(int id = 0, int phase = 0) : id(id), phase(phase)
    {
    }

    TaskId(std::string header)
    {
      if (std::count(header.begin(), header.end(), '#') == 1)
      {
        std::size_t pos = header.find("/");
        id = stoi(header.substr(0, pos));
        phase = stoi(header.substr(pos + 1));
      }
      else
      {
        id = 0;
        phase = 0;
      }
    }

    bool operator==(const TaskId& c)
    {
      return (id == c.id && phase == c.phase);
    }

    std::string Tostring()
    {
      return id + "#" + phase;
    }
  };
}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__DRAWER_STATE_HPP_
