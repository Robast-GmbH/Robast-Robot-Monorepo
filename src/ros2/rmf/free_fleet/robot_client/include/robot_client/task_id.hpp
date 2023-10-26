#ifndef ROBOT_CLIENT__TASK_ID_HPP_
#define ROBOT_CLIENT__TASK_ID_HPP_

namespace rmf_robot_client
{
  struct TaskId
  {
    int id;
    int step;

    // Constructor for convenience
    TaskId(int id = 0, int step = 0) : id(id), step(step)
    {
    }

    bool operator==(const TaskId &c)
    {
      return (id == c.id && step == c.step);
    }

    std::string Tostring()
    {
      return id + "#" + step;
    }
  };
}   // namespace rmf_robot_client
#endif   // ROBOT_CLIENT__DRAWER_STATE_HPP_
