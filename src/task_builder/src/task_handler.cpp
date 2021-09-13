#include "task.cpp"
#include "std_msgs/msg/string.hpp"

class TaskHandler
{  
      private:
         Task poi[1000];
         int poiSize =0;
      
      public:
         static Task rosToTask(std_msgs::msg::String RosMsg  )//ToDo
         {
            Task* task = new Task(); 
            return *task;
         }

         static std_msgs::msg::String TaskToRos(Task* task  )//ToDo
         {
            std_msgs::msg::String message= std_msgs::msg::String();
            return message;
         }
        
         bool addPoi(Task newPoi)
         {
            poi[poiSize++] = newPoi;
            return true;
         }

         Task buildTask(Task* rawTask) //ToDo
         {
            Task newTask = *rawTask;
            return newTask;     
         }
};