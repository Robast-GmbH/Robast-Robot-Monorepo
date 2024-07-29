#ifndef DRAWER_CONTROLLER_QUEUE_HPP
#define DRAWER_CONTROLLER_QUEUE_HPP

#include <memory>
#include <optional>

#include "debug/debug.hpp"

namespace drawer_controller
{
  template <typename T>
  class Queue
  {
   public:
    Queue();

    void add_element_to_queue(const T element);

    std::optional<T> get_element_from_queue();

   private:
    // Please mind: Normaly you would not built a queue yourself and use xQueue from FreeRTOS or std::queue
    // But: std:queue did not work and just permanently threw expections that made the ESP32 reboot
    // To use xQueue you need to use xQueueCreate to create a queue and you need to specify the size, in bytes,
    // required to hold each item in the queue, which is not possible for the CanMessage class because it contains a
    // vector of CanSignals which has a different length depending on the CanMessage. Therefore we built a queue
    // with a vector, which should be fine in this case as the queue usually only contains one or two feedback
    // messages and is rarely used. Furthermore we try to keep it as efficient as possible and try to follow what is
    // explained here: https://youtu.be/fHNmRkzxHWs?t=2541
    std::vector<T> _queue;
    uint8_t _head_of_queue;
  };

  // Please mind:
  // In C++ you need to include the implementation of the template class in the header file because the compiler
  // needs to know the implementation of the template class when it is used in another file
  template <typename T>
  Queue<T>::Queue()
  {
    _queue.clear();
    _head_of_queue = 0;
  }

  template <typename T>
  void Queue<T>::add_element_to_queue(const T element)
  {
    _queue.push_back(element);
  }

  template <typename T>
  std::optional<T> Queue<T>::get_element_from_queue()
  {
    uint8_t num_of_elements_in_queue = _queue.size();
    if (num_of_elements_in_queue == 0)
    {
      return std::nullopt;
    }

    if (_head_of_queue == (num_of_elements_in_queue - 1))
    {
      T can_element = _queue[_head_of_queue];
      _queue.clear();
      _head_of_queue = 0;
      return can_element;
    }
    else
    {
      return _queue[_head_of_queue++];
    }
  }

}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_QUEUE_HPP