#ifndef DRAWER_DEFINES_HPP_
#define DRAWER_DEFINES_HPP_

namespace drawer_bridge
{
  // TODO: This enum should be shared with the ESP32 code as well
  enum LedMode
  {
    steady_light = 0,
    fade_up = 1,
    running_led_from_mid_to_outside = 2,
    slow_fade_up_fade_down = 3,
  };

  // Some tests show that a timer period of 3 ms is to fast and asci cmds get lost at this speed, with 4 ms it worked,
  // so use 5ms with safety margin
#define TIMER_PERIOD_SEND_ASCII_CMDS 5ms

// Mind that this period should be slightly longer then the time that the serial read is waiting for a new message.
// Otherwise the node won't be responsive enough to do other tasks between the serial read
// At the time of writing this comment, the serial read waits for 50ms, so theoretically there is always a 10ms time gap
// for other tasks to be done, for example for message subscription callbacks
#define TIMER_PERIOD_RECEIVE_CAN_MSGS 60ms

// Drawer Module with 10cm Height, 40cm Depth, 1 Drawer
#define DRAWER_INSIDE_WIDTH_10x40x1  310
#define DRAWER_INSIDE_DEPTH_10x40x1  400
#define DRAWER_INSIDE_HEIGHT_10x40x1 74

// Drawer Module with 20cm Height, 40cm Depth, 1 Drawer
#define DRAWER_INSIDE_WIDTH_20x40x1  310
#define DRAWER_INSIDE_DEPTH_20x40x1  400
#define DRAWER_INSIDE_HEIGHT_20x40x1 177

// Drawer Module with 30cm Height, 40cm Depth, 1 Drawer
#define DRAWER_INSIDE_WIDTH_30x40x1  310
#define DRAWER_INSIDE_DEPTH_30x40x1  400
#define DRAWER_INSIDE_HEIGHT_30x40x1 282
}   // namespace drawer_bridge

#endif /* DRAWER_DEFINES_HPP_ */