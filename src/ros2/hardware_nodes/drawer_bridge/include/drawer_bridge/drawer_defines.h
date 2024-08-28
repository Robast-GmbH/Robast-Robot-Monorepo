#ifndef DRAWER_DEFINES_HPP_
#define DRAWER_DEFINES_HPP_

namespace drawer_bridge
{
  // TODO: This enum should be shared with the ESP32 code as well
  enum class LedMode
  {
    steady_light = 0,
    fade_up = 1,
    running_led_from_mid_to_outside = 2,
    slow_fade_up_fade_down = 3,
  };

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