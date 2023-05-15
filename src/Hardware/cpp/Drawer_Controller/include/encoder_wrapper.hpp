#if !defined(DRAWER_CONTROLLER_ENCODER_WRAPPER_HPP)
#define DRAWER_CONTROLLER_ENCODER_WRAPPER_HPP

#include <Encoder.h>

namespace drawer_controller
{
  class EncoderWrapper
  {
   public:
    long position = -999;

    void update_position()
    {
      long newPosition = encoder.read();
      if (newPosition != position)
      {
        position = newPosition;
        Serial.println(newPosition);
      }
    }
    Encoder encoder = Encoder(5, 6);
  };
}   // namespace drawer_controller

#endif   // DRAWER_CONTROLLER_ENCODER_HPP
