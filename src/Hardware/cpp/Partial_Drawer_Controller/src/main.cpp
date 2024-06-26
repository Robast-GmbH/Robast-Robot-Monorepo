#include <memory>

// #include "can/can_controller.hpp"
#include "debug/debug.hpp"
#include "drawer/drawer.hpp"
#include "drawer/electrical_drawer.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "led/led_strip.hpp"
#include "peripherals/gpio.hpp"
#include "peripherals/led_driver.hpp"
#include "utils/data_mapper.hpp"

#define MODULE_ID               1
#define LOCK_ID                 0
#define STEPPER_MOTOR_1_ADDRESS 0
#define USE_ENCODER             0

using drawer_ptr = std::shared_ptr<drawer_controller::IDrawer>;

// std::shared_ptr<robast_can_msgs::CanDb> can_db;

std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper;

// std::shared_ptr<drawer_controller::ElectricalDrawer> e_drawer_0;

// stepper_motor::StepperPinIdConfig stepper_1_pin_id_config = {
//   .stepper_enn_tmc2209_pin_id = STEPPER_1_ENN_TMC2209_PIN_ID,
//   .stepper_stdby_tmc2209_pin_id = STEPPER_1_STDBY_TMC2209_PIN_ID,
//   .stepper_spread_pin_id = STEPPER_1_SPREAD_PIN_ID,
//   .stepper_dir_pin_id = STEPPER_1_DIR_PIN_ID,
//   .stepper_diag_pin_id = STEPPER_1_DIAG_PIN_ID,
//   .stepper_index_pin_id = STEPPER_1_INDEX_PIN_ID,
//   .stepper_step_pin_id = STEPPER_1_STEP_PIN_ID};

// TODO@all: If you want to use a "normal" drawer uncomment this and comment out the e_drawer
// TODO@Andres: Write a script to automatically generate code with a drawer / e-drawer
// std::shared_ptr<drawer_controller::Drawer> drawer_0;

// std::vector<drawer_ptr> drawers = std::vector<drawer_ptr>();

std::unique_ptr<drawer_controller::LedStrip> led_strip;

// std::unique_ptr<drawer_controller::DataMapper> data_mapper;

std::unique_ptr<drawer_controller::LedDriver> led_driver;

// TODO@Jacob: This is kind of a temporary hack to ensure that the can messages are read from the
// controller fast enough
// TODO@Jacob: A much better solution would be to create paralle tasks for that like I suggested in
// this story: https://robast.atlassian.net/browse/RE-1775
uint16_t num_of_can_readings_per_cycle = 1;

uint16_t counter = 0;

void setup()
{
  debug_setup(115200);
  debug_println("\nStart...");

  // TODO: In the hardware design of v1 the pins of SDA and SCL are mixed up unfortunately for the port expander
  // TODO: In order to use the hardware anyway, we need to create two instances of the bus with different pin init
  std::shared_ptr<TwoWire> wire_led_driver = std::make_shared<TwoWire>(1);
  std::shared_ptr<TwoWire> wire_port_expander = std::make_shared<TwoWire>(2);
  wire_led_driver->begin(I2C_SDA, I2C_SCL);
  wire_port_expander->begin(I2C_SDA_PORT_EXPANDER, I2C_SCL_PORT_EXPANDER);

  led_driver = std::make_unique<drawer_controller::LedDriver>(wire_led_driver);
  gpio_wrapper = std::make_shared<drawer_controller::GPIO>(wire_port_expander);

  // can_db = std::make_shared<robast_can_msgs::CanDb>();
  // data_mapper = std::make_unique<drawer_controller::DataMapper>();

  // // TODO@all: If you want to use a "normal" drawer uncomment this and comment out the e_drawer
  // // TODO@Andres: Write a script to automatically generate code with a drawer / e-drawer
  // drawer_0 = std::make_shared<drawer_controller::Drawer>(MODULE_ID, LOCK_ID, can_db, gpio_wrapper);
  // drawer_0->init_electrical_lock(LOCK_1_OPEN_CONROL_PIN_ID,
  //                                LOCK_1_CLOSE_CONROL_PIN_ID,
  //                                SENSE_INPUT_LOCK_1_PIN_ID,
  //                                SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID);
  // drawers.push_back(drawer_0);

  led_strip = std::make_unique<drawer_controller::LedStrip>();

  // e_drawer_0 = std::make_shared<drawer_controller::ElectricalDrawer>(MODULE_ID,
  //                                                                    LOCK_ID,
  //                                                                    can_db,
  //                                                                    gpio_wrapper,
  //                                                                    stepper_1_pin_id_config,
  //                                                                    USE_ENCODER,
  //                                                                    DRAWER_1_ENCODER_A_PIN,
  //                                                                    DRAWER_1_ENCODER_B_PIN,
  //                                                                    STEPPER_MOTOR_1_ADDRESS);
  // e_drawer_0->init_electrical_lock(LOCK_1_OPEN_CONROL_PIN_ID,
  //                                  LOCK_1_CLOSE_CONROL_PIN_ID,
  //                                  SENSE_INPUT_LOCK_1_PIN_ID,
  //                                  SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID);
  // e_drawer_0->init_motor();
  // drawers.push_back(e_drawer_0);

  led_driver->initialize(
    []()
    {
      gpio_wrapper->set_pin_mode(ENABLE_ONBOARD_LED_VDD_PIN_ID, gpio_wrapper->get_gpio_output_pin_mode());
    });

  gpio_wrapper->set_pin_mode(SENSE_INPUT_LID_7_CLOSED_PIN_ID, gpio_wrapper->get_gpio_input_pin_mode());
  gpio_wrapper->set_pin_mode(LOCK_7_OPEN_CONTROL_PIN_ID, gpio_wrapper->get_gpio_output_pin_mode());
  gpio_wrapper->set_pin_mode(LOCK_7_CLOSE_CONTROL_PIN_ID, gpio_wrapper->get_gpio_output_pin_mode());

  gpio_wrapper->digital_write(LOCK_7_CLOSE_CONTROL_PIN_ID, false);
  gpio_wrapper->digital_write(LOCK_7_OPEN_CONTROL_PIN_ID, false);

  debug_println("Finished setup()!");
}

void loop()
{
  // led_strip->handle_led_control();

  // uint8_t value;
  // gpio_wrapper->digital_read(SENSE_INPUT_LID_7_CLOSED_PIN_ID, value);

  // if (value == 1)
  // {
  //   if (counter == 400)
  //   {
  //     debug_println("Lid 7 is open!");
  //     counter = 0;
  //   }
  // }
  // else
  // {
  //   if (counter == 400)
  //   {
  //     debug_println("Lid 7 is closed!");
  //     counter = 0;
  //   }
  // }
  // counter++;

  // if keyboard input toggle lock
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == 'o')
    {
      debug_println("Open lock 8");
      gpio_wrapper->digital_write(LOCK_7_CLOSE_CONTROL_PIN_ID, false);
      delay(100);
      gpio_wrapper->digital_write(LOCK_7_OPEN_CONTROL_PIN_ID, true);
    }
    else if (c == 'c')
    {
      debug_println("Close lock 8");
      gpio_wrapper->digital_write(LOCK_7_OPEN_CONTROL_PIN_ID, false);
      delay(100);
      gpio_wrapper->digital_write(LOCK_7_CLOSE_CONTROL_PIN_ID, true);
    }
    else if (c == 'l')
    {
      debug_println("Set led output color");
      led_driver->set_tray_led_brightness(counter, counter);
    }
    else if (c == 'k')
    {
      debug_println("Set led output color");
      led_driver->set_tray_led_brightness(counter, 0);
    }
    else if (c == 'j')
    {
      debug_println("Set led output brightness");
      led_driver->set_led_brigthness(1, counter);
    }
    else if (c == 'h')
    {
      debug_println("Set led output brightness");
      led_driver->set_led_brigthness(1, 0);
    }
    else if (c == 'r')
    {
      debug_println("Read registers");
      // loop throw all registers and print result
      for (int i = 0; i < 0x37; i++)
      {
        byte data_read;
        led_driver->read_register(i, data_read);
        debug_print("Register: ");
        debug_print(i);
        debug_print(" Data: ");
        debug_println(data_read);
      }

      debug_println("Finished reading register");
    }
    else if (c == 't')
    {
      debug_println("Reset registers");
      led_driver->reset_registers();
    }
    else if (c == 'u')
    {
      debug_println("Enable chip");
      led_driver->enable_chip();
    }
    else if (c == '+')
    {
      counter++;
      debug_print("Counter: ");
      debug_println(counter);
    }
    else if (c == '-')
    {
      counter--;
      debug_print("Counter: ");
      debug_println(counter);
    }
    else if (c == 'q')
    {
      debug_println("Set color of led 1");
      led_driver->set_led_output_color_by_number(1, counter);
    }
  }
}
