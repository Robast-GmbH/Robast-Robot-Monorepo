#include <memory>

#include "can/can_controller.hpp"
#include "debug/debug.hpp"
#include "drawer/drawer.hpp"
#include "drawer/electrical_drawer.hpp"
#include "interfaces/i_gpio_wrapper.hpp"
#include "led/led_strip.hpp"
#include "peripherals/gpio.hpp"
#include "utils/data_mapper.hpp"

#define MODULE_ID               1
#define LOCK_ID                 0
#define STEPPER_MOTOR_1_ADDRESS 0
#define USE_ENCODER             0

using drawer_ptr = std::shared_ptr<drawer_controller::IDrawer>;

std::shared_ptr<robast_can_msgs::CanDb> can_db;

std::shared_ptr<drawer_controller::IGpioWrapper> gpio_wrapper;

std::shared_ptr<drawer_controller::ElectricalDrawer> e_drawer_0;

stepper_motor::StepperPinIdConfig stepper_1_pin_id_config = {
  .stepper_enn_tmc2209_pin_id = STEPPER_1_ENN_TMC2209_PIN_ID,
  .stepper_stdby_tmc2209_pin_id = STEPPER_1_STDBY_TMC2209_PIN_ID,
  .stepper_spread_pin_id = STEPPER_1_SPREAD_PIN_ID,
  .stepper_dir_pin_id = STEPPER_1_DIR_PIN_ID,
  .stepper_diag_pin_id = STEPPER_1_DIAG_PIN_ID,
  .stepper_index_pin_id = STEPPER_1_INDEX_PIN_ID,
  .stepper_step_pin_id = STEPPER_1_STEP_PIN_ID};

// TODO@all: If you want to use a "normal" drawer uncomment this and comment out the e_drawer
// TODO@Andres: Write a script to automatically generate code with a drawer / e-drawer
std::shared_ptr<drawer_controller::Drawer> drawer_0;

std::vector<drawer_ptr> drawers = std::vector<drawer_ptr>();

std::unique_ptr<drawer_controller::LedStrip> led_strip;

std::unique_ptr<drawer_controller::DataMapper> data_mapper;

// TODO@Jacob: This is kind of a temporary hack to ensure that the can messages are read from the controller fast enough
// TODO@Jacob: A much better solution would be to create paralle tasks for that like I suggested in this story:
// https://robast.atlassian.net/browse/RE-1775
uint16_t num_of_can_readings_per_cycle = 1;

void setup()
{
  debug_setup(115200);
  debug_println("\nStart...");

  can_db = std::make_shared<robast_can_msgs::CanDb>();
  gpio_wrapper = std::make_shared<drawer_controller::GPIO>();
  data_mapper = std::make_unique<drawer_controller::DataMapper>();

  // TODO@all: If you want to use a "normal" drawer uncomment this and comment out the e_drawer
  // TODO@Andres: Write a script to automatically generate code with a drawer / e-drawer
  drawer_0 = std::make_shared<drawer_controller::Drawer>(MODULE_ID, LOCK_ID, can_db, gpio_wrapper);
  drawer_0->init_electrical_lock(LOCK_1_OPEN_CONROL_PIN_ID,
                                 LOCK_1_CLOSE_CONROL_PIN_ID,
                                 SENSE_INPUT_LOCK_1_PIN_ID,
                                 SENSE_INPUT_DRAWER_1_CLOSED_PIN_ID);
  drawers.push_back(drawer_0);

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

  debug_println("Finished setup()!");
}

void loop()
{
  led_strip->handle_led_control();
}
