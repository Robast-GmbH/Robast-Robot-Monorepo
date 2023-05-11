#ifndef ELECTRICAL_DRAWER_HPP
#define ELECTRICAL_DRAWER_HPP
#include "i_drawer.hpp"
#include "motor.hpp"
#include "lock.hpp"
#include "can/can_db.hpp"

class ElectricalDrawer : public IDrawer
{
public:
    ElectricalDrawer(uint32_t module_id, uint8_t id) : module_id_{module_id}, id_{id} {};

    void init_lock(uint8_t pwr_open_lock_pin, uint8_t pwr_close_lock_pin, uint8_t sensor_lock_pin, uint8_t sensor_drawer_closed_pin)
    {
        lock_.initialize_lock(pwr_open_lock_pin, pwr_close_lock_pin, sensor_lock_pin, sensor_drawer_closed_pin);
    }

    void init_motor(TMC2209Stepper *driver, uint8_t stepper_diag_pin, uint8_t stepper_enable_pin)
    {
        motor_.init(driver, stepper_diag_pin, stepper_enable_pin);
    }

    void can_in(robast_can_msgs::CanMessage msg) override
    {
        if (msg.get_id() == CAN_ID_ELECTRICAL_DRAWER_TASK)
        {
            handle_electrical_drawer_task_msg(msg);
            debug_prints_electric_drawer_task(msg);
        }
    }

    std::optional<robast_can_msgs::CanMessage> can_out() override
    {
        return feedback_msg_;
    }

    void update_state() override
    {
        if (is_moving_)
        {
            pos_ += direction_ ? speed_ : -speed_;
        }
        if (pos_ == 10000 && is_moving_)
        {
            create_electrical_drawer_feedback_msg();
            is_moving_ = false;
        }
        else if (pos_ == 0 && is_moving_)
        {
            create_electrical_drawer_feedback_msg();
            is_moving_ = false;
        }
    }

private:
    uint32_t module_id_;
    uint8_t id_;

    bool is_moving_ = false;
    bool direction_ = false;
    int pos_ = 0;
    uint8_t speed_ = 0;
    uint8_t target_position_ = 0;

    bool stall_guard_enabled_ = false;

    robast_can_msgs::CanDb can_db_ = robast_can_msgs::CanDb();

    std::optional<robast_can_msgs::CanMessage> feedback_msg_;

    Lock lock_ = Lock();

    Motor motor_ = Motor();


    void unlock()
    {
        if (lock_.is_drawer_opening_in_progress())
        {
            Serial.printf("Drawer%d opening is already in progress, so lock won't be opened again!\n", id_);
        }
        else
        {
            lock_.set_open_lock_current_step(true);
            lock_.set_timestamp_last_lock_change();
            lock_.set_drawer_opening_is_in_progress(true);
        }
    }

    void handle_electrical_drawer_task_msg(robast_can_msgs::CanMessage can_message)
    {
        target_position_ = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_GOTO_POSITION).get_data();
        speed_ = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_SPEED_MODE).get_data();
        stall_guard_enabled_ = can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).get_data() == CAN_DATA_ELECTRICAL_DRAWER_STALL_GUARD_ENABLED ? true : false;
        //motor_.setStallGuard(stall_guard_enabled_);
        if(pos_ == 0){
            unlock();
        }
        
    }

    void create_electrical_drawer_feedback_msg()
    {
        robast_can_msgs::CanMessage can_msg_electrical_drawer_feedback = can_db_.can_messages.at(CAN_MSG_ELECTRICAL_DRAWER_FEEDBACK);
        std::vector can_signals_electrical_drawer_feedback = can_msg_electrical_drawer_feedback.get_can_signals();

        can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(module_id_);
        can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(id_);

        const bool is_endstop_switch_pushed = lock_.is_endstop_switch_pushed();
        can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);

        const bool is_lock_switch_pushed = lock_.is_lock_switch_pushed();
        can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

        const bool is_drawer_stall_guard_triggered = motor_.getIsStalled();
        can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_IS_STALL_GUARD_TRIGGERED).set_data(is_lock_switch_pushed);

        can_signals_electrical_drawer_feedback.at(CAN_SIGNAL_DRAWER_POSITION).set_data(pos_);

        can_msg_electrical_drawer_feedback.set_can_signals(can_signals_electrical_drawer_feedback);

        feedback_msg_ = can_msg_electrical_drawer_feedback;
    }

    void debug_prints_electric_drawer_task(robast_can_msgs::CanMessage can_message)
    {
        Serial.print("Standard ID: ");
        Serial.print(can_message.get_id(), HEX);
        Serial.print(" rx_dlc: ");
        Serial.print(can_message.get_dlc(), DEC);
        Serial.print(" MODULE ID: ");
        Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
        Serial.print(" DRAWER ID: ");
        Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
        Serial.print(" GOTO POSITION: ");
        Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_GOTO_POSITION).get_data(), DEC);
        Serial.print(" SPEED MODE: ");
        Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_SPEED_MODE).get_data(), DEC);
        Serial.print(" STALL GUARD ENABLE: ");
        Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_STALL_GUARD_ENABLE).get_data(), DEC);
    }
};

#endif
