#ifndef DRAWER_CONTROLLER_DRAWER_HPP
#define DRAWER_CONTROLLER_DRAWER_HPP
#include "lock.hpp"
#include "i_drawer.hpp"
#include "can/can_db.hpp"

namespace drawer_controller
{
    class Drawer : public IDrawer
    {
    public:
        Drawer(uint32_t module_id, uint8_t id) : module_id_{module_id}, id{id} {};

        void init_lock(uint8_t pwr_open_lock_pin, uint8_t pwr_close_lock_pin, uint8_t sensor_lock_pin, uint8_t sensor_drawer_closed_pin)
        {
            lock.initialize_lock(pwr_open_lock_pin, pwr_close_lock_pin, sensor_lock_pin, sensor_drawer_closed_pin);
        }

        void can_in(robast_can_msgs::CanMessage msg) override
        {
            debug_prints_drawer_lock(msg);
            if (msg.get_id() == CAN_ID_DRAWER_UNLOCK)
            {
                unlock();

                debug_prints_drawer_lock(msg);
            }
        }

        std::optional<robast_can_msgs::CanMessage> can_out() override
        {
            return feedback_msg;
        };

        void update_state() override
        {
            lock.handle_lock_control();
            lock.handle_reading_sensors();
            feedback_msg.reset();
            handle_drawer_is_open_feedback();
            handle_drawer_is_closed_feedback();
        }

    private:
        uint32_t module_id_;
        uint8_t id;

        robast_can_msgs::CanDb can_db_ = robast_can_msgs::CanDb();

        std::optional<robast_can_msgs::CanMessage> feedback_msg;

        Lock lock = Lock();

        bool drawer_open_feedback_can_msg_sent = false;

        void unlock()
        {
            if (lock.is_drawer_opening_in_progress())
            {
                Serial.printf("Drawer%d opening is already in progress, so lock won't be opened again!\n", id);
            }
            else
            {
                lock.set_open_lock_current_step(true);
                lock.set_timestamp_last_lock_change();
                lock.set_drawer_opening_is_in_progress(true);
            }
        }

        void create_drawer_feedback_can_msg()
        {
            robast_can_msgs::CanMessage can_msg_drawer_feedback = this->can_db_.can_messages.at(CAN_MSG_DRAWER_FEEDBACK);
            std::vector can_signals_drawer_feedback = can_msg_drawer_feedback.get_can_signals();

            can_signals_drawer_feedback.at(CAN_SIGNAL_MODULE_ID).set_data(module_id_);
            can_signals_drawer_feedback.at(CAN_SIGNAL_DRAWER_ID).set_data(id);

            const bool is_endstop_switch_pushed = lock.is_endstop_switch_pushed();
            can_signals_drawer_feedback.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).set_data(is_endstop_switch_pushed);

            const bool is_lock_switch_pushed = lock.is_lock_switch_pushed();
            can_signals_drawer_feedback.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).set_data(is_lock_switch_pushed);

            can_msg_drawer_feedback.set_can_signals(can_signals_drawer_feedback);

            feedback_msg = can_msg_drawer_feedback;
        }

        void handle_drawer_is_open_feedback()
        {
            bool is_drawer_open = !lock.is_endstop_switch_pushed();
            if (lock.is_drawer_opening_in_progress() && is_drawer_open && !drawer_open_feedback_can_msg_sent)
            {
                lock.set_open_lock_current_step(false);   // this makes sure the lock automatically closes as soon as the drawer is opened
                drawer_open_feedback_can_msg_sent = true; // makes sure the feedback msg is only sent once
                create_drawer_feedback_can_msg();
            }
        }

        void handle_drawer_is_closed_feedback()
        {
            bool is_drawer_closed = lock.is_endstop_switch_pushed() && !lock.is_lock_switch_pushed();
            if (lock.is_drawer_opening_in_progress() && is_drawer_closed && drawer_open_feedback_can_msg_sent)
            {
                lock.set_drawer_opening_is_in_progress(false);
                drawer_open_feedback_can_msg_sent = false; // reset this flag for the next opening of the drawer
                create_drawer_feedback_can_msg();
            }
        }

        void debug_prints_drawer_lock(robast_can_msgs::CanMessage &can_message)
        {
            Serial.print("Standard ID: ");
            Serial.print(can_message.get_id(), HEX);
            Serial.print(" rx_dlc: ");
            Serial.print(can_message.get_dlc(), DEC);
            Serial.print(" MODULE ID: ");
            Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_MODULE_ID).get_data(), HEX);
            Serial.print(" DRAWER ID: ");
            Serial.print(can_message.get_can_signals().at(CAN_SIGNAL_DRAWER_ID).get_data(), HEX);
        }
    };
}
#endif