#ifndef LOGIC_HXX
#define LOGIC_HXX

#include "odometry.hxx"
#include "controller.hxx"
#include "common.hxx"
#include "signals.hxx"

namespace jlb
{

    class Logic
    {
    public:
        Controller controller;
        Odometry odometry;

        Logic(Direction direction_ = Direction::STRAIGHT, const float x_t_ = 0.0f, const float y_t_ = 0.0f, const float theta_t_ = 0.0f) : controller(direction_), odometry(x_t_, y_t_, theta_t_) {}

        void send_telemetry()
        {
            // char msg[5] = {0};
            // Value::packet_from_value(msg, 5, signal_library[TARGET_ANGLE_ID], controller.target_angle);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[TARGET_SPEED_ID], controller.target_speed);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[POSITION_X_ID], odometry.x_t);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[POSITION_Y_ID], odometry.y_t);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[POSITION_THETA_ID], odometry.theta_t);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[ODOM_LINEAR_VELOCITY_X_ID], odometry.vx_t);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[ODOM_ANGULAR_VELOCITY_Z_ID], odometry.w_t);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[MEAS_MOTOR_RPM_ID], odometry.meas_motor_rpm);
            // signal_sender.send(msg, 5);
            // Value::packet_from_value(msg, 5, signal_library[MEAS_ANGULAR_VELOCITY_Z_ID], odometry.meas_ang_vel_z);
            // signal_sender.send(msg, 5);

            // for (uint8_t i = 0; i < SENSOR_WIDTH; i++)
            // {
            //     if (i == controller.selected)
            //     {
            //         Value::packet_from_value(msg, 5, signal_library[LINE_SENSOR_1_ID + i], controller.detection[i] + 2.0f);
            //     }
            //     else
            //     {
            //         Value::packet_from_value(msg, 5, signal_library[LINE_SENSOR_1_ID + i], controller.detection[i]);
            //     }
            //     signal_sender.send(msg, 5);
            // }

            signal_sender.jlb_rx_t.measurements_1.line_sensor_1 = controller.detection[0];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_2 = controller.detection[1];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_3 = controller.detection[2];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_4 = controller.detection[3];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_5 = controller.detection[4];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_6 = controller.detection[5];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_7 = controller.detection[6];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_8 = controller.detection[7];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_9 = controller.detection[8];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_10 = controller.detection[9];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_11 = controller.detection[10];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_12 = controller.detection[11];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_13 = controller.detection[12];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_14 = controller.detection[13];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_15 = controller.detection[14];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_16 = controller.detection[15];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_17 = controller.detection[16];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_18 = controller.detection[17];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_19 = controller.detection[18];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_20 = controller.detection[19];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_21 = controller.detection[20];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_22 = controller.detection[21];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_23 = controller.detection[22];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_24 = controller.detection[23];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_25 = controller.detection[24];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_26 = controller.detection[25];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_27 = controller.detection[26];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_28 = controller.detection[27];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_29 = controller.detection[28];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_30 = controller.detection[29];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_31 = controller.detection[30];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_32 = controller.detection[31];
            signal_sender.jlb_rx_t.measurements_1.angular_velocity_x_ro = odometry.meas_ang_vel_x;
            signal_sender.jlb_rx_t.measurements_1.angular_velocity_y_ro = odometry.meas_ang_vel_y;

            signal_sender.jlb_rx_t.measurements_2.angular_velocity_z_ro = odometry.meas_ang_vel_z;
            signal_sender.jlb_rx_t.measurements_2.linear_acceleration_x_ro = odometry.meas_lin_acc_x;
            signal_sender.jlb_rx_t.measurements_2.linear_acceleration_y_ro = odometry.meas_lin_acc_y;
            signal_sender.jlb_rx_t.measurements_2.linear_acceleration_z_ro = odometry.meas_lin_acc_z;
            signal_sender.jlb_rx_t.measurements_3.motor_rpm_ro = odometry.meas_motor_rpm;

            uint8_t ide = measurements_1_IDE;
            uint8_t dlc = measurements_1_DLC;
            char data[measurements_1_DLC];
            Pack_measurements_1_jlb(&signal_sender.jlb_rx_t.measurements_1, reinterpret_cast<uint8_t *>(data), &dlc, &ide);
            signal_sender.send(data, dlc);
        }

    private:
        SignalLibrary &signal_library = SignalLibrary::get_instance();
        SignalSender &signal_sender = SignalSender::get_instance();
    };

} // namespace jlb

#endif // LOGIC_HXX