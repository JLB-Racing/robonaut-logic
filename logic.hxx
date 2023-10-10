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
            signal_sender.jlb_rx_t.measurements_1.line_sensor_1 = 0 == controller.selected ? controller.detection[0] + 2.0f : controller.detection[0];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_2 = 1 == controller.selected ? controller.detection[1] + 2.0f : controller.detection[1];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_3 = 2 == controller.selected ? controller.detection[2] + 2.0f : controller.detection[2];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_4 = 3 == controller.selected ? controller.detection[3] + 2.0f : controller.detection[3];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_5 = 4 == controller.selected ? controller.detection[4] + 2.0f : controller.detection[4];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_6 = 5 == controller.selected ? controller.detection[5] + 2.0f : controller.detection[5];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_7 = 6 == controller.selected ? controller.detection[6] + 2.0f : controller.detection[6];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_8 = 7 == controller.selected ? controller.detection[7] + 2.0f : controller.detection[7];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_9 = 8 == controller.selected ? controller.detection[8] + 2.0f : controller.detection[8];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_10 = 9 == controller.selected ? controller.detection[9] + 2.0f : controller.detection[9];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_11 = 10 == controller.selected ? controller.detection[10] + 2.0f : controller.detection[10];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_12 = 11 == controller.selected ? controller.detection[11] + 2.0f : controller.detection[11];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_13 = 12 == controller.selected ? controller.detection[12] + 2.0f : controller.detection[12];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_14 = 13 == controller.selected ? controller.detection[13] + 2.0f : controller.detection[13];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_15 = 14 == controller.selected ? controller.detection[14] + 2.0f : controller.detection[14];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_16 = 15 == controller.selected ? controller.detection[15] + 2.0f : controller.detection[15];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_17 = 16 == controller.selected ? controller.detection[16] + 2.0f : controller.detection[16];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_18 = 17 == controller.selected ? controller.detection[17] + 2.0f : controller.detection[17];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_19 = 18 == controller.selected ? controller.detection[18] + 2.0f : controller.detection[18];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_20 = 19 == controller.selected ? controller.detection[19] + 2.0f : controller.detection[19];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_21 = 20 == controller.selected ? controller.detection[20] + 2.0f : controller.detection[20];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_22 = 21 == controller.selected ? controller.detection[21] + 2.0f : controller.detection[21];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_23 = 22 == controller.selected ? controller.detection[22] + 2.0f : controller.detection[22];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_24 = 23 == controller.selected ? controller.detection[23] + 2.0f : controller.detection[23];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_25 = 24 == controller.selected ? controller.detection[24] + 2.0f : controller.detection[24];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_26 = 25 == controller.selected ? controller.detection[25] + 2.0f : controller.detection[25];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_27 = 26 == controller.selected ? controller.detection[26] + 2.0f : controller.detection[26];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_28 = 27 == controller.selected ? controller.detection[27] + 2.0f : controller.detection[27];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_29 = 28 == controller.selected ? controller.detection[28] + 2.0f : controller.detection[28];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_30 = 29 == controller.selected ? controller.detection[29] + 2.0f : controller.detection[29];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_31 = 30 == controller.selected ? controller.detection[30] + 2.0f : controller.detection[30];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_32 = 31 == controller.selected ? controller.detection[31] + 2.0f : controller.detection[31];

            char data[measurements_1_DLC + 2];
            for (unsigned int i = 0; i < measurements_1_DLC + 2; i++)
                data[i] = 0;
            uint8_t ide = measurements_1_IDE;
            uint8_t dlc = measurements_1_DLC;
            data[0] = measurements_1_CANID;
            data[1] = measurements_1_DLC;
            Pack_measurements_1_jlb(&signal_sender.jlb_rx_t.measurements_1, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, measurements_1_DLC + 2);

            signal_sender.jlb_rx_t.measurements_2.angular_velocity_x_ro = odometry.meas_ang_vel_x;
            signal_sender.jlb_rx_t.measurements_2.angular_velocity_y_ro = odometry.meas_ang_vel_y;
            signal_sender.jlb_rx_t.measurements_2.angular_velocity_z_ro = odometry.meas_ang_vel_z;

            for (unsigned int i = 0; i < measurements_1_DLC + 2; i++)
                data[i] = 0;
            ide = measurements_2_IDE;
            dlc = measurements_2_DLC;
            data[0] = measurements_2_CANID;
            data[1] = measurements_2_DLC;
            Pack_measurements_2_jlb(&signal_sender.jlb_rx_t.measurements_2, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, measurements_2_DLC + 2);

            signal_sender.jlb_rx_t.measurements_3.linear_acceleration_x_ro = odometry.meas_lin_acc_x;
            signal_sender.jlb_rx_t.measurements_3.linear_acceleration_y_ro = odometry.meas_lin_acc_y;
            signal_sender.jlb_rx_t.measurements_3.linear_acceleration_z_ro = odometry.meas_lin_acc_z;

            for (unsigned int i = 0; i < measurements_1_DLC + 2; i++)
                data[i] = 0;
            ide = measurements_3_IDE;
            dlc = measurements_3_DLC;
            data[0] = measurements_3_CANID;
            data[1] = measurements_3_DLC;
            Pack_measurements_3_jlb(&signal_sender.jlb_rx_t.measurements_3, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, measurements_3_DLC + 2);

            signal_sender.jlb_rx_t.measurements_4.motor_rpm_ro = odometry.meas_motor_rpm;

            for (unsigned int i = 0; i < measurements_1_DLC + 2; i++)
                data[i] = 0;
            ide = measurements_4_IDE;
            dlc = measurements_4_DLC;
            data[0] = measurements_4_CANID;
            data[1] = measurements_4_DLC;
            Pack_measurements_4_jlb(&signal_sender.jlb_rx_t.measurements_4, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, measurements_4_DLC + 2);

            signal_sender.jlb_rx_t.odometry_1.position_x_ro = odometry.x_t;
            signal_sender.jlb_rx_t.odometry_1.position_y_ro = odometry.y_t;
            signal_sender.jlb_rx_t.odometry_1.orientation_ro = odometry.theta_t;

            for (unsigned int i = 0; i < measurements_1_DLC + 2; i++)
                data[i] = 0;
            ide = odometry_1_IDE;
            dlc = odometry_1_DLC;
            data[0] = odometry_1_CANID;
            data[1] = odometry_1_DLC;
            Pack_odometry_1_jlb(&signal_sender.jlb_rx_t.odometry_1, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, odometry_1_DLC + 2);

            signal_sender.jlb_rx_t.odometry_2.linear_velocity_x_ro = odometry.vx_t;
            signal_sender.jlb_rx_t.odometry_2.angular_velocity_z_ro = odometry.w_t;

            for (unsigned int i = 0; i < measurements_1_DLC + 2; i++)
                data[i] = 0;
            ide = odometry_2_IDE;
            dlc = odometry_2_DLC;
            data[0] = odometry_2_CANID;
            data[1] = odometry_2_DLC;
            Pack_odometry_2_jlb(&signal_sender.jlb_rx_t.odometry_2, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, odometry_2_DLC + 2);

            signal_sender.jlb_rx_t.logic_1.target_angle_ro = controller.target_angle;
            signal_sender.jlb_rx_t.logic_1.target_speed_ro = controller.target_speed;

            for (unsigned int i = 0; i < measurements_1_DLC + 2; i++)
                data[i] = 0;
            ide = logic_1_IDE;
            dlc = logic_1_DLC;
            data[0] = logic_1_CANID;
            data[1] = logic_1_DLC;
            Pack_logic_1_jlb(&signal_sender.jlb_rx_t.logic_1, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, logic_1_DLC + 2);
        }

    private:
        SignalSender &signal_sender = SignalSender::get_instance();
    };

} // namespace jlb

#endif // LOGIC_HXX