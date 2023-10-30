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

        void measurements_1()
        {
            signal_sender.jlb_rx_t.measurements_1.line_sensor_1 = 0 == controller.selected_front ? controller.detection_front[0] + 2.0f : controller.detection_front[0];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_2 = 1 == controller.selected_front ? controller.detection_front[1] + 2.0f : controller.detection_front[1];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_3 = 2 == controller.selected_front ? controller.detection_front[2] + 2.0f : controller.detection_front[2];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_4 = 3 == controller.selected_front ? controller.detection_front[3] + 2.0f : controller.detection_front[3];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_5 = 4 == controller.selected_front ? controller.detection_front[4] + 2.0f : controller.detection_front[4];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_6 = 5 == controller.selected_front ? controller.detection_front[5] + 2.0f : controller.detection_front[5];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_7 = 6 == controller.selected_front ? controller.detection_front[6] + 2.0f : controller.detection_front[6];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_8 = 7 == controller.selected_front ? controller.detection_front[7] + 2.0f : controller.detection_front[7];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_9 = 8 == controller.selected_front ? controller.detection_front[8] + 2.0f : controller.detection_front[8];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_10 = 9 == controller.selected_front ? controller.detection_front[9] + 2.0f : controller.detection_front[9];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_11 = 10 == controller.selected_front ? controller.detection_front[10] + 2.0f : controller.detection_front[10];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_12 = 11 == controller.selected_front ? controller.detection_front[11] + 2.0f : controller.detection_front[11];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_13 = 12 == controller.selected_front ? controller.detection_front[12] + 2.0f : controller.detection_front[12];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_14 = 13 == controller.selected_front ? controller.detection_front[13] + 2.0f : controller.detection_front[13];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_15 = 14 == controller.selected_front ? controller.detection_front[14] + 2.0f : controller.detection_front[14];
            // signal_sender.jlb_rx_t.measurements_1.line_sensor_16 = 15 == controller.selected_front ? controller.detection_front[15] + 2.0f : controller.detection_front[15];

#ifndef SIMULATION
            signal_sender.jlb_rx_t.measurements_1.line_sensor_17 = 16 == controller.selected_front ? controller.detection_front[16] + 2.0f : controller.detection_front[16];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_18 = 17 == controller.selected_front ? controller.detection_front[17] + 2.0f : controller.detection_front[17];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_19 = 18 == controller.selected_front ? controller.detection_front[18] + 2.0f : controller.detection_front[18];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_20 = 19 == controller.selected_front ? controller.detection_front[19] + 2.0f : controller.detection_front[19];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_21 = 20 == controller.selected_front ? controller.detection_front[20] + 2.0f : controller.detection_front[20];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_22 = 21 == controller.selected_front ? controller.detection_front[21] + 2.0f : controller.detection_front[21];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_23 = 22 == controller.selected_front ? controller.detection_front[22] + 2.0f : controller.detection_front[22];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_24 = 23 == controller.selected_front ? controller.detection_front[23] + 2.0f : controller.detection_front[23];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_25 = 24 == controller.selected_front ? controller.detection_front[24] + 2.0f : controller.detection_front[24];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_26 = 25 == controller.selected_front ? controller.detection_front[25] + 2.0f : controller.detection_front[25];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_27 = 26 == controller.selected_front ? controller.detection_front[26] + 2.0f : controller.detection_front[26];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_28 = 27 == controller.selected_front ? controller.detection_front[27] + 2.0f : controller.detection_front[27];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_29 = 28 == controller.selected_front ? controller.detection_front[28] + 2.0f : controller.detection_front[28];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_30 = 29 == controller.selected_front ? controller.detection_front[29] + 2.0f : controller.detection_front[29];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_31 = 30 == controller.selected_front ? controller.detection_front[30] + 2.0f : controller.detection_front[30];
            signal_sender.jlb_rx_t.measurements_1.line_sensor_32 = 31 == controller.selected_front ? controller.detection_front[31] + 2.0f : controller.detection_front[31];
#endif

            char data[measurements_1_DLC + 2] = {0};
            uint8_t ide = measurements_1_IDE;
            uint8_t dlc = measurements_1_DLC;
            data[0] = measurements_1_CANID;
            data[1] = measurements_1_DLC;
            Pack_measurements_1_jlb(&signal_sender.jlb_rx_t.measurements_1, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, measurements_1_DLC + 2);
        }

        void measurements_2()
        {
            signal_sender.jlb_rx_t.measurements_2.line_sensor_1 = 0 == controller.selected_rear ? controller.detection_rear[0] + 2.0f : controller.detection_rear[0];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_2 = 1 == controller.selected_rear ? controller.detection_rear[1] + 2.0f : controller.detection_rear[1];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_3 = 2 == controller.selected_rear ? controller.detection_rear[2] + 2.0f : controller.detection_rear[2];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_4 = 3 == controller.selected_rear ? controller.detection_rear[3] + 2.0f : controller.detection_rear[3];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_5 = 4 == controller.selected_rear ? controller.detection_rear[4] + 2.0f : controller.detection_rear[4];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_6 = 5 == controller.selected_rear ? controller.detection_rear[5] + 2.0f : controller.detection_rear[5];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_7 = 6 == controller.selected_rear ? controller.detection_rear[6] + 2.0f : controller.detection_rear[6];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_8 = 7 == controller.selected_rear ? controller.detection_rear[7] + 2.0f : controller.detection_rear[7];
            // signal_sender.jlb_rx_t.measurements_2.line_sensor_9 = 8 == controller.selected_rear ? controller.detection_rear[8] + 2.0f : controller.detection_rear[8];
            // signal_sender.jlb_rx_t.measurements_2.line_sensor_10 = 9 == controller.selected_rear ? controller.detection_rear[9] + 2.0f : controller.detection_rear[9];
            // signal_sender.jlb_rx_t.measurements_2.line_sensor_11 = 10 == controller.selected_rear ? controller.detection_rear[10] + 2.0f : controller.detection_rear[10];
            // signal_sender.jlb_rx_t.measurements_2.line_sensor_12 = 11 == controller.selected_rear ? controller.detection_rear[11] + 2.0f : controller.detection_rear[11];
            // signal_sender.jlb_rx_t.measurements_2.line_sensor_13 = 12 == controller.selected_rear ? controller.detection_rear[12] + 2.0f : controller.detection_rear[12];
            // signal_sender.jlb_rx_t.measurements_2.line_sensor_14 = 13 == controller.selected_rear ? controller.detection_rear[13] + 2.0f : controller.detection_rear[13];
            // signal_sender.jlb_rx_t.measurements_2.line_sensor_15 = 14 == controller.selected_rear ? controller.detection_rear[14] + 2.0f : controller.detection_rear[14];
            // signal_sender.jlb_rx_t.measurements_2.line_sensor_16 = 15 == controller.selected_rear ? controller.detection_rear[15] + 2.0f : controller.detection_rear[15];

#ifndef SIMULATION
            signal_sender.jlb_rx_t.measurements_2.line_sensor_17 = 16 == controller.selected_rear ? controller.detection_rear[16] + 2.0f : controller.detection_rear[16];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_18 = 17 == controller.selected_rear ? controller.detection_rear[17] + 2.0f : controller.detection_rear[17];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_19 = 18 == controller.selected_rear ? controller.detection_rear[18] + 2.0f : controller.detection_rear[18];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_20 = 19 == controller.selected_rear ? controller.detection_rear[19] + 2.0f : controller.detection_rear[19];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_21 = 20 == controller.selected_rear ? controller.detection_rear[20] + 2.0f : controller.detection_rear[20];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_22 = 21 == controller.selected_rear ? controller.detection_rear[21] + 2.0f : controller.detection_rear[21];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_23 = 22 == controller.selected_rear ? controller.detection_rear[22] + 2.0f : controller.detection_rear[22];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_24 = 23 == controller.selected_rear ? controller.detection_rear[23] + 2.0f : controller.detection_rear[23];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_25 = 24 == controller.selected_rear ? controller.detection_rear[24] + 2.0f : controller.detection_rear[24];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_26 = 25 == controller.selected_rear ? controller.detection_rear[25] + 2.0f : controller.detection_rear[25];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_27 = 26 == controller.selected_rear ? controller.detection_rear[26] + 2.0f : controller.detection_rear[26];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_28 = 27 == controller.selected_rear ? controller.detection_rear[27] + 2.0f : controller.detection_rear[27];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_29 = 28 == controller.selected_rear ? controller.detection_rear[28] + 2.0f : controller.detection_rear[28];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_30 = 29 == controller.selected_rear ? controller.detection_rear[29] + 2.0f : controller.detection_rear[29];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_31 = 30 == controller.selected_rear ? controller.detection_rear[30] + 2.0f : controller.detection_rear[30];
            signal_sender.jlb_rx_t.measurements_2.line_sensor_32 = 31 == controller.selected_rear ? controller.detection_rear[31] + 2.0f : controller.detection_rear[31];
#endif

            char data[measurements_2_DLC + 2] = {0};
            uint8_t ide = measurements_2_IDE;
            uint8_t dlc = measurements_2_DLC;
            data[0] = measurements_2_CANID;
            data[1] = measurements_2_DLC;
            Pack_measurements_2_jlb(&signal_sender.jlb_rx_t.measurements_2, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, measurements_2_DLC + 2);
        }

        void measurements_3()
        {
            signal_sender.jlb_rx_t.measurements_3.angular_velocity_x_phys = odometry.meas_ang_vel_x;
            signal_sender.jlb_rx_t.measurements_3.angular_velocity_y_phys = odometry.meas_ang_vel_y;
            signal_sender.jlb_rx_t.measurements_3.angular_velocity_z_phys = odometry.meas_ang_vel_z;

            char data[measurements_2_DLC + 2] = {0};
            uint8_t ide = measurements_3_IDE;
            uint8_t dlc = measurements_3_DLC;
            data[0] = measurements_3_CANID;
            data[1] = measurements_3_DLC;
            Pack_measurements_3_jlb(&signal_sender.jlb_rx_t.measurements_3, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, measurements_3_DLC + 2);
        }

        void measurements_4()
        {
            signal_sender.jlb_rx_t.measurements_4.linear_acceleration_x_phys = odometry.meas_lin_acc_x;
            signal_sender.jlb_rx_t.measurements_4.linear_acceleration_y_phys = odometry.meas_lin_acc_y;
            signal_sender.jlb_rx_t.measurements_4.linear_acceleration_z_phys = odometry.meas_lin_acc_z;

            char data[measurements_3_DLC + 2] = {0};
            uint8_t ide = measurements_4_IDE;
            uint8_t dlc = measurements_4_DLC;
            data[0] = measurements_4_CANID;
            data[1] = measurements_4_DLC;
            Pack_measurements_4_jlb(&signal_sender.jlb_rx_t.measurements_4, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, measurements_4_DLC + 2);
        }

        void measurements_5()
        {
            signal_sender.jlb_rx_t.measurements_5.motor_rpm_phys = odometry.meas_motor_rpm;

            char data[measurements_4_DLC + 2] = {0};
            uint8_t ide = measurements_5_IDE;
            uint8_t dlc = measurements_5_DLC;
            data[0] = measurements_5_CANID;
            data[1] = measurements_5_DLC;
            Pack_measurements_5_jlb(&signal_sender.jlb_rx_t.measurements_5, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, measurements_5_DLC + 2);
        }

        void odometry_1()
        {
            signal_sender.jlb_rx_t.odometry_1.position_x_phys = odometry.x_t;
            signal_sender.jlb_rx_t.odometry_1.position_y_phys = odometry.y_t;
            signal_sender.jlb_rx_t.odometry_1.orientation_phys = odometry.theta_t;

            char data[odometry_1_DLC + 2] = {0};
            uint8_t ide = odometry_1_IDE;
            uint8_t dlc = odometry_1_DLC;
            data[0] = odometry_1_CANID;
            data[1] = odometry_1_DLC;
            Pack_odometry_1_jlb(&signal_sender.jlb_rx_t.odometry_1, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, odometry_1_DLC + 2);
        }

        void odometry_2()
        {
            signal_sender.jlb_rx_t.odometry_2.linear_velocity_x_phys = odometry.vx_t;
            signal_sender.jlb_rx_t.odometry_2.angular_velocity_z_phys = odometry.w_t;

            char data[odometry_2_DLC + 2] = {0};
            uint8_t ide = odometry_2_IDE;
            uint8_t dlc = odometry_2_DLC;
            data[0] = odometry_2_CANID;
            data[1] = odometry_2_DLC;
            Pack_odometry_2_jlb(&signal_sender.jlb_rx_t.odometry_2, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, odometry_2_DLC + 2);
        }

        void logic_1()
        {
            signal_sender.jlb_rx_t.logic_1.target_angle_phys = controller.target_angle;
            signal_sender.jlb_rx_t.logic_1.target_speed_phys = controller.target_speed;

            char data[logic_1_DLC + 2] = {0};
            uint8_t ide = logic_1_IDE;
            uint8_t dlc = logic_1_DLC;
            data[0] = logic_1_CANID;
            data[1] = logic_1_DLC;
            Pack_logic_1_jlb(&signal_sender.jlb_rx_t.logic_1, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            signal_sender.send(data, logic_1_DLC + 2);
        }

        void send_telemetry()
        {
            measurements_1();
            measurements_2();
            measurements_3();
            measurements_4();
            measurements_5();
            odometry_1();
            odometry_2();
            logic_1();
        }

    private:
        SignalSender &signal_sender = SignalSender::get_instance();
    };

} // namespace jlb

#endif // LOGIC_HXX