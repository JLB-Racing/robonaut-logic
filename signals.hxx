#ifndef SIGNALS_HXX
#define SIGNALS_HXX

#include <string>
#include <vector>

#include "as_state.hxx"
#include "common.hxx"
#include "controller.hxx"
#include "graph.hxx"
#include "jlb-binutil.h"
#include "measurements.hxx"
#include "odometry.hxx"
#include "udp.hxx"

#ifndef SIMULATION
#include "main.h"
#include "stm32l5xx_hal.h"
#include "cmsis_os.h"
extern UART_HandleTypeDef huart2;
#endif

namespace jlb
{
    class SignalSender
    {
    public:
        jlb_rx_t            jlb_rx;
        const Odometry     &odometry;
        const Controller   &controller;
        const ASState      &as_state;
        const Graph        &graph;
        const Measurements &measurements;

        std::vector<char> telemetry_data;

#ifndef SIMULATION
        // TODO: initialize UDPClient for STM32
        SignalSender(const Odometry     &odometry_,
                     const Controller   &controller_,
                     const ASState      &as_state_,
                     const Graph        &graph_,
                     const Measurements &measurements_)
            : odometry(odometry_), controller(controller_), as_state(as_state_), graph(graph_), measurements(measurements_)
        {
        }
#else
        SignalSender(const Odometry     &odometry_,
                     const Controller   &controller_,
                     const ASState      &as_state_,
                     const Graph        &graph_,
                     const Measurements &measurements_)
            : odometry(odometry_),
              controller(controller_),
              as_state(as_state_),
              graph(graph_),
              measurements(measurements_),
              client(SENDER_ADDRESS, SENDER_PORT)
        {
        }
#endif

        ~SignalSender() {}

        void send_telemetry()
        {
            telemetry_data.clear();
            measurements_1();
            measurements_2();
            measurements_3();
            measurements_4();
            measurements_5();
            odometry_1();
            odometry_2();
            logic_1();
            logic_2();
            logic3();

#ifndef SIMULATION
            uint32_t timestamp = HAL_GetTick();
#else
            uint32_t timestamp =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - std::chrono::steady_clock::time_point::min())
                    .count();
#endif
            telemetry_data.push_back((timestamp >> 24u) & 0xFF);
            telemetry_data.push_back((timestamp >> 16u) & 0xFF);
            telemetry_data.push_back((timestamp >> 8u) & 0xFF);
            telemetry_data.push_back(timestamp & 0xFF);
            send(telemetry_data.data(), telemetry_data.size());

        }

    private:
#ifndef SIMULATION
        // TODO: add UDPClient for STM32
#else
        UDPClient client;
#endif

        int send([[maybe_unused]] char *msg, [[maybe_unused]] size_t max_size)
        {
#ifndef SIMULATION
            HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(msg), max_size, HAL_MAX_DELAY);
            return 0;
#else
            return client.send(msg, max_size);
#endif
        }

        void measurements_1()
        {
            jlb_rx.measurements_1.line_sensor_1 =
                0 == controller.selected_front ? controller.detection_front[0] + 2.0f : controller.detection_front[0];
            jlb_rx.measurements_1.line_sensor_2 =
                1 == controller.selected_front ? controller.detection_front[1] + 2.0f : controller.detection_front[1];
            jlb_rx.measurements_1.line_sensor_3 =
                2 == controller.selected_front ? controller.detection_front[2] + 2.0f : controller.detection_front[2];
            jlb_rx.measurements_1.line_sensor_4 =
                3 == controller.selected_front ? controller.detection_front[3] + 2.0f : controller.detection_front[3];
            jlb_rx.measurements_1.line_sensor_5 =
                4 == controller.selected_front ? controller.detection_front[4] + 2.0f : controller.detection_front[4];
            jlb_rx.measurements_1.line_sensor_6 =
                5 == controller.selected_front ? controller.detection_front[5] + 2.0f : controller.detection_front[5];
            jlb_rx.measurements_1.line_sensor_7 =
                6 == controller.selected_front ? controller.detection_front[6] + 2.0f : controller.detection_front[6];
            jlb_rx.measurements_1.line_sensor_8 =
                7 == controller.selected_front ? controller.detection_front[7] + 2.0f : controller.detection_front[7];
             jlb_rx.measurements_1.line_sensor_9  = 8 == controller.selected_front ? controller.detection_front[8] + 2.0f :
             controller.detection_front[8]; jlb_rx.measurements_1.line_sensor_10 = 9 == controller.selected_front ? controller.detection_front[9]
             + 2.0f : controller.detection_front[9]; jlb_rx.measurements_1.line_sensor_11 = 10 == controller.selected_front ?
             controller.detection_front[10] + 2.0f : controller.detection_front[10]; jlb_rx.measurements_1.line_sensor_12 = 11 ==
             controller.selected_front ? controller.detection_front[11] + 2.0f : controller.detection_front[11];
             jlb_rx.measurements_1.line_sensor_13 = 12 == controller.selected_front ? controller.detection_front[12] + 2.0f :
             controller.detection_front[12]; jlb_rx.measurements_1.line_sensor_14 = 13 == controller.selected_front ? controller.detection_front[13]
             + 2.0f : controller.detection_front[13]; jlb_rx.measurements_1.line_sensor_15 = 14 == controller.selected_front ?
             controller.detection_front[14] + 2.0f : controller.detection_front[14]; jlb_rx.measurements_1.line_sensor_16 = 15 ==
             controller.selected_front ? controller.detection_front[15] + 2.0f : controller.detection_front[15];
#ifndef SIMULATION
            jlb_rx.measurements_1.line_sensor_17 =
                16 == controller.selected_front ? controller.detection_front[16] + 2.0f : controller.detection_front[16];
            jlb_rx.measurements_1.line_sensor_18 =
                17 == controller.selected_front ? controller.detection_front[17] + 2.0f : controller.detection_front[17];
            jlb_rx.measurements_1.line_sensor_19 =
                18 == controller.selected_front ? controller.detection_front[18] + 2.0f : controller.detection_front[18];
            jlb_rx.measurements_1.line_sensor_20 =
                19 == controller.selected_front ? controller.detection_front[19] + 2.0f : controller.detection_front[19];
            jlb_rx.measurements_1.line_sensor_21 =
                20 == controller.selected_front ? controller.detection_front[20] + 2.0f : controller.detection_front[20];
            jlb_rx.measurements_1.line_sensor_22 =
                21 == controller.selected_front ? controller.detection_front[21] + 2.0f : controller.detection_front[21];
            jlb_rx.measurements_1.line_sensor_23 =
                22 == controller.selected_front ? controller.detection_front[22] + 2.0f : controller.detection_front[22];
            jlb_rx.measurements_1.line_sensor_24 =
                23 == controller.selected_front ? controller.detection_front[23] + 2.0f : controller.detection_front[23];
            jlb_rx.measurements_1.line_sensor_25 =
                24 == controller.selected_front ? controller.detection_front[24] + 2.0f : controller.detection_front[24];
            jlb_rx.measurements_1.line_sensor_26 =
                25 == controller.selected_front ? controller.detection_front[25] + 2.0f : controller.detection_front[25];
            jlb_rx.measurements_1.line_sensor_27 =
                26 == controller.selected_front ? controller.detection_front[26] + 2.0f : controller.detection_front[26];
            jlb_rx.measurements_1.line_sensor_28 =
                27 == controller.selected_front ? controller.detection_front[27] + 2.0f : controller.detection_front[27];
            jlb_rx.measurements_1.line_sensor_29 =
                28 == controller.selected_front ? controller.detection_front[28] + 2.0f : controller.detection_front[28];
            jlb_rx.measurements_1.line_sensor_30 =
                29 == controller.selected_front ? controller.detection_front[29] + 2.0f : controller.detection_front[29];
            jlb_rx.measurements_1.line_sensor_31 =
                30 == controller.selected_front ? controller.detection_front[30] + 2.0f : controller.detection_front[30];
            jlb_rx.measurements_1.line_sensor_32 =
                31 == controller.selected_front ? controller.detection_front[31] + 2.0f : controller.detection_front[31];
#endif

            char    data[measurements_1_DLC + 2] = {0};
            uint8_t ide                          = measurements_1_IDE;
            uint8_t dlc                          = measurements_1_DLC;
            data[0]                              = measurements_1_CANID;
            data[1]                              = measurements_1_DLC;
            Pack_measurements_1_jlb(&jlb_rx.measurements_1, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            telemetry_data.insert(telemetry_data.end(), data, data + measurements_1_DLC + 2);
        }

        void measurements_2()
        {
            jlb_rx.measurements_2.line_sensor_1 = 0 == controller.selected_rear ? controller.detection_rear[0] + 2.0f : controller.detection_rear[0];
            jlb_rx.measurements_2.line_sensor_2 = 1 == controller.selected_rear ? controller.detection_rear[1] + 2.0f : controller.detection_rear[1];
            jlb_rx.measurements_2.line_sensor_3 = 2 == controller.selected_rear ? controller.detection_rear[2] + 2.0f : controller.detection_rear[2];
            jlb_rx.measurements_2.line_sensor_4 = 3 == controller.selected_rear ? controller.detection_rear[3] + 2.0f : controller.detection_rear[3];
            jlb_rx.measurements_2.line_sensor_5 = 4 == controller.selected_rear ? controller.detection_rear[4] + 2.0f : controller.detection_rear[4];
            jlb_rx.measurements_2.line_sensor_6 = 5 == controller.selected_rear ? controller.detection_rear[5] + 2.0f : controller.detection_rear[5];
            jlb_rx.measurements_2.line_sensor_7 = 6 == controller.selected_rear ? controller.detection_rear[6] + 2.0f : controller.detection_rear[6];
            jlb_rx.measurements_2.line_sensor_8 = 7 == controller.selected_rear ? controller.detection_rear[7] + 2.0f : controller.detection_rear[7];
             jlb_rx.measurements_2.line_sensor_9  = 8 == controller.selected_rear ? controller.detection_rear[8] + 2.0f :
             controller.detection_rear[8]; jlb_rx.measurements_2.line_sensor_10 = 9 == controller.selected_rear ? controller.detection_rear[9]
             + 2.0f : controller.detection_rear[9]; jlb_rx.measurements_2.line_sensor_11 = 10 == controller.selected_rear ?
             controller.detection_rear[10] + 2.0f : controller.detection_rear[10]; jlb_rx.measurements_2.line_sensor_12 = 11 ==
             controller.selected_rear ? controller.detection_rear[11] + 2.0f : controller.detection_rear[11]; jlb_rx.measurements_2.line_sensor_13 =
             12 == controller.selected_rear ? controller.detection_rear[12] + 2.0f : controller.detection_rear[12];
             jlb_rx.measurements_2.line_sensor_14 = 13 == controller.selected_rear ? controller.detection_rear[13] + 2.0f :
             controller.detection_rear[13]; jlb_rx.measurements_2.line_sensor_15 = 14 == controller.selected_rear ? controller.detection_rear[14]
             + 2.0f : controller.detection_rear[14]; jlb_rx.measurements_2.line_sensor_16 = 15 == controller.selected_rear ?
             controller.detection_rear[15] + 2.0f : controller.detection_rear[15];
#ifndef SIMULATION
            jlb_rx.measurements_2.line_sensor_9  = 8 == controller.selected_rear ? controller.detection_rear[8] + 2.0f : controller.detection_rear[8];
            jlb_rx.measurements_2.line_sensor_10 = 9 == controller.selected_rear ? controller.detection_rear[9] + 2.0f : controller.detection_rear[9];
            jlb_rx.measurements_2.line_sensor_11 =
                10 == controller.selected_rear ? controller.detection_rear[10] + 2.0f : controller.detection_rear[10];
            jlb_rx.measurements_2.line_sensor_12 =
                11 == controller.selected_rear ? controller.detection_rear[11] + 2.0f : controller.detection_rear[11];
            jlb_rx.measurements_2.line_sensor_13 =
                12 == controller.selected_rear ? controller.detection_rear[12] + 2.0f : controller.detection_rear[12];
            jlb_rx.measurements_2.line_sensor_14 =
                13 == controller.selected_rear ? controller.detection_rear[13] + 2.0f : controller.detection_rear[13];
            jlb_rx.measurements_2.line_sensor_15 =
                14 == controller.selected_rear ? controller.detection_rear[14] + 2.0f : controller.detection_rear[14];
            jlb_rx.measurements_2.line_sensor_16 =
                15 == controller.selected_rear ? controller.detection_rear[15] + 2.0f : controller.detection_rear[15];
            jlb_rx.measurements_2.line_sensor_17 =
                16 == controller.selected_rear ? controller.detection_rear[16] + 2.0f : controller.detection_rear[16];
            jlb_rx.measurements_2.line_sensor_18 =
                17 == controller.selected_rear ? controller.detection_rear[17] + 2.0f : controller.detection_rear[17];
            jlb_rx.measurements_2.line_sensor_19 =
                18 == controller.selected_rear ? controller.detection_rear[18] + 2.0f : controller.detection_rear[18];
            jlb_rx.measurements_2.line_sensor_20 =
                19 == controller.selected_rear ? controller.detection_rear[19] + 2.0f : controller.detection_rear[19];
            jlb_rx.measurements_2.line_sensor_21 =
                20 == controller.selected_rear ? controller.detection_rear[20] + 2.0f : controller.detection_rear[20];
            jlb_rx.measurements_2.line_sensor_22 =
                21 == controller.selected_rear ? controller.detection_rear[21] + 2.0f : controller.detection_rear[21];
            jlb_rx.measurements_2.line_sensor_23 =
                22 == controller.selected_rear ? controller.detection_rear[22] + 2.0f : controller.detection_rear[22];
            jlb_rx.measurements_2.line_sensor_24 =
                23 == controller.selected_rear ? controller.detection_rear[23] + 2.0f : controller.detection_rear[23];
            jlb_rx.measurements_2.line_sensor_25 =
                24 == controller.selected_rear ? controller.detection_rear[24] + 2.0f : controller.detection_rear[24];
            jlb_rx.measurements_2.line_sensor_26 =
                25 == controller.selected_rear ? controller.detection_rear[25] + 2.0f : controller.detection_rear[25];
            jlb_rx.measurements_2.line_sensor_27 =
                26 == controller.selected_rear ? controller.detection_rear[26] + 2.0f : controller.detection_rear[26];
            jlb_rx.measurements_2.line_sensor_28 =
                27 == controller.selected_rear ? controller.detection_rear[27] + 2.0f : controller.detection_rear[27];
            jlb_rx.measurements_2.line_sensor_29 =
                28 == controller.selected_rear ? controller.detection_rear[28] + 2.0f : controller.detection_rear[28];
            jlb_rx.measurements_2.line_sensor_30 =
                29 == controller.selected_rear ? controller.detection_rear[29] + 2.0f : controller.detection_rear[29];
            jlb_rx.measurements_2.line_sensor_31 =
                30 == controller.selected_rear ? controller.detection_rear[30] + 2.0f : controller.detection_rear[30];
            jlb_rx.measurements_2.line_sensor_32 =
                31 == controller.selected_rear ? controller.detection_rear[31] + 2.0f : controller.detection_rear[31];
#endif

            char    data[measurements_2_DLC + 2] = {0};
            uint8_t ide                          = measurements_2_IDE;
            uint8_t dlc                          = measurements_2_DLC;
            data[0]                              = measurements_2_CANID;
            data[1]                              = measurements_2_DLC;
            Pack_measurements_2_jlb(&jlb_rx.measurements_2, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            telemetry_data.insert(telemetry_data.end(), data, data + measurements_2_DLC + 2);
        }

        void measurements_3()
        {
            jlb_rx.measurements_3.angular_velocity_x_phys = odometry.meas_ang_vel_x;
            jlb_rx.measurements_3.angular_velocity_y_phys = odometry.meas_ang_vel_y;
            jlb_rx.measurements_3.angular_velocity_z_phys = odometry.meas_ang_vel_z;

            char    data[measurements_2_DLC + 2] = {0};
            uint8_t ide                          = measurements_3_IDE;
            uint8_t dlc                          = measurements_3_DLC;
            data[0]                              = measurements_3_CANID;
            data[1]                              = measurements_3_DLC;
            Pack_measurements_3_jlb(&jlb_rx.measurements_3, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            telemetry_data.insert(telemetry_data.end(), data, data + measurements_3_DLC + 2);
        }

        void measurements_4()
        {
            jlb_rx.measurements_4.linear_acceleration_x_phys = odometry.meas_lin_acc_x;
            jlb_rx.measurements_4.linear_acceleration_y_phys = odometry.meas_lin_acc_y;
            jlb_rx.measurements_4.linear_acceleration_z_phys = odometry.meas_lin_acc_z;

            char    data[measurements_3_DLC + 2] = {0};
            uint8_t ide                          = measurements_4_IDE;
            uint8_t dlc                          = measurements_4_DLC;
            data[0]                              = measurements_4_CANID;
            data[1]                              = measurements_4_DLC;
            Pack_measurements_4_jlb(&jlb_rx.measurements_4, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            telemetry_data.insert(telemetry_data.end(), data, data + measurements_4_DLC + 2);
        }

        void measurements_5()
        {
            jlb_rx.measurements_5.duty_cycle_phys    = measurements.duty_cycle;
            jlb_rx.measurements_5.motor_current_phys = measurements.motor_current;
            jlb_rx.measurements_5.object_range_phys  = measurements.object_range;
            jlb_rx.measurements_5.wheel_rpm_phys     = measurements.wheel_rpm;

            char    data[measurements_4_DLC + 2] = {0};
            uint8_t ide                          = measurements_5_IDE;
            uint8_t dlc                          = measurements_5_DLC;
            data[0]                              = measurements_5_CANID;
            data[1]                              = measurements_5_DLC;
            Pack_measurements_5_jlb(&jlb_rx.measurements_5, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            telemetry_data.insert(telemetry_data.end(), data, data + measurements_5_DLC + 2);
        }

        void odometry_1()
        {
            jlb_rx.odometry_1.position_x_phys  = odometry.x_t;
            jlb_rx.odometry_1.position_y_phys  = odometry.y_t;
            jlb_rx.odometry_1.orientation_phys = odometry.theta_t;

            char    data[odometry_1_DLC + 2] = {0};
            uint8_t ide                      = odometry_1_IDE;
            uint8_t dlc                      = odometry_1_DLC;
            data[0]                          = odometry_1_CANID;
            data[1]                          = odometry_1_DLC;
            Pack_odometry_1_jlb(&jlb_rx.odometry_1, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            telemetry_data.insert(telemetry_data.end(), data, data + odometry_1_DLC + 2);
        }

        void odometry_2()
        {
            jlb_rx.odometry_2.linear_velocity_x_phys  = odometry.vx_t;
            jlb_rx.odometry_2.angular_velocity_z_phys = odometry.w_t;

            char    data[odometry_2_DLC + 2] = {0};
            uint8_t ide                      = odometry_2_IDE;
            uint8_t dlc                      = odometry_2_DLC;
            data[0]                          = odometry_2_CANID;
            data[1]                          = odometry_2_DLC;
            Pack_odometry_2_jlb(&jlb_rx.odometry_2, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            telemetry_data.insert(telemetry_data.end(), data, data + odometry_2_DLC + 2);
        }

        void logic_1()
        {
            jlb_rx.logic_1.target_angle_phys      = controller.target_angle;
            jlb_rx.logic_1.target_speed_phys      = controller.target_speed;
            jlb_rx.logic_1.cross_track_error_phys = controller.cross_track_error;
            jlb_rx.logic_1.heading_error_phys     = controller.heading_error;

            char    data[logic_1_DLC + 2] = {0};
            uint8_t ide                   = logic_1_IDE;
            uint8_t dlc                   = logic_1_DLC;
            data[0]                       = logic_1_CANID;
            data[1]                       = logic_1_DLC;
            Pack_logic_1_jlb(&jlb_rx.logic_1, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            telemetry_data.insert(telemetry_data.end(), data, data + logic_1_DLC + 2);
        }

        void logic_2()
        {
            jlb_rx.logic_2.distance_traveled_phys = odometry.distance_local;
            jlb_rx.logic_2.labyrinth_state        = static_cast<uint8_t>(as_state.labyrinth_state);
            jlb_rx.logic_2.fast_state             = static_cast<uint8_t>(as_state.fast_state);
            jlb_rx.logic_2.next_node              = as_state.next_node;
            jlb_rx.logic_2.previous_node          = as_state.previous_node;
            jlb_rx.logic_2.direction              = static_cast<uint8_t>(controller.direction);
            jlb_rx.logic_2.mission                = static_cast<uint8_t>(as_state.mission);

            char    data[logic_2_DLC + 2] = {0};
            uint8_t ide                   = logic_2_IDE;
            uint8_t dlc                   = logic_2_DLC;
            data[0]                       = logic_2_CANID;
            data[1]                       = logic_2_DLC;
            Pack_logic_2_jlb(&jlb_rx.logic_2, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            telemetry_data.insert(telemetry_data.end(), data, data + logic_2_DLC + 2);
        }

        void logic3()
        {
            jlb_rx.logic_3.ang_error_norm_phys      = controller.ang_error_norm;
            jlb_rx.logic_3.dist_error_norm_phys     = controller.dist_error_norm;
            jlb_rx.logic_3.line_position_rear_phys  = controller.line_position_rear;
            jlb_rx.logic_3.line_position_front_phys = controller.line_position_front;
            jlb_rx.logic_3.at_cross_section         = as_state.at_cross_section;
            jlb_rx.logic_3.under_gate               = as_state.under_gate;

            char    data[logic_3_DLC + 2] = {0};
            uint8_t ide                   = logic_3_IDE;
            uint8_t dlc                   = logic_3_DLC;
            data[0]                       = logic_3_CANID;
            data[1]                       = logic_3_DLC;
            Pack_logic_3_jlb(&jlb_rx.logic_3, reinterpret_cast<uint8_t *>(data + 2), &dlc, &ide);
            telemetry_data.insert(telemetry_data.end(), data, data + logic_3_DLC + 2);
        }
    };

}  // namespace jlb

#endif  // SIGNALS_HXX
