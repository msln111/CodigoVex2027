#include "robot.h"
#include <cmath>
#include <cstdint>

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Iniciando");
    encoderhorizontal.reset_position();
    encodervertical.reset_position();
    imu.reset();
    while (imu.is_calibrating()) { pros::delay(20); }
    reset_odometry(0.0, 0.0, 0.0);
    pros::lcd::set_text(0, "IMU lista");
    pros::Task odometry_task(tareaOdometria, nullptr, "Tarea de odometria");
    pros::Task display_task(tareaPantalla, nullptr, "Tarea de pantalla");
}

void disabled() {
    stop_drive();
}

void competition_initialize() {
    stop_drive();
}

double apply_joystick_curve(double stick_value, double sensitivity_power) {
    double normalized = stick_value / 127.0;
    double sign = normalized >= 0.0 ? 1.0 : -1.0;
    double magnitude = std::pow(std::abs(normalized), sensitivity_power);
    return sign * magnitude * 127.0;
}

void opcontrol() {
    // MODIFICABLE: DEADBAND ignora movimientos pequeños de los joysticks.
    constexpr int DEADBAND = 10;
    // MODIFICABLE: estas sensibilidades cambian la respuesta de los joysticks.
    constexpr double FORWARD_SENSITIVITY = 2.0;
    constexpr double TURN_SENSITIVITY = 1.5;
    // MODIFICABLE: limita cuánto puede cambiar la velocidad de giro por ciclo.
    constexpr double MAX_TURN_ACCEL = 50.0;

    double previous_turn_speed = 0.0;
    bool previous_B = false;
    bool previous_DOWN = false;

    while (true) {
        // Pulsa B + ABAJO a la vez para activar la rutina autónoma.
        bool current_B = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool current_DOWN = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool both_pressed = current_B && current_DOWN;
        bool both_just_pressed = both_pressed && (!previous_B || !previous_DOWN);

        if (both_just_pressed) {
            pros::lcd::clear_line(7);
            pros::lcd::set_text(7, "Ejecutando autonomo");
            autonomous();
            stop_drive();
            previous_turn_speed = 0.0;
            pros::lcd::clear_line(7);
            pros::lcd::set_text(7, "Autonomo terminado");
        }

        previous_B = current_B;
        previous_DOWN = current_DOWN;

        int forward_raw = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn_raw = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (std::abs(forward_raw) < DEADBAND) { forward_raw = 0; }
        if (std::abs(turn_raw) < DEADBAND) { turn_raw = 0; }

        double forward_curved = apply_joystick_curve(forward_raw, FORWARD_SENSITIVITY);
        double turn_curved = apply_joystick_curve(turn_raw, TURN_SENSITIVITY);
        double forward_rpm = forward_curved * (MAX_MOTOR_RPM / 127.0);
        double turn_rpm = turn_curved * (MAX_TURN_RPM / 127.0);
        double turn_delta = turn_rpm - previous_turn_speed;

        if (turn_delta > MAX_TURN_ACCEL) {
            turn_rpm = previous_turn_speed + MAX_TURN_ACCEL;
        } else if (turn_delta < -MAX_TURN_ACCEL) {
            turn_rpm = previous_turn_speed - MAX_TURN_ACCEL;
        }

        previous_turn_speed = turn_rpm;
        double left_speed = forward_rpm - turn_rpm;
        double right_speed = forward_rpm + turn_rpm;
        set_tank_speed(left_speed, right_speed);
        pros::delay(20);
    }
}
