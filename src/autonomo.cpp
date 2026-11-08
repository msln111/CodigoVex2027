#include "robot.h"
#include <cmath>
#include <cstdint>

// valor variable: velocidad maxima y minima
constexpr double AUTO_MAX_RPM = 200.0;
constexpr double AUTO_MIN_RPM = 30.0;
constexpr double KP_DISTANCE = 12.0;
constexpr double KP_HEADING = 4.0;
constexpr double HEADING_CORRECTION_LIMIT = 100.0;
constexpr double DISTANCE_TOLERANCE = 0.75;
constexpr uint32_t TIMEOUT_MS = 5000;
// valor variable: cancelacion del autonomo
constexpr int JOYSTICK_ABORT = 60;

// avance de la distancia
void avanzar(double distancia){
    double x0, y0, theta0;
    get_position(x0, y0, theta0);
    const double start_heading_rad = theta0 * M_PI / 180.0;
    const uint32_t start_time = pros::millis();

    while (pros::millis() - start_time < TIMEOUT_MS){
        //mover el joystick izquierdo cancela
        if (std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > JOYSTICK_ABORT){ break; }

        double x, y, theta;
        get_position(x, y, theta);

        // distancia recorrida
        double traveled = ((x - x0) * std::sin(start_heading_rad)) + ((y - y0) * std::cos(start_heading_rad));
        double error = distancia - traveled;

        if (std::fabs(error) <= DISTANCE_TOLERANCE){ break; }
        // proteccion contra error
        if (std::fabs(error) > std::fabs(distancia) + 6){
            pros::lcd::set_text(6, "SENTIDO INVERTIDO");
            break;
        }
        double forward_rpm = clamp_speed(KP_DISTANCE * error, -AUTO_MAX_RPM, AUTO_MAX_RPM);
        if (std::fabs(forward_rpm) < AUTO_MIN_RPM){ forward_rpm = error > 0.0 ? AUTO_MIN_RPM : -AUTO_MIN_RPM; }

        // correccion de rumbo
        double heading_error = normalize_angle_degrees(theta0 - theta);
        double turn_rpm = clamp_speed(KP_HEADING * heading_error, -HEADING_CORRECTION_LIMIT, HEADING_CORRECTION_LIMIT);

        set_drive_rpm(forward_rpm, turn_rpm);
        pros::delay(20);
    }
    stop_drive();
}

void autonomous(){
    reset_odometry(0.0, 0.0, 0.0);
    pros::delay(200);
    avanzar(24.0);
    pros::delay(200);
    stop_drive();
}