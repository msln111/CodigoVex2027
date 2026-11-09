#include "robot.h"
#include <cmath>
#include <cstdint>

// valor variable: velocidad maxima y minima
constexpr double AUTO_MAX_RPM = 200.0;
constexpr double AUTO_MIN_RPM = 100.0;
constexpr double KP_DISTANCE = 12.0;
constexpr double KP_HEADING = 4.0;
constexpr double HEADING_CORRECTION_LIMIT = 100.0;
constexpr double DISTANCE_TOLERANCE = 0.75;
constexpr uint32_t TIMEOUT_MS = 5000;
// valor variable: cancelacion del autonomo
constexpr int JOYSTICK_ABORT = 60;

// valor variable: giro en sitio (KP en rpm por grado, KD en rpm por grado/segundo, velocidades en rpm, tolerancia en grados)
constexpr double KP_TURN = 3.0;
constexpr double KD_TURN = 0.08;
constexpr double TURN_MAX_RPM = 150.0;
constexpr double TURN_MIN_RPM = 100.0;
constexpr double TURN_TOLERANCE = 1.5;
constexpr uint32_t TURN_TIMEOUT_MS = 3000;
// valor variable: si el rumbo pedido difiere mas de estos grados, avanzar() gira primero
constexpr double ALIGN_TOLERANCE = 3.0;

// mover el joystick izquierdo cancela
static bool abortado(){
    return std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > JOYSTICK_ABORT;
}

// gira en sitio hasta el rumbo absoluto objetivoTheta (en grados)
void girarAngulo(double objetivoTheta){
    const uint32_t start_time = pros::millis();
    double previous_error = 0.0;
    bool first_loop = true;
    int settled_count = 0;

    while (pros::millis() - start_time < TURN_TIMEOUT_MS){
        if (abortado()){ break; }

        double x, y, theta;
        get_position(x, y, theta);
        double error = normalize_angle_degrees(objetivoTheta - theta);
        double derivative = first_loop ? 0.0 : (error - previous_error) / 0.02;
        previous_error = error;
        first_loop = false;

        // dentro de la tolerancia: frena y espera unos ciclos para confirmar que se quedo quieto
        if (std::fabs(error) <= TURN_TOLERANCE){
            stop_drive();
            settled_count++;
            if (settled_count >= 5){ break; }
            pros::delay(20);
            continue;
        }
        settled_count = 0;

        double turn_rpm = clamp_speed((KP_TURN * error) + (KD_TURN * derivative), -TURN_MAX_RPM, TURN_MAX_RPM);
        if (std::fabs(turn_rpm) < TURN_MIN_RPM){ turn_rpm = error > 0.0 ? TURN_MIN_RPM : -TURN_MIN_RPM; }

        set_drive_rpm(0.0, turn_rpm);
        pros::delay(20);
    }
    stop_drive();
}

// avanza la distancia (negativa = reversa) sobre el rumbo indicado; si el robot no mira a ese rumbo, gira primero
void avanzar(double distancia, double rumbo){
    double x0, y0, theta0;
    get_position(x0, y0, theta0);

    if (std::fabs(normalize_angle_degrees(rumbo - theta0)) > ALIGN_TOLERANCE){
        girarAngulo(rumbo);
        get_position(x0, y0, theta0);
    }

    const double heading_rad = rumbo * M_PI / 180.0;
    const uint32_t start_time = pros::millis();

    while (pros::millis() - start_time < TIMEOUT_MS){
        if (abortado()){ break; }

        double x, y, theta;
        get_position(x, y, theta);

        // distancia recorrida sobre el rumbo
        double traveled = ((x - x0) * std::sin(heading_rad)) + ((y - y0) * std::cos(heading_rad));
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
        double heading_error = normalize_angle_degrees(rumbo - theta);
        double turn_rpm = clamp_speed(KP_HEADING * heading_error, -HEADING_CORRECTION_LIMIT, HEADING_CORRECTION_LIMIT);

        set_drive_rpm(forward_rpm, turn_rpm);
        pros::delay(20);
    }
    stop_drive();
}

// avanza la distancia con el rumbo que tiene el robot en este momento
void avanzar(double distancia){
    double x, y, theta;
    get_position(x, y, theta);
    avanzar(distancia, theta);
}

// va a un punto: gira hacia el, avanza y al final gira a objetivoTheta (pasa NAN para no girar al final)
void moverAPunto(double objetivoX, double objetivoY, double objetivoTheta){
    double x, y, theta;
    get_position(x, y, theta);
    const double dx = objetivoX - x;
    const double dy = objetivoY - y;
    const double distance = std::hypot(dx, dy);

    if (distance > DISTANCE_TOLERANCE){
        // rumbo hacia el punto, con la misma convencion que la odometria
        const double rumbo_al_punto = std::atan2(dx, dy) * 180.0 / M_PI;
        avanzar(distance, rumbo_al_punto);
    }
    if (!std::isnan(objetivoTheta)){
        girarAngulo(objetivoTheta);
    }
}

void autonomous(){
    reset_odometry(0.0, 0.0, 0.0);
    pros::delay(200);
    avanzar(24.0);                  // recto 24 in con el rumbo actual
    girarAngulo(90.0);              // gira a la derecha hasta quedar a 90 grados
    avanzar(24.0, 90.0);            // 24 in sobre el rumbo de 90 grados
    pros::delay(200);
    stop_drive();
}
