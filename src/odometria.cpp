#include "robot.h"
#include <cmath>
#include <string>

// Posición global del robot.
double globalX = 0.0;
double globalY = 0.0;
double globalTheta = 0.0;
pros::Mutex odometry_mutex;

// Sensores.
// MODIFICABLE: cambia estos puertos si conectas los encoders o el IMU a otros puertos.
pros::Rotation encoderhorizontal(11);
pros::Rotation encodervertical(-12);
pros::IMU imu(10);

// Motores.
// MODIFICABLE: cambia los puertos o signos para ajustarlos a la configuración del robot.
pros::MotorGroup leftMotors({-3, 4, -16, 8});
pros::MotorGroup rightMotors({1, 6, -2, -5});
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Calibración de las ruedas de seguimiento.
// MODIFICABLE: ajusta el diámetro si las ruedas de los encoders tienen otro tamaño.
constexpr double TRACKING_WHEEL_DIAMETER = 2.0;
constexpr double CENTIDEGREES_PER_REVOLUTION = 36000.0;
constexpr double TRACKING_WHEEL_CIRCUMFERENCE = M_PI * TRACKING_WHEEL_DIAMETER;
constexpr double INCHES_PER_CENTIDEGREE = TRACKING_WHEEL_CIRCUMFERENCE / CENTIDEGREES_PER_REVOLUTION;

// Ubicación de las ruedas de seguimiento respecto al centro de gravedad.
// MODIFICABLE: estos desplazamientos corrigen el movimiento medido durante los giros.
constexpr double WHEEL_FORWARD_OFFSET = 0.875;
constexpr double WHEEL_SIDE_OFFSET = 1.875;
constexpr pros::Rotation &ENCODER_FOR_X = encoderhorizontal;
constexpr pros::Rotation &ENCODER_FOR_Y = encodervertical;

// Lecturas anteriores de los sensores.
static double previousEncoderX = 0.0;
static double previousEncoderY = 0.0;
static double previousHeadingDegrees = 0.0;

double clamp_speed(double value, double minimum, double maximum) {
    if (value > maximum) { return maximum; }
    if (value < minimum) { return minimum; }
    return value;
}

double normalize_angle_degrees(double angle) {
    while (angle > 180.0) { angle -= 360.0; }
    while (angle < -180.0) { angle += 360.0; }
    return angle;
}

void get_position(double &x, double &y, double &theta) {
    odometry_mutex.take();
    x = globalX;
    y = globalY;
    theta = globalTheta;
    odometry_mutex.give();
}

void reset_odometry(double x, double y, double theta_degrees) {
    odometry_mutex.take();
    globalX = x;
    globalY = y;
    globalTheta = normalize_angle_degrees(theta_degrees);
    odometry_mutex.give();

    previousEncoderX = ENCODER_FOR_X.get_position();
    previousEncoderY = ENCODER_FOR_Y.get_position();
    previousHeadingDegrees = imu.get_heading();
}

void set_tank_speed(double left_speed, double right_speed) {
    double largest_requested_speed = std::fmax(std::fabs(left_speed), std::fabs(right_speed));

    if (largest_requested_speed > MAX_MOTOR_RPM) {
        double scale = MAX_MOTOR_RPM / largest_requested_speed;
        left_speed *= scale;
        right_speed *= scale;
    }

    left_speed = clamp_speed(left_speed, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
    right_speed = clamp_speed(right_speed, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
    leftMotors.move_velocity(left_speed);
    rightMotors.move_velocity(right_speed);
}

void stop_drive() {
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
}

// Tarea de actualización de odometría.
void tareaOdometria(void *param) {
    (void)param;
    previousEncoderX = ENCODER_FOR_X.get_position();
    previousEncoderY = ENCODER_FOR_Y.get_position();
    previousHeadingDegrees = imu.get_heading();

    while (true) {
        double currentEncoderX = ENCODER_FOR_X.get_position();
        double currentEncoderY = ENCODER_FOR_Y.get_position();
        double currentHeadingDegrees = imu.get_heading();
        double deltaEncoderX = currentEncoderX - previousEncoderX;
        double deltaEncoderY = currentEncoderY - previousEncoderY;

        // Ambos encoders usan la misma conversión porque sus ruedas tienen el mismo diámetro.
        double rawDeltaX = deltaEncoderX * INCHES_PER_CENTIDEGREE;
        double rawDeltaY = deltaEncoderY * INCHES_PER_CENTIDEGREE;
        double deltaHeadingDegrees = normalize_angle_degrees(currentHeadingDegrees - previousHeadingDegrees);
        double deltaHeadingRadians = deltaHeadingDegrees * M_PI / 180.0;
        double correctedDeltaX = rawDeltaX - (WHEEL_SIDE_OFFSET * deltaHeadingRadians);
        double correctedDeltaY = rawDeltaY - (WHEEL_FORWARD_OFFSET * deltaHeadingRadians);
        double headingRadians = currentHeadingDegrees * M_PI / 180.0;
        double cos_heading = std::cos(headingRadians);
        double sin_heading = std::sin(headingRadians);
        double fieldDeltaX = (correctedDeltaX * cos_heading) - (correctedDeltaY * sin_heading);
        double fieldDeltaY = (correctedDeltaX * sin_heading) + (correctedDeltaY * cos_heading);

        odometry_mutex.take();
        globalX += fieldDeltaX;
        globalY += fieldDeltaY;
        globalTheta += deltaHeadingDegrees;
        globalTheta = normalize_angle_degrees(globalTheta);
        odometry_mutex.give();

        previousEncoderX = currentEncoderX;
        previousEncoderY = currentEncoderY;
        previousHeadingDegrees = currentHeadingDegrees;
        pros::delay(10);
    }
}

// Tarea de visualización de la posición en la pantalla.
void tareaPantalla(void *param) {
    (void)param;

    while (true) {
        double x, y, theta;
        get_position(x, y, theta);
        pros::lcd::set_text(0, "X: " + std::to_string(x) + " in");
        pros::lcd::set_text(1, "Y: " + std::to_string(y) + " in");
        pros::lcd::set_text(2, "Orientacion: " + std::to_string(theta) + " grados");
        pros::delay(100);
    }
}