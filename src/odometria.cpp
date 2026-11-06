#include "robot.h"
#include <cmath>
#include <string>

// posicion global del robot
double globalX = 0.0;
double globalY = 0.0;
double globalTheta = 0.0;

// velocidades filtradas del robot (avance en in/s y giro en rad/s)
double globalV = 0.0;
double globalOmega = 0.0;
pros::Mutex odometry_mutex;

// sensores
pros::Rotation encoderhorizontal(11);
pros::Rotation encodervertical(-12);
pros::IMU imu(10);

// motores
pros::Motor garraMotor(4);
pros::MotorGroup upMotors({13, 14});
pros::MotorGroup cadenaMotors({15, 17});
pros::adi::DigitalOut pistonNeumatico(1);
pros::MotorGroup leftMotors({-3, 7, 16, -8});
pros::MotorGroup rightMotors({1, 6, -2, -5});
pros::Controller master(pros::E_CONTROLLER_MASTER);

// valor variable: ajusta el diametro si las ruedas de los encoders tienen otro tamaño
constexpr double TRACKING_WHEEL_DIAMETER = 2.0;
constexpr double CENTIDEGREES_PER_REVOLUTION = 36000.0;
constexpr double TRACKING_WHEEL_CIRCUMFERENCE = M_PI * TRACKING_WHEEL_DIAMETER;
constexpr double INCHES_PER_CENTIDEGREE = TRACKING_WHEEL_CIRCUMFERENCE / CENTIDEGREES_PER_REVOLUTION;

// valor variable: ubicacion de las ruedas de seguimiento respecto al centro de gravedad
constexpr double WHEEL_FORWARD_OFFSET = 0.875;
constexpr double WHEEL_SIDE_OFFSET = 1.875;

// valor variable: encoder que mide el avance y encoder que mide el lado izquierdo (intercambiar si tu montaje es al reves)
constexpr pros::Rotation &ENCODER_FORWARD = encoderhorizontal;
constexpr pros::Rotation &ENCODER_SIDE = encodervertical;

// valor variable: filtro de las velocidades medidas (0 a 1, mas alto responde mas rapido pero con mas ruido)
constexpr double ODOM_VEL_ALPHA = 0.3;

// lecturas anteriores de los sensores, del tiempo y offset del rumbo
static double previousForwardRaw = 0.0;
static double previousSideRaw = 0.0;
static double previousThetaDeg = 0.0;
static double headingOffsetDeg = 0.0;
static uint32_t previousTimeMs = 0;

// rumbo del IMU en grados antihorarios continuos, con el offset del ultimo reset
static double read_theta_deg() { return -imu.get_rotation() + headingOffsetDeg; }

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

// estado completo del robot (posicion, rumbo y velocidades) leido de forma segura
void get_state(RobotState &state) {
    odometry_mutex.take();
    state.x = globalX;
    state.y = globalY;
    state.thetaRad = globalTheta * M_PI / 180.0;
    state.v = globalV;
    state.omega = globalOmega;
    odometry_mutex.give();
}

// reinicia posicion, rumbo y velocidades; todo bajo el mutex para no chocar con la tarea de odometria
void reset_odometry(double x, double y, double theta_degrees) {
    odometry_mutex.take();
    headingOffsetDeg = theta_degrees + imu.get_rotation();
    previousThetaDeg = theta_degrees;
    globalX = x;
    globalY = y;
    globalTheta = normalize_angle_degrees(theta_degrees);
    globalV = 0.0;
    globalOmega = 0.0;
    previousForwardRaw = ENCODER_FORWARD.get_position();
    previousSideRaw = ENCODER_SIDE.get_position();
    previousTimeMs = pros::millis();
    odometry_mutex.give();
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

// convierte avance (in/s) y giro antihorario (rad/s) en RPM de cada lado y los manda a los motores
void set_drive_velocity(double v_in_s, double omega_rad_s) {
    double half_track = TRACK_WIDTH * 0.5;
    double left_rpm = (v_in_s - (omega_rad_s * half_track)) / IN_PER_S_PER_RPM;
    double right_rpm = (v_in_s + (omega_rad_s * half_track)) / IN_PER_S_PER_RPM;
    set_tank_speed(left_rpm, right_rpm);
}

void stop_drive() {
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}

// tarea de actualizacion de odometria
void tareaOdometria(void *param) {
    (void)param;

    while (true) {
        odometry_mutex.take();
        uint32_t now = pros::millis();
        double dt = (now - previousTimeMs) / 1000.0;
        double thetaDeg = read_theta_deg();
        double forwardRaw = ENCODER_FORWARD.get_position();
        double sideRaw = ENCODER_SIDE.get_position();

        if (dt > 0.0 && std::isfinite(thetaDeg)) {
            double dThetaDeg = normalize_angle_degrees(thetaDeg - previousThetaDeg);
            double dThetaRad = dThetaDeg * M_PI / 180.0;
            // cada encoder se corrige por su distancia al centro, igual que antes
            double dForward = ((forwardRaw - previousForwardRaw) * INCHES_PER_CENTIDEGREE) - (WHEEL_SIDE_OFFSET * dThetaRad);
            double dLeft = ((sideRaw - previousSideRaw) * INCHES_PER_CENTIDEGREE) - (WHEEL_FORWARD_OFFSET * dThetaRad);
            // se rota con el rumbo medio del intervalo para aproximar el arco
            double midRad = (previousThetaDeg + (0.5 * dThetaDeg)) * M_PI / 180.0;
            globalX += (dForward * std::cos(midRad)) - (dLeft * std::sin(midRad));
            globalY += (dForward * std::sin(midRad)) + (dLeft * std::cos(midRad));
            globalTheta = normalize_angle_degrees(thetaDeg);
            // velocidades filtradas que usa el MPC
            globalV = ((1.0 - ODOM_VEL_ALPHA) * globalV) + (ODOM_VEL_ALPHA * (dForward / dt));
            globalOmega = ((1.0 - ODOM_VEL_ALPHA) * globalOmega) + (ODOM_VEL_ALPHA * (dThetaRad / dt));
            previousForwardRaw = forwardRaw;
            previousSideRaw = sideRaw;
            previousThetaDeg = thetaDeg;
            previousTimeMs = now;
        }

        odometry_mutex.give();
        pros::delay(10);
    }
}

// tarea de visualizacion de la posicion en la pantalla
void tareaPantalla(void *param) {
    (void)param;

    while (true) {
        double x, y, theta;
        get_position(x, y, theta);
        pros::lcd::set_text(0, "X: " + std::to_string(x) + " in");
        pros::lcd::set_text(1, "Y: " + std::to_string(y) + " in");
        pros::lcd::set_text(2, "Orientacion: " + std::to_string(theta) + " grados");
        RobotState state;
        get_state(state);
        pros::lcd::set_text(3, "V: " + std::to_string(state.v) + " in/s");
        pros::lcd::set_text(4, "W: " + std::to_string(state.omega) + " rad/s");
        pros::delay(100);
    }
}