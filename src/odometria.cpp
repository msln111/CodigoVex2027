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
pros::Rotation encoderhorizontal(9);
pros::Rotation encodervertical(10);
pros::IMU imu(15);

// motores
pros::Motor garraMotor(4);
pros::MotorGroup upMotors({1, -2});
pros::MotorGroup cadenaMotors({8, 7});
pros::adi::DigitalOut pistonNeumatico(1);
pros::MotorGroup leftMotors({-14, 13, 12, -11}, pros::v5::MotorGears::blue);
pros::MotorGroup rightMotors({-19, -18, 17, 16}, pros::v5::MotorGears::blue);
pros::Controller master(pros::E_CONTROLLER_MASTER);

// valor variable: ajusta el diametro si las ruedas de los encoders tienen otro tamaño
constexpr double TRACKING_WHEEL_DIAMETER = 2.0;
constexpr double CENTIDEGREES_PER_REVOLUTION = 36000.0;
constexpr double INCHES_PER_CENTIDEGREE = (M_PI * TRACKING_WHEEL_DIAMETER) / CENTIDEGREES_PER_REVOLUTION;

// valor variable: encoder de avance
constexpr pros::Rotation &ENCODER_FORWARD = encodervertical;
constexpr pros::Rotation &ENCODER_SIDE = encoderhorizontal;
// valor variable: sentido del encoder
constexpr double FORWARD_SIGN = 1.0;
constexpr double SIDE_SIGN = 1.0;
// valor variables: offset de los encoders
constexpr double SIDE_OFFSET_IN = 0.0;
constexpr double FORWARD_OFFSET_IN = 0.0;
// valor variable: sentido del imu
constexpr double IMU_SIGN = 1.0;
// valor variable: sentido de los motores en auton
constexpr double DRIVE_POLARITY = -1.0;

// lecturas anteriores del encoder, rumbo y offset
static double previousForwardRaw = 0.0;
static double previousSideRaw = 0.0;
static double previousThetaDeg = 0.0;
static double headingOffsetDeg = 0.0;

// rumbo del IMU
static double read_theta_deg() {
    return (IMU_SIGN * imu.get_rotation()) + headingOffsetDeg;
}
double clamp_speed(double value, double minimum, double maximum){
    if (value > maximum){ return maximum; }
    if (value < minimum){ return minimum; }
    return value;
}
double normalize_angle_degrees(double angle){
    while (angle > 180.0){ angle -= 360.0; }
    while (angle < -180.0){ angle += 360.0; }
    return angle;
}

// posicion leida
void get_position(double &x, double &y, double &theta){
    odometry_mutex.take();
    x = globalX;
    y = globalY;
    theta = globalTheta;
    odometry_mutex.give();
}

//reinicia posicion y rumbo
void reset_odometry(double x, double y, double theta_degrees){
    odometry_mutex.take();
    headingOffsetDeg = theta_degrees - (IMU_SIGN * imu.get_rotation());
    previousThetaDeg = theta_degrees;
    globalX = x;
    globalY = y;
    globalTheta = normalize_angle_degrees(theta_degrees);
    globalV = 0.0;
    globalOmega = 0.0;
    previousForwardRaw = ENCODER_FORWARD.get_position();
    previousSideRaw = ENCODER_SIDE.get_position();
    odometry_mutex.give();
}

void set_tank_speed(double left_speed, double right_speed){
    double largest_requested_speed = std::fmax(std::fabs(left_speed), std::fabs(right_speed));

    if (largest_requested_speed > MAX_MOTOR_RPM){
        double scale = MAX_MOTOR_RPM / largest_requested_speed;
        left_speed *= scale;
        right_speed *= scale;
    }
    left_speed = clamp_speed(left_speed, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
    right_speed = clamp_speed(right_speed, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
    leftMotors.move_velocity(left_speed);
    rightMotors.move_velocity(right_speed);
}

// avance positivo
void set_drive_rpm(double forward_rpm, double turn_rpm){
    set_tank_speed(DRIVE_POLARITY * (forward_rpm + turn_rpm), DRIVE_POLARITY * (forward_rpm - turn_rpm));
}

void stop_drive(){
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}

// tarea odometria
void tareaOdometria(void *param){
    (void)param;

    while (true){
        odometry_mutex.take();
        double thetaDeg = read_theta_deg();
        double forwardRaw = ENCODER_FORWARD.get_position();
        double sideRaw = ENCODER_SIDE.get_position();

        if (std::isfinite(thetaDeg) && forwardRaw != PROS_ERR && sideRaw != PROS_ERR){
            double dThetaDeg = thetaDeg - previousThetaDeg;
            double dThetaRad = dThetaDeg * M_PI / 180.0;
            // medicion
            double dForwardMeasured = FORWARD_SIGN * (forwardRaw - previousForwardRaw) * INCHES_PER_CENTIDEGREE;
            double dSideMeasured = SIDE_SIGN * (sideRaw - previousSideRaw) * INCHES_PER_CENTIDEGREE;
            double dForward = dForwardMeasured + (FORWARD_OFFSET_IN * dThetaRad);
            double dSide = dSideMeasured - (SIDE_OFFSET_IN * dThetaRad);
            double midRad = (previousThetaDeg + (0.5 * dThetaDeg)) * M_PI / 180.0;
            globalX += (dForward * std::sin(midRad)) + (dSide * std::cos(midRad));
            globalY += (dForward * std::cos(midRad)) - (dSide * std::sin(midRad));
            globalTheta = normalize_angle_degrees(thetaDeg);
            previousForwardRaw = forwardRaw;
            previousSideRaw = sideRaw;
            previousThetaDeg = thetaDeg;
        }

        odometry_mutex.give();
        pros::delay(10);
    }
}

// tarea de la pantalla
void tareaPantalla(void *param){
    (void)param;

    while (true){
        double x, y, theta;
        get_position(x, y, theta);
        pros::lcd::print(0, "X: %.2f in", x);
        pros::lcd::print(1, "Y: %.2f in", y);
        pros::lcd::print(2, "Grados: %.2f deg", theta);
        pros::delay(100);
    }
}