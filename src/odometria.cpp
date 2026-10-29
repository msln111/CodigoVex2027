#include "robot.h"

#include <cmath>
#include <string>

// --------------------------------------------------
// Global position
// --------------------------------------------------

double globalX = 0.0;
double globalY = 0.0;
double globalTheta = 0.0;

pros::Mutex odometry_mutex;

// --------------------------------------------------
// Sensors and drivetrain
// --------------------------------------------------

pros::Rotation encoderhorizontal(11);
pros::Rotation encodervertical(-12);
pros::IMU imu(10);

// -----------------------------
// Motors and controller// -----------------------------

pros::MotorGroup leftMotors({-3, 4, -16, 8});
pros::MotorGroup rightMotors({1, 6, -2, -5});

pros::Controller master(
    pros::E_CONTROLLER_MASTER
);

// --------------------------------------------------
// Tracking-wheel calibration
// --------------------------------------------------

constexpr double VERTICAL_WHEEL_DIAMETER =
    2.0;

constexpr double HORIZONTAL_WHEEL_DIAMETER =
    2.0;

constexpr double CENTIDEGREES_PER_REVOLUTION = 36000.0;

constexpr double VERTICAL_WHEEL_CIRCUMFERENCE =
    M_PI * VERTICAL_WHEEL_DIAMETER;

constexpr double HORIZONTAL_WHEEL_CIRCUMFERENCE =
    M_PI * HORIZONTAL_WHEEL_DIAMETER;

constexpr double VERTICAL_INCHES_PER_CENTIDEGREE =
    VERTICAL_WHEEL_CIRCUMFERENCE /
    CENTIDEGREES_PER_REVOLUTION;

constexpr double HORIZONTAL_INCHES_PER_CENTIDEGREE =
    HORIZONTAL_WHEEL_CIRCUMFERENCE /
    CENTIDEGREES_PER_REVOLUTION;

// --------------------------------------------------
// Tracking-wheel placement (gravity centers)
// --------------------------------------------------

constexpr double WHEEL_FORWARD_OFFSET =
    0.875;

constexpr double WHEEL_SIDE_OFFSET =
    1.875;

constexpr pros::Rotation &ENCODER_FOR_X = encoderhorizontal;
constexpr pros::Rotation &ENCODER_FOR_Y = encodervertical;

// --------------------------------------------------
// Previous sensor readings
// --------------------------------------------------

static double previousEncoderX = 0.0;
static double previousEncoderY = 0.0;
static double previousHeadingDegrees = 0.0;

// --------------------------------------------------
// Utility functions
// --------------------------------------------------

double clamp_speed(
    double value,
    double minimum,
    double maximum
) {
    if (value > maximum) {
        return maximum;
    }

    if (value < minimum) {
        return minimum;
    }

    return value;
}

double normalize_angle_degrees(double angle) {
    while (angle > 180.0) {
        angle -= 360.0;
    }

    while (angle < -180.0) {
        angle += 360.0;
    }

    return angle;
}

void get_position(
    double &x,
    double &y,
    double &theta
) {
    odometry_mutex.take();

    x = globalX;
    y = globalY;
    theta = globalTheta;

    odometry_mutex.give();
}

void reset_odometry(
    double x,
    double y,
    double theta_degrees
) {
    odometry_mutex.take();

    globalX = x;
    globalY = y;
    globalTheta = normalize_angle_degrees(theta_degrees);

    odometry_mutex.give();

    previousEncoderX =
        ENCODER_FOR_X.get_position();

    previousEncoderY =
        ENCODER_FOR_Y.get_position();

    previousHeadingDegrees =
        imu.get_heading();
}

void set_tank_speed(
    double left_speed,
    double right_speed
) {
    double largest_requested_speed =
        std::fmax(
            std::fabs(left_speed),
            std::fabs(right_speed)
        );

    if (largest_requested_speed > MAX_MOTOR_RPM) {
        double scale =
            MAX_MOTOR_RPM /
            largest_requested_speed;

        left_speed *= scale;
        right_speed *= scale;
    }

    left_speed = clamp_speed(
        left_speed,
        -MAX_MOTOR_RPM,
        MAX_MOTOR_RPM
    );

    right_speed = clamp_speed(
        right_speed,
        -MAX_MOTOR_RPM,
        MAX_MOTOR_RPM
    );

    leftMotors.move_velocity(left_speed);
    rightMotors.move_velocity(right_speed);
}

void stop_drive() {
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
}

// --------------------------------------------------
// Odometry task
// --------------------------------------------------

void tareaOdometria(void *param) {
    (void)param;

    previousEncoderX =
        ENCODER_FOR_X.get_position();

    previousEncoderY =
        ENCODER_FOR_Y.get_position();

    previousHeadingDegrees =
        imu.get_heading();

    while (true) {
        double currentEncoderX =
            ENCODER_FOR_X.get_position();

        double currentEncoderY =
            ENCODER_FOR_Y.get_position();

        double currentHeadingDegrees =
            imu.get_heading();

        double deltaEncoderX =
            currentEncoderX -
            previousEncoderX;

        double deltaEncoderY =
            currentEncoderY -
            previousEncoderY;

        double rawDeltaX =
            deltaEncoderX *
            VERTICAL_INCHES_PER_CENTIDEGREE;

        double rawDeltaY =
            deltaEncoderY *
            HORIZONTAL_INCHES_PER_CENTIDEGREE;

        double deltaHeadingDegrees =
            normalize_angle_degrees(
                currentHeadingDegrees -
                previousHeadingDegrees
            );

        double deltaHeadingRadians =
            deltaHeadingDegrees *
            M_PI /
            180.0;

        double correctedDeltaX =
            rawDeltaX -
            (WHEEL_SIDE_OFFSET * deltaHeadingRadians);

        double correctedDeltaY =
            rawDeltaY -
            (WHEEL_FORWARD_OFFSET * deltaHeadingRadians);

        double headingRadians =
            currentHeadingDegrees *
            M_PI /
            180.0;

        double cos_heading =
            std::cos(headingRadians);

        double sin_heading =
            std::sin(headingRadians);

        double fieldDeltaX =
            (correctedDeltaX * cos_heading) -
            (correctedDeltaY * sin_heading);

        double fieldDeltaY =
            (correctedDeltaX * sin_heading) +
            (correctedDeltaY * cos_heading);

        odometry_mutex.take();

        globalX += fieldDeltaX;
        globalY += fieldDeltaY;
        globalTheta += deltaHeadingDegrees;

        globalTheta =
            normalize_angle_degrees(globalTheta);

        odometry_mutex.give();

        previousEncoderX =
            currentEncoderX;

        previousEncoderY =
            currentEncoderY;

        previousHeadingDegrees =
            currentHeadingDegrees;

        pros::delay(10);
    }
}

// --------------------------------------------------
// LCD display task
// --------------------------------------------------

void tareaPantalla(void *param) {
    (void)param;

    while (true) {
        double x;
        double y;
        double theta;

        get_position(x, y, theta);

        pros::lcd::set_text(
            0,
            "X: " + std::to_string(x) + " in"
        );

        pros::lcd::set_text(
            1,
            "Y: " + std::to_string(y) + " in"
        );

        pros::lcd::set_text(
            2,
            "Heading: " + std::to_string(theta) + " deg"
        );

        pros::delay(100);
    }
}