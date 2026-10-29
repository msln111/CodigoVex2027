#include "robot.h"

#include <cmath>
#include <cstdint>

// --------------------------------------------------
// DRIVE STRAIGHT: Use IMU heading to maintain line
// --------------------------------------------------

void driveStraightWithHeadingHold(double targetDistance) {
    constexpr double DISTANCE_TOLERANCE = 2.0;
    constexpr double MAX_DRIVE_SPEED = 300.0;
    constexpr double MIN_DRIVE_SPEED = 50.0;
    constexpr uint32_t TIMEOUT_MS = 10000;
    constexpr double DISTANCE_KP = 2.5;
    constexpr double HEADING_KP = 8.0;  // STRONG heading correction

    uint32_t start_time = pros::millis();

    double startX, startY, startTheta;
    get_position(startX, startY, startTheta);

    while (
        pros::millis() - start_time < TIMEOUT_MS
    ) {
        double currentX, currentY, currentTheta;
        get_position(currentX, currentY, currentTheta);

        // Calculate distance traveled
        double distanceTraveled = std::sqrt(
            (currentX - startX) * (currentX - startX) +
            (currentY - startY) * (currentY - startY)
        );

        double distance_error = 
            targetDistance - distanceTraveled;

        // Stop if we've reached target distance
        if (
            std::fabs(distance_error) <= 
            DISTANCE_TOLERANCE
        ) {
            break;
        }

        // Forward speed from distance error
        double forward_speed =
            DISTANCE_KP * distance_error;

        if (
            std::fabs(forward_speed) < MIN_DRIVE_SPEED &&
            forward_speed != 0.0
        ) {
            forward_speed =
                forward_speed >= 0.0
                    ? MIN_DRIVE_SPEED
                    : -MIN_DRIVE_SPEED;
        }

        forward_speed = clamp_speed(
            forward_speed,
            -MAX_DRIVE_SPEED,
            MAX_DRIVE_SPEED
        );

        // CRITICAL: Heading correction to keep robot straight
        // startTheta is the initial heading we want to maintain
        double heading_error =
            normalize_angle_degrees(
                startTheta - currentTheta
            );

        double turn_speed =
            HEADING_KP * heading_error;

        turn_speed = clamp_speed(
            turn_speed,
            -200.0,  // Limit max turn speed
            200.0
        );

        // Apply speeds
        double left_speed = forward_speed - turn_speed;
        double right_speed = forward_speed + turn_speed;

        set_tank_speed(left_speed, right_speed);

        pros::delay(20);
    }

    stop_drive();
}

// --------------------------------------------------
// Turn to a heading
// --------------------------------------------------

void girarAngulo(double objetivoTheta) {
    constexpr double ANGLE_TOLERANCE = 2.0;
    constexpr double MAX_TURN_SPEED = 250.0;
    constexpr double MIN_TURN_SPEED = 25.0;
    constexpr uint32_t TIMEOUT_MS = 4000;
    constexpr double TURN_KP = 3.0;

    uint32_t start_time = pros::millis();

    while (
        pros::millis() - start_time < TIMEOUT_MS
    ) {
        double current_x, current_y, current_theta;
        get_position(current_x, current_y, current_theta);

        double error =
            normalize_angle_degrees(
                objetivoTheta - current_theta
            );

        if (
            std::fabs(error) <= ANGLE_TOLERANCE
        ) {
            break;
        }

        double turn_speed = TURN_KP * error;

        turn_speed = clamp_speed(
            turn_speed,
            -MAX_TURN_SPEED,
            MAX_TURN_SPEED
        );

        if (
            std::fabs(turn_speed) < MIN_TURN_SPEED &&
            std::fabs(error) > 5.0
        ) {
            turn_speed =
                turn_speed >= 0.0
                    ? MIN_TURN_SPEED
                    : -MIN_TURN_SPEED;
        }

        set_tank_speed(-turn_speed, turn_speed);
        pros::delay(20);
    }

    stop_drive();
}

// --------------------------------------------------
// Autonomous routine
// --------------------------------------------------

void autonomous() {
    // Start at (0, 0) facing forward (90°)
    reset_odometry(0.0, 0.0, 90.0);
    pros::delay(200);

    // Drive straight 24 inches while maintaining heading
    driveStraightWithHeadingHold(24.0);

    pros::delay(500);
    stop_drive();
}