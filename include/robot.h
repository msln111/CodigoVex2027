#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"
#include <cmath>
#include <cstdint>

// MODIFICABLE: límites máximos de velocidad del motor y de giro, en RPM.
constexpr double MAX_MOTOR_RPM = 600.0;
constexpr double MAX_TURN_RPM = 300.0;

// MODIFICABLE: ajustes de corrección para mantener el rumbo.
constexpr double HEADING_HOLD_KP = 5.0;
constexpr double HEADING_CORRECTION_SIGN = 1.0;

// Posición global del robot.
extern double globalX;
extern double globalY;
extern double globalTheta;
extern pros::Mutex odometry_mutex;

// Motores de tracción.
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// Controlador y sensores.
extern pros::Controller master;
extern pros::IMU imu;
extern pros::Rotation encoderhorizontal;
extern pros::Rotation encodervertical;

// Funciones auxiliares.
double clamp_speed(double value, double minimum, double maximum);
double normalize_angle_degrees(double angle);
double apply_joystick_curve(double stick_value, double sensitivity_power);
void get_position(double &x, double &y, double &theta);
void reset_odometry(double x = 0.0, double y = 0.0, double theta_degrees = 0.0);
void set_tank_speed(double left_speed, double right_speed);
void stop_drive();

// Tareas de fondo.
void tareaOdometria(void *param);
void tareaPantalla(void *param);

// Funciones autónomas.
void girarAngulo(double objetivoTheta);
void moverAPunto(double objetivoX, double objetivoY, double objetivoTheta);

#endif