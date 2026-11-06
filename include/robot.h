#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"
#include <cmath>
#include <cstdint>

// valor variable: limites maximos de velocidad del motor y de giro, en RPM
constexpr double MAX_MOTOR_RPM = 600.0;
constexpr double MAX_TURN_RPM = 300.0;

// valor variable: geometria del tren de traccion, medir en el robot (diametro y distancia entre ruedas en pulgadas, relacion de engranes)
constexpr double DRIVE_WHEEL_DIAMETER = 3.25;
constexpr double TRACK_WIDTH = 12.0;
constexpr double DRIVE_GEAR_RATIO = 1.0;

// conversion de RPM del motor a pulgadas por segundo en la rueda (no modificar)
constexpr double IN_PER_S_PER_RPM = (M_PI * DRIVE_WHEEL_DIAMETER) / (60.0 * DRIVE_GEAR_RATIO);

// valor variable: horizonte de prediccion del MPC (cantidad de pasos y segundos por paso)
constexpr int MPC_HORIZON = 12;
constexpr double MPC_DT = 0.05;

// valor variable: retardo del motor en segundos (sale de la prueba de escalon)
constexpr double MPC_ACTUATOR_TAU = 0.10;

// valor variable: velocidad maxima de cada rueda en RPM (empezar con 150 en las primeras pruebas)
constexpr double MPC_MAX_WHEEL_RPM = 300.0;

// velocidad de avance maxima en in/s, se calcula sola (no modificar)
constexpr double MPC_V_MAX = MPC_MAX_WHEEL_RPM * IN_PER_S_PER_RPM;

// valor variable: avance minimo en in/s (0.0 prohibe la reversa), giro maximo en rad/s y aceleraciones maximas (in/s^2 y rad/s^2)
constexpr double MPC_V_MIN = -MPC_V_MAX;
constexpr double MPC_W_MAX = 6.0;
constexpr double MPC_A_MAX = 120.0;
constexpr double MPC_ALPHA_MAX = 25.0;

// valor variable: pesos del costo (posicion, rumbo, costo terminal, penalizacion de avance y de giro)
constexpr double MPC_Q_POS = 1.0;
constexpr double MPC_Q_HEADING = 40.0;
constexpr double MPC_QF = 3.0;
constexpr double MPC_R_V = 0.002;
constexpr double MPC_R_W = 0.02;

// valor variable: distancia en pulgadas a la que el rumbo final empieza a pesar en el costo
constexpr double MPC_HEADING_BLEND_DIST = 8.0;

// posicion global del robot
extern double globalX;
extern double globalY;
extern double globalTheta;
extern pros::Mutex odometry_mutex;

// velocidades filtradas del robot (avance en in/s y giro en rad/s)
extern double globalV;
extern double globalOmega;

// estado del robot para el MPC: posicion, rumbo antihorario y velocidades
struct RobotState { double x, y, thetaRad, v, omega; };

// motores de traccion
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup cadenaMotors;
extern pros::MotorGroup upMotors;
extern pros::Motor garraMotor;
extern pros::adi::DigitalOut pistonNeumatico;

// controlador y sensores
extern pros::Controller master;
extern pros::IMU imu;
extern pros::Rotation encoderhorizontal;
extern pros::Rotation encodervertical;

// funciones auxiliares
double clamp_speed(double value, double minimum, double maximum);
double normalize_angle_degrees(double angle);
double apply_joystick_curve(double stick_value, double sensitivity_power);
void get_position(double &x, double &y, double &theta);
void get_state(RobotState &state);
void reset_odometry(double x = 0.0, double y = 0.0, double theta_degrees = 0.0);
void set_tank_speed(double left_speed, double right_speed);
void set_up_speed(double up_speed);
void set_drive_velocity(double v_in_s, double omega_rad_s);
void stop_drive();

// tareas de fondo
void tareaOdometria(void *param);
void tareaPantalla(void *param);

// funciones autonomas
void girarAngulo(double objetivoTheta);
void moverAPunto(double objetivoX, double objetivoY, double objetivoTheta);

#endif