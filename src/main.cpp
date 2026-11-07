#include "robot.h"
#include <algorithm>
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
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    garraMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    garraMotor.tare_position();
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
    // valor variable: DEADBAND ignora movimientos pequeños de los joysticks
    constexpr int DEADBAND = 10;
    // valor variable: VELOCIDAD de la garra
    constexpr double GarraRPM = 100.0;
    // valor variable: GRADOS que gira el motor.   
    constexpr double RelacionGarra = 3.0;
    // valor variable: La direccion del giro
    constexpr double SentidoGarra = -1.0;
    // valor variable: Recorrido de la garra
    constexpr double GarraGradosArriba = 90.0;
    constexpr double GarraGradosAbajo = 350.0;
    // valor variable: VELOCIDAD de la subida
    constexpr double SubidaRPM = 100.0;
    // valor variable: VELOCIDAD del intake
    constexpr double IntakeRPM = 100.0;
    // valor variable: estas sensibilidades cambian la respuesta de los joysticks
    constexpr double FORWARD_SENSITIVITY = 2.0;
    constexpr double TURN_SENSITIVITY = 1.0;

    bool previous_A = false;
    bool previous_B = false;
    bool previous_DOWN = false;
    int PosicionGarra = 1;
    bool anteriorDerecha = false;
    bool anteriorIzquierda = false;
    bool subidaEncendida = false;
    bool intakeEncendido = false;
    bool previous_UP = false;
    bool previous_X = false;

    while (true) {
        // pulsa B + ABAJO a la vez para activar la rutina autonoma
        bool current_B = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool current_DOWN = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool both_pressed = current_B && current_DOWN;
        bool both_just_pressed = both_pressed && (!previous_B || !previous_DOWN);

        if (both_just_pressed) {
            pros::lcd::clear_line(7);
            pros::lcd::set_text(7, "Ejecutando autonomo");
            autonomous();
            stop_drive();
            pros::lcd::clear_line(7);
            pros::lcd::set_text(7, "Autonomo terminado");
        }

        previous_B = current_B;
        previous_DOWN = current_DOWN;

        // controlador de la garra
        bool GarraMas = master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        bool GarraMenos = master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
        if (GarraMas && !anteriorDerecha && !GarraMenos && PosicionGarra > 0){
            PosicionGarra--;
        }
        if (GarraMenos && !anteriorIzquierda && !GarraMas && PosicionGarra < 2){
            PosicionGarra++;
        }
        double garra_objetivo = PosicionGarra == 0 ? GarraGradosArriba * RelacionGarra * SentidoGarra : (PosicionGarra == 2 ? -GarraGradosAbajo * RelacionGarra * SentidoGarra : 0.0);
        if ((GarraMas && !anteriorDerecha && !GarraMenos && PosicionGarra < 2) || (GarraMenos && !anteriorIzquierda && !GarraMas && PosicionGarra > 0)){
            garraMotor.move_absolute(garra_objetivo, GarraRPM);
        }
        anteriorDerecha = GarraMas;
        anteriorIzquierda = GarraMenos;

        // subida
        bool current_UP = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        if (current_UP && !previous_UP){
            subidaEncendida = !subidaEncendida;
                upMotors.move_velocity(subidaEncendida ? SubidaRPM : 0);
            }
            previous_UP = current_UP;
    
        // intake
        bool current_X = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
         if (current_X && !previous_X){
            intakeEncendido = !intakeEncendido;
            cadenaMotors.move_velocity(intakeEncendido ? IntakeRPM : 0);
            }
        previous_X = current_X;

        // estados del piston
        bool current_A = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        static bool pistonAbierto = false;
        if (current_A && !previous_A){
            pistonAbierto = !pistonAbierto;
            pistonNeumatico.set_value(pistonAbierto);
        }
        previous_A = current_A;

        int forward_raw = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn_raw = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (std::abs(forward_raw) < DEADBAND) { 
            forward_raw = 0; 
        }
        if (std::abs(turn_raw) < DEADBAND) { 
            turn_raw = 0; 
        }

        // si ambos joysticks estan en reposo, frena para que no avance por inercia.
        if (forward_raw == 0 && turn_raw == 0) {
            stop_drive();
            pros::delay(20);
            continue;
        }

        double forward_curved = apply_joystick_curve(forward_raw, FORWARD_SENSITIVITY);
        double turn_curved = apply_joystick_curve(turn_raw, TURN_SENSITIVITY);
        double forward_rpm = forward_curved * (MAX_MOTOR_RPM / 127.0);
        double turn_rpm = clamp_speed(turn_curved * (MAX_TURN_RPM / 127.0), -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
        double left_speed = forward_rpm - turn_rpm;
        double right_speed = forward_rpm + turn_rpm;

        // si la suma de avance + giro pasa del maximo, reduce ambos lados en proporcion 
        // para conservar el giro mientras avanza (en vez de recortar un solo lado)
        double max_speed = std::max(std::abs(left_speed), std::abs(right_speed));
        if (max_speed > MAX_MOTOR_RPM && forward_rpm != 0.0) {
            double exceso = (max_speed - MAX_MOTOR_RPM) * (forward_rpm > 0.0 ? 1.0 : -1.0);
            left_speed -= exceso;
            right_speed -= exceso;
        }
        
        set_tank_speed(left_speed, right_speed);
        pros::delay(20);
    }
}