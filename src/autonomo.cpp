#include "robot.h"
#include <cmath>
#include <cstdint>

// Pose del robot dentro del modelo del MPC (pulgadas y radianes).
struct MpcPose { double x, y, th; };

// Comando de velocidad del MPC (avance en in/s y giro en rad/s).
struct MpcCmd { double v, w; };

// Normaliza un ángulo en radianes al rango de -pi a pi.
static double wrap_rad(double angle) {
    while (angle > M_PI) { angle -= 2.0 * M_PI; }
    while (angle < -M_PI) { angle += 2.0 * M_PI; }
    return angle;
}

// Modelo del robot: monociclo con retardo de primer orden y límite de aceleración.
static void step_model(MpcPose &pose, double &v, double &w, const MpcCmd &cmd) {
    const double lag = MPC_DT / (MPC_ACTUATOR_TAU + MPC_DT);
    v += clamp_speed(lag * (cmd.v - v), -MPC_A_MAX * MPC_DT, MPC_A_MAX * MPC_DT);
    w += clamp_speed(lag * (cmd.w - w), -MPC_ALPHA_MAX * MPC_DT, MPC_ALPHA_MAX * MPC_DT);
    const double mid_theta = pose.th + (0.5 * w * MPC_DT);
    pose.x += v * std::cos(mid_theta) * MPC_DT;
    pose.y += v * std::sin(mid_theta) * MPC_DT;
    pose.th += w * MPC_DT;
}

// Costo de una pose: distancia a la meta y, cerca de ella, error de rumbo.
static double stage_cost(const MpcPose &pose, const MpcPose &goal) {
    double distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
    double heading_error = wrap_rad(goal.th - pose.th);
    double blend = clamp_speed(1.0 - (distance / MPC_HEADING_BLEND_DIST), 0.0, 1.0);
    return (MPC_Q_POS * distance * distance) + (MPC_Q_HEADING * blend * heading_error * heading_error);
}

// Costo de mantener un comando constante durante todo el horizonte.
static double rollout_cost(MpcPose pose, double v, double w, const MpcCmd &cmd, const MpcPose &goal) {
    double cost = 0.0;
    double last_stage = 0.0;

    for (int step = 0; step < MPC_HORIZON; ++step) {
        step_model(pose, v, w, cmd);
        last_stage = stage_cost(pose, goal);
        cost += last_stage;
    }

    cost += MPC_QF * last_stage;
    cost += MPC_HORIZON * ((MPC_R_V * cmd.v * cmd.v) + (MPC_R_W * cmd.w * cmd.w));
    return cost;
}

// Búsqueda en malla de comandos (más densa cerca de cero); siempre incluye el comando (0, 0).
static MpcCmd solve_mpc(const MpcPose &pose, double v, double w, const MpcPose &goal) {
    // MODIFICABLE: tamaño de la malla de búsqueda (debe ser impar; más grande es más preciso pero más lento).
    constexpr int GRID_SIZE = 13;
    MpcCmd best = {0.0, 0.0};
    double best_cost = 1e18;

    for (int i = 0; i < GRID_SIZE; ++i) {
        double sv = (2.0 * i / (GRID_SIZE - 1)) - 1.0;
        double candidate_v = clamp_speed(sv * std::fabs(sv) * MPC_V_MAX, MPC_V_MIN, MPC_V_MAX);

        for (int j = 0; j < GRID_SIZE; ++j) {
            double sw = (2.0 * j / (GRID_SIZE - 1)) - 1.0;
            double candidate_w = sw * std::fabs(sw) * MPC_W_MAX;

            // Ninguna rueda puede superar la velocidad máxima.
            if (std::fabs(candidate_v) + (std::fabs(candidate_w) * TRACK_WIDTH * 0.5) > MPC_V_MAX) { continue; }

            MpcCmd candidate = {candidate_v, candidate_w};
            double cost = rollout_cost(pose, v, w, candidate, goal);
            if (cost < best_cost) { best_cost = cost; best = candidate; }
        }
    }

    return best;
}

// MODIFICABLE: tolerancias (in y rad), velocidades para dar la meta por alcanzada, tiempo límite y umbral de cancelación con el joystick.
void moverAPunto(double objetivoX, double objetivoY, double objetivoTheta) {
    constexpr double POS_TOLERANCE = 1.5;
    constexpr double ANGLE_TOLERANCE = 2.0 * M_PI / 180.0;
    constexpr double SETTLED_SPEED = 3.0;
    constexpr double SETTLED_TURN = 0.3;
    constexpr uint32_t TIMEOUT_MS = 6000;
    constexpr int JOYSTICK_ABORT = 60;
    // Periodo del ciclo de control en segundos (debe coincidir con el pros::delay).
    constexpr double CTRL_DT = 0.02;

    const MpcPose goal = {objetivoX, objetivoY, objetivoTheta * M_PI / 180.0};
    MpcCmd applied = {0.0, 0.0};
    uint32_t start_time = pros::millis();

    while (pros::millis() - start_time < TIMEOUT_MS) {
        // Mover el joystick izquierdo cancela el movimiento (en competencia el control no envía nada).
        if (std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > JOYSTICK_ABORT) { break; }

        RobotState state;
        get_state(state);
        MpcPose pose = {state.x, state.y, state.thetaRad};
        double distance_error = std::hypot(goal.x - pose.x, goal.y - pose.y);
        double heading_error = wrap_rad(goal.th - pose.th);
        bool in_position = distance_error <= POS_TOLERANCE && std::fabs(heading_error) <= ANGLE_TOLERANCE;
        bool settled = std::fabs(state.v) < SETTLED_SPEED && std::fabs(state.omega) < SETTLED_TURN;

        if (in_position && settled) { break; }

        MpcCmd best = solve_mpc(pose, state.v, state.omega, goal);
        // Solo se aplica el primer paso, con límite de aceleración (horizonte deslizante).
        applied.v += clamp_speed(best.v - applied.v, -MPC_A_MAX * CTRL_DT, MPC_A_MAX * CTRL_DT);
        applied.w += clamp_speed(best.w - applied.w, -MPC_ALPHA_MAX * CTRL_DT, MPC_ALPHA_MAX * CTRL_DT);
        set_drive_velocity(applied.v, applied.w);
        pros::delay(20);
    }

    stop_drive();
}

// Gira en sitio hasta el rumbo indicado (grados antihorarios): ir a la posición actual con otro rumbo.
void girarAngulo(double objetivoTheta) {
    double x, y, theta;
    get_position(x, y, theta);
    moverAPunto(x, y, objetivoTheta);
}

// Avanza la distancia indicada (pulgadas) manteniendo el rumbo actual.
void driveStraightWithHeadingHold(double targetDistance) {
    double x, y, theta;
    get_position(x, y, theta);
    double theta_radians = theta * M_PI / 180.0;
    moverAPunto(x + (targetDistance * std::cos(theta_radians)), y + (targetDistance * std::sin(theta_radians)), theta);
}

// MODIFICABLE: cambia las llamadas y los valores aquí para definir la rutina autónoma.
void autonomous() {
    reset_odometry(0.0, 0.0, 0.0);
    pros::delay(200);
    moverAPunto(0.0, 24.0, 0.0);
    pros::delay(500);
    stop_drive();
}