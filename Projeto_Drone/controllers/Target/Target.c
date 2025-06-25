#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <javino.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <termios.h>
#include <math.h>

#define TIME_STEP 32
#define SERIAL_PORT "/dev/ttyExogenous0"

typedef enum {
    STATE_LANDED,
    STATE_TAKING_OFF,
    STATE_HOVERING,
    STATE_MOVING_FORWARD,
    STATE_MOVING_TO_TARGET,
    STATE_LANDING
} DroneState;

void normalize_string(char *str) {
    int len = strlen(str);
    while (len > 0 && isspace((unsigned char)str[len - 1])) {
        str[len - 1] = '\0';
        len--;
    }
    for (char *p = str; *p; p++) *p = tolower(*p);
}

double calculate_rate(double current, double previous, int timestep_ms) {
    return (current - previous) / (timestep_ms / 1000.0);
}

double limit(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void apply_motor_speeds(WbDeviceTag *motors, double *speeds) {
    wb_motor_set_velocity(motors[0], speeds[0]);
    wb_motor_set_velocity(motors[1], -speeds[1]);
    wb_motor_set_velocity(motors[2], -speeds[2]);
    wb_motor_set_velocity(motors[3], speeds[3]);
}

int main() {

    // ------------------ Motores ------------------

    wb_robot_init();
    int basic_timestep = wb_robot_get_basic_time_step();

    WbDeviceTag propellers[4] = {
        wb_robot_get_device("front left propeller"),
        wb_robot_get_device("front right propeller"),
        wb_robot_get_device("rear left propeller"),
        wb_robot_get_device("rear right propeller"),
    };

    for (int i = 0; i < 4; i++) {
        if (propellers[i] == 0)
            printf("Motor %d não encontrado!\n", i);
        wb_motor_set_position(propellers[i], INFINITY);
        wb_motor_set_velocity(propellers[i], 0.0);
    }

    // ------------------ Sensores ------------------

    WbDeviceTag imu_sensor = wb_robot_get_device("inertial unit");
    WbDeviceTag gps_sensor = wb_robot_get_device("gps");

    if (imu_sensor == 0) printf("Sensor inercial não encontrado!\n");
    else wb_inertial_unit_enable(imu_sensor, basic_timestep);

    if (gps_sensor == 0) printf("GPS não encontrado!\n");
    else wb_gps_enable(gps_sensor, basic_timestep);

    // ------------------ Comunicação Serial ------------------

    int serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        perror("Erro ao abrir porta serial");
        return 1;
    }

    struct termios tty;

    tcgetattr(serial_fd, &tty);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;
    tcsetattr(serial_fd, TCSANOW, &tty);

    javino_init(serial_fd);
    printf("Porta serial configurada\n");

    // ------------------ Variáveis de Estado ------------------

    DroneState state = STATE_LANDED;

    double state_start_time = 0;
    double motor_speeds[4] = {0};

    const double TARGET_ALTITUDE = 0;
    const double BASE_THRUST = 80.0;
    const double KP_STABILIZE = 15.0;
    const double KD_STABILIZE = 3.0;
    const double KP_ALTITUDE = 8.0;
    const double NAVIGATION_SPEED = 0.3;

    double previous_roll = 0, previous_pitch = 0;
    double target_x = 0, target_y = 0;
    double initial_x = 0, initial_y = 0;

    printf("Calibrando sensores...\n");

    for (int i = 0; i < 100; i++) {
        wb_robot_step(basic_timestep);
        for (int j = 0; j < 4; j++) {
            wb_motor_set_velocity(propellers[j], 0.0);
        }
    }

    printf("Calibração completa\n");


    
    
    // ------------------ Iteração do Drone ------------------

    while (wb_robot_step(basic_timestep) != -1) {
        double current_time = wb_robot_get_time();
        double time_in_state = current_time - state_start_time;

        // ------------------ Processamento de Mensagens ------------------

        if (javino_avaliable_msg()) {
            char *msg = javino_get_msg();

            if (msg) {
                printf("Mensagem recebida: %s\n", msg);
                normalize_string(msg);

                if (strcmp(msg, "takeoff") == 0 && state == STATE_LANDED) {
                    state = STATE_TAKING_OFF;
                    state_start_time = current_time;
                    printf("Iniciando decolagem\n");

                    if (gps_sensor) {
                        const double *pos = wb_gps_get_values(gps_sensor);
                        initial_x = pos[0];
                        initial_y = pos[1];
                    }
                } else if (strncmp(msg, "goto(", 5) == 0) {
                    sscanf(msg, "goto(%lf,%lf)", &target_x, &target_y);
                    printf("Novo destino: X=%.2f, Y=%.2f\n", target_x, target_y);
                    state = STATE_MOVING_TO_TARGET;
                    state_start_time = current_time;
                } else if (strcmp(msg, "land") == 0 && state != STATE_LANDED && state != STATE_LANDING) {
                    state = STATE_LANDING;
                    state_start_time = current_time;
                    printf("Iniciando pouso\n");
                } else if (strcmp(msg, "getpos") == 0) {
                    if (gps_sensor) {
                        const double *pos = wb_gps_get_values(gps_sensor);
                        char response[100];
                        snprintf(response, sizeof(response), "pos(%.2f,%.2f,%.2f)", pos[0], pos[1], pos[2]);
                        javino_send_msg(response);
                    }
                }

                free(msg);
            }
        }

        // ------------------ Leitura dos Sensores ------------------

        double roll = 0, pitch = 0;
        double pos_x = 0, pos_y = 0, altitude = 0;

        if (imu_sensor) {
            const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_sensor);
            roll = rpy[0];
            pitch = rpy[1];
        }

        if (gps_sensor) {
            const double *pos = wb_gps_get_values(gps_sensor);
            pos_x = pos[0];
            pos_y = pos[1];
            altitude = pos[2];

            static int step_counter = 0;
            step_counter++;
            if (step_counter >= 5) {
                char percept[128];
                snprintf(percept, sizeof(percept), "gps(%.2f,%.2f,%.2f)", pos_x, pos_y, altitude);
                javino_send_msg(percept);
                step_counter = 0;
            }
        }

        // ------------------ Controle do Estado do Drone ------------------

        if (state != STATE_LANDED) {

            // Atualiza velocidades angulares
            double roll_rate  = calculate_rate(roll, previous_roll, basic_timestep);
            double pitch_rate = calculate_rate(pitch, previous_pitch, basic_timestep);
            previous_roll = roll;
            previous_pitch = pitch;

            // Controle de altitude
            double thrust_correction = 0.0;
            double altitude_error = 0.0;

            if (state == STATE_LANDING) {

                double descent_altitude = TARGET_ALTITUDE * (1.0 - fmin(1.0, time_in_state / 4.0));
                altitude_error = descent_altitude - altitude;
                
                if (altitude < 0.1) {
                    state = STATE_LANDED;
                    printf("Pouso completo\n");
                }

            } else {
                altitude_error = TARGET_ALTITUDE - altitude;
                if (state == STATE_TAKING_OFF && altitude >= TARGET_ALTITUDE * 0.9) {
                    state = STATE_HOVERING;
                    printf("Drone estabilizado em voo\n");
                }
            }

            thrust_correction = KP_ALTITUDE * altitude_error;

            // --- Correções de estabilização
            double roll_correction  = KP_STABILIZE * (-roll)  + KD_STABILIZE * (-roll_rate);
            double pitch_correction = KP_STABILIZE * (-pitch) + KD_STABILIZE * (-pitch_rate);

            // --- Correções de navegação
            if (state == STATE_MOVING_TO_TARGET) {

                double dx = target_x - pos_x;
                double dy = target_y - pos_y;
                double distance = sqrt(dx * dx + dy * dy);

                if (distance < 0.2) {
                    
                    state = STATE_HOVERING;
                    printf("Alvo alcançado!\n");

                } else {
                    // Ainda voando até target
                    double angle = atan2(dy, dx);
                    double target_pitch = -NAVIGATION_SPEED * cos(angle);
                    double target_roll  = -NAVIGATION_SPEED * sin(angle);

                    pitch_correction += KP_STABILIZE * (target_pitch - pitch);
                    roll_correction  += KP_STABILIZE * (target_roll - roll);
                }

            } else if (state == STATE_HOVERING) {
                double error_x = initial_x - pos_x;
                double error_y = initial_y - pos_y;
                pitch_correction += error_x * 0.1;
                roll_correction  += error_y * 0.1;
            }



            // --- Atualiza velocidades dos motores
            double motor_speeds[4];
            motor_speeds[0] = BASE_THRUST + thrust_correction - pitch_correction + roll_correction;
            motor_speeds[1] = BASE_THRUST + thrust_correction - pitch_correction - roll_correction;
            motor_speeds[2] = BASE_THRUST + thrust_correction + pitch_correction + roll_correction;
            motor_speeds[3] = BASE_THRUST + thrust_correction + pitch_correction - roll_correction;

            for (int i = 0; i < 4; i++) {
                motor_speeds[i] = limit(motor_speeds[i], 5.0, 100.0);
            }

            apply_motor_speeds(propellers, motor_speeds);

        }
        else {
            printf("Desligando motores do drone!");

            for (int i = 0; i < 4; i++) {
                wb_motor_set_velocity(propellers[i], 0.0);
            }
        }

        // ------------------ Debug ------------------

        static double last_log_time = 0;

        if (current_time - last_log_time > 0) {
            printf("Estado: %d | Altura: %.2fm | X: %.2f | Y: %.2f\n",
                   state, altitude, pos_x, pos_y);
            last_log_time = current_time;
        }
    }

    // ------------------ Finalização ------------------

    close(serial_fd);
    wb_robot_cleanup();
    return 0;
}