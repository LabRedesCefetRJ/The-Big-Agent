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
#define TTY_EXOGENOUS_PORT "/dev/ttyExogenous0"

typedef enum {
    LANDED,
    TAKING_OFF,
    HOVERING,
    MOVING_FORWARD,
    MOVING_TO_TARGET,
    LANDING
} DroneState;

void clean_string(char *str) {
    int len = strlen(str);
    while (len > 0 && isspace((unsigned char)str[len - 1])) {
        str[len - 1] = '\0';
        len--;
    }
    for (char *p = str; *p; p++) *p = tolower(*p);
}

int main() {
    wb_robot_init();
    int timestep = wb_robot_get_basic_time_step();

    WbDeviceTag motors[4] = {
        wb_robot_get_device("front left propeller"),
        wb_robot_get_device("front right propeller"),
        wb_robot_get_device("rear left propeller"),
        wb_robot_get_device("rear right propeller"),
    };

    for (int i = 0; i < 4; i++) {
        if (motors[i] == 0) {
            printf("Motor %d não encontrado!\n", i);
        }
        wb_motor_set_position(motors[i], INFINITY);
        wb_motor_set_velocity(motors[i], 0.0);
    }

    // Sensores
    WbDeviceTag imu = wb_robot_get_device("inertial unit");
    WbDeviceTag gps = wb_robot_get_device("gps");

    if (imu == 0) printf("Sensor inercial não encontrado!\n");
    else wb_inertial_unit_enable(imu, timestep);

    if (gps == 0) printf("GPS não encontrado!\n");
    else wb_gps_enable(gps, timestep);

    // Configuração serial
    int serial = open(TTY_EXOGENOUS_PORT, O_RDWR | O_NOCTTY);
    if (serial < 0) {
        perror("Erro ao abrir porta serial");
        return 1;
    }

    struct termios tty;
    tcgetattr(serial, &tty);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;
    tcsetattr(serial, TCSANOW, &tty);

    javino_init(serial);
    printf("Porta serial configurada\n");

    DroneState state = LANDED;
    double state_start_time = 0;
    double motor_speeds[4] = {0};

    const double TARGET_ALTITUDE = 0;
    const double BASE_THRUST = 80.0;
    const double KP_STABILIZE = 15.0;
    const double KD_STABILIZE = 3.0;
    const double KP_ALTITUDE = 8.0;
    const double NAV_SPEED = 0.3;

    double last_roll = 0, last_pitch = 0;
    double target_x = 0, target_y = 0;
    double initial_x = 0, initial_y = 0;

    printf("Calibrando sensores...\n");
    for (int i = 0; i < 100; i++) {
        wb_robot_step(timestep);
        for (int j = 0; j < 4; j++) {
            wb_motor_set_velocity(motors[j], 0.0);
        }
    }
    printf("Calibração completa\n");

    while (wb_robot_step(timestep) != -1) {
        double current_time = wb_robot_get_time();
        double elapsed_time = current_time - state_start_time;

        // Processamento de mensagens
        if (javino_avaliable_msg()) {
            char *msg = javino_get_msg();
            if (msg) {
                printf("Mensagem recebida: %s\n", msg);
                clean_string(msg);

                if (strcmp(msg, "takeoff") == 0 && state == LANDED) {
                    state = TAKING_OFF;
                    state_start_time = current_time;
                    printf("Iniciando decolagem\n");

                    if (gps) {
                        const double *pos = wb_gps_get_values(gps);
                        initial_x = pos[0];
                        initial_y = pos[1];
                    }
                } else if (strncmp(msg, "goto(", 5) == 0) {
                    sscanf(msg, "goto(%lf,%lf)", &target_x, &target_y);
                    printf("Novo destino: X=%.2f, Y=%.2f\n", target_x, target_y);
                    state = MOVING_TO_TARGET;
                    state_start_time = current_time;
                } else if (strcmp(msg, "land") == 0 && state != LANDED && state != LANDING) {
                    state = LANDING;
                    state_start_time = current_time;
                    printf("Iniciando pouso\n");
                } else if (strcmp(msg, "getpos") == 0) {
                    if (gps) {
                        const double *pos = wb_gps_get_values(gps);
                        char response[100];
                        snprintf(response, sizeof(response), "pos(%.2f,%.2f,%.2f)", pos[0], pos[1], pos[2]);
                        javino_send_msg(response);
                    }
                }

                free(msg);
            }
        }

        double roll = 0, pitch = 0;
        double altitude = 0;
        double pos_x = 0, pos_y = 0, pos_z = 0;

        if (imu) {
            const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
            roll = rpy[0];
            pitch = rpy[1];
        }

        if (gps) {
            const double *pos = wb_gps_get_values(gps);
            pos_x = pos[0];
            pos_y = pos[1];
            pos_z = pos[2];
            altitude = pos_z;

            static int step_counter = 0;
            step_counter++;
            if (step_counter >= 5) {
                char percept[128];
                snprintf(percept, sizeof(percept), "gps(%.2f,%.2f,%.2f)", pos_x, pos_y, pos_z);
                javino_send_msg(percept);
                step_counter = 0;
            }
        }

        if (state != LANDED) {
            double roll_rate = (roll - last_roll) / (timestep / 1000.0);
            double pitch_rate = (pitch - last_pitch) / (timestep / 1000.0);
            last_roll = roll;
            last_pitch = pitch;

            double altitude_error = 0;
            double thrust_correction = 0;

            if (state != LANDING) {
                altitude_error = TARGET_ALTITUDE - altitude;
                thrust_correction = KP_ALTITUDE * altitude_error;

                if (state == TAKING_OFF && altitude >= TARGET_ALTITUDE * 0.9) {
                    state = HOVERING;
                    printf("Drone estabilizado em voo\n");
                }
            } else {
                double target_alt = TARGET_ALTITUDE * (1.0 - fmin(1.0, elapsed_time / 4.0));
                altitude_error = target_alt - altitude;
                thrust_correction = KP_ALTITUDE * altitude_error;

                if (altitude < 0.1) {
                    state = LANDED;
                    printf("Pouso completo\n");
                }
            }

            double roll_correction = KP_STABILIZE * (-roll) + KD_STABILIZE * (-roll_rate);
            double pitch_correction = KP_STABILIZE * (-pitch) + KD_STABILIZE * (-pitch_rate);

            if (state == MOVING_TO_TARGET) {
                double dx = target_x - pos_x;
                double dy = target_y - pos_y;
                double distance = sqrt(dx * dx + dy * dy);

                if (distance < 0.2) {
                    state = HOVERING;
                    printf("Alvo alcançado!\n");
                } else {
                    double direction = atan2(dy, dx);
                    double target_pitch = -NAV_SPEED * cos(direction);
                    double target_roll = -NAV_SPEED * sin(direction);

                    pitch_correction += KP_STABILIZE * (target_pitch - pitch);
                    roll_correction += KP_STABILIZE * (target_roll - roll);
                }
            } else if (state == HOVERING) {
                double pos_error_x = initial_x - pos_x;
                double pos_error_y = initial_y - pos_y;
                pitch_correction += pos_error_x * 0.1;
                roll_correction += pos_error_y * 0.1;
            }

            motor_speeds[0] = BASE_THRUST + thrust_correction - pitch_correction + roll_correction;
            motor_speeds[1] = BASE_THRUST + thrust_correction - pitch_correction - roll_correction;
            motor_speeds[2] = BASE_THRUST + thrust_correction + pitch_correction + roll_correction;
            motor_speeds[3] = BASE_THRUST + thrust_correction + pitch_correction - roll_correction;

            for (int i = 0; i < 4; i++) {
                if (motor_speeds[i] > 100.0) motor_speeds[i] = 100.0;
                if (motor_speeds[i] < 5.0) motor_speeds[i] = 5.0;
            }

            wb_motor_set_velocity(motors[0], motor_speeds[0]);
            wb_motor_set_velocity(motors[1], -motor_speeds[1]);
            wb_motor_set_velocity(motors[2], -motor_speeds[2]);
            wb_motor_set_velocity(motors[3], motor_speeds[3]);
        } else {
            for (int i = 0; i < 4; i++) {
                wb_motor_set_velocity(motors[i], 0.0);
            }
        }

        static double last_debug = 0;
        if (current_time - last_debug > 0) {
            printf("Estado: %d | Altura: %.2fm | X: %.2f | Y: %.2f\n",
                   state, altitude, pos_x, pos_y);
            last_debug = current_time;
        }
    }

    close(serial);
    wb_robot_cleanup();
    return 0;
}
