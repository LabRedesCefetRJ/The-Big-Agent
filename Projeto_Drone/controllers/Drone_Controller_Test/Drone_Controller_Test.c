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
    
    // Inicialização segura dos motores (todos desligados)
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

    const double TARGET_ALTITUDE = 1.0;  // Altura alvo em metros
    const double BASE_THRUST = 62.0;     // Força base para hover
    const double KP_STABILIZE = 15.0;    // Ganho para estabilização
    const double KD_STABILIZE = 3.0;     // Ganho derivativo
    const double KP_ALTITUDE = 8.0;      // Ganho para controle de altura

    double last_roll = 0, last_pitch = 0;
    double initial_x = 0, initial_z = 0;

    printf("Calibrando sensores...\n");
    for(int i = 0; i < 100; i++) {
        wb_robot_step(timestep);
        // Garante motores desligados durante calibração
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
                    
                    // Captura posição inicial para hover
                    if (gps) {
                        const double *pos = wb_gps_get_values(gps);
                        initial_x = pos[0];
                        initial_z = pos[2];
                    }
                } else if (strcmp(msg, "goahead") == 0 && state == HOVERING) {
                    state = MOVING_FORWARD;
                    state_start_time = current_time;
                    printf("Movendo para frente\n");
                } else if (strcmp(msg, "land") == 0 && (state == HOVERING || state == MOVING_FORWARD)) {
                    state = LANDING;
                    state_start_time = current_time;
                    printf("Iniciando pouso\n");
                } else if (strcmp(msg, "getpercepts") == 0) {
                    char response[100];
                    if (gps) {
                        const double *pos = wb_gps_get_values(gps);
                        snprintf(response, sizeof(response), "pos:%.2f,%.2f,%.2f", pos[0], pos[1], pos[2]);
                    } else {
                        snprintf(response, sizeof(response), "state:%d", state);
                    }
                    javino_send_msg(response);
                    printf("Enviando dados: %s\n", response);
                }

                free(msg);
            }
        }

        // Leitura dos sensores
        double roll = 0, pitch = 0;
        double altitude = 0;
        double pos_x = 0, pos_z = 0;

        if (imu) {
            const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
            roll = rpy[0];
            pitch = rpy[1];
        }

        if (gps) {
            const double *pos = wb_gps_get_values(gps);
            altitude = pos[1];
            pos_x = pos[0];
            pos_z = pos[2];
        }

        // Controle dos motores - só ativo se não estiver no estado LANDED
        if (state != LANDED) {
            // Cálculo de derivadas para controle D
            double roll_rate = (roll - last_roll) / (timestep / 1000.0);
            double pitch_rate = (pitch - last_pitch) / (timestep / 1000.0);
            last_roll = roll;
            last_pitch = pitch;

            // Controle de altitude
            double altitude_error = 0;
            double thrust_correction = 0;

            if (state == TAKING_OFF || state == HOVERING || state == MOVING_FORWARD) {
                altitude_error = TARGET_ALTITUDE - altitude;
                thrust_correction = KP_ALTITUDE * altitude_error;

                // Transição para hover quando atingir a altura alvo
                if (state == TAKING_OFF && altitude >= TARGET_ALTITUDE * 0.9) {
                    state = HOVERING;
                    printf("Drone estabilizado em voo\n");
                }
            }

            // Estabilização com controle PD
            double roll_correction = KP_STABILIZE * (-roll) + KD_STABILIZE * (-roll_rate);
            double pitch_correction = KP_STABILIZE * (-pitch) + KD_STABILIZE * (-pitch_rate);

            // Manter posição durante hover
            if (state == HOVERING && gps) {
                double pos_error_x = initial_x - pos_x;
                double pos_error_z = initial_z - pos_z;
                pitch_correction += pos_error_z * 0.1;
                roll_correction += pos_error_x * 0.1;
            }

            // Movimento para frente controlado
            double target_pitch = 0;
            if (state == MOVING_FORWARD) {
                if (elapsed_time < 1.0) {
                    target_pitch = -0.1 * elapsed_time;  // Aceleração suave
                } else if (elapsed_time < 4.0) {
                    target_pitch = -0.1;  // Velocidade constante
                } else if (elapsed_time < 5.0) {
                    target_pitch = -0.1 * (5.0 - elapsed_time);  // Desaceleração
                } else {
                    state = HOVERING;
                    printf("Movimento concluído\n");
                }
                pitch_correction += KP_STABILIZE * (target_pitch - pitch);
            }

            // Cálculo das velocidades dos motores
            motor_speeds[0] = BASE_THRUST + thrust_correction - pitch_correction + roll_correction;
            motor_speeds[1] = BASE_THRUST + thrust_correction - pitch_correction - roll_correction;
            motor_speeds[2] = BASE_THRUST + thrust_correction + pitch_correction + roll_correction;
            motor_speeds[3] = BASE_THRUST + thrust_correction + pitch_correction - roll_correction;

            // Limites de segurança
            for (int i = 0; i < 4; i++) {
                if (motor_speeds[i] > 100.0) motor_speeds[i] = 100.0;
                if (motor_speeds[i] < 5.0) motor_speeds[i] = 5.0;
            }

            // Aplicar velocidades nos motores
            wb_motor_set_velocity(motors[0], motor_speeds[0]);
            wb_motor_set_velocity(motors[1], -motor_speeds[1]);
            wb_motor_set_velocity(motors[2], -motor_speeds[2]);
            wb_motor_set_velocity(motors[3], motor_speeds[3]);
        } else {
            // Estado LANDED - garante motores desligados
            for (int i = 0; i < 4; i++) {
                wb_motor_set_velocity(motors[i], 0.0);
            }
        }

        // Debug
        static double last_debug = 0;
        if (current_time - last_debug > 5) {
            printf("Estado: %d | Altura: %.2fm | Roll: %.2f° | Pitch: %.2f°\n",
                   state, altitude, roll * 180 / M_PI, pitch * 180 / M_PI);
            if (state != LANDED) {
                printf("Motores: FL=%.1f, FR=%.1f, RL=%.1f, RR=%.1f\n",
                       motor_speeds[0], motor_speeds[1],
                       motor_speeds[2], motor_speeds[3]);
            }
            last_debug = current_time;
        }
    }

    close(serial);
    wb_robot_cleanup();
    return 0;
}