#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <javino.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define TTY_EXOGENOUS_PORT "/dev/ttyExogenous0"

int main(int argc, char **argv) {

  // Configuração serial
  int serial = open(TTY_EXOGENOUS_PORT, O_RDWR | O_NOCTTY);
  if (serial < 0) {
    perror("Erro ao abrir porta serial");
    return 1;
  }

  wb_robot_init();
  int timeStep = (int)wb_robot_get_basic_time_step();

  // Intervalo de impressão
  double last_print_time = 0.0;
  double print_interval = 5.0;

  // Dispositivos: motores e sensores
  WbDeviceTag motor_fl = wb_robot_get_device("front left propeller");
  WbDeviceTag motor_fr = wb_robot_get_device("front right propeller");
  WbDeviceTag motor_rl = wb_robot_get_device("rear left propeller");
  WbDeviceTag motor_rr = wb_robot_get_device("rear right propeller");
  wb_motor_set_position(motor_fl, INFINITY);
  wb_motor_set_position(motor_fr, INFINITY);
  wb_motor_set_position(motor_rl, INFINITY);
  wb_motor_set_position(motor_rr, INFINITY);
  wb_motor_set_velocity(motor_fl, 0.0);
  wb_motor_set_velocity(motor_fr, 0.0);
  wb_motor_set_velocity(motor_rl, 0.0);
  wb_motor_set_velocity(motor_rr, 0.0);

  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timeStep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timeStep);

  javino_init(serial);

  const double targetAltitude = 3.0;
  const double targetX = -3.0;
  const double targetZ = 0.3;
  const double Kp_att = 8.0, Kd_att = 4.0;
  const double Kp_alt = 5.0, Kd_alt = 3.0;
  const double Kp_land = 3.0, Kd_land = 1.0;
  const double Knav = 0.1;
  const double baseVel = 150.0;

  int state = 0;
  double prevRoll = 0.0, prevPitch = 0.0;
  double prevAlt = 0.0;
  double dt = timeStep / 1000.0;

  while (wb_robot_step(timeStep) != -1) {
    char *msg = javino_get_msg();
    if (msg && msg[0] != '\0') {
      if (strcmp(msg, "takeoff") == 0 && state == 0) {
        printf("Iniciando decolagem\n");
        state = 1;
      } else if (strcmp(msg, "goAhead") == 0 && state == 2) {
        printf("Se movimentando\n");
        state = 3;
      } else if (strcmp(msg, "land") == 0 && (state == 2 || state == 3)) {
        printf("Pousando\n");
        state = 4;
      }
    }

    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double roll = rpy[0];
    double pitch = rpy[1];
    const double *gps_val = wb_gps_get_values(gps);
    double currX = gps_val[0];
    double currAlt = gps_val[1];
    double currZ = gps_val[2];

    double rollRate = (roll - prevRoll) / dt;
    double pitchRate = (pitch - prevPitch) / dt;
    double vertVel = (currAlt - prevAlt) / dt;

    double altError = targetAltitude - currAlt;
    double vertCorr = Kp_alt * altError - Kd_alt * vertVel;

    double pitchCorr = 0.0, rollCorr = 0.0;
    double motorVelFL = 0.0, motorVelFR = 0.0, motorVelRL = 0.0, motorVelRR = 0.0;

    if (state == 0) {
      motorVelFL = motorVelFR = motorVelRL = motorVelRR = 0.0;
    } else if (state == 1) {
      double v = baseVel + vertCorr;
      motorVelFL = motorVelFR = motorVelRL = motorVelRR = v;
      if (fabs(altError) < 0.05) {
        printf("Estabilizando");
        state = 2;
      }
    } else if (state == 2) {
      printf("Estabilizando");
      
      double v = baseVel + vertCorr;
      pitchCorr = - (Kp_att * pitch + Kd_att * pitchRate);
      rollCorr  = - (Kp_att * roll  + Kd_att * rollRate);
      motorVelFL = v - pitchCorr + rollCorr;
      motorVelFR = v - pitchCorr - rollCorr;
      motorVelRL = v + pitchCorr + rollCorr;
      motorVelRR = v + pitchCorr - rollCorr;
    } else if (state == 3) {
      double v = baseVel + vertCorr;
      double errX = targetX - currX;
      double errZ = targetZ - currZ;
      double desiredPitch = Knav * errX;
      double desiredRoll  = Knav * errZ;
      pitchCorr = Kp_att * (desiredPitch - pitch) - Kd_att * pitchRate;
      rollCorr  = Kp_att * (desiredRoll  - roll ) - Kd_att * rollRate;
      motorVelFL = v - pitchCorr + rollCorr;
      motorVelFR = v - pitchCorr - rollCorr;
      motorVelRL = v + pitchCorr + rollCorr;
      motorVelRR = v + pitchCorr - rollCorr;
      if (fabs(errX) < 0.1 && fabs(errZ) < 0.1) {
        state = 2;
      }
    } else if (state == 4) {
      double altErrorL = 0.3 - currAlt;
      double vertCorrL = Kp_land * altErrorL - Kd_land * vertVel;
      double v = baseVel + vertCorrL;
      motorVelFL = motorVelFR = motorVelRL = motorVelRR = v;
      if (currAlt <= 0.31) {
        motorVelFL = motorVelFR = motorVelRL = motorVelRR = 0.0;
        state = 0;
      }
    }

    wb_motor_set_velocity(motor_fl, motorVelFL);
    wb_motor_set_velocity(motor_fr, motorVelFR);
    wb_motor_set_velocity(motor_rl, motorVelRL);
    wb_motor_set_velocity(motor_rr, motorVelRR);

    // Impressão periódica a cada 1 segundo
    double current_time = wb_robot_get_time();
    if (current_time - last_print_time >= print_interval) {
      printf("[%.2f] Estado: %d | Altitude: %.2f | ErroAlt: %.2f | v: %.2f\n",
             current_time, state, currAlt, altError, baseVel + vertCorr);
      last_print_time = current_time;
    }

    prevRoll = roll;
    prevPitch = pitch;
    prevAlt  = currAlt;
  }

  close(serial);
  wb_robot_cleanup();
  return 0;
}
