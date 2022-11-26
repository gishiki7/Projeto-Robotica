#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>

#define TIME_STEP 256
#define TIME_STEP_CAM 64
#define MAX_SPEED 6.28
#define QtddSensoresProx 8

int main(int argc, char **argv) {
  // Configuracao das variaveis
  int i = 0, j = 0, vermelho = 0, verde = 0, azul = 0, repeat = 0;
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
  bool ObstaculoFrente = false, ObstaculoDireita45 = false, ObstaculoDireita90 = false, FaixaDetectada = false;
  
  wb_robot_init();
  
  // Configuracao dos motores
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo, 0.0);
  wb_motor_set_velocity(MotorDireito, 0.0);
  
  // Configuracao dos sensores de proximidade
  WbDeviceTag SensorProx[8];
  char NomeSensorProx[4];
  for (i = 0; i < 8 ; i++) {
    sprintf(NomeSensorProx, "ps%d", i);
    SensorProx[i] = wb_robot_get_device(NomeSensorProx);
    wb_distance_sensor_enable(SensorProx[i], TIME_STEP);
  }
  
  // Configuracoes da camera
  WbDeviceTag Camera;
  unsigned short Largura, Altura, LimiteAltura;
  const unsigned char *image;
  Camera = wb_robot_get_device("camera");
  wb_camera_enable(Camera, TIME_STEP_CAM);
  Largura = wb_camera_get_width(Camera);
  Altura = wb_camera_get_height(Camera);
  LimiteAltura = Altura - (Altura / 5);
 
  
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    for (i = 0; i < 8 ; i++)
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]);

    // detecta obstaculos
    ObstaculoFrente = LeituraSensorProx[0] > 85.0;
    ObstaculoDireita45 = LeituraSensorProx[1] > 85.0;
    ObstaculoDireita90 = LeituraSensorProx[2] > 85.0;
    
    image = wb_camera_get_image(Camera);
    
    for (i = 0; i < Largura; i++) {
      for (j = LimiteAltura; j < Altura; j++) {
        vermelho = wb_camera_image_get_red(image, Largura, i, j);
        verde = wb_camera_image_get_green(image, Largura, i, j);
        azul = wb_camera_image_get_blue(image, Largura, i, j);
        FaixaDetectada = (vermelho >= 128 && verde >= 128 && azul < 64);
        if (FaixaDetectada)
          break;
      }
      if (FaixaDetectada)
          break;
    }
    
    if (FaixaDetectada) { // vira a direita
      AceleradorEsquerdo = 1;
      AceleradorDireito = -1;
      repeat = 0;
    }
    if (ObstaculoFrente) { // vira a esquerda
      AceleradorEsquerdo = -1;
      AceleradorDireito = 1;
      repeat = 0;
    }
    else {
      if (ObstaculoDireita90) { // segue em frente
        AceleradorEsquerdo = 1;
        AceleradorDireito = 1;
        repeat = 0;
      }
      else if (repeat < 5) { // vira a direita
        AceleradorEsquerdo = 1;
        AceleradorDireito = 0.125;
        repeat++;
      }
      else { // segue em frente
        AceleradorEsquerdo = 1;
        AceleradorDireito = 1;
      }
      if (ObstaculoDireita45) { // vira a esquerda
        AceleradorEsquerdo = 0.125;
        AceleradorDireito = 1;
        repeat = 0;
      }
    }

    wb_motor_set_velocity(MotorEsquerdo, AceleradorEsquerdo * MAX_SPEED);
    wb_motor_set_velocity(MotorDireito, AceleradorDireito * MAX_SPEED);
  }

  wb_robot_cleanup();
  return 0;
}
