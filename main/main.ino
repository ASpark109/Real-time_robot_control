#include "GY_85.h"
#include "math.h"
#include <Wire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#define TIME_STEP  50
#define CALIB_ITER 100
#define CALIB_Z 15000
#define FK 0.3

GY_85 GY85;     //create the object
RF24 radio(9, 10); // "создать" модуль на пинах 9 и 10 Для Уно

byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб
bool button1_state = false;
bool button2_state = false;
bool flag1 = true;
bool flag2 = true;
bool flag3 = true;
bool flag4 = true;

int y = 0;

float period_time = 400;
float button_t = 0;
float Angel_X_A = 0;
float Angel_Y_A = 0;

float Angel_X = 0;
float Angel_Y = 0;
float Angel_Z = 0;

double time_s = 0;
double time_f = 0;

float angle_ax = 0;
float angle_ay = 0;

float calibration_acc_x_val = 0;
float calibration_acc_y_val = 0;

float calibration_gyr_x_val = 0;
float calibration_gyr_y_val = 0;
double offset_Z = 0;
float calibration_values[CALIB_ITER];

void calibration()
{  
   for(int i = 0; i < CALIB_ITER; i++)
   {
      calibration_values[i] = GY85.accelerometer_x(GY85.readFromAccelerometer());
   }
   for(int i = 0; i < CALIB_ITER - 1; i++)
   {
      calibration_values[i + 1] = calibration_values[i] + calibration_values[i + 1];
   }
   calibration_acc_x_val = calibration_values[CALIB_ITER - 1] / CALIB_ITER;


   
   for(int i = 0; i < CALIB_ITER; i++)
   {
      calibration_values[i] = GY85.accelerometer_y(GY85.readFromAccelerometer());
   }
   for(int i = 0; i < CALIB_ITER - 1; i++)
   {
      calibration_values[i + 1] = calibration_values[i] + calibration_values[i + 1];
   }
   calibration_acc_y_val = calibration_values[CALIB_ITER - 1] / CALIB_ITER;

   

   for(int i = 0; i < CALIB_ITER; i++)
   {
      calibration_values[i] = GY85.gyro_x(GY85.readGyro());
   }
   for(int i = 0; i < CALIB_ITER - 1; i++)
   {
      calibration_values[i + 1] = calibration_values[i] + calibration_values[i + 1];
   }
   calibration_gyr_x_val = calibration_values[CALIB_ITER - 1] / CALIB_ITER;

   

   for(int i = 0; i < CALIB_ITER; i++)
   {
      calibration_values[i] = GY85.gyro_y(GY85.readGyro());
   }
   for(int i = 0; i < CALIB_ITER - 1; i++)
   {
      calibration_values[i + 1] = calibration_values[i] + calibration_values[i + 1];
   }
   calibration_gyr_y_val = calibration_values[CALIB_ITER - 1] / CALIB_ITER;
   delay(1500);
}

void setup()
{
    Wire.begin();
    delay(10);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(3, INPUT);
    pinMode(2, INPUT);
    pinMode(13, INPUT);
    
    delay(10);
    Serial.begin(9600);
    delay(10);
    GY85.init();
    delay(10);
    calibration();
    radio.begin(); //активировать модуль
    radio.setAutoAck(1);         //режим подтверждения приёма, 1 вкл 0 выкл
    radio.setRetries(0, 15);    //(время между попыткой достучаться, число попыток)
    radio.enableAckPayload();    //разрешить отсылку данных в ответ на входящий сигнал
    radio.setPayloadSize(32);     //размер пакета, в байтах
  
    radio.openWritingPipe(address[0]);   //мы - труба 0, открываем канал для передачи данных
    radio.setChannel(0x5f);  //выбираем канал (в котором нет шумов!)
  
    radio.setPALevel (RF24_PA_MAX); //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radio.setDataRate (RF24_2MBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
    //должна быть одинакова на приёмнике и передатчике!
    //при самой низкой скорости имеем самую высокую чувствительность и дальность!!
  
    radio.powerUp(); //начать работу
    radio.stopListening();  //не слушаем радиоэфир, мы передатчик
}
float data[] = {100, 5.21, 10.77, 8.32};
void loop()
{
    float ax = GY85.accelerometer_x(GY85.readFromAccelerometer()) - calibration_acc_x_val;
    float ay = GY85.accelerometer_y(GY85.readFromAccelerometer()) - calibration_acc_y_val;
    float az = GY85.accelerometer_z(GY85.readFromAccelerometer()) - 255;
    
    float gx = GY85.gyro_x(GY85.readGyro()) - calibration_gyr_x_val;
    float gy = GY85.gyro_y(GY85.readGyro()) - calibration_gyr_y_val;
    float gz = GY85.gyro_z(GY85.readGyro()) - offset_Z;

    time_f = millis();
    Angel_X = Angel_X + gx * (time_f - time_s) * 0.001;
    Angel_Y = Angel_Y + gy * (time_f - time_s) * 0.001;
    Angel_Z = Angel_Z + gz * (time_f - time_s) * 0.001;
    time_s = millis();
    Angel_Z -= 0.00309329608;
    
    ay = ay / 255;
    ax = ax / 255;
    az = az / 255;
  
    ay = constrain(ay, -1.0, 1.0);
    ax = constrain(ax, -1.0, 1.0);

    angle_ax = 90 - acos(ay) * 180 / PI;
    angle_ay = 90 - acos(ax) * 180 / PI;
    
    Angel_X_A = Angel_X_A * (1 - FK) + angle_ax * FK;
    Angel_Y_A = Angel_Y_A *(1 - FK) + angle_ay * FK;



    if(digitalRead(3) == true && flag1 == true)
    {
      if(millis() - button_t <= 300)
      {
        flag3 = !flag3;
      }
      button_t = millis();
      delay(30);
      flag1 = false;
      button1_state = !button1_state;
    }
    
    if(digitalRead(2) == true && flag2 == true)
    {
      delay(30);
      flag2 = false;
      button2_state = !button2_state;
    }
  
    if(digitalRead(3) == false && flag1 == false)
    {
      delay(30);
      flag1 = true;
    }
    
    if(digitalRead(2) == false && flag2 == false)
    {
      delay(30);
      flag2 = true;
      delay(10);
    }

    if(flag3 == true)
    {
      y = 4;
      if (millis() - button_t >= period_time) 
      {
         button_t = millis();
         flag4 = !flag4;
      }
      digitalWrite(4, flag4);
    }
    else if(button1_state == true && button2_state == true)
    {
      y = 0;
      digitalWrite(4, HIGH);
      digitalWrite(5, HIGH);
    }
    else if(button1_state == true && button2_state == false)
    {
      y = 1;
      digitalWrite(4, HIGH);
      digitalWrite(5, LOW);
    }
    else if(button1_state == false && button2_state == true)
    {
      y = 2;
      digitalWrite(5, HIGH);
      digitalWrite(4, LOW);
    }

    else if(button1_state == false && button2_state == false)
    {
      y = 3;
      digitalWrite(5, LOW);
      digitalWrite(4, LOW);
    }
    
    data[0] = y;
    data[1] = Angel_X_A;
    data[2] = Angel_Y_A;
    data[3] = Angel_Z;
    radio.write(&data, sizeof(data));

    delay(TIME_STEP);
}
