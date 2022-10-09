#include"ServoTimer2.h"
#include <LiquidCrystal.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>
#include <string.h>

#define MOTOR_STEP 10
#define SERVO_STEP 10
#define MOTOR_PIN A4
#define SERVO_PIN A3

ServoTimer2 motor;
ServoTimer2 servo;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
float lat, lon;
NMEAGPS gps;
NeoSWSerial gpsPort( A1, A2 );

void setup() {
  gpsPort.begin(9600);
  //motor and servo setup
  motor.attach(MOTOR_PIN);
  servo.attach(SERVO_PIN);
  //LCD and Keypad setup
  lcd.begin(16, 2);
}

void loop() {
  int motor_pwm = 1500;
  static int servo_position = 1800;
  static int time_held = 0;
  static int forward = 0;
  static int display_timer = 0;

  get_coordinate();
  key_pad_input(motor_pwm, servo_position, time_held, forward);
  motor.write(motor_pwm);
  servo.write(servo_position); 

  swap_display(display_timer, motor_pwm, servo_position);
  delay(100);
}

void key_pad_input(int &motor_pwm, int &servo_position, int &time_held, int &forward){
  int analog_range = analogRead (A0);

  if(analog_range < 100){ //right
    servo_position = servo_right(servo_position);
  }
  else if(analog_range < 300){//up
    if(!forward){
      forward = 1;
      time_held = 0;
    }
    time_held++;
    motor_pwm = motor_FORWARD(motor_pwm, time_held);
  }
  else if(analog_range < 500){//down
    if(forward){
      forward = 0;
      time_held = 0;
    }
    time_held++;
    motor_pwm = motor_REVERSE(motor_pwm, time_held);
  }
  else if(analog_range < 700){//left
    servo_position = servo_left(servo_position);
  }
  else if(analog_range < 900){//select
    reset(motor_pwm, servo_position);
  }

}

//Max motor forward = 750.
int motor_FORWARD(int motor_pwm, int time_held){
  if(motor_pwm - (time_held * MOTOR_STEP) > 750){
    return motor_pwm -= (time_held * MOTOR_STEP);
  }
  return motor_pwm;
}

//Max motor reverse = 2400.
int motor_REVERSE(int motor_pwm, int time_held){
  if(motor_pwm + (time_held * MOTOR_STEP) < 2400){
    return motor_pwm += (time_held * MOTOR_STEP);
  }
  return motor_pwm;
}

//Max servo left = 2000.
int servo_left(int servo_pwm){
  if(servo_pwm < 2000){
    return servo_pwm += SERVO_STEP;
  }
  return servo_pwm;
}

//Max servo right = 1600.
int servo_right(int servo_pwm){
  if(servo_pwm > 1600){
    return servo_pwm -= SERVO_STEP;
  }
  return servo_pwm;
}

void reset(int &motor_pwm, int &servo_pwm){
  motor_pwm = 1500;
  servo_pwm = 1800;
}

void get_coordinate(){
  while(gps.available( gpsPort )){
    gps_fix fix = gps.read();
    if(fix.valid.location){
      lat = fix.latitude(); 
      lon = fix.longitude(); 
    }
  }
}

void swap_display(int &display_timer, int motor_val, int servo_val){
  if(display_timer < 40){
    display_gps_value();
  }
  else{
    display_servo_value(motor_val, servo_val);
  }
  if(display_timer >= 80){
    display_timer = 0;
  }
  else{
    display_timer++;
  }
}

void display_servo_value(int motor_val, int servo_val){
  lcd.setCursor(0,0);
  lcd.print("MOTOR:");
  lcd.setCursor(0,1);
  lcd.print("SERVO:");
  lcd.setCursor(6,0);
  lcd.print(motor_val);
  lcd.setCursor(6,1);
  lcd.print(servo_val);
}

void display_gps_value(){
  lcd.setCursor(0,0);
  lcd.print("LON:");
  lcd.setCursor(0,1);
  lcd.print("LAT:");
  lcd.setCursor(4,0);
  lcd.print(lat);
  lcd.setCursor(4,1);
  lcd.print(lon);
}