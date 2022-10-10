#include"ServoTimer2.h"
#include <LiquidCrystal.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>
#include <string.h>
#include <math.h>

#define MOTOR_STEP 10
#define SERVO_STEP 10
#define MOTOR_PIN A4
#define SERVO_PIN A3

const float LAT = -31.97855;
const float LONG = 115.81772;
NeoGPS::Location_t des( -31.97855, 115.81772 );
ServoTimer2 motor;
ServoTimer2 servo;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
NMEAGPS gps;
NeoSWSerial gpsPort( A1, A2 );

void setup() {
  //gps setup
  gpsPort.begin(9600);
  Serial.begin(9600);
  //motor and servo setup
  motor.attach(MOTOR_PIN);
  servo.attach(SERVO_PIN);
  //LCD and Keypad setup
  lcd.begin(16, 2);
}

void loop() {
  int motor_pwm = 1500;
  float angle;
  static int servo_position = 1800;
  static int time_held = 0;
  static int forward = 0;
  static int display_timer = 0;

  angle = get_coordinate();
  key_pad_input(motor_pwm, servo_position, time_held, forward);
  motor.write(motor_pwm);
  servo.write(servo_position); 

  swap_display(display_timer, motor_pwm, servo_position);
  delay(100);
}

float get_direction(float lat, float lon){
  float dx, dy, radians, degree;
  dy = LAT - lat;
  dx = LONG - lon;
  radians = atan2(dy, dx);
  degree = radians*(180/ M_PI);
  return degree < 0 ? degree + 360 : degree;
}

float get_coordinate(){
  float lat, lon;
  float angle;
  while(gps.available( gpsPort )){
    gps_fix fix = gps.read();
    if(fix.valid.location){
      lat = fix.latitude(); 
      lon = fix.longitude();
      angle = get_direction(lat, lon);
      display_gps_value(lat, lon);
      //display_direction(angle);
      double distanceToDestination = fix.location.DistanceKm( des );
      double courseToDestination   = fix.location.BearingToDegrees( des );
      const __FlashStringHelper *directionToDestination = compassDir(courseToDestination);
      int courseChangeNeeded = (int)(360 + courseToDestination - fix.heading()) % 360;
      Serial.print( F("DEBUG: Course2Dest: ") );
      Serial.print(courseToDestination);
      Serial.print( F("  CurCourse: ") );
      if (fix.valid.heading)
        Serial.print( fix.heading() );
      Serial.print( F("  Dir2Dest: ") );
      Serial.print(directionToDestination);
      Serial.print( F("  RelCourse: ") );
      Serial.println(courseChangeNeeded);
      /* Serial.print( F(" Lat: ") );
      Serial.print( fix.latitude(), 6 );
      Serial.print( F("  Lon: ") );
      Serial.println( fix.longitude(), 6 );
      Serial.print( F("  CurSpd: ") );
      if (fix.valid.speed)
        Serial.print( fix.speed_kph() );
      Serial.println('\n');
      if (distanceToDestination <= 20.0)
      {
        Serial.println( F("CONGRATULATIONS: You've arrived!") );
        exit(1);
      }

      Serial.print( F("DISTANCE: ") );
      Serial.print(distanceToDestination);
      Serial.println( F(" meters to go.") );
      Serial.print( F("INSTRUCTION: ") );

      // Standing still? Just indicate which direction to go.
      if (fix.speed_kph() < 5.0)
      {
        Serial.print( F("Head ") );
        Serial.print(directionToDestination);
        Serial.println( '.' );

      } else // suggest a course change
      if ((courseChangeNeeded >= 345) || (courseChangeNeeded < 15))
        Serial.println( F("Keep on straight ahead!") );
      else if ((courseChangeNeeded >= 315) && (courseChangeNeeded < 345))
        Serial.println( F("Veer slightly to the left.") );
      else if ((courseChangeNeeded >= 15) && (courseChangeNeeded < 45))
        Serial.println( F("Veer slightly to the right.") );
      else if ((courseChangeNeeded >= 255) && (courseChangeNeeded < 315))
        Serial.println( F("Turn to the left.") );
      else if ((courseChangeNeeded >= 45) && (courseChangeNeeded < 105))
        Serial.println( F("Turn to the right.") );
      else
        Serial.println( F("Turn completely around.") );
        */
    } 
  }
  return angle;
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

void swap_display(int &display_timer, int motor_val, int servo_val){
  if(display_timer < 40){
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

void display_direction(double angle){
  if(angle > 0){
    Serial.print("ANGLE:");
    Serial.println(angle);
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

void display_gps_value(float lat, float lon){
  lcd.setCursor(0,0);
  lcd.print("LAT:");
  lcd.setCursor(4,0);
  lcd.print(lat);
  
  lcd.setCursor(0,1);
  lcd.print("LON:");
  lcd.setCursor(4,1);
  lcd.print(lon);
}

//------------------------------------------------------------
//  This snippet is from NMEAaverage.  It keeps all the
//    compass direction strings in FLASH memory, saving RAM.

const char nCD  [] PROGMEM = "N";
const char nneCD[] PROGMEM = "NNE";
const char neCD [] PROGMEM = "NE";
const char eneCD[] PROGMEM = "ENE";
const char eCD  [] PROGMEM = "E";
const char eseCD[] PROGMEM = "ESE";
const char seCD [] PROGMEM = "SE";
const char sseCD[] PROGMEM = "SSE";
const char sCD  [] PROGMEM = "S";
const char sswCD[] PROGMEM = "SSW";
const char swCD [] PROGMEM = "SW";
const char wswCD[] PROGMEM = "WSW";
const char wCD  [] PROGMEM = "W";
const char wnwCD[] PROGMEM = "WNW";
const char nwCD [] PROGMEM = "NW";
const char nnwCD[] PROGMEM = "NNW";

const char * const dirStrings[] PROGMEM =
  { nCD, nneCD, neCD, eneCD, eCD, eseCD, seCD, sseCD, 
    sCD, sswCD, swCD, wswCD, wCD, wnwCD, nwCD, nnwCD };

const __FlashStringHelper *compassDir( uint16_t bearing ) // degrees CW from N
{
  const int16_t directions    = sizeof(dirStrings)/sizeof(dirStrings[0]);
  const int16_t degreesPerDir = 360 / directions;
        int8_t  dir           = (bearing + degreesPerDir/2) / degreesPerDir;

  while (dir < 0)
    dir += directions;
  while (dir >= directions)
    dir -= directions;

  return (const __FlashStringHelper *) pgm_read_ptr( &dirStrings[ dir ] );

} // compassDir