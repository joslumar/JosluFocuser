/***********************************************************************
 * 
 * microsteps 32u4 direct pwm code & heater controller (c) joslumar
 * 
 * License GPL: http://www.gnu.org/licenses/gpl.html
 * 
 * Pending: new GPL parser & ASCOM driver robocofus independent
 * 
 * A special thanks to the published work of nando (nandofocus8), which greatly facilitated the start of this project
 * Site: http://es.groups.yahoo.com/group/nandofocus_group/
 * License: http://creativecommons.org/licenses/by-sa/3.0/deed.es
 * 
 * // testing cmds
 * 
 * 
 * # robofocus CMDs:
 * F{C}{VVVVVVV}
 * C command
 * V argument/value (6 bytes)
 * 
 * FV000000#
 * FP000000#
 * FC000000#
 * FT000000#
 * FS000000#
 * FS010000#
 * FI002000#
 * FO002000#
 * FI000010#
 * FO000010#
 * FI000100#
 * FO000100#
 * FI002000#
 * FO002000#
 * FD000000#
 * FR000000#
 * FM500500#
 * FM000000#
 *
 * No robocofus:
 * FM500500#   // microsteps + motor loop timming
 * 
 * 
 * // sodeco-saia -> 120 pasos / rev => pasos de 3º
 * 
 **************************************************************************/


//#define _FOCUSER_DEBUG
#ifdef _FOCUSER_DEBUG
#define _FOCUSER_MOTOR_DEBUG
//#define _FOCUSER_MOTOR_LOOP_DEBUG
//#define _FOCUSER_LOOP_DEBUG
#endif

#include <Arduino.h>
#include <EEPROM.h>
#include <TimerOne.h> // Warning!! this disables the PWM 9 and 10 outputs // http://playground.arduino.cc/Code/Timer1
#include <DHT.h>      // http://playground.arduino.cc/Main/DHTLib
#include "JosluFocuser.h"

#define VERSION "14"
#define ROBOFOCUS_VERSION "000111" //don't change

//#define DHT_TYPE DHT22   // AM2302
#define DHT_TYPE AM2301    // DHT21, AM2301
#define DHT_PIN        7   // what pin we're connected to DHT

#define PIN_LED       12

// ATmega32u4 timers & PWM
// http://www.atmel.com/images/7766s.pdf
//
// http://arduino.cc/en/pmwiki.php?n=Hacking/PinMapping32u4
// http://provideyourown.com/2012/arduino-leonardo-versus-uno-whats-new/
//
// 3 -> PD0 (SCL)          OC0B         timer0
// 5 -> PC6 PWM 16 bit     OC3A/!OC4A   timer3      !A timer4 opt?
// 6 -> PD7 PWM HiR        OC4D         timer4
// 9 -> PB5 PWM 16 bit     OC1A/!OC4B   timer1      !B timer4 opt?
// 10 -> PB6 PWM 16bit     OC1B/OC4B    timer1      timer4 opt
// 11 -> PB7 PWM 8/16bit   OC0A/OC1C    timer0      timer1 opt
// 13 -> PC7 PWM 10 bit    OC4A         timer4
// 12 -> PD6 PWM Hi (?)    !OC4D                    !D timer4 opt?
//
// timer 2 don't exist in the 32u4 controller
// timer 0 -> this is used by millis() function

#define PIN_BOBINA_1A  5 // timer 3
#define PIN_BOBINA_1B  6 // timer 4
#define PIN_BOBINA_2A 11 // timer 0
#define PIN_BOBINA_2B 13 // timer 4

// For PWM every motor step, pwm period coil control must be mucho smaller than interrupt motor loop timer
// then I must play with avr (32u4) timiers internals
/*PWM SETUP
 timers 0,1 & 3:
 Setting    Divisor    Frequency [Hz](phase-correct mode)
 0x01        1        31372.55 *  3.2us
 0x02        8         3921.16 * 25.5us
 0x03       64          490.20
 0x04      256          122.55
 0x05     1024           30.64
 timer 4:
 0x01        1        31372.55 * 3.2us
 0x02        2        15686.27
 0x03        4         7843.13
 0x04        8         3921.16 * 25.5us
 0x05       16         1960.78
 */
#define TIMERSPRESCALER 8 // 1 8

#if TIMERSPRESCALER == 1
#define TIMERPRESCALER0_VALUE B001
#define TIMERPRESCALER4_VALUE B001
#elif TIMERSPRESCALER == 8
#define TIMERPRESCALER0_VALUE B010
#define TIMERPRESCALER4_VALUE B100
#endif

#define millis() ( ( millis() << 1 ) / TIMERSPRESCALER )

// microsteps table
// octave:
// n=128; A=[round(sin([0:n/2-1]*pi/n*2)*255), zeros(1,n/2)]; A
// MS=16 ; B=[]; for i = 0:MS-1 ; B=[B A(i*n/MS+1)] ; endfor ; 
#define MAXMICROSTEPS 32 // son 128 micropasos / ciclo
volatile const byte pwmStepsTable[MAXMICROSTEPS<<1]={
  0,  13,  25,  37,  50,  62,  74,  86,
  98, 109, 120, 131, 142, 152, 162, 171,
  180, 189, 197, 205, 212, 219, 225, 231,
  236, 240, 244, 247, 250, 252, 254, 255,
  255, 255, 254, 252, 250, 247, 244, 240,
  236, 231, 225, 219, 212, 205, 197, 189,
  180, 171, 162, 152, 142, 131, 120, 109,
  98 ,  86,  74,  62,  50,  37,  25,  13
};

#define MAXRELAX  1000

#define PIN_B0 A3
#define PIN_B1 A5
#define PIN_B2 A4
// #define PIN_B3 4

#define SENSORTEMP_A_PIN A0
#define SENSORTEMP_B_PIN A1
#define SENSORTEMP_C_PIN A2

#define HEATER_A_PIN   2   // PWM bitbang
#define HEATER_B_PIN   9   // PWM bitbang // timer 1 pwm
#define HEATER_C_PIN   10  // PWM bitbang // timer 1 pwm

#define CONFIG_VERSION "101"
struct EEpromStore{
  char version[4];
  int reset;
  int step;
  unsigned int position;
  unsigned int backslash_dir;
  unsigned int backslash;
  unsigned int limit;
  byte sw1, sw2, sw3, sw4;
  unsigned int microSteps;
  unsigned int timerPeriod;
  byte hsw1, hsw2, hsw3;
  byte hpw1, hpw2, hpw3;
};

EEpromStore mieeprom;
EEpromStore reset = {
  CONFIG_VERSION, 1, 0, 20000, '2', 0, 65000, 0, 0, 0, 0, 16, 500, 0, 0, 0, 0, 0, 0 };

DHT dht(DHT_PIN, DHT_TYPE);

#define MAXLEN_SERIAL_INPUT_STRING 80
String serialInputString = "";         // a string to hold incoming data
int serialInputCount = 0;

cmdInputModeType cmdInputMode = robofocus_mode; // default mode, robofocus


// motor control

#define SPEED_MAX  8
//#define TICK_USEC 6493+2 // half step 8x
////#define TICK_USEC 12985+0 // single step 8x

// deberia poder corregir el decimal del tamaño preciso de tick haciendo perder un tick de vez en cuando

volatile unsigned long t1_ticks = 0;
volatile boolean interrupting_t1 = false;

volatile byte motion_speed  = 4;   // 2, 4, 8
volatile boolean priomotion = false;   // motor a toda leche

volatile int compensa_backslash = 0;
volatile unsigned int targetPos = 30000;

volatile int relax_ticks = 0;

volatile short int step = 0;   // 0 .. ULTIMOPASO
volatile unsigned int motorPos = 30000;
volatile unsigned int backslash_dir;
volatile unsigned int backslash = 30;
volatile unsigned int limit = 65536;

volatile int delay_tick;  // tiempo muerto mientras espera el siguiente slot tick para cumplir con la velocidad programada

volatile unsigned long timer1Period=500; // 1000us step size (a 250us pierde par)
volatile unsigned int microSteps=16;    // 1 2 4 8 16 32 micropasos/paso => 4 8 16 32 64 128 micropasos / ciclo ; 1 ciclo = 4 pasos enteros


volatile float temperature = NAN;
volatile float humidity = NAN;
volatile float dewPointValue = NAN;

// for inputs
volatile int    lastSensorATemp_Value,lastSensorBTemp_Value,lastSensorCTemp_Value;
volatile float lastSensorATemp,lastSensorBTemp,lastSensorCTemp;

// for filters
int    xSensorATempValues[4],xSensorBTempValues[4],xSensorCTempValues[4];

volatile boolean loop_halfsecond_ready = true;

// for soft PWM (heaters)
volatile unsigned int pwmHeatersCounter = 0; // (0..64)

// soft pwm heaters (0..64)
volatile int heaterA, heaterB, heaterC  = 0; // value < 0 => OFF  -64 .. 64

volatile byte sw1, sw2, sw3, sw4;

char response_str[10]={
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

char tmp_str[256]; // resulting string limited to 256 chars

/*
// http://playground.arduino.cc/Main/DHT11Lib
 // dewPoint function NOAA
 // reference (1) : http://wahiduddin.net/calc/density_algorithms.htm
 // reference (2) : http://www.colorado.edu/geography/weather_station/Geog_site/about.htm
 //
 float dewPoint(float celsius, float humidity)
 {
 // (1) Saturation Vapor Pressure = ESGG(T)
 double RATIO = 373.15 / (273.15 + celsius);
 double RHS = -7.90298 * (RATIO - 1);
 RHS += 5.02808 * log10(RATIO);
 RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
 RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
 RHS += log10(1013.246);
 
 // factor -3 is to adjust units - Vapor Pressure SVP * humidity
 double VP = pow(10, RHS - 3) * humidity;
 
 // (2) DEWPOINT = F(Vapor Pressure)
 double T = log(VP/0.61078);   // temp var
 return (241.88 * T) / (17.558 - T);
 }
 */

// delta max = 0.6544 wrt dewPoint()
// 6.9 x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double dewPointFast(double celsius, double humidity)
{
  const double a = 17.271;
  const double b = 237.7;
  double temp = (a * celsius) / (b + celsius) + log(humidity*0.01);
  double Td = (b * temp) / (a - temp);
  return Td;
}

int medianFilter (int xValues[], int value) {
  if (xValues[3] < 0 ) { // inicializacion
    xValues[0] = xValues[1] = xValues[2] = xValues[3] = value;
  }
  xValues[0] = xValues[1];
  xValues[1] = xValues[2];
  xValues[2] = xValues[3];
  xValues[3] = value;

  int result=0;
  for (int i=0;i<4;i++)
    result += xValues[i];
  result /= 4;

  return (result);
}

// redondeo al decimal. MODELAR !!!!!
float TempNTC(int RawADC) { // HOT
  const static double b = 790.06;
  const static double m = 5.06;
  return (int)( ((RawADC - b) / m ) * 10 ) / 10.0; // celsius
  // 826 -> 7.1
  // 908 -> 23.3
}

void loadEEStatus( )
{
  int i;
  byte *v;
  v = (byte *) &mieeprom;
  for ( i=0; i < sizeof(mieeprom); i++)
    *v++ = (byte) EEPROM.read(i);

  step                 = mieeprom.step;
  motorPos = targetPos = mieeprom.position;
  backslash_dir        = mieeprom.backslash_dir;
  backslash            = mieeprom.backslash;
  limit                = mieeprom.limit;
  sw1                  = mieeprom.sw1;
  sw2                  = mieeprom.sw2;
  sw3                  = mieeprom.sw3;
  sw4                  = mieeprom.sw4;
  microSteps           = mieeprom.microSteps;
  timer1Period         = mieeprom.timerPeriod;
  heaterA              = mieeprom.hpw1;
  heaterB              = mieeprom.hpw2;
  heaterC              = mieeprom.hpw3;
}

void writeEE()
{
  byte *v;
  v = (byte *) &mieeprom;
  for (int i=0; i < sizeof(mieeprom);i++,v++) {
    if( EEPROM.read(i) != *v ) // to save eeprom life
      EEPROM.write(i, *v);
    //EEPROM.update(i,*v);
  }
}

void saveEEStatus()
{
  mieeprom.step          = step;
  mieeprom.position      = motorPos;
  mieeprom.backslash_dir = backslash_dir;
  mieeprom.backslash     = backslash;
  mieeprom.limit         = limit;
  mieeprom.sw1           = sw1;
  mieeprom.sw2           = sw2;
  mieeprom.sw3           = sw3;
  mieeprom.sw4           = sw4;
  mieeprom.microSteps    = microSteps;
  mieeprom.timerPeriod   = timer1Period;
  mieeprom.hpw1          = heaterA;
  mieeprom.hpw2          = heaterB;
  mieeprom.hpw3          = heaterC;
  writeEE();
}

void resetFocuser()
{
  memcpy(&mieeprom,&reset,sizeof(mieeprom));
  writeEE();
  loadEEStatus();
  cancelAndStopFocuser();
  Timer1.setPeriod(timer1Period);
  for( int i = 0; i<10; i++ )
  {
    digitalWrite(PIN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(20);                     // wait for a 1/20 second
    digitalWrite(PIN_LED, LOW);    // turn the LED off by making the voltage LOW
    delay(20);                     // wait for a 1/20 second
  }
  //asm volatile ("  jmp 0"); // software reset. Don't reset devices
}


void motorRelax()
{
  digitalWrite(PIN_BOBINA_1A, LOW);
  digitalWrite(PIN_BOBINA_1B, LOW);  
  digitalWrite(PIN_BOBINA_2A, LOW);
  digitalWrite(PIN_BOBINA_2B, LOW);
}

// pasos completos
void doMotorStep4(int step)
{
  switch(step)
  {
  case 0:         //paso 0
    digitalWrite(PIN_BOBINA_1A, HIGH);  
    digitalWrite(PIN_BOBINA_1B, LOW);
    digitalWrite(PIN_BOBINA_2A, HIGH);
    digitalWrite(PIN_BOBINA_2B, LOW);
    break;
  case 1:         //paso 1
    digitalWrite(PIN_BOBINA_1A, HIGH);  
    digitalWrite(PIN_BOBINA_1B, LOW);
    digitalWrite(PIN_BOBINA_2A, LOW);
    digitalWrite(PIN_BOBINA_2B, HIGH);
    break;
  case 2:         //paso 2
    digitalWrite(PIN_BOBINA_1A, LOW);  
    digitalWrite(PIN_BOBINA_1B, HIGH);
    digitalWrite(PIN_BOBINA_2A, LOW);
    digitalWrite(PIN_BOBINA_2B, HIGH);
    break;
  case 3:         //paso 3
    digitalWrite(PIN_BOBINA_1A, LOW);  
    digitalWrite(PIN_BOBINA_1B, HIGH);
    digitalWrite(PIN_BOBINA_2A, HIGH);
    digitalWrite(PIN_BOBINA_2B, LOW);
    break;
  }
}

// medios pasos
void doMotorStep8(int step)
{
  switch(step)
  {
  case 0:         //paso 0
    digitalWrite(PIN_BOBINA_1A, HIGH);
    digitalWrite(PIN_BOBINA_1B, LOW);
    digitalWrite(PIN_BOBINA_2A, HIGH);
    digitalWrite(PIN_BOBINA_2B, LOW);
    break;
  case 1:         //paso 1
    digitalWrite(PIN_BOBINA_1A, HIGH);  
    digitalWrite(PIN_BOBINA_1B, LOW);
    digitalWrite(PIN_BOBINA_2A, LOW);
    digitalWrite(PIN_BOBINA_2B, LOW);
    break;
  case 2:         //paso 2
    digitalWrite(PIN_BOBINA_1A, HIGH);  
    digitalWrite(PIN_BOBINA_1B, LOW);
    digitalWrite(PIN_BOBINA_2A, LOW);
    digitalWrite(PIN_BOBINA_2B, HIGH);
    break;
  case 3:         //paso 3
    digitalWrite(PIN_BOBINA_1A, LOW);  
    digitalWrite(PIN_BOBINA_1B, LOW);
    digitalWrite(PIN_BOBINA_2A, LOW);
    digitalWrite(PIN_BOBINA_2B, HIGH);
    break;
  case 4:         //paso 4
    digitalWrite(PIN_BOBINA_1A, LOW);  
    digitalWrite(PIN_BOBINA_1B, HIGH);
    digitalWrite(PIN_BOBINA_2A, LOW);
    digitalWrite(PIN_BOBINA_2B, HIGH);
    break;
  case 5:         //paso 5
    digitalWrite(PIN_BOBINA_1A, LOW);  
    digitalWrite(PIN_BOBINA_1B, HIGH);
    digitalWrite(PIN_BOBINA_2A, LOW);
    digitalWrite(PIN_BOBINA_2B, LOW);
    break;
  case 6:         //paso 6
    digitalWrite(PIN_BOBINA_1A, LOW);  
    digitalWrite(PIN_BOBINA_1B, HIGH);
    digitalWrite(PIN_BOBINA_2A, HIGH);
    digitalWrite(PIN_BOBINA_2B, LOW);
    break;
  case 7:         //paso 7
    digitalWrite(PIN_BOBINA_1A, LOW);  
    digitalWrite(PIN_BOBINA_1B, LOW);
    digitalWrite(PIN_BOBINA_2A, HIGH);
    digitalWrite(PIN_BOBINA_2B, LOW);
    break;
  }
}

// devuelve el valor de la potencia de pulso para la posicion del micropaso absoluto pos
int getpwmbyStep(int pos) {

  pos = pos % (microSteps<<2); // microSteps => 1 cuadrante => x4
  if (pos == 0)
    pos = microSteps<<2;

  unsigned int index = ((pos - 1) * MAXMICROSTEPS) / microSteps;
  return ( index < (MAXMICROSTEPS<<1) ? pwmStepsTable[index] : 0 );  // solo tengo medio cuadrante en el array, el resto es cero
}

void doMotorStep(int step)
{
  if (microSteps>2) {
    analogWrite(PIN_BOBINA_1A, getpwmbyStep(step));
    analogWrite(PIN_BOBINA_1B, getpwmbyStep(step + (microSteps<<1) )); // microSteps * 2 ; microSteps => 1 cuadrante
    analogWrite(PIN_BOBINA_2A, getpwmbyStep(step +  microSteps     )); // microSteps
    analogWrite(PIN_BOBINA_2B, getpwmbyStep(step +  microSteps * 3 )); // microSteps * 3

#ifdef _FOCUSER_MOTOR_DEBUG
      sprintf( tmp_str, "(%03u:%03u:%03u:%03u)",
    getpwmbyStep(step),getpwmbyStep(step+(microSteps<<1)), getpwmbyStep(step+microSteps), getpwmbyStep(step+microSteps*3));
    Serial.print(tmp_str);
#endif
  } 
  else if (microSteps==2) {
    doMotorStep8(step);
  }
  else if (microSteps==1) {
    doMotorStep4(step);
  }
}

int calcMotorStep(int direction) {
  int rc=0;

  if ( direction ) {
    if ( step < (microSteps<<2) - 1 )
      step++;
    else
      step = 0;
    if ( motorPos < limit )
      motorPos++;
    else
      return(0);
  }
  else {
    if ( step > 0 )
      step--;
    else
      step = (microSteps<<2) - 1;
    if ( motorPos > 1 )
      motorPos--;
    else
      return(0);
  }
  return(1);
}

// main motor control loop
void interrup_t1() {
  int diff = 0;

  if (!interrupting_t1) {
    interrupting_t1 = true;

    if ( delay_tick == 0 || priomotion ) {  // delay tick es cero o movimiento prioritario
      delay_tick =  SPEED_MAX/motion_speed - 1;  // velocidad 8x => no espera (11111111) ; velocidad 2x => espera 3 (10001000)
      diff = targetPos - motorPos; // calcula ticks (pasos) pendientes
      if ( diff ) {  // si hay pasos pendientes, mueve el motor
        if ( calcMotorStep( diff > 0 ) == 1 ) { // calcula siguiente paso de motor
          doMotorStep(step); // mueve al siguiente paso de motor
          if (( diff > 0 ) && ( cmdInputMode == robofocus_mode))
            Serial.print('O');
          else
            Serial.print('I');
        }
#ifdef _FOCUSER_MOTOR_DEBUG
        sprintf( tmp_str, "\t%3d,%05u,%05u/%03u\t%05u\t%u,%d,%01u\t%2u\t%ld\r\n",
        diff,targetPos,motorPos,step, limit, backslash,backslash_dir,compensa_backslash, motion_speed, t1_ticks);
        Serial.print(tmp_str);
        // stty -F /dev/ttyACM5 raw ; echo "FI00100#" > /dev/ttyACM5 &  ts "%.s" < /dev/ttyACM5
#endif
        diff = targetPos - motorPos;  // vuelve a calcular pasos pendientes

          relax_ticks = MAXRELAX;  // se ha movido, entonces no relajes por MAXRELAX
        if ( ! diff ) {
          if( compensa_backslash ) { // resulta en un movimiento adicional, una vez completado el principal
            compensa_backslash = 0;
            switch( backslash_dir)
            {
            case '3':
              targetPos = motorPos + backslash;
              break;
            case '2':
              targetPos = motorPos - backslash;
              break;
            default:
              //targetPos = motorPos;
              break;
            }
          }
          else if (cmdInputMode == robofocus_mode) {
            sprintf(response_str,"FD0%05u", motorPos);  // devuelve posicion una vez completado movimiento
            robofocusResponse(response_str);
            saveEEStatus(); // When the movement is completed, the new position is update to eeprom (bad)
          }
        }
      }
    }
    else { // hay que esperar ranura
      if ( delay_tick > 0 ) {
        delay_tick--;
      }
    }
    if ( relax_ticks > 0 ) { // si estoy esperando relajacion, comienza la cuenta atras ...
      relax_ticks--;
      if ( relax_ticks == 0 ) {
#ifdef _FOCUSER_MOTOR_DEBUG
        Serial.print("Relaxing motor\n"); 
#endif
        motorRelax();
      }
    }
    else
      relax_ticks = MAXRELAX; // espera un tiempo (t1_ticks) hasta relajacion de motor

    interrupting_t1 = false; // fin interrupcion
  }

  if( millis()/(timer1Period<<1) % 2) { // cada segundo keepalive (
    digitalWrite(PIN_LED, HIGH);   // turn the LED on
  }
  else
    digitalWrite(PIN_LED, LOW);    // turn the LED off

  // calibracion
  // stty -F /dev/ttyUSB0 raw ;  ts "%.s" < /dev/ttyUSB0  | \
  // awk 'BEGIN {t=0} ; {if (t==0) {s=$1;t=$3} else {print "s:"$1" t:"$3" T:"($1-s)/($3-t)*1000}}'

#ifdef _FOCUSER_MOTOR_LOOP_DEBUG
  //if( t1_ticks % 1000 == 0) { // cada 1000 ticks (1/2 sec, si timer1Period=500)
  if( t1_ticks % (timer1Period<<1) == 0) { // cada 1/2 sec )
    Serial.print(millis());
    Serial.print(" ");
    Serial.print(t1_ticks);
    Serial.print("\r\n");
  }
#endif

  t1_ticks++; // every interrupt
}


//--------------------------------------------------------------------------------------------------------
//-- MANAGE COMMANDS


unsigned char roboFocusCheckSum(char *rbfcmd)
{
  char substr[255] ; 
  unsigned char val = 0 ;

  for(int i=0; i<8; i++)
    substr[i]=rbfcmd[i] ;

  val = roboFocusCalculateSum( substr) ;

  return val ;
}

unsigned char roboFocusCalculateSum(char *rbfcmd)
{
  unsigned char val = 0;

  for(int i=0; i<8; i++)
    val = val + (unsigned char) rbfcmd[i];

  return val % 256 ;
}


void robofocusResponse(char *rbfcmd)
{
  for(int i=0; i<8 ; i++)
    Serial.write((byte)rbfcmd[i]);

  Serial.write((byte)(roboFocusCalculateSum(rbfcmd)));
}

void respondeRobococusVersion( char *version )
{
  sprintf(response_str,"FV%s", version);
  robofocusResponse(response_str);
}

void cancelAndStopFocuser()
{
  motorPos = targetPos;
}

void moveIn(int steps)
{
  if (!steps)
    return;

  if (steps && ( motorPos - steps - ( backslash_dir == '3'? backslash: 0)) >= 1 ) {
    switch( backslash_dir) {
    case '3':
      targetPos = motorPos - steps - backslash;
      compensa_backslash = 1;
      break;
    case '2':
      targetPos = motorPos - steps;
      compensa_backslash = 0;
      break;
    default:
      targetPos = motorPos - steps;
      break;
    }
  }
}

void moveOut(int steps)
{
  if (steps && ( motorPos + steps + ( backslash_dir == '2'?  backslash: 0)) <= limit)  {
    switch( backslash_dir) {
    case '3':
      targetPos = motorPos + steps;
      compensa_backslash = 0;
      break;
    case '2':
      targetPos = motorPos + steps + backslash;
      compensa_backslash = 1;
      break;
    default:
      targetPos = motorPos + steps;
      break;
    }
  }
}

void moveRel(int steps)
{
  if (steps>0)
    moveIn(steps);
  else
    moveOut(-steps);
}

void moveAbs(int absPosition)
{
  if ( (absPosition == 0) || (absPosition == motorPos) ) {
    if (cmdInputMode == robofocus_mode) {   
      sprintf(response_str,"FD0%05u", motorPos);
      robofocusResponse(response_str);
    }
  }
  else {
    if ( absPosition < motorPos )
      moveIn(motorPos - absPosition);
    else
      moveOut(absPosition - motorPos);
  }
}

void setPosition(int absPosition)
{
  if (( absPosition > 0 ) && (absPosition <= limit))
    motorPos = targetPos = absPosition;

  if (cmdInputMode == robofocus_mode) {   
    sprintf(response_str,"FS0%05u", motorPos);
    robofocusResponse(response_str);
  }
}

void setLimit(int maxLmit)
{
  if ((maxLmit >= motorPos) && (maxLmit <= 65000))
    limit = maxLmit;

  if (cmdInputMode == robofocus_mode) {   
    sprintf(response_str,"FL0%05u", limit);
    robofocusResponse(response_str);
  }
}

void robofocusSwitches(unsigned char * arg)
{
  if ( arg[0] == '0' && arg[1] == '0' &&  ( arg[2] != '0' || arg[3] != '0' || arg[4] != '0' || arg[5] != '0' )) {
    if ( arg[2] == '2' )
      sw1=2;
    else
      sw1=1;

    if ( arg[3] == '2' )
      sw2=2;
    else
      sw2=1;

    if ( arg[4] == '2' )
      sw3=2;
    else
      sw3=1;

    if ( arg[5] == '2' )
      sw4=2;
    else
      sw4=1;
  }
  sprintf(response_str,"FP%06u", sw1*1000+sw2*100+sw3*10+sw4);
  robofocusResponse(response_str);
}

void heatersSwitches(unsigned char * arg)
{
  if ( arg[0] == '0' && arg[1] == '0' && ( arg[2] != '0' || arg[3] != '0' || arg[4] != '0' )) {

    if ( arg[2] == '2' )
      heaterA = abs(heaterA);
    else
      heaterA = -abs(heaterA);

    if ( arg[3] == '2' )
      heaterB = abs(heaterB);
    else
      heaterB = -abs(heaterB);

    if ( arg[4] == '2' )
      heaterC = abs(heaterC);
    else
      heaterC = -abs(heaterC);
  }

  boolean swA = ( heaterA > 0 ? true : false );
  boolean swB = ( heaterB > 0 ? true : false );
  boolean swC = ( heaterC > 0 ? true : false );

  sprintf(response_str,"FHP%05u", swA*100+swB*10+swC);
  robofocusResponse(response_str);
}

void printDebugging() {
  sprintf( tmp_str, "VERSION: v%s\r\n", VERSION);
  Serial.print(tmp_str);

  sprintf( tmp_str, "targetPos,motorPos,limit \tbslash,backslash_dir \tmicroSteps,timer1Period\r\n");
  Serial.print(tmp_str);
  sprintf( tmp_str, "%05u,%05u,%05u \t%u,%c \t%d,%d\r\n",
  targetPos, motorPos, limit, backslash, backslash_dir, microSteps ,timer1Period);
  Serial.print(tmp_str);

  sprintf( tmp_str, "TEMP,HUM,DEWP \tTEMPA,TEMPB,TEMPC \tTEMPVA,TEMPVB,TEMPVC \tPWMA,PWMB,PWMC\r\n");
  Serial.print(tmp_str);
  sprintf( tmp_str, "%d,\t%d,\t%d  \t%d,\t%d,\t%d  \t%d,\t%d,\t%d \t%d,\t%d,\t%d\r\n",
  (int)(temperature*10), (int)(humidity*10), (int)(dewPointValue*10),
  (int)(lastSensorATemp*10), (int)(lastSensorBTemp*10), (int)(lastSensorCTemp*10),
  lastSensorATemp_Value, lastSensorBTemp_Value, lastSensorCTemp_Value,
  heaterA, heaterB , heaterC);
  Serial.print(tmp_str);
}

boolean processRobofocusCommand (char * command) {
  // Robofocus cmds
  // F{C}{N}{VVVVV}#
  // C orden (1 byte)
  // N valor1/dir/0 (1 byte)
  // V valor2/posicion (5 bytes)    1 .. 65536

  int d,v = 0;
  unsigned char c[6];

  switch( command[1] ) {
  case 'V':
    if ( command[1] == 'J' )
      respondeRobococusVersion(VERSION);
    else
      respondeRobococusVersion(ROBOFOCUS_VERSION);
    break;

  case 'G':          // position absolute
    if (sscanf(command,"FG%6u",&v) == 1 )
      moveAbs(v);
    break;

  case 'I':          // posicion IN relative
    if (sscanf(command,"FI0%5u",&v) == 1 )
      moveIn(v);
    /*
#ifdef _FOCUSER_DEBUG
     Serial.print("processRobofocusCommand:");
     Serial.print(command);
     Serial.print(":");
     Serial.print(v);
     Serial.print ( ":\r\n" );
     #endif
     */
    break;

  case 'O':          // posicion OUT relative
    if (sscanf(command,"FO0%5u",&v) == 1 )
      moveOut(v);
    break;

  case 'S':          // set position without motion
    if (sscanf(command,"FS0%5u",&v) == 1 ) {
      setPosition(v);
      saveEEStatus(); // write eeprom
    }
    break;

  case 'L':          // set limits FL000000#
    if (sscanf(command,"FL0%5u",&v) == 1 ) {
      setLimit(v);
      saveEEStatus(); // write eeprom
    }
    break;

  case 'B':          // req backlash info FB000000#
    if (sscanf(command,"FB%1u00%3u",&d,&v) == 2 ) { // FC|(byte)dir,bs|
      if ( d != 0) {
        backslash_dir = d;
        backslash = v;
        saveEEStatus(); // write eeprom
      } 
      sprintf(response_str,"FB%c%05u", backslash_dir+'0', backslash);
      robofocusResponse(response_str);
    }
    break;

  case 'R':          // reset
    if (sscanf(command,"FR%6d",&v) == 1 )
      if ( v == 0 ) {
        resetFocuser();
        sprintf(response_str,"FR001111");
        robofocusResponse(response_str);
      } 
    break;

  case 'C':          // configuracion
    sprintf(response_str,"FC%c%c%c000",0xff,0,1); // FC|(byte)dutty,(byte)delay,(byte)ticks|000
    robofocusResponse(response_str);
    break;

  case 'P':          // powermodule FP000000#
    if (sscanf(command,"FP%6c",c) == 1 )
      robofocusSwitches(c);
    break;

  case 'T':          // req temperatura FT000000#
    switch( command[2] ) {
    case 'H':        // no robofocus
      if (isnan(humidity)) {
        sprintf(response_str,"FTH00000"); // FTH00000#
        robofocusResponse(response_str);
      }
      else {
        sprintf(response_str,"FTH%5u",(unsigned int)(humidity));
        robofocusResponse(response_str);
      }
      break;
    default:
      if (isnan(temperature)) {
        sprintf(response_str,"FT000000");
        robofocusResponse(response_str);
      }
      else {
        sprintf(response_str,"FT%6u",(unsigned int)((temperature + 273.15) * 2)); // Kelvin*2
        robofocusResponse(response_str);
      }
      break;
    }
    break;

  case 'M':          // no robofocus // FM000000# FM500500#
    // timer1Period=500; // 1000us step size (a 250us pierde par)
    // microSteps=16;    // 1 2 4 8 16 32 micropasos/paso => 4 8 16 32 64 128 micropasos / ciclo ; 1 ciclo = 4 pasos enteros
    char steps;
    if (sscanf(command,"FM%1u%5u",&steps,&v) == 2 ) {
      if ( (steps < 6) && (steps > 0)) {// max 32
        microSteps = 1<<steps;
        timer1Period = v;
        Timer1.setPeriod(timer1Period);
      }
      steps=0;
      byte microSteps2;
      microSteps2 = microSteps;
      while ( (microSteps2=microSteps2>>1) > 0)
        steps++;

      sprintf(response_str,"FM%1u%05u", steps, timer1Period); 
      robofocusResponse(response_str);
      saveEEStatus(); // write eeprom

      // FM500500#
      // steps: 5
      // microSteps: 32
      // timer1Period: 500
      /*
      Serial.print ( "steps: " );
       Serial.print ( (int)steps );
       Serial.print ( "\r\n" );
       Serial.print ( "microSteps: " );
       Serial.print ( microSteps );
       Serial.print ( "\r\n" );
       Serial.print ( "timer1Period: " );
       Serial.print ( timer1Period );
       Serial.print ( "\r\n" );
       */
    }
    break;

  case 'H':       // no robofocus FHP0000#
    switch( command[2] ) {
    case 'P': // FHP00000#   // like power module. Switching
      if (sscanf(command,"FHP%5c",c) == 1 )
        heatersSwitches(c);
      break;
    case 'W': // HW{N}000#  // level
      switch( command[3] ) {
      case 'A':
        if (sscanf(command,"FHWA0%3u",&v) == 1 ) {  // FHWA1000#
          v = ( v < 64 ? v : 64 );
          heaterA = ( heaterA < 0 ? -v : v);
        }
        sprintf(response_str,"FHWA0%03u", heaterA);
        robofocusResponse(response_str);
        break;
      case 'B':
        if (sscanf(command,"FHWB0%3u",&v) == 1 ) {  // FHWB1000# FHWB0100#
          v = ( v < 64 ? v : 64 );
          heaterB = ( heaterB < 0 ? -v : v);
        }
        sprintf(response_str,"FHWB0%03u", heaterB);
        robofocusResponse(response_str);
        break;
      case 'C':
        if (sscanf(command,"FHWC0%3u",&v) == 1 ) {  // FHWC0000#
          v = ( v < 64 ? v : 64 );
          heaterC = ( heaterC < 0 ? -v : v);
        }
        sprintf(response_str,"FHWC0%03u", heaterC);
        robofocusResponse(response_str);
        break;
      }
      break;
    case 'T': // HT{N}000#  // temperature
      switch( command[3] ) {
      case 'A':
        sprintf(response_str,"FHT%05u", (int)(lastSensorATemp*10)); // FHTA0000#
        robofocusResponse(response_str);
        break;
      case 'B':
        sprintf(response_str,"FHT%05u", (int)(lastSensorBTemp*10)); // FHTB0000#
        robofocusResponse(response_str);
        break;
      case 'C':
        sprintf(response_str,"FHT%05u", (int)(lastSensorCTemp*10)); // FHTC0000#
        robofocusResponse(response_str);
        break;
      }
      break;
    }
    break;

  case 'D': 
    printDebugging();
    break;

  default:
    sprintf( tmp_str, "F_Error\r\n");
    Serial.print(tmp_str);
    break;
  }
  return 0;

}

// prototipe commands
boolean processJosluCommand (String command) {
  String param = command.substring(2);
  String param1;
  String param2;
  int isep = command.indexOf(':');
  if ( isep > -1 ) {
    isep--;
    param1 = param.substring(0,isep-1); 
    param2 = param.substring(isep);
  }
  // JB300:4600

  switch( command.charAt(1) ) {
  case 'O': // move outward
    moveIn(param.toInt());
    break;
  case 'I': // move inward
    moveOut(param.toInt());
    break;
  case 'P': // absolute pos
    moveAbs(param.toInt());
    break;
  case 'S': // absolute pos wo motion
    if (param.length() > 0) {
      setPosition(param.toInt());
      sprintf( tmp_str, "%d\r\n",motorPos);
      Serial.print(tmp_str);
    }
    break;
  case 'B': // backlash
    if (param2.length() > 0) {
      backslash_dir = param1.toInt();
      backslash     = param2.toInt();
    }
    sprintf( tmp_str, "%d:%d\r\n",backslash_dir ,backslash);
    Serial.print(tmp_str);
    break;
  case 'T': // temperature/humidity
    sprintf( tmp_str, "%d:%d\r\n",temperature*10 ,humidity);
    Serial.print(tmp_str);
    break;

    break;
  case 'M': // microsteps/period
    if (param2.length() > 0) {
      microSteps = param1.toInt();
      timer1Period = param2.toInt();
      Timer1.setPeriod(timer1Period);
    }
    sprintf( tmp_str, "%d:%d\r\n",microSteps ,timer1Period);
    Serial.print(tmp_str);
    break;
  case 'H': // hotters
    if (param2.length() > 0) {
    } 
    sprintf( tmp_str, "%d:%d:%d:%d:%d:%d\r\n",
    heaterA, heaterB , heaterC, (int)(lastSensorATemp*10), (int)(lastSensorBTemp*10), (int)(lastSensorCTemp*10));
    Serial.print(tmp_str);
    break;
  case 'X': // halt
    cancelAndStopFocuser();
    break;

  case 'V':
    sprintf( tmp_str, "VERSION: v%s\r\n", VERSION);
    Serial.print(tmp_str);
    break;

  case 'D': 
    printDebugging();
    break;

  }
  return 0;
}

// FV000000# | FV000000{check}
cmdInputModeType processSerialCommand(String command) {

  cmdInputModeType result = unknownserialinput_mode;
  char rbfCommand[10] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00                                                                                   };

  switch(command.charAt(0)) {
  case 'F':
    if (command.length() == 9) {
      command.toCharArray(rbfCommand,10);

#ifdef _FOCUSER_DEBUG
      Serial.print("processSerialCommand:");
      Serial.print(command.substring(0,8));
      Serial.print ( ":\r\n" );
#endif

      if (roboFocusCheckSum(rbfCommand) || (rbfCommand[8] == '#'))
        if ( ! processRobofocusCommand(rbfCommand) )
          result = robofocus_mode;
    }
    break;
  case 'J':
    if ( ! processJosluCommand(command) ) 
      result = joslufocus_mode;
    break;
  }
  return(result); 
}


//--------------------------------------------------------------------------------------------------------

void setup() {

  // initialize the button pin as a input:
  //pinMode(PIN_B0, INPUT);
  //digitalWrite(PIN_B0, HIGH);       // turn on pullup resistors
  pinMode(PIN_B1, INPUT);
  digitalWrite(PIN_B1, HIGH);       // turn on pullup resistors
  pinMode(PIN_B2, INPUT);
  digitalWrite(PIN_B2, HIGH);       // turn on pullup resistors
  //pinMode(PIN_B3, INPUT);
  //digitalWrite(PIN_B3, HIGH);       // turn on pullup resistors

  // initialize the LED as an output:
  pinMode(PIN_LED, OUTPUT);  
  digitalWrite(PIN_LED, HIGH);   // turn the LED on

  // initialize the motor phases:
  pinMode(PIN_BOBINA_1A, OUTPUT);
  pinMode(PIN_BOBINA_1B, OUTPUT);
  pinMode(PIN_BOBINA_2A, OUTPUT);
  pinMode(PIN_BOBINA_2B, OUTPUT);

  //TCCR0B = TCCR0B & B11111000 | B001; // timer0 prescaler = 1 f: 31372.55 Hz 8bit phase-correct pwm / 62500 Hz fast pwwm
  //TCCR3B = TCCR3B & B11111000 | B001; // timer3 prescaler = 1 f: 31372.55 Hz 8bit phase-correct hardware pwm
  //TCCR4B = TCCR4B & B11111000 | B001; // timer4 prescaler = 1 f: 31372.55 Hz 8bit phase-correct hardware pwm
  //TCCR0B = TCCR0B & B11111000 | B011; // prescaler = 64 (default) f: 976.5625 Hz fast pwm

  TCCR0A = TCCR0A & B11111000 | B001; // timer0 setup 8bit phase-correct hardware pwm
  TCCR0B = TCCR0B & B11111000 | TIMERPRESCALER0_VALUE;
  TCCR3B = TCCR3B & B11111000 | TIMERPRESCALER0_VALUE;
  TCCR4B = TCCR4B & B11111000 | TIMERPRESCALER4_VALUE;

  pinMode(HEATER_A_PIN, OUTPUT);
  pinMode(HEATER_B_PIN, OUTPUT);
  pinMode(HEATER_C_PIN, OUTPUT);

  // initialize serial communication:

  serialInputString.reserve(200);

  Serial.begin(9600);
  Serial.setTimeout(2000);

  digitalWrite(PIN_LED, HIGH);   // turn the LED off
  delay(1000);  
  digitalWrite(PIN_LED, LOW);   // turn the LED off

  // filters setup
  for (int i=0;i<4;i++) {
    xSensorATempValues[i] = -1;
    xSensorBTempValues[i] = -1;
    xSensorCTempValues[i] = -1;
  }

  loadEEStatus();
  if ( mieeprom.reset !=1 )
    resetFocuser();

  dht.begin();

  doMotorStep(step);
  Timer1.initialize(timer1Period); // This breaks analogWrite() for digital pins 9 and 10 on Arduino !!!
  Timer1.attachInterrupt(interrup_t1);

  respondeRobococusVersion(ROBOFOCUS_VERSION);
  digitalWrite(PIN_LED, HIGH);   // turn the LED on
}


// Serial event loop
void serialEvent() {

  while (Serial.available() > 0) {
    char inChar = (char)Serial.read(); 

    /*
#ifdef _FOCUSER_DEBUG
     Serial.print("Serial Event count:");
     Serial.print(serialInputCount);
     Serial.print(" Char:");
     Serial.print(inChar);
     Serial.print ( "\r\n" );
     #endif
     */

    if ( !serialInputCount && ( inChar=='\r') && (cmdInputMode == robofocus_mode )) {
      cancelAndStopFocuser();
    }
    else  {
      serialInputString += inChar;
      serialInputCount++;

      if (inChar == '\r' || inChar == 0x00 || ((cmdInputMode == robofocus_mode) && (serialInputCount > 8))
        || (serialInputCount > MAXLEN_SERIAL_INPUT_STRING) )
      {
        cmdInputMode = processSerialCommand(serialInputString);
        serialInputString = "";
        serialInputCount = 0;
        if (!cmdInputMode)
          cmdInputMode = robofocus_mode;

      } 
      else if ((inChar == 'J') && (cmdInputMode == robofocus_mode)) { // sync robofocus mode
        serialInputString = "J";
        serialInputCount = 1;
      }
    }
  }
}

// Software PWM loop (low frequency)
void pwmHeaterLoop() {
  // heaterX < 0 => always LOW
  digitalWrite(HEATER_A_PIN, heaterA > pwmHeatersCounter ? HIGH : LOW); 
  digitalWrite(HEATER_B_PIN, heaterB > pwmHeatersCounter ? HIGH : LOW);
  digitalWrite(HEATER_C_PIN, heaterC > pwmHeatersCounter ? HIGH : LOW);
  pwmHeatersCounter < 64 ? pwmHeatersCounter++ : pwmHeatersCounter=0;
}

// main loop
void loop() {
  unsigned long currentTime = millis();

  // DO every 5 seconds (cada 2 x 2500ms)
  if ( currentTime / 2500 % 2)  {
    if (loop_halfsecond_ready) {
#ifdef _FOCUSER_LOOP_DEBUG
      Serial.print ( "Time: " );
      Serial.print ( currentTime );
      Serial.print ( "\r\n" );
#endif
      float t = dht.readTemperature();
      float h =  dht.readHumidity();
      if (!isnan(h) && !isnan(t)) {
        temperature = t;
        humidity =  h;
        dewPointValue=dewPointFast(temperature,humidity);
      }

      int v = analogRead(SENSORTEMP_A_PIN);
      if (v>15) {
        lastSensorATemp_Value = medianFilter(xSensorATempValues,v);
        lastSensorATemp = TempNTC(lastSensorATemp_Value);
      } 
      else
        lastSensorATemp_Value = NAN;

      v = analogRead(SENSORTEMP_B_PIN);
      if (v>15) {
        lastSensorBTemp_Value = medianFilter(xSensorBTempValues,v);
        lastSensorBTemp = TempNTC(lastSensorBTemp_Value);
      } 
      else
        lastSensorBTemp_Value = NAN;

      v = analogRead(SENSORTEMP_C_PIN);
      if (v>15) {
        lastSensorCTemp_Value = medianFilter(xSensorCTempValues,v);
        lastSensorCTemp = TempNTC(lastSensorCTemp_Value);
      }
      else
        lastSensorCTemp_Value = NAN;
      /*
      if (lastSensorATemp >= dewPointValue) {
       heaterA = 25;
       }
       else
       heaterA = 0;
       if (temperature > dewPointValue) {
       heaterB = 50;
       heaterC = 64;
       } 
       else {
       heaterB = 0;
       heaterC = 0;
       }
       
       //heaterA = 25;
       //heaterB = 50;
       //heaterC = 64;
       */

      // Cable disconnected
      if (lastSensorATemp_Value == NAN )
        heaterA = -abs(heaterA);
      if (lastSensorBTemp_Value == NAN )
        heaterB = -abs(heaterB);
      if (lastSensorCTemp_Value == NAN )
        heaterC = -abs(heaterC);

      //Serial.println(currentTime);
      loop_halfsecond_ready = false;
    }
  } 
  else
    loop_halfsecond_ready = true;

  // Other periodic tasks
  serialEvent();    // serial control loop. Arduino Micro, Esplora or Leonardo
  pwmHeaterLoop();  // PWM heaters control loop

    delay(10);
}

































































