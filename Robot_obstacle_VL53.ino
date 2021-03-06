/*
Standing on the shoulders of giants:
 
Stepper Motor Control - one revolution
 This is the examply by Tom Igoe delivered with th Arduino IDE.

 Adapted by Stefan Thesen for 28BYJ-48 stepper motor
 https://blog.thesen.eu

again adapted by wtlx for flexible movement of the SMARS Robot
 */

#include <Stepper.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
#define LONG_RANGE

#define PIN1 23


/////#define ledverde 45//LED pin-D45
//////#define ledAzul 47//LED pin-D47
#define BUZZER_PIN  49  //pin-49 AX-1205-2 5V


// Get steps per turn and RPM out of datasheet:
// ============================================
// 5.625/64 deg per step --> 4096 steps for 360 deg
// step relates to half-step sequencing, which means 8 steps
// We use 4 full steps ==> 2048 steps per turn
//
// recommended frequency range 600-1000Hz for 8 half-steps
// 1000Hz/4096 half-steps --> approx 4sec per turn is max speed 
// ==> 15rpm
// Note: actually we will use ~500Hz using the 4 step approach
// but motor will not be able to turn faster.

// Get stepping sequence out of datasheet:
// =======================================
// Stepping sequence as given in datasheet
// this is an 8-step half-step approach
//         Steps
// Wire  1  2  3  4  5  6  7  8
// 1                    x  x  x
// 2              x  x  x 
// 3        x  x  x
// 4     x  x                 x
// 
// We use only even / full steps thus:
//         Steps
// Wire  2  4  6  8
//   1         x  x
//   2      x  x 
//   3   x  x 
//   4   x        x
// 
// Code of Arduino Stepper Lib has implemented:
//         Steps
// Wire  1  2  3  4
//   1         x  x
//   2   x  x      
//   3      x  x 
//   4   x        x
//
// ==> Simple Solution: exchange wire 2&3


const int stepsPerRevolution = 2048;  // here go the 2048 steps
// for your motor

// inicializar a biblioteca stepper nos pinos 8 a 11-> IN1, IN2, IN3, IN4
// como mostrado acima, precisamos trocar os fios 2 e 3, o que fazemos no construtor
Stepper leftStepper(stepsPerRevolution, 8, 10, 9, 11);
//IN1--D8
//IN2--D9
//IN3--D10
//IN4--D11

///Stepper rightStepper(stepsPerRevolution, 4, 6, 5, 7);
//IN1--D4
//IN2--D5
//IN3--D6
//IN4--D7

Stepper rightStepper(stepsPerRevolution, 3, 5, 4, 6);
//IN1--D3
//IN2--D4
//IN3--D5
//IN4--D6




int stepsToDo;  //vari??vel para ser flex??vel no loop (por exemplo, dependendo da dist??ncia aos obst??culos)


 
// definir fun????es para movimento b??sico
// usando 'for' para mover steppers (pseudo) simultaneamente  
    
    void goForward(int x){              
           for(int s=0; s<x; s++){
             leftStepper.step(1); // um positivo move o motor em um sentido rota??ao
             rightStepper.step(-1);// um negativo move o motor em um novo sentido rota??ao
             }       
    }
        
    
    void goBack(int x){              
           for(int s=0; s<x; s++){
             leftStepper.step(-1);
             rightStepper.step(1);
             }       
    }


//virar ?? esquerda e ?? direita no mesmo lugar - com ambos os motores
  void turnLeft(int x){              
           for(int s=0; s<x; s++){
             leftStepper.step(-1);
             rightStepper.step(-1);
             }       
    }
    
    void turnRight(int x){              
           for(int s=0; s<x; s++){
             leftStepper.step(1);
             rightStepper.step(1);
             }       
    } 




// HC-SR04 Sensor
///int trigger=12; //D12
///int echo=13; //D13
///float duration=0; 
float distance=0; 
float realDistance=0;



int whatDistance(){
  ///  digitalWrite(trigger, LOW); 
  ///  delayMicroseconds(2); 
  ///  digitalWrite(trigger, HIGH); 
 ///   delayMicroseconds(10); 
 ///   digitalWrite(trigger, LOW); 
///    duration = pulseIn(echo, HIGH); 
///    distance = (duration/2) * 0.03432;    
    distance = sensor1.readRangeContinuousMillimeters();
    return distance;
    }




void setup() 
{
  //---- defina a velocidade em 15 rpm:
 /// rightStepper.setSpeed(15);
 /// leftStepper.setSpeed(15);
  rightStepper.setSpeed(20);
  leftStepper.setSpeed(20);
  
  // set pinmodes for HC-04
////  pinMode(trigger, OUTPUT); 
//////  pinMode(echo, INPUT); 
    
Serial.begin(9600); // Starts the serial communication
pinMode(PIN1, OUTPUT);
digitalWrite(PIN1, LOW);
pinMode(BUZZER_PIN, OUTPUT); 
tone(BUZZER_PIN, 200, 200); delay(200); 
tone(BUZZER_PIN, 500, 400); delay(500);  
delay(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor1.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);//default
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);//default
  //sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 28);//default
  //sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 20);//default
#endif

pinMode(PIN1, INPUT);
delay(150);
Serial.println("00");
sensor1.init(true);

Serial.println("01");
delay(100);
sensor1.setAddress((uint8_t)45);
Serial.println("02");
Serial.println("addresses set");

// end configuration
sensor1.setTimeout(500);
sensor1.startContinuous();

// scan i2c
Serial.println ("I2C scanner. Scanning ...");
byte count = 0;

for (byte i = 1; i < 120; i++) {

Wire.beginTransmission (i);
if (Wire.endTransmission () == 0) {
Serial.print ("Found address: ");
Serial.print (i, DEC);
Serial.print (" (0x");
Serial.print (i, HEX);
Serial.println (")");
count++;
delay (1); // maybe unneeded?
} // end of good response
} // end of for loop
Serial.println ("Done.");
Serial.print ("Found ");
Serial.print (count, DEC);
Serial.println (" device(s).");
}

void loop() 
{
   int distance1 =sensor1.readRangeContinuousMillimeters();// get distance for sensor 1 
//-------------------------------------------------------------
Serial.print(sensor1.readRangeContinuousMillimeters());
if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT S1"); }
Serial.println();
Serial.print("\t");
//----------------------------------------------------------------------------------------
Serial.print("distance: ");
Serial.println(distance);

realDistance = whatDistance();
Serial.println(realDistance);    
            
if (realDistance<=0) {
    realDistance = whatDistance();
    }   
    
///while (realDistance>=5) {
while (realDistance>25) {
    stepsToDo = 2048;  // passos para uma revolu????o completa
    goForward(stepsToDo);
    realDistance = whatDistance();
    }    
    
    
    
//////stepsToDo = 512;    // no caso de haver apenas 5 cm ou menos para o obst??culo, vire rund 45 graus para a direita
///stepsToDo = 1512;
////stepsToDo = 1812;
stepsToDo = 1912;
turnRight(stepsToDo);  
    
    
realDistance = whatDistance(); //obter uma nova dist??ncia
 
/// while (realDistance<=15) {
 while (realDistance<=30) {
    stepsToDo = 512;  // passos para 1/4 de revolu????o
/// stepsToDo = 1512;  // passos para 1/4 de revolu????o
///stepsToDo = 2012;  // passos para  de revolu????o
    turnRight(stepsToDo);
    realDistance = whatDistance();
    } 
    
}
