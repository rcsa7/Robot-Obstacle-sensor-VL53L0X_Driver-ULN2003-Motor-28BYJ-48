/*
Standing on the shoulders of giants:
 
Stepper Motor Control - one revolution
 This is the examply by Tom Igoe delivered with th Arduino IDE.

 Adapted by Stefan Thesen for 28BYJ-48 stepper motor
 https://blog.thesen.eu

again adapted by wtlx for flexible movement of the SMARS Robot

 * PINOUT SHIELD BUZZER LED 
 * BUZZER---PIN49
 * LED1---48
 * LED2---46
 * LED3---44
 * LED4---42
 * LED5---40
 * LED6---38
 * PINOT----PIN23
 * PINOT----PIN25
 * PINOT----PIN27
 * PINOT----PIN29
 * PINOT----PIN33
 * PINOT----PIN37
 * PINOT----PIN41
 * PINOT----PIN45
 */
#include <SPI.h>
#include <Stepper.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

VL53L0X frontSensor;// SENSOR DA FRENTE--PIN1
VL53L0X driverSensor;// SENSOR LADO ESQUERDO--MOTORISTA--PIN2
VL53L0X passengerSensor;// SENSOR LADO DIREITO--PASSAGEIRO--PIN3

//--usando este pins para alterar o endereço I2C do sensor
#define PIN1 23 //-PIN--XSHUT ----USAr conversor de nivel lógico
#define PIN2 25 //-PIN--XSHUT ----USAr conversor de nivel lógico
#define PIN3 27 //-PIN--XSHUT ----USAr conversor de nivel lógico


#define ledverde 45//LED pin-D45
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




int stepsToDo;  //variável para ser flexível no loop (por exemplo, dependendo da distância aos obstáculos)


 
// definir funções para movimento básico
// usando 'for' para mover steppers (pseudo) simultaneamente  
    
    void goForward(int x){              
           for(int s=0; s<x; s++){
             leftStepper.step(1); // um positivo move o motor em um sentido rotaçao
             rightStepper.step(-1);// um negativo move o motor em um novo sentido rotaçao
             }       
    }
        
    
    void goBack(int x){              
           for(int s=0; s<x; s++){
             leftStepper.step(-1);
             rightStepper.step(1);
             }       
    }


//virar à esquerda e à direita no mesmo lugar - com ambos os motores
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






float DistanciaReal=0;
float range=0;

int qualDistancia()
{
   range = frontSensor.readRangeContinuousMillimeters(); //mm
  return range;
}


void setup() 
{
  //---- defina a velocidade em 15 rpm:
 /// rightStepper.setSpeed(15);
 /// leftStepper.setSpeed(15);
  rightStepper.setSpeed(20);
  leftStepper.setSpeed(20);
  
 
Wire.begin();     
Serial.begin(9600); // Starts the serial communication
  //-----sensores---vl53l0x-----------------------------
pinMode(PIN1, OUTPUT);
pinMode(PIN2, OUTPUT);
pinMode(PIN3, OUTPUT);

digitalWrite(PIN1, LOW);
digitalWrite(PIN2, LOW);
digitalWrite(PIN3, LOW);

pinMode(ledverde, OUTPUT);
digitalWrite(ledverde, LOW);
pinMode(BUZZER_PIN, OUTPUT); 
tone(BUZZER_PIN, 200, 200); delay(200); 
tone(BUZZER_PIN, 500, 400); delay(500);  
delay(500);

//--------------------------------------
pinMode(PIN1, INPUT);
///delay(150);
///Serial.println("00");
frontSensor.init(true);

///Serial.println("01");
///delay(100);
frontSensor.setAddress((uint8_t)24); //0X16 novo endereço frontSensor
//---------------------------------------------
///Serial.println("02");

pinMode(PIN2, INPUT);
///delay(150);
driverSensor.init(true);
///Serial.println("03");
///delay(100);
driverSensor.setAddress((uint8_t)28);//0X19 novo endereço driverSensor
///Serial.println("04");
//----------------------------------------------------------
//-------------------------------------------------------------
pinMode(PIN3, INPUT);
///delay(150);
passengerSensor.init(true);

///Serial.println("----");
///delay(100);
passengerSensor.setAddress((uint8_t)32);// 0x17 novo endereço passengerSensor
//-------------------------------------------------------------
// end configuration
frontSensor.setTimeout(500);
driverSensor.setTimeout(500);
passengerSensor.setTimeout(500);
frontSensor.startContinuous();
driverSensor.startContinuous();
passengerSensor.startContinuous();
 
 callI2Cscanner();
/*
//------- scan i2c
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

*/
}
void callI2Cscanner()
{
  // scan i2c
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++) 
  {  
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    }             // end of good response
  }               // end of for loop
  
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");  
}

void loop() 
{
  float distance_1= frontSensor.readRangeContinuousMillimeters();// SENSOR DA FRENTE
  float distance_2= driverSensor.readRangeContinuousMillimeters();// SENSOR LADO ESQUERDO--MOTORISTA
  float distance_3= passengerSensor.readRangeContinuousMillimeters();// SENSOR LADO DIREITO--PASSAGEIRO
  /*
Serial.print(frontSensor.readRangeContinuousMillimeters());
if (frontSensor.timeoutOccurred()) { Serial.print(" TIMEOUT S1"); }
Serial.print("\t");
Serial.print(driverSensor.readRangeContinuousMillimeters());
if (driverSensor.timeoutOccurred()) { Serial.print(" TIMEOUT S2"); }

Serial.print(passengerSensor.readRangeContinuousMillimeters());
if (passengerSensor.timeoutOccurred()) { Serial.print(" TIMEOUT S3"); }
Serial.print("\t");

Serial.println();

*/
//-----local vars sensor vl53l0x----------------------------------------------

  uint16_t frontDistance_1; // SENSOR DA FRENTE
  uint16_t driverDistance_2;// SENSOR LADO ESQUERDO--MOTORISTA
  uint16_t passengerDistance_3;// SENSOR LADO DIREITO--PASSAGEIRO
 
   
  // Obtenha distância dos sensores
  frontDistance_1 = frontSensor.readRangeContinuousMillimeters();
  driverDistance_2 = driverSensor.readRangeContinuousMillimeters();
  passengerDistance_3 = passengerSensor.readRangeContinuousMillimeters();

  
 //------------
/////////   int distance1 =sensor1.readRangeContinuousMillimeters();// get distance for sensor 1 
//-------------------------------------------------------------
//////Serial.print(sensor1.readRangeContinuousMillimeters());
/////////if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT S1"); }
/////Serial.println();
///////Serial.print("\t");
//----------------------------------------------------------------------------------------
////////DistanciaReal = qualDistancia();
////Serial.print("distancia real: ");   
///////Serial.println(DistanciaReal); 


///if (DistanciaReal<=0) {
///    DistanciaReal = qualDistancia();
 ///   digitalWrite(ledverde, HIGH); 
 /// }   
    


  if (frontSensor.readRangeContinuousMillimeters() > 300) {
    stepsToDo = 2048;  // passos para uma revolução completa
    goForward(stepsToDo);
    DistanciaReal = qualDistancia();
    digitalWrite(ledverde, HIGH); 
  }    

if ( driverSensor.readRangeContinuousMillimeters() <=250){
//   stepsToDo = 512;  // passos para 1/4 de revolução
/// stepsToDo = 1512;  // passos para 
stepsToDo = 1612;  // passos para 
////stepsToDo = 2000;  // passos para  de revolução 180
turnRight(stepsToDo);
DistanciaReal = qualDistancia();
      
}

if ( passengerSensor.readRangeContinuousMillimeters() <=210){
//   stepsToDo = 512;  // passos para 1/4 de revolução
 stepsToDo = 1612;  // passos para 
////stepsToDo = 2000;  // passos para  de revolução 180
turnLeft(stepsToDo);
DistanciaReal = qualDistancia();
      
}

if ( driverSensor.readRangeContinuousMillimeters() <=180){
//   stepsToDo = 512;  // passos para 1/4 de revolução
/// stepsToDo = 1512;  // passos para 180
stepsToDo = 2040;  // passos para  de revolução 180
goBack(stepsToDo);
DistanciaReal = qualDistancia();
      
}

}
