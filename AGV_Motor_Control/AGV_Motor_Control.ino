//Arduino code for Nissei BLDC Motor Control - RPM input to Left & Right motors, PID Controller integrated in this code

#include <Messenger.h>
#include <AutoPID.h>

Messenger Messenger_Handler = Messenger();

//Encoder Pins 

//Left Motor and Encoder
int lefEncAPin = 19;   // Continuous pulses
int lefEncBPin = 18;   // Direction Switching Pulse 
int lefMotPowPin = 12; // Motor Power pin
int lefMotCCWDirPin = 9; // Counter clockwise pin
int lefMotCWDirPin = 8;   // Clockwise pin

volatile float lefEncCou = 0, lefPreCou = 0;
volatile float dir_lef = 1;
long copy_Left_Encoder_Ticks = 0;
float lefCurTime=0, lefPreTime=0;


//Right Motor and Encoder
int rigEncAPin = 3;   // Continuous pulses
int rigEncBPin = 2;   // Direction Switching Pulse 
int rigMotPowPin = 13; // Motor Power pin
int rigMotCCWDirPin = 7; // Counter clockwise pin
int rigMotCWDirPin = 6; // Clockwise pin

volatile float rigEncCou = 0, rigPreCou = 0;
volatile float dir_rig = 1;
long copy_Right_Encoder_Ticks = 0;
float rigCurTime=0, rigPreTime=0;

//Variables for calculating RPM and setting the frequency of the loop
float N = 0.0;    // Used for getting RPMs to set
const float radWhe = 0.1; // in metres
const float wheBas = 0.43;  // in metres
const float pi = 3.1415;
const float geaRat = 25; // gear ratio of motor

unsigned long TT;//frequency variable
int frequency = 30;

//Variables from ROS

float linear = 0.0;
float angular = 0.0;
float ang = 0.0;

//AutoPID Variables and Objects

//Left
double lefMotRPM, lefMotSetRPM, lefOutPWM, lefWheRPM, lefMotRPMabs;

#define lefOutPWMMax 50
#define lefOutPWMMin 0
#define lefKP 0.02765
#define lefKD 0.035
#define lefKI 0.07432

//Right 
double rigMotRPM, rigMotSetRPM, rigOutPWM, rigWheRPM, rigMotRPMabs;

#define rigOutPWMMax 50
#define rigOutPWMMin 0
#define rigKP 0.02765
#define rigKD 0.035
#define rigKI 0.07432

//Variable for setting the direction of the robot

int lefMotDirInd = 1, rigMotDirInd = 1; // 1 means forward rotation of a wheel
int preLefMotDirInd = 1, preRigMotDirInd = 1;


//---------------Starting PID Objects-------------------------//

AutoPID lefMotPID(&lefMotRPMabs, &lefMotSetRPM, &lefOutPWM, lefOutPWMMin, lefOutPWMMax, lefKP, lefKI, lefKD);
AutoPID rigMotPID(&rigMotRPMabs, &rigMotSetRPM, &rigOutPWM, rigOutPWMMin, rigOutPWMMax, rigKP, rigKI, rigKD);


//Real-Time Calculation of Motors
void lefMotRPMCal() 
{ // RPM =  No. of count difference in a time interval divided by the time taken in the time interval in Minutes
  // There are 18 ticks in 1 revolution, Gear ratio is 25:1
  noInterrupts();
  copy_Left_Encoder_Ticks = lefEncCou;
  lefCurTime = micros();
  interrupts();
   lefMotRPM = (60 * 1000000 * (copy_Left_Encoder_Ticks - lefPreCou) / (lefCurTime - lefPreTime)) /18;
   lefMotRPMabs = abs(lefMotRPM);
   lefWheRPM = lefMotRPM / geaRat; 
   lefPreTime = lefCurTime;
   lefPreCou = lefEncCou; 
 
}

void rigMotRPMCal() 
{
  noInterrupts();
  copy_Right_Encoder_Ticks = rigEncCou;
  rigCurTime = micros();
  interrupts();
   rigMotRPM = (60 * 1000000 * (copy_Right_Encoder_Ticks - rigPreCou) / (rigCurTime - rigPreTime)) /18;
   rigMotRPMabs = abs(rigMotRPM);
   rigWheRPM = rigMotRPM / geaRat; 
   rigPreTime = rigCurTime;
   rigPreCou = rigEncCou; 
}

//Interrupt Routine for Left and Right Encoders

//Left Encoder

void ISRLefEncCou()   // Interrupt routine for left encoder
{
  lefEncCou = lefEncCou + dir_lef;
}

void ISRLefEncDir()
{
  if (digitalRead(lefEncBPin) == 0)
  {
    dir_lef = 1;
  }
  else {
  dir_lef = -1;
}
}

//Right Encoder

void ISRRigEncCou()   // Interrupt routine for left encoder
{
  rigEncCou = rigEncCou + dir_rig;
}

void ISRRigEncDir()
{
  if (digitalRead(rigEncBPin) == 0)
  {
    dir_rig = 1;
  }
  else {
  dir_rig = -1;
}
}

//left ccw = -1, right cw = +1
//left cw = +1, right ccw = -1

void(* resetFunc) (void) = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000); // Delay to allow for initialisation of serial
  
  //Left Encoder
  
  pinMode(lefEncAPin, INPUT); 
  digitalWrite(lefEncAPin, HIGH); // Turn on pullup resistor
  pinMode(lefEncBPin, INPUT); 
  digitalWrite(lefEncBPin, HIGH); // Turn on pullup resistor
  lefPreTime = micros();
  attachInterrupt(digitalPinToInterrupt(lefEncAPin),ISRLefEncCou , FALLING);
  attachInterrupt(digitalPinToInterrupt(lefEncBPin),ISRLefEncDir , CHANGE);
  
  //Right Encoder

  pinMode(rigEncAPin, INPUT); 
  digitalWrite(rigEncAPin, HIGH); // Turn on pullup resistor
  pinMode(rigEncBPin, INPUT); 
  digitalWrite(rigEncBPin, HIGH); // Turn on pullup resistor
  rigPreTime = micros();
  attachInterrupt(digitalPinToInterrupt(rigEncAPin),ISRRigEncCou , FALLING);
  attachInterrupt(digitalPinToInterrupt(rigEncBPin),ISRRigEncDir , CHANGE);
  
  //Setup Motors

  //Left Motor

  pinMode(lefMotPowPin, OUTPUT);
  pinMode(lefMotCCWDirPin,OUTPUT);
  pinMode(lefMotCWDirPin,OUTPUT);

  //Right Motor
  
  pinMode(rigMotPowPin, OUTPUT);
  pinMode(rigMotCCWDirPin,OUTPUT);
  pinMode(rigMotCWDirPin,OUTPUT);

  //Setup Messenger
  Messenger_Handler.attach(OnMessageCompleted);

  //AutoPID
  lefMotPID.setTimeStep(20);  // Calculations will be done and
  rigMotPID.setTimeStep(20);  //variables will be changed after this time has passed
  
  //Frequency setting variable according to ROS
  TT = micros();
}

void loop() {
  
  //Updating ROS and printing all the variables
  
  Serial.print("r"); //RPM
  Serial.print("\t");
  Serial.print(long(lefMotRPM));
  Serial.print("\t");
  Serial.print(long(rigMotRPM));
  Serial.print("\n");

  Serial.print("a"); //SetRPM
  Serial.print("\t");
  Serial.print(long(lefMotSetRPM));
  Serial.print("\t");
  Serial.print(long(rigMotSetRPM));
  Serial.print("\n");

  Serial.print("p"); //PWM
  Serial.print("\t");
  Serial.print(long(lefOutPWM));
  Serial.print("\t");
  Serial.print(long(rigOutPWM));
  Serial.print("\n");

  Serial.print("e");
  Serial.print("\t");
  Serial.print(long(lefEncCou));
  Serial.print("\t");
  Serial.print(long(rigEncCou));
  Serial.print("\n");
/*
  Serial.print("dir");
  Serial.print("\t");
  Serial.print(dir_lef);
  Serial.print("\t");
  Serial.print(dir_rig);
  Serial.print("\n");
  */

  //Reading From Serial

  Read_From_Serial();

  //Frequency setting of the loop
  
  while(micros()-TT<1000000/frequency){
   }
  TT = micros();
  
}

void Read_From_Serial()
{
  while(Serial.available())
  {
    int data = Serial.read();
    Messenger_Handler.process(data);
  }
}

void OnMessageCompleted()
{
  {
  
  char reset[] = "r";
  char set_speed[] = "s";
  
  
  if(Messenger_Handler.checkString("r"))
  {
    resetFunc();  
    
  }
  
  if(Messenger_Handler.checkString("s"))
  {

     Set_Speed();
     if(linear == 0 && ang == 0)
     {
  // Vehicle at rest
    lefMotSetRPM = 0;
    rigMotSetRPM = 0;
    lefMotDirInd = 0;
    rigMotDirInd = 0;
  
  }

  if(abs(linear)>0 && ang == 0)
  {
  // Straight line motion, also a turn of infinity radius
    N = 25*60 * linear / (2 * pi * radWhe);
    lefMotSetRPM = abs(N);
    rigMotSetRPM = abs(N);
    if(N>=0){
    lefMotDirInd = 1;
    rigMotDirInd = 1;
    }
    else 
    {
    lefMotDirInd = -1;
    rigMotDirInd = -1;
    }
  }

  if(linear == 0 && abs(ang)>0)
  {
  // Turn of radius zero - in place rotation
    N = 25*60 * ang * wheBas / (4 * pi * radWhe);
    
    lefMotSetRPM = abs(N);
    rigMotSetRPM = abs(N);
    if(N>=0)
    {
    lefMotDirInd = -1;
    rigMotDirInd = 1;
    }
    else 
    {
    lefMotDirInd = 1;
    rigMotDirInd = -1;
    }
  }

  if(abs(linear)>0 && abs(ang)>0)
  {
  // Here vehicle would go for a turn of a non zero radius
    float L = linear/ang; // radius of turn
    float Nl = 0.0 , Nr = 0.0;
 
    Nl = 25*60*(L-wheBas/2.0)*ang/(2*pi*radWhe);
    Nr = 25*60*(L+wheBas/2.0)*ang/(2*pi*radWhe);
    lefMotSetRPM = abs(Nr);
    rigMotSetRPM = abs(Nl);
    lefMotDirInd = Nl / abs(Nl);
    rigMotDirInd = Nr / abs(Nr);

  }

   lefMotRPMCal();
   rigMotRPMCal();
   
   lefMotPID.run(); //call every loop, updates automatically at certain time interval
   rigMotPID.run();


   if (lefMotDirInd == preLefMotDirInd && rigMotDirInd == preRigMotDirInd) 
   {
    // follow the code
    // This if...else... ensures that whenever wheel change their turning direction,the wheels first come to stop
  } 
  else {
    analogWrite(lefMotPowPin, 0);
    analogWrite(rigMotPowPin, 0);
    lefMotPID.stop();
    rigMotPID.stop();
    //Serial.println(" Stop Called ");
  }

  //-------Functions to call acoording to direction indicator variables------------
  // The last two conditions of assingment ensures that wheels are not stopped if the function is called again------
  if (lefMotDirInd == 1) {
    if (rigMotDirInd == 1) {
      forMov();     // forward movement
      
    } 
    else {
      rigMov();     // right movement
      
    }
  }

  if (lefMotDirInd == -1) 
  {
    if (rigMotDirInd == -1) {
      bacMov();     // backward movement
      
    } 
    else {
      lefMov();     // left movement
      
    }
  }
  
  if (lefMotDirInd == 0 && rigMotDirInd == 0) 
  {
    stopMov();
    
    }
}
  }
}

void Set_Speed()
{
  linear = Messenger_Handler.readLong() ;
  linear = linear * 0.1;
  
  angular = Messenger_Handler.readLong();
  ang = angular * 0.1;
  
  Serial.print("i");
  Serial.print("\t");
  Serial.print(long(linear*10));
  Serial.print("\t");
  Serial.print(long(ang*10));
  Serial.print("\n");
}

//Directions functions

void forMov()
{
  digitalWrite(lefMotCCWDirPin, LOW);
  digitalWrite(lefMotCWDirPin, HIGH);
  
  digitalWrite(rigMotCCWDirPin, HIGH);
  digitalWrite(rigMotCWDirPin, LOW);

  analogWrite(lefMotPowPin, lefOutPWM);
  analogWrite(rigMotPowPin, rigOutPWM);
  Serial.println(" Forward Called");
  preLefMotDirInd = lefMotDirInd;
  preRigMotDirInd = rigMotDirInd;
  

}
void bacMov()
{
  digitalWrite(lefMotCCWDirPin, HIGH);
  digitalWrite(lefMotCWDirPin, LOW);
  
  digitalWrite(rigMotCCWDirPin, LOW);
  digitalWrite(rigMotCWDirPin, HIGH);

  analogWrite(lefMotPowPin, lefOutPWM);
  analogWrite(rigMotPowPin, rigOutPWM);
  
  Serial.println(" Backward Called");
  preLefMotDirInd = lefMotDirInd;
  preRigMotDirInd = rigMotDirInd;
}

void lefMov()
{
  digitalWrite(lefMotCCWDirPin, HIGH);
  digitalWrite(lefMotCWDirPin, LOW);
  
  digitalWrite(rigMotCCWDirPin, HIGH);
  digitalWrite(rigMotCWDirPin, LOW);

  analogWrite(lefMotPowPin, lefOutPWM);
  analogWrite(rigMotPowPin, rigOutPWM);
  
  Serial.println(" left Called");
  preLefMotDirInd = lefMotDirInd;
  preRigMotDirInd = rigMotDirInd;
}
void rigMov()
{
  digitalWrite(lefMotCCWDirPin, LOW);
  digitalWrite(lefMotCWDirPin, HIGH);
  
  digitalWrite(rigMotCCWDirPin, LOW);
  digitalWrite(rigMotCWDirPin, HIGH);

  analogWrite(lefMotPowPin, lefOutPWM);
  analogWrite(rigMotPowPin, rigOutPWM);
  
  Serial.println(" right Called");
  preLefMotDirInd = lefMotDirInd;
  preRigMotDirInd = rigMotDirInd;
}

void stopMov()
{
  lefOutPWM=0;
  rigOutPWM=0;
  digitalWrite(lefMotCCWDirPin, LOW);
  digitalWrite(lefMotCWDirPin, LOW);
  digitalWrite(rigMotCCWDirPin, LOW);
  digitalWrite(rigMotCWDirPin, LOW);

  analogWrite(lefMotPowPin, lefOutPWM);
  analogWrite(rigMotPowPin, rigOutPWM);
  
  Serial.println(" Stop Called");
  preLefMotDirInd = lefMotDirInd;
  preRigMotDirInd = rigMotDirInd;
  
  lefMotPID.stop();
  rigMotPID.stop();
  }

  //-------------THE END-----------
