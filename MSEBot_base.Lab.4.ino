#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;    
Servo servo_GripMotor;
Servo servo_RotArmMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_MOTOR_CALIBRATION


//PIN VARIABLES
const int pin_Ultrasonic_Ping = 2;   //input plug
const int pin_Ultrasonic_Data = 3;   //output plug
const int pin_HallEffect_rightCl = 4;
const int pin_HallEffect_leftCl = 5;
const int pin_HallEffect_rightCh = 6;
const int pin_HallEffect_leftCh = 8;
const int pin_Mode_Button = 7;
const int pin_Right_Motor = 11;
const int pin_Left_Motor = 10;
const int pin_Arm_Motor = 9;
const int pin_Grip_Motor = 12;
const int pin_Rot_Arm_Motor = 13;
const int pin_Right_Line_Tracker = A0;
const int pin_Middle_Line_Tracker = A1;
const int pin_Left_Line_Tracker = A2;
const int pin_I2C_SDA = A4;         // I2C data = white
const int pin_I2C_SCL = A5;         // I2C clock = yellow

// Charlieplexing LED assignments
const int CP_Heartbeat_LED = 1;
const int CP_Indicator_LED = 4;
const int CP_Right_Line_Tracker_LED = 6;
const int CP_Middle_Line_Tracker_LED = 9;
const int CP_Left_Line_Tracker_LED = 12;

//constants

// EEPROM addresses
const int Left_Line_Tracker_Dark_Address_L = 0;
const int Left_Line_Tracker_Dark_Address_H = 1;
const int Left_Line_Tracker_Light_Address_L = 2;
const int Left_Line_Tracker_Light_Address_H = 3;
const int Middle_Line_Tracker_Dark_Address_L = 4;
const int Middle_Line_Tracker_Dark_Address_H = 5;
const int Middle_Line_Tracker_Light_Address_L = 6;
const int Middle_Line_Tracker_Light_Address_H = 7;
const int Right_Line_Tracker_Dark_Address_L = 8;
const int Right_Line_Tracker_Dark_Address_H = 9;
const int Right_Line_Tracker_Light_Address_L = 10;
const int Right_Line_Tracker_Light_Address_H = 11;

const int Left_Motor_Offset_Address_L = 12;
const int Left_Motor_Offset_Address_H = 13;
const int Right_Motor_Offset_Address_L = 14;
const int Right_Motor_Offset_Address_H = 15;

const int const_Grip_Motor_Open = 140;    
const int const_Grip_Motor_Closed = 90;      
const int const_Arm_Servo_Retracted = 55;      
const int const_Arm_Servo_Extended = 120;      
const int const_Display_Time = 500;
const int const_Line_Tracker_Calibration_Interval = 100;
const int const_Line_Tracker_Cal_Measures = 20;
const int const_Line_Tracker_Tolerance = 50;  
const int const_Motor_Calibration_Cycles = 3;
const int const_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long Echo_Time;
unsigned int Left_Line_Tracker_Data;
unsigned int Middle_Line_Tracker_Data;
unsigned int Right_Line_Tracker_Data;
long Left_Motor_Position;
long Right_Motor_Position;

unsigned long Display_Time;
unsigned long Calibration_Time;
unsigned long Left_Motor_Offset;
unsigned long Right_Motor_Offset;

unsigned int Cal_Count;
unsigned int Cal_Cycle;
unsigned int Left_Line_Tracker_Dark;
unsigned int Left_Line_Tracker_Light;
unsigned int Middle_Line_Tracker_Dark;
unsigned int Middle_Line_Tracker_Light;
unsigned int Right_Line_Tracker_Dark;
unsigned int Right_Line_Tracker_Light;
unsigned int Line_Tracker_Tolerance;

unsigned int  Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,65536};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  // set up ultrasonic
  pinMode(pin_Ultrasonic_Ping, OUTPUT);
  pinMode(pin_Ultrasonic_Data, INPUT);

  // set up drive motors
  pinMode(pin_Right_Motor, OUTPUT);
  servo_RightMotor.attach(pin_Right_Motor);
  pinMode(pin_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(pin_Left_Motor);

  // set up arm motors
  pinMode(pin_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(pin_Arm_Motor);
  pinMode(pin_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(pin_Grip_Motor);
  pinMode(pin_Rot_Arm_Motor, OUTPUT);
  servo_RotArmMotor.attach(pin_Rot_Arm_Motor);


  // set up encoders. Must be initialized in order that they are chained together, 
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

  // set up line tracking sensors
  pinMode(pin_Right_Line_Tracker, INPUT);
  pinMode(pin_Middle_Line_Tracker, INPUT);
  pinMode(pin_Left_Line_Tracker, INPUT);
  Line_Tracker_Tolerance = const_Line_Tracker_Tolerance;

  // read saved values from EEPROM
  b_LowByte = EEPROM.read(Left_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Left_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Middle_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte); 
  b_LowByte = EEPROM.read(Middle_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Right_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Right_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(Left_Motor_Offset_Address_H);
  Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(Right_Motor_Offset_Address_H);
  Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
 
  // button-based mode selection
  if(CharliePlexM::ui_Btn)
  {
    if(bt_Do_Once == false)
    {
      bt_Do_Once = true;
      Robot_State_Index++;
      Robot_State_Index = Robot_State_Index & 7;
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
      Cal_Cycle = 0;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  
  // Button Modes 
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level.




  switch(Robot_State_Index)
  {
    /*case 0:    //Robot stopped
    {
      readLineTrackers();
      UltrasonicPing();
      servo_LeftMotor.writeMicroseconds(1500); 
      servo_RightMotor.writeMicroseconds(1500); 
      servo_ArmMotor.write(const_Arm_Servo_Retracted);
      servo_GripMotor.write(const_Grip_Motor_Closed);
      encoder_LeftMotor.zero();
      encoder_RightMotor.zero();
      Mode_Indicator_Index = 0;
      break;
    } 
*/
  
    case 0:    //Robot Run after 3 seconds
    {
      //turn the motors on, drive straight until the bumper switch is hit
      servo_LeftMotor.writeMicroseconds(1700); 
      servo_RightMotor.writeMicroseconds(1700); 
      
      
      
    } 














































    /*
    case 2:    //Calibrate line tracker light levels after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          Left_Line_Tracker_Light = 0;
          Middle_Line_Tracker_Light = 0;
          Right_Line_Tracker_Light = 0;
          Calibration_Time = millis();
          Cal_Count = 0;
        }
        else if((millis() - Calibration_Time) > const_Line_Tracker_Calibration_Interval)
        {
          Calibration_Time = millis();
          readLineTrackers();
          Left_Line_Tracker_Light += Left_Line_Tracker_Data;
          Middle_Line_Tracker_Light += Middle_Line_Tracker_Data;
          Right_Line_Tracker_Light += Right_Line_Tracker_Data;
          Cal_Count++;
        }
        if(Cal_Count == const_Line_Tracker_Cal_Measures)
        {
          Left_Line_Tracker_Light /= const_Line_Tracker_Cal_Measures;
          Middle_Line_Tracker_Light /= const_Line_Tracker_Cal_Measures;
          Right_Line_Tracker_Light /= const_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
          Serial.print("Light Levels: Left = ");
          Serial.print(Left_Line_Tracker_Light,DEC);
          Serial.print(", Middle = ");
          Serial.print(Middle_Line_Tracker_Light,DEC);
          Serial.print(", Right = ");
          Serial.println(Right_Line_Tracker_Light,DEC);
#endif           
          EEPROM.write(ci_Left_Line_Tracker_Light_Address_L, lowByte(Left_Line_Tracker_Light));
          EEPROM.write(ci_Left_Line_Tracker_Light_Address_H, highByte(Left_Line_Tracker_Light));
          EEPROM.write(ci_Middle_Line_Tracker_Light_Address_L, lowByte(Middle_Line_Tracker_Light));
          EEPROM.write(ci_Middle_Line_Tracker_Light_Address_H, highByte(Middle_Line_Tracker_Light));
          EEPROM.write(ci_Right_Line_Tracker_Light_Address_L, lowByte(Right_Line_Tracker_Light));
          EEPROM.write(ci_Right_Line_Tracker_Light_Address_H, highByte(Right_Line_Tracker_Light));
          Robot_State_Index = 0;    // go back to Mode 0
        }
        Mode_Indicator_Index = 2; 
      }
      break;
    }


    
    
    case 3:    // Calibrate line tracker dark levels after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          Left_Line_Tracker_Dark = 0;
          Middle_Line_Tracker_Dark = 0;
          Right_Line_Tracker_Dark = 0;
          Calibration_Time = millis();
          Cal_Count = 0;
        }
        else if((millis() - Calibration_Time) > const_Line_Tracker_Calibration_Interval)
        {
          Calibration_Time = millis();
          readLineTrackers();
          Left_Line_Tracker_Dark += Left_Line_Tracker_Data;
          Middle_Line_Tracker_Dark += Middle_Line_Tracker_Data;
          Right_Line_Tracker_Dark += Right_Line_Tracker_Data;
          Cal_Count++;
        }
        if(Cal_Count == const_Line_Tracker_Cal_Measures)
        {
          Left_Line_Tracker_Dark /= const_Line_Tracker_Cal_Measures;
          Middle_Line_Tracker_Dark /= const_Line_Tracker_Cal_Measures;
          Right_Line_Tracker_Dark /= const_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
          Serial.print("Dark Levels: Left = ");
          Serial.print(Left_Line_Tracker_Dark,DEC);
          Serial.print(", Middle = ");
          Serial.print(Middle_Line_Tracker_Dark,DEC);
          Serial.print(", Right = ");
          Serial.println(Right_Line_Tracker_Dark,DEC);
#endif           
          EEPROM.write(Left_Line_Tracker_Dark_Address_L, lowByte(Left_Line_Tracker_Dark));
          EEPROM.write(Left_Line_Tracker_Dark_Address_H, highByte(Left_Line_Tracker_Dark));
          EEPROM.write(Middle_Line_Tracker_Dark_Address_L, lowByte(Middle_Line_Tracker_Dark));
          EEPROM.write(Middle_Line_Tracker_Dark_Address_H, highByte(Middle_Line_Tracker_Dark));
          EEPROM.write(Right_Line_Tracker_Dark_Address_L, lowByte(Right_Line_Tracker_Dark));
          EEPROM.write(Right_Line_Tracker_Dark_Address_H, highByte(Right_Line_Tracker_Dark));
          Robot_State_Index = 0;    // go back to Mode 0
        }
        Mode_Indicator_Index = 3;
      }
      break;
    }
*/




 
  } //end of switch statement

  
}//end of loop() 
//---------------------------------------------------------------------------------


//---------FUNCTIONS--------------------------------------------------------------



// read values from line trackers and update status of line tracker LEDs
void readLineTrackers()
{
  Left_Line_Tracker_Data = analogRead(pin_Left_Line_Tracker);
  Middle_Line_Tracker_Data = analogRead(pin_Middle_Line_Tracker);
  Right_Line_Tracker_Data = analogRead(pin_Right_Line_Tracker);

  if(Left_Line_Tracker_Data < (Left_Line_Tracker_Dark - Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(CP_Left_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(CP_Left_Line_Tracker_LED, LOW);
  }
  if(Middle_Line_Tracker_Data < (Middle_Line_Tracker_Dark - Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(CP_Middle_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(CP_Middle_Line_Tracker_LED, LOW);
  }
  if(Right_Line_Tracker_Data < (Right_Line_Tracker_Dark - Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(CP_Right_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(CP_Right_Line_Tracker_LED, LOW);
  }

#ifdef DEBUG_LINE_TRACKERS
  Serial.print("Trackers: Left = ");
  Serial.print(Left_Line_Tracker_Data,DEC);
  Serial.print(", Middle = ");
  Serial.print(Middle_Line_Tracker_Data,DEC);
  Serial.print(", Right = ");
  Serial.println(Right_Line_Tracker_Data,DEC);
#endif

}


// measure distance to target using ultrasonic sensor  
void UltrasonicPing()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(pin_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(pin_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  Echo_Time = pulseIn(pin_Ultrasonic_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(Echo_Time/148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(Echo_Time/58); //divide time by 58 to get distance in cm 
#endif
}  

