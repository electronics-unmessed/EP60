//
// ********************************** Arduino Nano and Stepper Gauge **************************************************
// Stepper Motor X27 directly driven by Arduino Nano
// 0.5° per step  -> 90° -> 180 steps
// Rotation Angle: Max 315°
// 0.5°/full step
// Coil resistance: 280 +/- 20Ω: 5V -> 18mA
// White LEDs power consumption 4 * 2.5V/390 Ohms -> 0 .. 26mA with full brightness
// Arduino Nano 15mA
// Total consumption: 18mA + (0..13mA) + 15mA + 5mA = 38..100 mA
// Features:
//       Start-up procedure shows 0% -> 100% and relative settings of potentiometers
//       SW definition for gain, zero adjust,LED brightness
//       LED several features
//
// *********************************************************************************************************************

#include <Stepper.h>
#define STEPS 720  // steps per revolution (limited to 315°) 720
#define COIL1 4    // was
#define COIL2 2    // was
#define COIL3 7    // was
#define COIL4 12   // was

// create an instance of the stepper class:
Stepper stepper(STEPS, COIL1, COIL2, COIL3, COIL4);

// +++++ Software Definitions, which you can adjust +++++++++++++++++++++++++++

float Gain = 1.0;       // 0 .. 10 Gain of Gauge to calibrate  (typical 0.5 .. 5)
int zero_adjust = 0;    // adjust pointer to zero

// Definitions for the Dial Angles

int Full_Deflection = 540;  // No of Steps for full deflection 270 degrees
int zero_offset = 20;       // Offset from left block to Zero on Scale
int Zero_Under = 5;         // Clipping at Zero Underschoot
int Max_Over = 565;         // Clipping at Maximum Overshoot

int Red_Blink_Area = 500; // Blinking Red area, red LED comes up blinking
int Red_Area = 432;     // Red area, red LED comes up
int Yellow_Area = 324;  // Yellow area, yellow LED comes up
int Blink = 0;

int pos = 0;  //Position in steps(0-630)= (0°-315°)
int pos_Old = 0;
int val = 0;  // to be Position in Steps 0..180 = 90°
int val_Old = 0;

// int Turn_Indicator = 0;  // indicates turning direction

int speed = 70;        // Max Dynamic RPM = 70 * 720 / 60 = 840 Steps/sec = 420 °/sec  (X27 dynamic Maximum = 600 °/sec)
int start_speed = 30;  // Max Start RPM = 30 * 720 / 60 = 360 Steps/sec = 180 °/sec    (X27 static Maximum = 200 °/sec)

int DelayMicro = 5500;

float Slope_Steep_forward = 2.0;   // 0.8 .. 2
float Slope_Steep_backward = 2.0;  // 0.8 .. 2

float Speed_max = 800;  // 800 Steps/sec * 0.5°/Step = 400 °/sec
float Speed_min = 200;  // 200 Steps/sec * 0.5°/Step = 100 °/sec
float Speed_var = 0;
float val_pos_diff = 0;

// Acceleration Definitions

float Acc_Steps = 20;      //
float no_average = 50.0;   // averaging for input signal
float acc_average = 50.0;  // averaging for acceleration
float val_1 = 0;
float val_1_Old = 0;
float val_1_av_Old = 0;
float val_1_av = 0;

// Analog Input

float voltA0 = 0;
float voltA0_average = 0;

// Lights Control

int LED_Light_D3 = 3;    // defines PIN for LED
int LED_Light_D5 = 5;    // defines PIN for LED
int LED_Light_D6 = 6;    // defines PIN for LED
int LED_Light_D11 = 11;  // defines PIN for LED

int Brightness_D3 = 50;          // Brightness for LEDs PWM 0..255
int Blink_Brightness_D3 = 250;    // Brightness for LEDs PWM 0..255
int Brightness_D5 = 30;           // Brightness for LEDs PWM 0..255
int Brightness_D6 = 30;           // Brightness for LEDs PWM 0..255
int Brightness_D11 = 50;         // Brightness for LEDs PWM 0..255

// int Brightness_D6_max = 30;   // Brightness for LEDs PWM 0..255
// int Brightness_D11_max = 30;  // Brightness for LEDs PWM 0..255

const int analogInPin0 = A0;  // Analog in Voltage

float counter = 0;
long count = 0;

float FilterInput_0 = 0;

// ******************* End Definitions ***********************************************************************************

// ******************** Set-up *******************************************************************************************

void setup() {
  Serial.flush();
  // Serial.begin(115200);
  Serial.begin(9600);

  Serial.setTimeout(200);  // Standard is 1000m
  Serial.flush();

  pinMode(LED_Light_D3, OUTPUT);   // LED rot
  pinMode(LED_Light_D5, OUTPUT);   // 2x LED weiss
  pinMode(LED_Light_D6, OUTPUT);   // 2x LED weiss
  pinMode(LED_Light_D11, OUTPUT);  // LED rot oder grün
  //  pinMode(LED_Light_D16, OUTPUT); // would be possible

  stepper.setSpeed(start_speed);  // set the motor speed to 30 RPM (360 PPS aprox.).
  stepper.step(630);              //Reset Position(630 steps counter-clockwise).

  stepper.setSpeed(speed);
  Speed_var = Speed_min;

  Serial.flush();
}

// ***************************** Main Program ***************************************************************************
// **********************************************************************************************************************

void loop() {

  A0_Read_Voltage();

  Lights();

  FilterInput_0 = Gain * 0.5279 * voltA0_average;

  val = int(FilterInput_0);  // signal comes without filter

  Start_Up();  // moves pointer -> 0 -> 100% -> 

  val = val + zero_offset + zero_adjust;

  Clipping();

  Acceleration();

  Drive_Stepper();

  delayMicroseconds(DelayMicro);

   /*
  // for Serial Plot:

  Serial.print(900);
  Serial.print(",");
  Serial.print(-100);
  Serial.print(",");
  Serial.print(val);
  Serial.print(",");
  Serial.print(pos);
  Serial.print(",");
  Serial.println(Speed_var);

  delay(15);

   */
}

// ************ End Main Loop ********************************************************************************************
// ***********************************************************************************************************************

// ********* Calculation for Optimum Acceleration ************************************************************************

void Acceleration() {

  /*
  // ** Floating Averaging (second best method) ****

  val_1_av = val;
  val_1_av = (acc_average - 1) / acc_average * val_1_av_Old + 1 / acc_average * val_1_av;
  val_1_av_Old = val_1_av;

  val = int(val_1_av);
  */

  // ** Exact Step Control (best method) *****

  val_1 = val;

  if (val_1_Old - val_1 > Slope_Steep_backward) { val_1 = val_1_Old - Slope_Steep_backward; };  // limits falling slope
  if (-val_1_Old + val_1 > Slope_Steep_forward) { val_1 = val_1_Old + Slope_Steep_forward; };   // limits rising slope

  val_1_Old = val_1;
  val = int(val_1);

  val_pos_diff = abs(val - pos);
  Speed_var = val_pos_diff * ((Speed_max - Speed_min) / Acc_Steps);  // calculates variable Speed
  if (Speed_var > Speed_max) { Speed_var = Speed_max; };             // respects upper speed limit
  if (Speed_var < Speed_min) { Speed_var = Speed_min; };             // sets lower speed limit

  DelayMicro = 1000000 / Speed_var;
}

// ********* Drive Stepper Motor **************************************************************************

void Drive_Stepper() {

  float diff = 1.0;  // ??_3: 0..10 difference val-pos for stepper to get active

  if (abs(val - pos) > diff) {  // ??_3:  moves stepper, if diference is greater than diff steps.

    if ((val - pos) > 0) {
      stepper.step(-1);  // move one step to the right.
      pos++;
    };

    if ((val - pos) < 0) {
      stepper.step(1);  // move one step to the left.
      pos--;
      // delayMicroseconds(DelayMicro_Back);  // in case back turning speed ist limited
    };
  };
}


void A0_Read_Voltage() {

  float inputA0 = 0;  // Variable for Analog Input

  float voltA0_average_OLD = 0;

  inputA0 = analogRead(analogInPin0);
  voltA0 = 1.03 * map(inputA0, 0, 1023, 0, 1023);  // ??_0: any input signal can be chosen instead

  voltA0_average_OLD = voltA0_average;
  voltA0_average = (no_average - 1) / no_average * voltA0_average_OLD + 1 / no_average * voltA0;
}

void Clipping() {

  if (val < Zero_Under) { val = Zero_Under; };
  if (val > Max_Over) { val = Max_Over; };
}

void Start_Up() {
  count = count + 1;

  if (count < 750) { val = Full_Deflection; };
  if (count < 150) { val = 0; };
  if (count > 2000) { count = 2000; };
}

void Lights() {

  analogWrite(LED_Light_D5, Brightness_D5); // 2 white LEDs
  analogWrite(LED_Light_D6, Brightness_D6); // 2 white LEDs

  if ((pos >= Red_Area) && (pos < Red_Blink_Area)) {
    analogWrite(LED_Light_D3, Brightness_D3); // 1 red LED
  } else {
    analogWrite(LED_Light_D3, 0);
  };

  if (pos >= Red_Blink_Area) {
    Blink++;
    if (Blink > 7) {
      analogWrite(LED_Light_D3, Blink_Brightness_D3); // same red LED blinking brighter
    }
    if (Blink > 14) {
      analogWrite(LED_Light_D3, 0);
      Blink = 0;
    }
  };

  if ((pos >= Yellow_Area) && (pos < Red_Area)) {
    analogWrite(LED_Light_D11, Brightness_D11); // 1 red LED
    // Serial.println("Yellow Area");
  } else {
    analogWrite(LED_Light_D11, 0);
  };

  // analogWrite(LED_Light_D11, Brightness_D11);
}


void SerialPrint() {

  counter = counter + 1;

  if (counter > 100) {
    counter = 0;

    Serial.print("VoltA0_Av: ");
    Serial.print(voltA0_average);
    Serial.print(", ");
    Serial.print("Zero: ");
    Serial.print(zero_adjust);
    Serial.print(", ");
    Serial.print("Gain: ");
    Serial.print(Gain);
    Serial.print(", ");
    Serial.print("Fi-In: ");
    Serial.print(FilterInput_0);
    Serial.print(", ");
    Serial.print("Val: ");
    Serial.print(val);
    Serial.print(", ");
    Serial.print("Pos: ");
    Serial.print(pos);
    Serial.println("");
  };
}

// ******** End **********************************************************************************************************
