#include <LedControl.h> // 8x8 LED Matrix library
#include <LiquidCrystal.h> // 2x16 LCD Display library

#define BUTTON_1 4 // Button is connected to D4 pin
#define BUTTON_2 5 // Button is connected to D5 pin
#define BUTTON_3 6 // Button is connected to D6 pin

#define PIN_TRIG 3 // HC-SR04 trig pin is connected to D2 pin
#define PIN_ECHO 2 // HC-SR04 echo pin is connected to D3 pin

#define PIN_IN1 24 // L298N IN1 pin is connected to D42 pin
#define PIN_IN2 22 // L298N IN2 pin is connected to D40 pin
#define PIN_ENA 7  // L298N ENA pin is connected to D7 pin

LiquidCrystal lcd(52, 50, 48, 46, 44, 42); // 2x16 LCD Display VSS pin is connected to GND pin
                                           // 2x16 LCD Display VDD pin is connected to 5V pin
                                           // 2x16 LCD Display V0 pin is connected to 10K pot mid pin
                                           // 2x16 LCD Display RS pin is connected to D52 pin
                                           // 2x16 LCD Display RW pin is connected to GND pin  
                                           // 2x16 LCD Display E pin is connected to D50 pin
                                           // 2x16 LCD Display D4 pin is connected to D48 pin 
                                           // 2x16 LCD Display D5 pin is connected to D46 pin
                                           // 2x16 LCD Display D6 pin is connected to D44 pin
                                           // 2x16 LCD Display D7 pin is connected to D42 pin
                                           // 2x16 LCD Display A pin is connected to 5V pin 
                                           // 2x16 LCD Display K pin is connected to GND pin

LedControl lc = LedControl(11, 12, 13, 1); // 8x8 LED Matrix DIN pin is connected to D11 pin
                                           // 8x8 LED Matrix CLK pin is connected to D12 pin
                                           // 8x8 LED Matrix CS pin is connected to D13 pin

int target_floor = 0; // Target floor number, in the interval of 1-3
int motor_signal; // Signal to drive the motor, in the interval of 0-255

long duration; // HC-SR04 response duration

unsigned long previous_millis = 0; // Arduino previous runtime, for serial port monitor

const long millis_interval = 500; // Arduino runtime interval, for serial port monitor

float height = 0; // Cabin height

float kP = 0.0; // P-Controller gain
float kD = 0.0; // D-Controller gain
float kI = 0.0; // I-Controller gain

float error_P = 0.0; // Difference in target floor height and current cabin height, for P-controller
float error_I = 0.0; // Difference in target floor height and current cabin height, for I-controller
float error_PI = 0.0; // Difference in target floor height and current cabin height, for PI-controller
float error_PD = 0.0; // Difference in target floor height and current cabin height, for PD-controller
float error_PID = 0.0; // Difference in target floor height and current cabin height, for PID-controller

float integral_I = 0.0; // Integral of error_I, cumulative sum of it, over time, for I-controller
float integral_PI = 0.0; // Integral of error_PI, cumulative sum of it, over tiem, for PI-controller
float integral_PID = 0.0; // Integral of error_PID, cumulative sum of it, over time, for PID-controller

float previous_error_PD = 0.0; // Previous value of error_PD, for PD-controller
float previous_error_PID = 0.0; // Previous value of error_PID, for PID-controller

float derivative_PD = 0.0; // Derivative of error_PD, difference between current and previous value of it, with respect to time, for PD-controller
float derivative_PID = 0.0; // Derivative of error_PID, difference between current and previous value of it, with respect to time, for PID-controller

float motor_voltage; // Voltage to drive the motor, in the interval of 0-7

void setup()
{
  pinMode(BUTTON_1, INPUT_PULLUP); // D4 pin is set as input pin that enables the internal pull-up resistor
  pinMode(BUTTON_2, INPUT_PULLUP); // D5 pin is set as input pin that enables the internal pull-up resistor
  pinMode(BUTTON_3, INPUT_PULLUP); // D6 pin is set as input pin that enables the internal pull-up resistor

  pinMode(PIN_TRIG, OUTPUT); // D2 pin is set as output pin
  pinMode(PIN_ECHO, INPUT);  // D3 pin is set as input pin

  pinMode(PIN_IN1, OUTPUT); // D42 pin is set as output pin
  pinMode(PIN_IN2, OUTPUT); // D40 pin is set as output pin
  pinMode(PIN_ENA, OUTPUT); // D7 pin is set as output pin
  
  lcd.begin(16, 2); // Initialize 2x16 LCD Display with 16 characters and 2 lines
  
  lc.shutdown(0, false); // Activate 8x8 LED Matrix
  lc.setIntensity(0, 8); // Set brightness level of 8x8 LED MAtrix to 8
  lc.clearDisplay(0); // Clear 8x8 LED Matris

  Serial.begin(9600); // Start serial communication and set baud rate to 9600
}

void loop()
{
  if (digitalRead(BUTTON_1) == LOW) // If BUTTON_1 is pressed
  {
    target_floor = 1; // target_floor is set to 1, indicating 1. floor
    delay(200); // Wait for debounce of the button
    Serial.println("Button 1 is pressed, target floor is 1"); // Print current state to serial port monitor
  }
  else if (digitalRead(BUTTON_2) == LOW) // If BUTTON_2 is pressed
  {
    target_floor = 2; // target_floor is set to 2, indicating 2. floor
    delay(200); // Wait for debounce of the button
    Serial.println("Button 2 is pressed, target floor is 2"); // Print current state to serial port monitor
  } 
  else if (digitalRead(BUTTON_3) == LOW) // If BUTTON_3 is pressed
  {
    target_floor = 3; // target_floor is set to 3, indicating 3. floor
    delay(200); // Wait for debounce of the button
    Serial.println("Button 3 is pressed, target floor is 3"); // Print current state to serial port monitor
  }

  digitalWrite(PIN_TRIG, LOW); // HC-SR04 trig pin is set to LOW
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(PIN_TRIG, HIGH); // HC-SR04 trig pin is set to HIGH
  delayMicroseconds(10); // Wait for 10 microseconds
  digitalWrite(PIN_TRIG, LOW); // HC-SR04 trig pin is set to LOW

  duration = pulseIn(PIN_ECHO, HIGH); // Read HC-SR04 response duration from echo pin in microseconds

  height = floor(((duration * 0.03432) / 2) * 10) / 10.0; // Calculate the height of the cabin in centimeters, (cabin height) =  ((HC-SR04 response duration) * (sound speed in air)) / 2, and neglect the hundredths place using floor function
  
  if (height < 29.5) show_floor_number(1); // If cabin height is less than 25 cm, show 1 on 8x8 LED Matrix, indicatin 1. floor
  else if (height < 54.5) show_floor_number(2); // Else if cabin height is less than 50 cm, show 2 on 8x8 LED Matrix, indicating 2. floor
  else show_floor_number(3); // Else, show 3 on 8x8 LED Matrix, indicating 3. floor

  kP = map(analogRead(A0), 0, 1023, 0, 20); // Read potentiometer value for kP in the interval of 0-20

  float kI_values[] = {0.000, 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009};
  delay(5);
  analogRead(A1);
  delay(5);
  kI = kI_values[map(analogRead(A1), 0, 1023, 0, 10)]; // Read potentiometer value for kI with the elements of kI_values array

  float kD_values[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
  delay(5);
  analogRead(A5);
  delay(5);
  kD = kD_values[map(analogRead(A5), 0, 1023, 0, 10)]; // Read potentiometer value for kD with the elements of kD_values array

  if (target_floor) // If target_floor variable is not zero or if any button is pressed
  {
    if (kP == 0 && kD == 0.00 && kI == 0.00) ON_OFF_controller((25 * (target_floor - 1)) + 5); // If kP, kI and kD are all zero, all potentiometerz are at the lowest level, move the cabin using ON-OFF controller
    else if (kP > 0 && kD == 0.00 && kI == 0.00) P_controller((25 * (target_floor - 1)) + 5); // If kP is not zero while kI and kD are zero, move the cabin using P controller
    else if (kP == 0 && kD == 0.00 && kI > 0.00) I_controller((25 * (target_floor - 1)) + 5); // If kI is not zero while kP and kD are zero, move the cabin using I controller
    else if (kP > 0 && kD == 0.00 && kI > 0.00) PI_controller((25 * (target_floor - 1)) + 5); // If kP and kI are not zero while kD is zero, move the cabin using PI controller
    else if (kP > 0 && kD > 0.00 && kI == 0.00) PD_controller((25 * (target_floor - 1)) + 5); // If kP and kD are not zero while kI is zero, move the cabin using PD controller
    else if (kP > 0 && kD > 0.00 && kI > 0.00) PID_controller((25 * (target_floor - 1)) + 5); // If kP, kI and kD are all not zero, all potentiometerz are at a level different than the lowest, move the cabin using PID controller
  }

  motor_voltage = (motor_signal * 7.0) / 255.0; // Calculate the motor voltage to drive the motor, in the interval of 0-7
  
  unsigned long current_millis = millis(); // Arduino current runtime, for serial port monitor

  if (current_millis - previous_millis >= millis_interval) // If Arduino runtime is in the interval of 0.5 seconds, for serial port monitor
  {
    previous_millis = current_millis; // Arduino runtime update, for serial port monitor
    
    // Print cabin height to serial port monitor
    Serial.print("Height: ");
    Serial.print(height);
    Serial.println(" cm");

    // Print floor number to serial port monitor
    Serial.print("Floor: ");
    Serial.println(target_floor);

    // Print gains to serial port monitor
    Serial.print("kP = ");
    Serial.print(int(kP));
    Serial.print(" | kI = ");
    Serial.print(int(kI * 1000));
    Serial.print("x10^(-3)");
    Serial.print(" | kD = ");
    Serial.print(int(kD * 10));
    Serial.println("x10^(-1)");

    // Print motor voltage to serial port monitor
    Serial.print("Motor voltage: ");
    Serial.print(motor_voltage);
    Serial.println(" V");

    // Print motor signal to serial port monitor
    Serial.print("Motor signal: ");
    Serial.println(motor_signal);
    
    Serial.print("\n");
  }

  update_LCD(height, motor_voltage, kP, kI, kD); // Update 2x16 LCD Display with new data of height, motor voltage, kP, kI and kD values
    
  // delay(100);
}

void update_LCD(float cabin_height, float voltage, float P_gain, float I_gain, float D_gain) // 2x16 LCD Display function
{
  // 1. line
  lcd.setCursor(0, 0); // Start printing from 1. line 1. character
  lcd.print("H:");
  lcd.print(cabin_height);
  lcd.print(" V:");
  lcd.print(voltage);
  lcd.print("   ");

  // 2. line
  lcd.setCursor(0, 1); // Start printing from 2. line 1. character
  lcd.print(int(P_gain));
  lcd.print(" ");
  lcd.print(int(I_gain * 1000));
  lcd.print("(-3) ");
  lcd.print(int(D_gain * 10));
  lcd.print("(-1)");
  lcd.print(" ");
}

void ON_OFF_controller(int target_height) // ON-OFF controller function
{
  if (height < target_height) // If current heighr is less than target height
  {
    // Drive the motor, full power, to lift the cabin
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 255);
  }
  else if (height > target_height) // Else if current height is greater than target height
  {
    // Drive the motor, full power, to lower the cabin
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, 255); // Motoru hızla çalıştırma
  }
  else // Else, if the cabin is at target height
  {
    // Do not drive the motor
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0); // Motoru durdurma
  }

  motor_signal = 255; // Signal to drive the motor
}

void P_controller(int target_height) // P-controller function
{
  error_P = target_height - height; // P error, difference between target height and current height
  
  if (error_P > 0.5) // If P error is greater than 0.5
  {
    // Drive the motor to lift the cabin
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  else if (error_P < 0) // If P error is less than 0
  {
    // Drive the motor to lower the cabin
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  else // Else, if the cabin is at or very close to target height
  {
    // Do not drive the motor
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }
  
  motor_signal = constrain(abs(kP * error_P), 0, 255); // Calculate the signal by constraining the P equation in the interval of 0-255 to drive the motor
  
  analogWrite(PIN_ENA, motor_signal); // Drive the motor
}

void I_controller(int target_height) // I-controller function
{
  error_I = target_height - height; // I error, difference between target height and current height

  integral_I += error_I; // I integral, cumulative sum of I error over time

  if (error_I > 0.5) // If I error is greater than 0.5
  {
    // Drive the motor to lift the cabin
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  else if (error_I < 0) // If I error is less than 0
  {
    // Drive the motor to lower the cabin
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  else // Else, if the cabin is at or very close to target height
  {
    // Do not drive the motor
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }

  motor_signal = constrain(abs(kI * integral_PI), 0, 255); // Calculate the signal by constraining the I equation in the interval of 0-255 to drive the motor

  analogWrite(PIN_ENA, motor_signal); // Drive the motor
}

void PI_controller(int target_height) // PI-controller function
{
  error_PI = target_height - height; // PI error, difference between target height and current height
  
  integral_PI += error_PI; // PI integral, cumulative sum of PID error over time
  
  if (error_PI > 0.5) // If PI error is greater than 0.5
  {
    // Drive the motor to lift the cabin
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  else if (error_PI < 0) // If PI error is less than 0
  {
    // Drive the motor to lower the cabin
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  else // Else, if the cabin is at or very close to target height
  {
    // Do not drive the motor
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }

  motor_signal = constrain(abs((kP * error_PI) + (kI * integral_PI)), 0, 255); // Calculate the signal by constraining the PI equation in the interval of 0-255 to drive the motor
  
  analogWrite(PIN_ENA, motor_signal); // Drive the motor
}

void PD_controller(int target_height) // PD-controller function
{
  error_PD = target_height - height; // PD error, difference between target height and current height

  derivative_PD = error_PD - previous_error_PD; // PD derivative, difference between current PD error and previous PID error with respect to time

  if (error_PD > 0.5) // If PD error is greater than 0.5
  {
    // Drive the motor to lift the cabin
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  else if (error_PD < 0) // If PD error is less than 0
  {
    // Drive the motor to lower the cabin
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  else // Else, if the cabin is at or very close to target height
  {
    // Do not drive the motor
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }

  motor_signal = constrain(abs((kP * error_PD) + (kD * derivative_PD)), 0, 255); // Calculate the signal by constraining the PD equation in the interval of 0-255 to drive the motor

  analogWrite(PIN_ENA, motor_signal); // Drive the motor

  previous_error_PD = error_PD; // Update the previous PD error
}

void PID_controller(int target_height) // PID-controller function
{
  error_PID = target_height - height; // PID error, difference between target height and current height

  integral_PID += error_PID; // PID integral, cumulative sum of PID error over time

  derivative_PID = error_PID - previous_error_PID; // PID derivative, difference between current PID error and previous PID error with respect to time

  if (error_PID > 0.5) // If PID error is greater than 0.5
  {
    // Drive the motor to lift the cabin
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  else if (error_PID < 0) // If PID error is less than 0
  {
    // Drive the motor to lower the cabin
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  else // Else, if the cabin is at or very close to target height
  {
    // Do not drive the motor
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }

  motor_signal = constrain(abs((kP * error_PID) + (kI * integral_PID) + (kD * derivative_PID)), 0, 255); // Calculate the signal by constraining the PID equation in the interval of 0-255 to drive the motor

  analogWrite(PIN_ENA, motor_signal); // Drive the motor

  previous_error_PID = error_PID; // Update the previous PID error
}

void show_floor_number(int floor_number) // 8x8 LED Matrix function
{
  if (floor_number == 1) // If BUTTON_1 is pressed, target floor is 1
  {
    lc.clearDisplay(0); // Clear the 8x8 LED Matrix display
    
    byte floor_1[8] = {
                      B00000000,
                      B00011000,
                      B00111000,
                      B00011000,
                      B00011000,
                      B00011000,
                      B00111110,
                      B00000000
                      };
                      
    for (int i = 0; i < 8; i++)
    {
      lc.setRow(0, i, floor_1[i]);
    }  
  }
  else if (floor_number == 2) // Else if BUTTON_2 is pressed, target floor is 2
  {
    lc.clearDisplay(0); // Clear the 8x8 LED Matrix display
    
    byte floor_2[8] = {
                      B00000000,
                      B00111100,
                      B01100110,
                      B00000110,
                      B00001100,
                      B00110000,
                      B01111110,
                      B00000000
                      };
                      
    for (int i = 0; i < 8; i++)
    {
      lc.setRow(0, i, floor_2[i]);
    }  
  }
  else if (floor_number == 3) // Else if BUTTON_3 is pressed, target floor is 3
  {
    lc.clearDisplay(0); // Clear the 8x8 LED Matrix display
    
    byte floor_3[8] = {
                      B00000000,
                      B00111100,
                      B01100110,
                      B00001100,
                      B00001100,
                      B01100110,
                      B00111100,
                      B00000000
                      };
                      
    for (int i = 0; i < 8; i++)
    {
      lc.setRow(0, i, floor_3[i]);
    }    
  }
}
