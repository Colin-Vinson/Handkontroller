/*
  motor_start_stop_timer_lcd

  Pressing the "start" button will run a motor for 10 seconds. The "stop" button
  will interupt the timer and stop the motor. The motor state and timer will be
  displayed on the lcd.

  The circuit:
  - "start" button
    - output attached to pin 8 to ground through a 10k Ohm resistor
  - "stop" button
    - output attached to pin 9 to ground through a 10k Ohm resistor
  - Tip122 base attached to pin 6
  - LCD i2c attached to Arduino i2c
    - SDA <--> SDA
    - SCL <--> SCL

  created 7 Feb 2022
  by Colin Vinson
*/

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// constants won't change. They're used here to set pin numbers:
const int startButton = 8;    // the number of the pushbutton pin
const int stopButton = 9;
const int motorPin = 6;      // the number of the LED pin

// Variables will change:
int motor_speed;            // the PWM value for the motor
int start_button_state;            // the current reading from the input pin
int last_start_button_state = LOW;  // the previous reading from the input pin
int stop_button_state;            
int last_stop_button_state = LOW;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long startDebounceTime = 0;  // the last time the output pin was toggled
unsigned long stopDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

unsigned long start_millis = 0;
unsigned long stop_millis = 0;
unsigned long run_time;
unsigned long timer;

boolean flag;

void setup() {
  pinMode(startButton, INPUT);
  pinMode(stopButton, INPUT);
  pinMode(motorPin, OUTPUT);

  // set initial LED state
  digitalWrite(motorPin, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  Serial.begin(9600);
}

void loop() {
  // read the state of the buttond into a local variables:
  int start_button_reading = digitalRead(startButton);
  int stop_button_reading = digitalRead(stopButton);

  // check to see if you just pressed a button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (start_button_reading != last_start_button_state) {
    // reset the debouncing timer
    startDebounceTime = millis();
  }

  if ((millis() - startDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (start_button_reading != start_button_state) {
      start_button_state = start_button_reading;

      // only toggle the LED if the new button state is HIGH
      if (start_button_state == HIGH) {
        run_time = 10;
        start_millis = millis();
        flag = true;
        lcd.clear();
      }
    }
  }

  if (stop_button_reading != last_stop_button_state) {
    // reset the debouncing timer
    stopDebounceTime = millis();
  }

  if ((millis() - stopDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (stop_button_reading != stop_button_state) {
      stop_button_state = stop_button_reading;

      // only toggle the LED if the new button state is HIGH
      if (stop_button_state == HIGH) {
        run_time = 0;
        stop_millis = millis();
        flag = false;
        lcd.clear();
      }
    }
  }

  unsigned long current_millis = millis();

  unsigned long difference = (current_millis - start_millis)/1000;
  
  if (difference < run_time){ 
        motor_speed = 255;
   }
   else{
    motor_speed = 0;
    run_time = 0;
   }

  // set the Motor:
  analogWrite(motorPin, motor_speed);

  // save the reading. Next time through the loop, it'll be the last_start_button_state:
  last_start_button_state = start_button_reading;
  last_stop_button_state = stop_button_reading;

  if (run_time > 0 && difference < run_time){
    timer = difference + 1;   //starts count form 0
  }
  else{
    //clear the lcd once the time reaches the run_time value
    if (flag == true){
      lcd.clear();
      flag = false;
    }
    timer = 0;
  }

  int duty_cycle = map(motor_speed,0,255,0,100);
  
  //display vlaues on lcd
  lcd.setCursor(0,0);
  lcd.print("Duty Cycle:");
  lcd.setCursor(12,0);
  lcd.print(duty_cycle);
  lcd.setCursor(16,0);
  lcd.print("%");
  lcd.setCursor(0,1);
  lcd.print("Timer:");
  lcd.setCursor(7,1);
  lcd.print(timer);

/*Error checking
  Serial.print("start_button_state: ");
  Serial.print(start_button_state);
  Serial.print("  ");
  Serial.print("stop_button_state: ");
  Serial.print(stop_button_state);
  Serial.print("  ");
  Serial.print("timer: ");
  Serial.print(timer);
  Serial.print("  ");
  Serial.print("motor_speed: ");
  Serial.println(motor_speed);
*/
}