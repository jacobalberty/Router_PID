/*
 * PWM PID control using an optical sensor, a PWM enable AC triac power control, LCD panel....more info soon.
 * 
 * Details for now, https://www.v1engineering.com/forum/topic/hardware-needed-for-a-software-fix/
 * 
 * V1 Engineering
 * 
 * Updated on August 18, 2018
 */

#include <PID_v1.h>                                  // Arduino library PID Library by Brett Beauregard
#include <LiquidCrystal.h>                           // LCD Library

const long US_PERIOD_TO_RPM = 60000000;              // Convert from us of one rotation to RPM.

const int SPINDLE_SENSOR_PIN = 2;                    // Spindle IR Sensor input pin
const int PWM_IN_PIN = 3;                            // PWM Input pin from Rambo\Marlin
const int SPINDLE_ENABLE_PIN = 5;                    // Spindle Enable Pin from Rambo\Marlin
const int ROUTER_PWM_OUT_PIN = 9;                    // PWM Output to Dimmer Boart for Spindle

unsigned long current_rpm_time = 0;                  // Spindle RPM PWM input calc. Initialize to 0
unsigned long prev_rpm_time = 0;                     // Spindle RPM PWM input calc. Initialize to 0
unsigned long prev_time=0;

volatile unsigned int rpm_math=0;                    // RPMs Calculated from pulse times from Spindle Sensor.  Volatile to read outside ISR.
int optical_pwm = 0;                                 // Spindle RPM represented by pulse width setting (0-255)8bit, (0-4095)12bit. Initialize to 0                                               
volatile int PWM_IN_MARLIN = 0;                      // Stores Measured PWM input from Rambo\Marlin.  Time HIGH in microseconds.  Max from Marlin is 2024us.

const long PWM_OUT_MAX = 4095;                       // Setting for PWM Max #. 12bit = 4095.....Max 16bit resolution on Timer1....only on pin 9 &
int DIMMER_MIN=0;                                    // Used to set dimmer min mapping in setup based on PWM max value setting.
int DIMMER_MAX=0;                                    // Used to set dimmer max mapping in setup based on PWM max value setting.
int DIMMER_MAPPED_OUTPUT = 0;                        // Dimmer board is 20-240(on 255 scale).  This is mapped later based on PWM resolution setting.

//const int rs = 11, en = 12, d4 = 10, d5 = 9, d6 = 8, d7 = 7; // Initialize LCD pins
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                   // LCD library pins
//volatile unsigned long current_lcd_time = 0;                 // LCD refresh timer
//volatile unsigned long prev_lcd_time = 0;                    // LCD refresh timer
//const int LCD_Refresh = 200;                                 // LCD refresh rate in milliseconds

// PID variables
double Setpoint = 0.0;                           
double Input = 0.0;
double Output = 0.0;

//PID Gains
double Kp=2.1;    //....Needs tuning?
double Ki=7.0;    //....Needs tuning?
double Kd=0.025;   //....Needs tuning?

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);                   // PID library

const int MAX_TOOL_RPM = 30000;                                              // Max RPM for Spindle\Router
volatile int MAX_PWM_INPUT_US = -1;                                          // Settings the microseconds of the max PWM from Marlin.
const int MIN_TOOL_RPM = 5000;                                               // Min Spindle RPM to control to. \\Not yet implemented

void setup()
{
    
    pinMode(SPINDLE_SENSOR_PIN, INPUT_PULLUP);                                        // Spindle RPM input pin
    attachInterrupt(digitalPinToInterrupt(SPINDLE_SENSOR_PIN), spindleRPM, FALLING);  // Spindle RPM interrupt
    
    pinMode(PWM_IN_PIN, INPUT);                                              // Marlin PWM Input pin
    attachInterrupt(digitalPinToInterrupt(PWM_IN_PIN), rising, RISING);         // Marlin PWM interrupt
    
    pinMode(SPINDLE_ENABLE_PIN, INPUT);                                      // Marlin Spindle enabled pin

    setupPWM16();                                                             //Setup PWM out to use 16bit capability....only using 12bits at 4095
    analogWrite16(ROUTER_PWM_OUT_PIN,0);                                      // Spindle Off initially.

    DIMMER_MIN = map(20, 0, 255, 0, PWM_OUT_MAX);                             //Maps the dimmer min of 20 on 0-255 scale to PWM max setting
    DIMMER_MAX = map(240, 0, 255, 0, PWM_OUT_MAX);                            //Maps the dimmer max of 20 on 0-255 scale to PWM max setting
    
    myPID.SetMode(AUTOMATIC);                                                // PID auto/manual *TODO change with lcd button
    myPID.SetSampleTime(8.2);                                                // PID Loop time this is faster than 60Hz Double check
    myPID.SetOutputLimits(0,PWM_OUT_MAX);                                   // Set PID to use the range matching the PWM out max setting.
    
    //lcd.begin(20, 4);                                                        // 2004 LCD size
    //Serial.begin(115200);
}

void loop() {
  
   //Read the Spindle Enable pin from Marlin
   int spindle_enable = digitalRead(SPINDLE_ENABLE_PIN);

   //Read PWM Input from Marlin if Spindle pin is enabled
   if (spindle_enable==1){
     if (!myPID.GetMode()){myPID.SetMode(AUTOMATIC);};                                          // Enable the PID if disabled}
     optical_pwm =  (rpm_math * (unsigned long)PWM_OUT_MAX)/((unsigned long)MAX_TOOL_RPM);     // Compute the spindle's RPM value as a PWM correlated value

     // One iteration of the PID.
     Input = optical_pwm;                                                       // PID Input from router
     Setpoint = map(PWM_IN_MARLIN, 0, MAX_PWM_INPUT_US, 0, PWM_OUT_MAX);    // PID setpoit from Marlin
     myPID.Compute();                                                          // PID Run the Loop  

     //Adjust the PWM to the Dimmer Board                 
     DIMMER_MAPPED_OUTPUT = map(Output, 0, PWM_OUT_MAX, DIMMER_MIN, DIMMER_MAX);               // Traic scaling 20-240(8bit),  80-963(10bit)

     //Using Greater than 8bit so necessary to turn off interrupts to complete automically.
     noInterrupts(); //Disable Interrupts
     analogWrite16(ROUTER_PWM_OUT_PIN, DIMMER_MAPPED_OUTPUT);                             // Set the PWM out to the dimmer to the new value.
     interrupts(); // Enable Interrupts
     
   } else {
     noInterrupts(); //Disable Interrupts.
     analogWrite16(ROUTER_PWM_OUT_PIN, 0); // Turn off Spindle AC.
     interrupts(); //Enable Interrupts
     if (myPID.GetMode()){myPID.SetMode(MANUAL);}; //Disable PID if not already disabled
     PWM_IN_MARLIN=0;       //Clear PWM In Setting.
     rpm_math=0;            //Clear RPM
     optical_pwm=0L;        //Clear PID Values
     Input=0.0;             //Clear PID Values
     Output=0.0;            //Clear PID Values
     Setpoint=0.0;          //Clear PID Values
     MAX_PWM_INPUT_US = -1; //Clear PWM timing
     prev_time = 0;         //Clear PWM Previous time
                         
   }

   // Debug to console.
    //Serial.print("RPM = ");                                    // PWM In debug
    //Serial.println(rpm_math);                                  // Spindle, Display RPM
   // Serial.print("PWM In = ");                               // PWM In debug
   // Serial.println(PWM_IN_MARLIN);                           // PWM In Debug
   // Serial.print("Optical PWM = ");                          // PWM In debug
   // Serial.println(optical_pwm);                             // Mapped setpoint for PID
   // Serial.print("PID Input = ");                            // PWM In debug
   // Serial.println(Input);                                   // Mapped setpoint for PID
   // Serial.print("PID Setpoint = ");                         // PWM In debug
   // Serial.println(Setpoint);                                // Mapped setpoint for PID
   // Serial.print("PID Output = ");                           // PWM In debug
   // Serial.println(Output);                                  //PID Output
   
   
    //current_lcd_time = millis();                             // LCD timer
       
    //if (current_lcd_time - prev_lcd_time > LCD_Refresh){     // LCD check refresh rate
    //   lcd.setCursor(3, 0);                                  // LCD position
    //   lcd.print("V1 ENGINEERING");
    //   lcd.setCursor(1, 2);
    //   lcd.print("RPM");
    //   lcd.setCursor(0, 3);
    //  if (rpm_math > 0 ){                                    // LCD used to clear display at RPM=0
    //     lcd.print(rpm_math);
    //     }
    //   else {
    //     lcd.print("  0  ");
    //   }
    //   prev_lcd_time = current_lcd_time;                     // LCD timer reset
    //   }
    
}
/* Configure digital pins 9 and 10 as 16-bit PWM outputs. */
void setupPWM16() {
    DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
    TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
        | _BV(WGM11);                   /* mode 14: fast PWM, TOP=ICR1 */
    TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS10);                    /* no prescaling */
    ICR1 = 0x0fff;                      /* TOP counter value. Set to 4095*/
}

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */
void analogWrite16(uint8_t pin, uint16_t val)
{
    switch (pin) {
        case  9: OCR1A = val; break;
        case 10: OCR1B = val; break;
    }
}

// spindleRPM gets called on a falling edge of the SPINDLE_SENSOR_PIN, and records the amount of time
// between edges in the rpm_value field (in microsecinds). ISR.
void spindleRPM() {
 
       // Capture the time as "now"
       current_rpm_time = micros();
 
       // Calculate the RPM of the Spindle based on the time from the last interrupt.
       rpm_math = (US_PERIOD_TO_RPM / (current_rpm_time - prev_rpm_time))-1;

       // Start a new "timer" by setting the previous to "now".
       prev_rpm_time = current_rpm_time;
       
}

// rising() is called on the rising edge of the PHOTO_PIN. Basically starts the timer for measuring the
// PWM duty cycle. ISR.
void rising() {
   unsigned long new_pulse = micros();
   // Capture when this is rising.
   if (MAX_PWM_INPUT_US == -1 && prev_time != 0) {
     MAX_PWM_INPUT_US = (new_pulse-prev_time)*.99;
   }
   prev_time = new_pulse;

   // Set the next interrupt.
   attachInterrupt(digitalPinToInterrupt(PWM_IN_PIN), falling, FALLING);
   }

// falling() is called on the falling edge of the PWM_PIN. Records the amount of time since the
// rising(). ISR.  
 void falling() {
   // Measure the time since the last rising edge, in microseconds.
   PWM_IN_MARLIN = micros()- prev_time;

   // Set the next interrupt.
   attachInterrupt(digitalPinToInterrupt(PWM_IN_PIN), rising, RISING);
  }
