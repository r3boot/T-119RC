/*
 * Wiring diagram
 * 
 *                                                               +-----| usb |-----+
 *                            +--------+   +---------+           |gnd           vin|
 * +------------+             |  R-ESC |   |  L-ESC  |           |0            agnd|
 * |        elev|-----\       +--------+   +---------+           |1            3.3v|
 * |        aile|---\  \           ^            ^                |2              23|
 * |        thro|    \  \          |            |                |3              22|
 * |        rudd|     \  \         |            |                |4              21|
 * |        gear|      \  \        |            \----------------|5              20|
 * |        aux1|       \  \       \-----------------------------|6              19|
 * |        aux2|        \  \----------------------------------->|7              18|
 * +------------+         \------------------------------------->|8              17|------> signal led blue
 *                                                               |9              16|------> signal led green
 *                                                               |10             A1|------> signal led red
 *                                                               |11             A0|<---- lipo voltage
 *                                                               |12             13|     
 *                                                               +-----------------+
 */

#include "Servo.h"

#define DEBUG         // Enable a debug build

// Pins on which we receive input from the RC receiver
#define THROTTLE_IN_PIN 7   // Connected to elevation channel on receiver
#define STEERING_IN_PIN 8   // Connected to aileron channel on receiver

// Pins on which we control the motor ESC for the tracks
#define L_ESC_CONTROL_PIN 5 // Connected to servo control of left motor
#define R_ESC_CONTROL_PIN 6 // Connected to servo control of right motor

// Pins on which the voltage of the LIPO is measured using a resistor divider
#define LIPO_VOLTAGE_PIN A0 // Connected to LIPO via resistor-divider circuit

// Pin used to signal that the controller is powered on
#define MCU_LED_PIN 13

// Pins used for the RGB signalling led
#define SIGNAL_R_PIN A1 // Meh .. pin broken?
#define SIGNAL_G_PIN A2
#define SIGNAL_B_PIN A3

// Bit flags used for signalling changes on receiver channels
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define VOLTAGE_FLAG  4

// Input voltage as measured from DC-DC converter
#define INPUT_VOLTAGE 8

// LIPO undervoltage cutoff value
#define LIPO_CUTOFF_VOLTAGE 6.6

// Maximum value for ADC read
#define ADC_MAX 475.0

// RX701 min/max readings
#define THROTTLE_MIN      1165
#define THROTTLE_NEUTRAL  1565
#define THROTTLE_MAX      1966

#define STEERING_MIN      1091
#define STEERING_NEUTRAL  1489
#define STEERING_MAX      1892

// Midpoint tolerance for servo stick position
#define SERVO_MIDPOINT_TOLERANCE 3

// Various timing values for the motor ESCs
#define ESC_MIN           1100
#define ESC_NEUTRAL       1500
#define ESC_MAX           1800

#define ESC_FWD_MAX_VALUE 1800  // in microseconds
#define ESC_RWD_MAX_VALUE 1100  // in microseconds
#define ESC_NEUTRAL_VALUE 1500  // in microseconds
#define ESC_ARMING_DELAY  500   // in microseconds

// Values used for timing of signal led flash
#define FLASH_PERIOD    750  // in ms
#define FLASH_DURATION  75   // in ms

#define MOTOR_UPDATE_STEPS    5
#define MOTOR_UPDATE_INTERVAL 20000

// Color values
#define BLACK       0
#define RED         1
#define GREEN       2
#define BLUE        3
#define MAGENTA     4
#define LIGHT_GREEN 5
#define ORANGE      6

// Timing values used for flashing the signal led during bootup
#define RECEIVER_ACTIVE_FLASH_DURATION    25    // in ms
#define RECEIVER_ACTIVE_FLASH_PERIOD      750   // in ms

// Timing values used for controlling the ESC's
#define ESC_UPDATE_PERIOD                 20    // in ms

// Timing values used for measuring the voltage
#define VOLTMETER_UPDATE_PERIOD           1000   // in ms

// Macro used to get the voltage
#define READ_VOLTAGE (analogRead(LIPO_VOLTAGE_PIN) * INPUT_VOLTAGE) / ADC_MAX

struct color {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} colors[] = {
  {   0,   0,   0 },    // Black
  { 255,   0,   0 },    // Red
  {   0, 255,   0 },    // Green
  {   0,   0, 255 },    // Blue
  { 255,   0, 255 },    // Magenta
  { 255, 255,   0 },    // Light green
  { 0,     0, 128 },    // Orange
};

// Status register for incoming receiver signals
volatile uint8_t SharedStateFlags;

// Shared variables, written by ISR and read by main loop
volatile int ThrottleInShared;
volatile int SteeringInShared;

volatile int WantedLeftThrottle;
int CurrentLeftThrottle;

volatile int WantedRightThrottle;
int CurrentRightThrottle;

volatile int ThrottleLeftShared;
volatile int ThrottleRightShared;

// Variables used inside ISR to record rising edge of pulse
uint32_t ThrottleInStart;
uint32_t SteeringInStart;

float LipoVoltage;                // Used to check the voltage of the lipo battery

// The currently selected signal color
color currentColor;

// Timing value used for flashing the led
uint16_t FlashStart;

// Various timing values used to trigger events
int ReceiverActiveTimer = 0;    // Used for checking that the receiver is transmitting values
int ESCUpdateTimer = 0;         // Used for updating the ESCs in a regular manner
int VoltMeterTimer = 0;         // Used for measuring the voltage

// Used for debugging inside of the mainloop
uint32_t DebugTimer;

// Used to hold the debug message in the mainloop
#ifdef DEBUG
  String debugMessage;
#endif

// Servo objects used to control the motor ESCs
Servo LeftESC;
Servo RightESC;

// ISR used to read the incoming PWM signal for the throttle channel
void readThrottle()
{
  if (digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ThrottleInStart = micros();
  }
  else
  {
    ThrottleInShared = (uint16_t)(micros() - ThrottleInStart);
    SharedStateFlags |= THROTTLE_FLAG;
  }
}

// ISR used to read the incoming PWM signal for the steering channel
void readSteering()
{
  if (digitalRead(STEERING_IN_PIN) == HIGH)
  {
    SteeringInStart = micros();
  }
  else
  {
    SteeringInShared = (uint16_t)(micros() - SteeringInStart);
    SharedStateFlags |= STEERING_FLAG;
  }
}

// ISR used to flash the signal led if required
void FlashSignalLed()
{
  if (!FlashStart) {
    FlashStart = millis();
    analogWrite(SIGNAL_R_PIN, constrain(currentColor.red, 0, 255));
    analogWrite(SIGNAL_G_PIN, constrain(currentColor.green, 0, 255));
    analogWrite(SIGNAL_B_PIN, constrain(currentColor.blue, 0, 255));
  }
    
  if ((millis() - FlashStart) > FLASH_DURATION) {
    analogWrite(SIGNAL_R_PIN, 0);
    analogWrite(SIGNAL_G_PIN, 0);
    analogWrite(SIGNAL_B_PIN, 0);
  }
    
  if ((millis() - FlashStart) > FLASH_PERIOD) {
    FlashStart = 0;
  }
}

// Set the color of the signal RGB led to a certain value
void setSignalColor(color c)
{
  analogWrite(SIGNAL_R_PIN, constrain(c.red, 0, 255));
  analogWrite(SIGNAL_G_PIN, constrain(c.green, 0, 255));
  analogWrite(SIGNAL_B_PIN, constrain(c.blue, 0, 255));
  currentColor = c;
}

// Routine that runs whenever a failsafe condition is needed. This routine never returns
void RunFailsafeLoop(color c)
{
  // Disable interrupts to prevent further actions
  noInterrupts();

  // Make sure we show the correct color
  setSignalColor(c);

  // Disable all currently running ISRs
  detachInterrupt(digitalPinToInterrupt(THROTTLE_IN_PIN));
  detachInterrupt(digitalPinToInterrupt(STEERING_IN_PIN));
  
  // Stop motors from running
  LeftESC.writeMicroseconds(ESC_NEUTRAL);
  RightESC.writeMicroseconds(ESC_NEUTRAL);

#ifdef DEBUG
  Serial.print("Controller halted, manual recovery required");
#endif
  for (;;) {
    setSignalColor(c);
    delay(FLASH_DURATION);
    setSignalColor(colors[BLACK]);
    delay(FLASH_PERIOD-FLASH_DURATION);
  }
}

int PercentToPWM(int percentValue)
{
  // Input value between -100 and 100, center on 0
  // Output value between 1100 and 1800, center on 1500
  if (percentValue >= 0) {
    return map(percentValue, 0, 100, ESC_NEUTRAL, ESC_MIN);
  } else {
    return map(percentValue, 0, -100, ESC_NEUTRAL, ESC_MAX);
  }
}

int DampenAtNeutral(int pwmValue, int neutralValue)
{  
  // Prevents jitter around the servo neutral point
  if ((pwmValue > neutralValue) && ((pwmValue - neutralValue) < SERVO_MIDPOINT_TOLERANCE)) {
    return neutralValue;
  }
  
  if ((pwmValue < neutralValue) && ((neutralValue - pwmValue) < SERVO_MIDPOINT_TOLERANCE)) {
    return neutralValue;
  }

  return pwmValue;
}

void setup()
{
  bool ThrottleTriggered = false;   // Used to check if throttle signal is received
  bool SteeringTriggered = false;   // Used to check if steering signal is received
  
  // Configure pins
  pinMode(THROTTLE_IN_PIN, INPUT);            // Attached to RX-701 elevation channel
  pinMode(STEERING_IN_PIN, INPUT);            // Attached to RX-701 aileron channel
  pinMode(LIPO_VOLTAGE_PIN, INPUT_PULLDOWN);  // Attached to voltage divider connected to main voltage
  pinMode(MCU_LED_PIN, OUTPUT);               // Attached to the onboard mcu led
  pinMode(SIGNAL_R_PIN, OUTPUT);              // Attached to signal led via transistor  (something is broken here)
  pinMode(SIGNAL_G_PIN, OUTPUT);              // Attached to signal led via transistor
  pinMode(SIGNAL_B_PIN, OUTPUT);              // Attached to signal led via transistor

  // Signal that the controller is activated
  digitalWrite(MCU_LED_PIN, HIGH);            // Signal that the MCU is active

  // Signal that the controller is booting
  setSignalColor(colors[BLUE]);               // Signal that the controller is booting

#ifdef DEBUG
  // Initialize serial port to 9600 8n1
  Serial.begin(9600);
  delay(1000);
  Serial.println("Controller booting");
#endif

  // Activate the failsave mechanism if a lipo undervoltage condition is detected
  LipoVoltage = READ_VOLTAGE;
  if (LipoVoltage <= LIPO_CUTOFF_VOLTAGE) {
    RunFailsafeLoop(colors[BLUE]);
  }

  // Activate ISRs to read incoming receiver values
  attachInterrupt(digitalPinToInterrupt(THROTTLE_IN_PIN), readThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_IN_PIN), readSteering, CHANGE);

  // Make sure we are connected to the transmitter before continuing bootup
  ReceiverActiveTimer = millis();
  setSignalColor(colors[GREEN]);
  Serial.print("Waiting for receiver to become active");
  while ((!ThrottleTriggered) || (!SteeringTriggered)) {
    if (SharedStateFlags & THROTTLE_FLAG) {
      ThrottleTriggered = true;
    }
    if (SharedStateFlags & STEERING_FLAG) {
      SteeringTriggered = true;
    }

    if ((millis() - ReceiverActiveTimer) > RECEIVER_ACTIVE_FLASH_DURATION) {
      setSignalColor(colors[BLACK]);
    }
    
    if ((millis() - ReceiverActiveTimer) > RECEIVER_ACTIVE_FLASH_PERIOD) {
      setSignalColor(colors[GREEN]);
      ReceiverActiveTimer = millis();
      Serial.print(".");
    }
  }
  Serial.println(" attached to transmitter");

  // Signal that the receiver is connected
  setSignalColor(colors[GREEN]);

  // Initialize timer variables used inside of the mainloop
  ESCUpdateTimer = millis();
  VoltMeterTimer = millis();
  
  // Initialize motors to the neutral position
  CurrentLeftThrottle = 1500;
  CurrentRightThrottle = 1500;

#ifdef DEBUG
  Serial.print("Arming motor controllers: ");
#endif  
  LeftESC.attach(L_ESC_CONTROL_PIN);
  LeftESC.writeMicroseconds(1500);
  RightESC.attach(R_ESC_CONTROL_PIN);
  RightESC.writeMicroseconds(1500);
  delay(1000);
#ifdef DEBUG
  Serial.println("done");
#endif
}

void loop()
{
  static int ThrottleIn;        // Elevation servo value as read from the receiver
  static int SteeringIn;        // Aileron servo value as read from the receiver
  static float ThrottlePercent; // Amount of input throttle, from 100(full forward), 0(neutral) to -100(full backward)
  static float SteeringPercent; // Amount of input steering, from 100(full left), 0(neutral) to -100(full right)
  static float LeftPercent;     // Amount of left throttle, from 100(full forward), 0(neutral) to -100(full backward)
  static float RightPercent;    // Amount of right throttle, from 100(full forward), 0(neutral) to -100(full backward)
  static float CV;              // Used for intermediary calculations
  static float CW;              // Used for intermediary calculations
  static int ThrottleLeft;      // Amount of left throttle, expressed in servo values
  static int ThrottleRight;     // Amount of right throttle, expressed in servo values
  
  uint8_t StateFlags;           // Local copy of ISR status register
  bool ThrottleChanged = false; // State flag used to to determine if the input throttle changed
  bool SteeringChanged = false; // State flag used to determine if the input steering changed

  /* Check if we have updated values for any receiver channels. Disable interrupts while doing so, so we can
   *  safely read the values that are written by ISR's
   */
  if (SharedStateFlags)
  {
    // Disable processing of interrupts
    noInterrupts();

    // Grab a local copy of the ISR state flags
    StateFlags = SharedStateFlags;

    // Update throttle value if changed and signal this
    if (StateFlags & THROTTLE_FLAG)
    {
      ThrottleIn = constrain(ThrottleInShared, THROTTLE_MIN, THROTTLE_MAX);
      ThrottleChanged = true;
    }

    // Update steering value if changed and signal this
    if (StateFlags & STEERING_FLAG)
    {
      SteeringIn = constrain(SteeringInShared, STEERING_MIN, STEERING_MAX);
      SteeringChanged = true;
    }

    // Clear shared status register
    SharedStateFlags = 0;
    interrupts();
  }

  // Convert throttle servo values to percentages
  if (ThrottleChanged) {
    ThrottleIn = DampenAtNeutral(ThrottleIn, THROTTLE_NEUTRAL);
    if (ThrottleIn <= 1500) {
      ThrottlePercent = constrain(map(ThrottleIn, THROTTLE_NEUTRAL, THROTTLE_MIN, 0, 100), 0, 100);
    } else {
      ThrottlePercent = constrain(map(ThrottleIn, THROTTLE_NEUTRAL, THROTTLE_MAX, 0, -100), -100, 0);
    }
  }
  
  // Convert steering servo values to percentages
  if (SteeringChanged) {
    SteeringIn = DampenAtNeutral(SteeringIn, STEERING_NEUTRAL);
    if (SteeringIn <= 1500) {
      SteeringPercent = constrain(map(SteeringIn, STEERING_NEUTRAL, STEERING_MIN, 0, 100), 0, 100);
    } else {
      SteeringPercent = constrain(map(SteeringIn, STEERING_NEUTRAL, STEERING_MAX, 0, -100), -100, 0);
    }
  }

  // Recalculate new values for left and right throttle
  if (ThrottleChanged || SteeringChanged) {
    CV = (100-abs(ThrottlePercent)) * (SteeringPercent / 100) + SteeringPercent;
    CW = (100-abs(SteeringPercent)) * (ThrottlePercent / 100) + ThrottlePercent;
    LeftPercent = (CV - CW) / 2;
    RightPercent = (CV + CW) / 2;

    // Invert left esc value
    if (LeftPercent <= 0) {
      LeftPercent = map(LeftPercent, 0, -100, 0, 100);
    } else {
      LeftPercent = map(LeftPercent, 0, 100, 0, -100);
    }
      
    // Convert the left+right throttle percentages to servo values
    ThrottleLeft = PercentToPWM(LeftPercent);
    ThrottleRight = PercentToPWM(RightPercent);
  }

  // Update the ESC's with new values every 20ms
  if ((millis() - ESCUpdateTimer) > ESC_UPDATE_PERIOD) {
    LeftESC.writeMicroseconds(ThrottleLeft);
    RightESC.writeMicroseconds(ThrottleRight);
    ESCUpdateTimer = millis();
  }

  // Read the voltage of the lipo every second
  if ((millis() - VoltMeterTimer) > VOLTMETER_UPDATE_PERIOD) {
    LipoVoltage = READ_VOLTAGE;
    VoltMeterTimer = millis();
  }

  // Activate failsafe mode if an undervoltage situation occurs.
  if (LipoVoltage <= LIPO_CUTOFF_VOLTAGE) {
#ifdef DEBUG
    Serial.print("LIPO under-voltage condition detected! Measured voltage is ");
    Serial.print(LipoVoltage);
    Serial.println("v");
#endif
    RunFailsafeLoop(colors[BLUE]);
  }
  
  // Finally, clear the local copy of the status register
  StateFlags = 0;
  
#ifdef DEBUG  
  if ((millis() - DebugTimer) > 1000)
  {
    debugMessage = "T:";
    debugMessage += ThrottlePercent;
    debugMessage += ",";
    debugMessage += ThrottleIn;

    debugMessage += "; S:";
    debugMessage += SteeringPercent;
    debugMessage += ",";
    debugMessage += SteeringIn;

    debugMessage += "; L:";
    debugMessage += LeftPercent;
    debugMessage += ",";
    debugMessage += ThrottleLeft;

    debugMessage += "; R:";
    debugMessage += RightPercent;
    debugMessage += ",";
    debugMessage += ThrottleRight;

    debugMessage += "; V:";
    debugMessage += LipoVoltage;
    
    Serial.println(debugMessage);

    DebugTimer = millis();
  }
#endif
}
