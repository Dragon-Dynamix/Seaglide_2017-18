
#include <IRremote.h>
#include <Servo.h>
Servo Myservo;

// Constants
static int minCoast =  1000;         // if the pot is turned all the way to the counter-clockwise the glider will coast for 1 seccond
static int maxCoast = 30000;         // if the pot is turned all the way to the clockwise the glider will coast for 10 secconds
static int totalEncoderCounts = 149; //
static byte servoDiveCommand = 0;    // this is the angle value that the dive method sends to the servo
static byte servoRiseCommand = 180;  // this is the angle value that the rise method sends to the servo
static int riseDriveTime = 13000;    //
static byte countsPrev = 6;          // state change on the reed switch
float upperDepth = 12;               //in inches (72 = 6 feet) threshhold for top
float lowerDepth = 84;               //in inches (72 = 6 feet) threshhold for botom


//pins
static byte SERVO_PIN = 10;          // pin the servo is on
static byte DIVE_STOP = 11;          // the buton in the silinder to stop the dive rotation
static byte encoderPin =12;          // the magnetic rotery encoder pin
static byte pausePin = 6;            // pin used to send pause comand
static byte POT_PIN = A3;            // the pin that the wiper of the little orange trim pot is attached to
static byte RECV_PIN = 2;            // IR reciever signal pin
static byte IR_GND = 3;              // IR middle pin, ground
static byte IR_PWR = 4;              // positive IR pin
int PRESSURE_SENSOR = A2;            // the PRESSURE_SENSOR

// IR definitions
IRrecv irrecv(RECV_PIN);             // all IR commands and coresponding remote buttons
decode_results results;
#define PAUSE 0xFD807F
#define ONE 0xFD08F7
#define TWO 0xFD8877
#define THREE 0xFD48B7
#define UP 0xFDA05F
#define DOWN 0xFDB04F
#define LEFT 0xFD10EF
#define RIGHT 0xFD50AF


void setup() {
  Serial.begin(9600);                // fire up the serial port. This allows us to print values to the serial console
  // pinMode setup
    IRsetup();                         // Start the Infa-Red reciever
    pinMode(POT_PIN, INPUT);           // initialize the potentiometer, this pot will determine the coast time turn it right to coast longer
    pinMode(SERVO_PIN, OUTPUT);        // initialize the continuous rotation servo, this motor drives the buoyancy engine
    pinMode(DIVE_STOP, INPUT_PULLUP);  // initialize the dive stop switch, and turn on the internal pull-up resistor. This limmit switch is lets the Arduino know when the buoyancy engine reaches the end of its travle in the dive direction
    pinMode(pausePin, INPUT_PULLUP);   // initialize the dive stop switch, and turn on the internal pull-up resistor. This limmit switch is lets the Arduino know when the buoyancy engine reaches the end of its travle in the dive direction
    pinMode(encoderPin, INPUT_PULLUP); // reeds the magnetic pulses
    pinMode(RED_LED, OUTPUT);          // initialise the RED
    pinMode(GREEN_LED, OUTPUT);        //                GREEN
    pinMode(BLUE_LED, OUTPUT);         //            and BLUE pins on the LED
    pinMode(LED_BASE, OUTPUT);         // initialize the common pin of the LED

    // initialize RGB LED
     ledRGB_Write(0, 0, 0);             // set the R, G, & B LEDs to OFF
     digitalWrite(LED_BASE, HIGH);      // set the LED Base pin to HIGH this LED it is a common anode, providing +5V to all 3 LEDs
     readPot(POT_PIN);
     delay(50);                        // wait for 0.2 sec

}

void loop() {
  dive();                      // DIVE-DIVE-DIVE: Run the "dive" method. This will start turning the servo to take in water & pitch the glider down
   delay(readPot(POT_PIN));     // read the pot and delay bassed on it's position, coast
   rise(totalEncoderCounts);  // Rise: Run the "rise" method. This will start turning the servo to push out water & pitch the glider up
   delay(readPot(POT_PIN)*1.1);     // Read the pot and delay bassed on it's position, coast
}

void dive(){                                    // Dive: Run the "dive" method. This will start turning the servo to take in water & pitch the glider down
  ledRGB_Write(255, 0, 0);                      // set LED to RED to indicate that the glider is diving
  Serial.println("diving");                     // print status change to the serial port
  myservo.attach(SERVO_PIN);                    // attaches the servo on "SERVO_PIN" to the servo object so that we can command the servo to turn
  myservo.write(servoDiveCommand);              // drive servo clockwise, take in water & pull weight forward (pull counterweight & plunger towards servo, at the bow of the glider)
  while (digitalRead(DIVE_STOP) == HIGH){       // keep checking the DIVE_STOP pin to see if the button is pressed
    if (checkIR()){
      myservo.attach(SERVO_PIN);                    // attaches the servo on SERVO_PIN to the servo object
      myservo.write(servoDiveCommand);              // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
      ledRGB_Write(255, 0, 0);                      // set LED to RED to indicate that the glider is diving
    }
    // wait...                                  // just keep checking: when the button is pressed, continue to the next line
  }
  myservo.detach();                             // stop the servo, detaches the servo on SERVO_PIN from the servo object
    Serial.println("coasting (dive)");            // print status change to the serial port
    ledRGB_Write(255, 80, 0);                     // set LED to ORANGE to indicate that the glider is coasting in a dive
  }                                               // end of method

  boolean checkEncoder(){                   // looks for magnetic signal from encoder wheel
    if (!digitalRead(encoderPin)){          // if magnetic force felt
      return 1;                             // give a 1 or posotive
    }
    else{                                   // if no magnetic force felt
      return 0;                             // give a 0 or OFF or negative
    }























    void ledRGB_Write(byte R, byte G, byte B){      // This method takes care of the details of setting a color and intensity of the RGB LED
      analogWrite(RED_LED, 255-R);                  // These are backwards because you write low values to turn these LEDs on
      analogWrite(GREEN_LED, 255-G);                // This method reverses the counterintuitive nature of the LEDs
      analogWrite(BLUE_LED, 255-B);                 // If using common anode rather than common anode LEDs remove the "255-"es
    }                                               // end of method
