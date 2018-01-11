#include <boarddefs.h>
#include <ir_Lego_PF_BitStreamEncoder.h>
#include <IRremote.h>
#include <IRremoteInt.h>

#include <Servo.h>                   // include the stock "Servo" library for use in this sketch
Servo myservo;                       // create a new Servo object called myservo

// Constants
static int totalEncoderCounts = 140; // the number of magnetic counts that the encoder reeds
static byte servoDiveCommand = 0;    // this is the angle value that the dive method sends to the servo
static byte servoRiseCommand = 180;  // this is the angle value that the rise method sends to the servo
static byte countsPrev = 6;          // state change on the reed switch
float upperDepth = 12;               //in inches (72 = 6 feet) threshhold for top
float lowerDepth = 84;               //in inches (72 = 6 feet) threshhold for botom

// Pins 
static byte SERVO_PIN = 10;          // the pin that the "continuous rotation servo" is attached to, this motor drives the buoyancy engine
static byte DIVE_STOP = 11;          // the pin that the dive limmit switch (round push button) is attached to
static byte encoderPin =12;          // the magnetic rotery encoder pin
static byte pausePin = 6;            // pin used to send pause comand
static byte RECV_PIN = 2;            // IR reciever signal pin
static byte IR_GND = 3;              // IR middle pin, ground
static byte IR_PWR = 4;              // IR power pin
int PRESSURE_SENSOR = A2;            // the PRESSURE_SENSOR

static byte RED_LED = 9;             // these are the three pins that the RED
static byte GREEN_LED = 6;           //                                   GREEN
static byte BLUE_LED = 5;            //                               and BLUE LED cathodes are attached to
static byte LED_BASE = 7;            // this is the pin that the "common anode" of the RGB LED is attached to

// IR definitions
IRrecv irrecv(RECV_PIN);              // all IR commands and coresponding remote buttons
decode_results results;
#define PAUSE 0xFD807F 
#define ONE 0xFD08F7
#define TWO 0xFD8877
#define THREE 0xFD48B7
#define UP 0xFDA05F
#define DOWN 0xFDB04F
#define LEFT 0xFD10EF
#define RIGHT 0xFD50AF


void setup() {                       // begin setup method
  Serial.begin(9600);                // fire up the serial port. This allows us to print values to the serial console
  IRsetup();                         // Start the Infa-Red reciever
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

  delay(50);                        // wait for 0.2 sec
}                                    // end setup method

void loop(){                   // begin main loop
 
 
 if((getDepth())<=(upperDepth)){      // the curent deapth is at a smaller number than the limit deapth
  dive();                             // DIVE-DIVE-DIVE: Run the "dive" method. This will start turning the servo to take in water & pitch the glider down
 }
 
 if((getDepth())>=(lowerDepth)){      //
  rise(totalEncoderCounts);           // Rise: Run the "rise" method. This will start turning the servo to push out water & pitch the glider up
}
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

boolean checkEncoder(){                         // looks for magnetic signal from encoder wheel
  if (!digitalRead(encoderPin)){                // if magnetic force felt
    return 1;                                   // give a 1 or posotive
  }
  else{                                         // if no magnetic force felt
    return 0;                                   // give a 0 or null or negative
  }
}

void rise(int cnts){      // , byte cnts){                  // Rise: Run the "rise" method. This will start turning the servo to push out water & pitch the glider up
  ledRGB_Write(0, 200, 0);                      // set LED to GREEN to indicate that the glider is rising
  Serial.println("rising");                     // print status change to the serial port
  myservo.attach(SERVO_PIN);                    // attaches the servo on SERVO_PIN to the servo object
  myservo.write(servoRiseCommand);              // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
  boolean previousState = checkEncoder();
  int count = 1;
  while (count <= (cnts)){                      // keep checking to see if the RISE_STOP_SENSOR reading is < the riseStopThreshold
    if (previousState != checkEncoder()){
      count++;
      previousState = !previousState;
      digitalWrite(13, previousState);
      delay(1);
    }
    if (checkIR()){
      myservo.attach(SERVO_PIN);                // attaches the servo on SERVO_PIN to the servo object
      myservo.write(servoRiseCommand);          // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
      ledRGB_Write(0, 200, 0);                  // set LED to GREEN to indicate that the glider is rising
    }
    
    // wait...                                  // just keep checking, when the sensor sees the edge, continue to the next line
  }
  myservo.detach();                             // stop the servo, detaches the servo on SERVO_PIN from the servo object
  Serial.println("coasting (rise)");            // print status change to the serial port
  ledRGB_Write(0, 0, 255);                      // set LED to BLUE to indicate that the glider is coasting in a rise
}                                               // end of method

void ledRGB_Write(byte R, byte G, byte B){      // This method takes care of the details of setting a color and intensity of the RGB LED
  analogWrite(RED_LED, 255-R);                  // These are backwards because you write low values to turn these LEDs on
  analogWrite(GREEN_LED, 255-G);                // This method reverses the counterintuitive nature of the LEDs
  analogWrite(BLUE_LED, 255-B);                 // If using common anode rather than common anode LEDs remove the "255-"es
}                                               // end of method

void IRsetup(){
  irrecv.enableIRIn();                           // alow to reseve IR signal
  pinMode(IR_GND, OUTPUT);                       // sets to OUTPUT
  pinMode(IR_PWR, OUTPUT);                       // sets to OUTPUT
  digitalWrite(IR_GND, 0);                       // sets pin IR_GND to 0
  digitalWrite(IR_PWR, 1);                       // sets pin IR_PWR to 1
}

boolean checkIR(){                                    // gets signal from IR
      if (checkPause()){                                  // checks the checkPause comand line
        myservo.detach();                                 // detach servo
        ledRGB_Write(9, 0, 20);                           // RGB color
        boolean paused = true;                            // if pause is true
        while (paused){                                   // what to do when PAUSED
          if (irrecv.decode(&results)) {                  // if the IR sensor finds signal decode
            if (results.value == PAUSE){                  // if value of signal is equal to the pause function
                Serial.println("PLAY");                   // write play in serial monitor
                delay(100);                               // 0.1s
                paused = false;                           // if not stop
            }
            if (results.value == UP){                     // if IR signal is for the UP command
              Serial.println("up");                       // write up
              rise(20);                                   // rise command 20 times
              flash(1, 50);                               // flash LED
            }
            if (results.value == ONE){                    // if IR signal is the ONE command
                Serial.println("1");                      // write 1
                flash(1, 200);                            // flash LED
               // dive();
                rise(1);                                  // rise comand 1 time
            }
            if (results.value == TWO){                    // if IR signal is the TWO command
                Serial.println("2");                      // write 2
                flash(2, 150);                            // flash LED
                rise(10);                                 // rise command 10 times
               // delay(150);
               // dive();
               // rise(3);
            }
            if (results.value == THREE){                   // if IR signal is the THREE command
                Serial.println("3");                       // write 3
                flash(3, 150);                             // flash LED
                dive();                                    // dive command untill otherwise told
                delay(150);                                // a 0.150s delay
                rise(75);                                  // rise command 75 times
            }
            if (results.value == RIGHT){                   // if IR signal is the RIGHT command
                Serial.println("Right");                   // write Right
                rise(60);                                  // rise command 60 times
            }
            irrecv.resume();                               // start looking for IR signals again
          }
        }
        ledRGB_Write(60, 50, 50);                          // RGB color
        return true;                                       // mark as true if true
      }
      else{                                                // eanything else
        return false;                                      //mark false if false
      }
    }

    boolean checkPause(){                                   // looks for the pause comand
      if (irrecv.decode(&results)) {                        // decodes the ir signal
        if (results.value == PAUSE) {                       // if signal is equal to pause
          Serial.println("PAUSE");                          // prints pause
          irrecv.resume();                                  // starts to look for IR signals again
          return true;                                      // say if this hapend its true
        }
        else{                                               // if did not equal pause
          irrecv.resume();                                  // start looking for IR signal again
          return false;                                     // if not pause tell us false
        }
      }
    }

    void flash(int flashes, int time){                      // command flash and reletive neded input
      for(int i = flashes; i > 0; i--){                     // first number is number of flashes next number is time of flashes. EX flash(30,2);
        digitalWrite(13, 1);                                // write to pin 13 a 1 and/or true
        delay(time);                                        // stop for number that intiger (time) represents
        digitalWrite(13, 0);                                // wright to pin 13 a 0 and or false
        delay(time);                                        //stop for number that intiger (time) represents
      }
    }

 float getDepth() {                                      // math formula to calculate depth of the seaglide
      //
      int raw = analogRead(PRESSURE_SENSOR);                // raw number reed from preshure sencer
      return (float) (1.59625 * raw) - 380.975;             // the formula to calculate preshure
    }
