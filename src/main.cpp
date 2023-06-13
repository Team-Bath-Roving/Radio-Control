#include <Arduino.h>

/* -------------------------------- Libraries ------------------------------- */

#include <Ewma.h>
#include <EwmaT.h>
#include <vector>

#include <ServoInput.h>

#include <Adafruit_TinyUSB.h>

/* --------------------------------- Headers -------------------------------- */

#include "channel.h"
#include "motor.h"

/* -------------------------------- Settings -------------------------------- */

#define MOTOR_SERIAL_TX 26
#define MOTOR_SERIAL_RX 27

#define MOTOR_SERIAL_BAUD 230400
#define USB_SERIAL_BAUD 230400

#define MAX_CHANGE_TIME 1000 // max time between changes on PWM inputs before controller considered disconnected

/* --------------------------------- Objects -------------------------------- */

SerialPIO motorSer(MOTOR_SERIAL_TX,MOTOR_SERIAL_RX);

ServoInputPin<1> rawLeftDial;
ServoInputPin<2> rawRightDial;
ServoInputPin<3> rawLeftX;
ServoInputPin<4> rawLeftY;
ServoInputPin<5> rawRightY;
ServoInputPin<6> rawRightX;

channel LeftDial  ("LeftDial"  ,MODE           );
channel RightDial ("RightDial" ,ENABLE         );
channel LeftY     ("LeftY"     ,DRIVE_OR_ARM  );
channel LeftX     ("LeftX"     ,TURN_OR_BRUSH );
channel RightY    ("RightY"    ,SCOOP_FRONT    );
channel RightX    ("RightX"    ,SCOOP_REAR     );

channel * channels [6] = {
	& LeftDial,
	& RightDial,
	& LeftX,
	& LeftY,
	& RightY,
	& RightX
};

// Payload
Motor Brush     (motorSer,0,false);
Motor Arm       (motorSer,1,false);
Motor ScoopFront(motorSer,2,false);
Motor ScoopRear (motorSer,3,false);
// Propulsion
Motor FrontLeft  (motorSer,4,true );
Motor FrontRight (motorSer,5,true );
Motor RearLeft   (motorSer,6,true );
Motor RearRight  (motorSer,7,false);

Motor * motors [8] = {
	&Brush,     
	&Arm,       
	&ScoopFront,
	&ScoopRear, 
	&FrontLeft, 
	&FrontRight,
	&RearLeft,  
	&RearRight 
};

/* -------------------------------- Variables ------------------------------- */

int driveSpeed=0;
int turnSpeed=0;

bool connected = false;
unsigned long prevChangeTime=0; // time of last change to PWM inputs
unsigned long printTime=0;
unsigned long sampleTime=0;

enum Mode {DRIVE,PAYLOAD};
Mode mode=DRIVE;
Mode prevMode=DRIVE;
bool enabled=false;

/* -------------------------------- Functions ------------------------------- */

void drive() {
	int left=driveSpeed-turnSpeed;
	int right=driveSpeed+turnSpeed;
	FrontLeft.setDuty(left);
	FrontRight.setDuty(right);
	RearLeft.setDuty(left);
	RearRight.setDuty(right);
}
// Pair the device with the controller
void connect() {
	digitalWrite(0,LOW);
	delay(5000);
	digitalWrite(0,HIGH);
}
void stop() {
	for (auto m : motors) {
		m->setDuty(0);
	}
}

/* ---------------------------------- Setup --------------------------------- */

void setup() {
	Serial.begin(USB_SERIAL_BAUD);
	motorSer.begin(MOTOR_SERIAL_BAUD); // gp27 RX
	TinyUSBDevice.setID(0x2E8A,0x000B);
	TinyUSBDevice.setManufacturerDescriptor("Team Bath Roving");
	TinyUSBDevice.setProductDescriptor("HKT6A-V2 RX");
	delay(500);
	Serial.println("INFO: Intialised, awaiting controller connection");
}

/* -------------------------------- Main Loop ------------------------------- */

void loop() {
	if (micros()-sampleTime>500) {
		sampleTime=micros();

		// Check for updates
		channels[0]->update(rawLeftDial.getAngle());
		channels[1]->update(rawRightDial.getAngle());
		channels[2]->update(rawLeftX.getAngle());
		channels[3]->update(rawLeftY.getAngle());
		channels[4]->update(rawRightY.getAngle());
		channels[5]->update(rawRightX.getAngle());

		// Measure sizes of changes to see if significant
		uint8_t changes=0;
		uint8_t smallChanges=0;
		for (auto chan : channels) {
			// Detect if channels have changed at all (if not, RC controller not connected as no noise in signal)
			if (chan->detectChange(0.5,false)) {
				smallChanges++;
			}
			// Detect if channels have changed enough to send new value to rover
			if (chan->detectChange()) {
				changes++;
			}
		}

		// Only set state to connected if multiple channels have changed
		if (smallChanges>3) {
			prevChangeTime=millis();
			if (!connected) {
				connected=true;
				Serial.println("INFO: Radio Control Established");
			}  
		}
		// Check if disconnected, if so disable motors
		if (millis()-prevChangeTime>MAX_CHANGE_TIME) {
			if (connected) {
				connected=false;
				Serial.println("WARN: Radio Control Lost");

				// Disable motors
				stop();
			} 
		}

		// anyChange=true;
		
		uint8_t printedChanges=0;
		if (changes && millis()-printTime>100){
			printTime=millis();    
			Serial.print("{");
			for (auto chan : channels) {
				if (chan->change) {
					printedChanges++;
					chan->printRawAngle();
					if (printedChanges!=changes)
						Serial.print(",");
					// Serial.print(" ");
					switch(chan->action) {
						case MODE:
							prevMode=mode;
							// set new mode
							if (chan->getOutput()>0)
								mode=PAYLOAD;
							else
								mode=DRIVE;
							// check if changed
							if (prevMode!=mode) {
								// if previously in drive mode, stop the wheels
								if (prevMode==DRIVE) {
									driveSpeed=0;
									turnSpeed=0;
									drive();
								}
								// if previously in payload mode, stop the brush and arm
								else {
									Brush.setDuty(0);
									Arm.setDuty(0);
								}
							}
							break;					
						case ENABLE:
							if (chan->getOutput()>0) {
								if (!enabled)
									enabled=true;
							} else {
								if (enabled) {
									enabled=false;
									stop();
								}
							}
							break;
						case DRIVE_OR_ARM:
							if (mode==DRIVE) {
								driveSpeed=chan->getOutput();
								drive();
							}
							else {
								Arm.setDuty(chan->getOutput());
							}
							break;
						case TURN_OR_BRUSH:
							if (mode==DRIVE){
								turnSpeed=chan->getOutput();
								drive();
							}
							else {
								Brush.setDuty(chan->getOutput());
							}
							break;
						case SCOOP_FRONT:
							ScoopFront.setDuty(chan->getOutput());
							break;
						case SCOOP_REAR:
							ScoopRear.setDuty(chan->getOutput());
							break;
					}
				}
			}
			Serial.println("}");

			// Serial.println();
		}
		
	}
}
