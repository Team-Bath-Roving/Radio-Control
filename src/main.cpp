#include <Arduino.h>

#include <Ewma.h>
#include <EwmaT.h>
#include <vector>

#include <ServoInput.h>

#include <Adafruit_TinyUSB.h>

#define FILTERING 0.006
#define DEFAULT_THRESHOLD 1.5

#define MAX_CHANGE_TIME 1000 // max time between changes on PWM inputs before controller considered disconnected

ServoInputPin<1> rawLeftDial;
ServoInputPin<2> rawRightDial;
ServoInputPin<3> rawLeftX;
ServoInputPin<4> rawLeftY;
ServoInputPin<5> rawRightY;
ServoInputPin<6> rawRightX;

// Adafruit_USBD_CDC USBSer1;

class channel {
	Ewma filter;
	float previousValue;
	float threshold;
public:
	String name;
	bool change;
	channel(String name, float threshold=DEFAULT_THRESHOLD) : name(name), filter(FILTERING), threshold(threshold) {};
	void update(float angle) {
		filter.filter(angle);
	};
	bool detectChange(float changeThreshold=-1,bool updateVal=true) {
		if (changeThreshold==-1)
			changeThreshold=threshold;
		if (previousValue-filter.output>=changeThreshold || filter.output-previousValue>=changeThreshold) {
			if (updateVal)
				previousValue=filter.output;
			change=true;
		} else {
			change=false;
		}
		return change;
	};
	void printAngle() {
		Serial.print("\"");
		Serial.print(name);
		Serial.print("\"");
		Serial.print(":");
		Serial.print((uint8_t)filter.output);
	};
};

channel LeftDial("LeftDial");
channel RightDial("RightDial");
channel LeftY("LeftY");
channel LeftX("LeftX");
channel RightY("RightY");
channel RightX("RightX");

channel * channels [6] = {
	& LeftDial,
	& RightDial,
	& LeftX,
	& LeftY,
	& RightY,
	& RightX
};

// Pair the device with the controller
void connect() {
	digitalWrite(0,LOW);
	delay(5000);
	digitalWrite(0,HIGH);
}

void setup() {
	// Serial.setStringDescriptor("RC Reciever");
	// Serial.print(Serial.getInterfaceDescriptor()):

	Serial.begin(115200);
	TinyUSBDevice.setID(0x2E8A,0x000B);
	TinyUSBDevice.setManufacturerDescriptor("Team Bath Roving");
	TinyUSBDevice.setProductDescriptor("HKT6A-V2 RX");
	delay(500);
	Serial.println("INFO: Intialised, awaiting controller connection");
}

// void printChannel(String name, float angle) {
//     Serial.print(name);
//     Serial.print(":");
//     Serial.print(angle);
//     Serial.print(",");
//   };


bool connected = false;
unsigned long prevChangeTime=0; // time of last change to PWM inputs
unsigned long printTime=0;
unsigned long sampleTime=0;
void loop() {
	if (micros()-sampleTime>500) {
		sampleTime=micros();
		channels[0]->update(rawLeftDial.getAngle());
		channels[1]->update(rawRightDial.getAngle());
		channels[2]->update(rawLeftX.getAngle());
		channels[3]->update(rawLeftY.getAngle());
		channels[4]->update(rawRightY.getAngle());
		channels[5]->update(rawRightX.getAngle());

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
		if (millis()-prevChangeTime>MAX_CHANGE_TIME) {
			if (connected) {
				connected=false;
				Serial.println("WARN: Radio Control Lost");
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
					chan->printAngle();
					if (printedChanges!=changes)
						Serial.print(",");
					// Serial.print(" ");
				}
			}
			Serial.println("}");

			// Serial.println();
		}
		
	}
}
