#include "Arduino.h"
#include "Ewma.h"

#define FILTERING 0.006
#define DEFAULT_THRESHOLD 1.5

enum Action{DRIVE,TURN,SCOOP_FRONT,SCOOP_REAR,BRUSH,ARM};

class channel {
	Ewma filter;
	float previousValue;
	float threshold;
	uint8_t zero;
	uint deadzone;
	uint range;
public:
	Action action;
	String name;
	bool change;
	channel(String name, Action action, uint8_t zero=127, uint deadzone=6, uint range=4095, float threshold=DEFAULT_THRESHOLD) : 
		action(action),name(name), zero(zero),deadzone(deadzone),range(range),filter(FILTERING), threshold(threshold) {}
	void update(float angle) {
		filter.filter(angle);
	}
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
	}
	int getOutput() {
		int output = map(getRawAngle(),zero,255,0,range);
		if ((output>0 && output<deadzone/2) || (output<0 && output> -deadzone/2) )
			output=0;
		return output;
	}
	void printOutput() {
		Serial.print("\"");
		Serial.print(name);
		Serial.print("\"");
		Serial.print(":");
		Serial.print(getOutput());
	}
	uint8_t getRawAngle() {
		return filter.output;
	}
	void printRawAngle() {
		Serial.print("\"");
		Serial.print(name);
		Serial.print("\"");
		Serial.print(":");
		Serial.print(getRawAngle());
	}
};