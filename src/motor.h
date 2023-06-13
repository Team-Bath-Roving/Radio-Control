#include "Arduino.h"

class Motor {
	uint8_t mIdx;
	bool invert;
    HardwareSerial & ser;
public:
	Motor(HardwareSerial & ser,uint8_t mIdx,bool invert) : 
        ser(ser), mIdx(mIdx),invert(invert){}
	void setDuty(int value) {
		value=constrain(value,-4095,4095);
		if (invert)
			value=-value;
		ser.printf("M%u D=%i\n",mIdx,value);
	}
};