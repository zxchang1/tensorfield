/*Reads temperatures from thermistor and voltage divider circuit, then sends PWM to control the
 * heaters to maintain aluminium block within specific temperature range.
*/

#include <math.h>

double setTemp = 180;
int H_Left = 3;
int H_Right = 5;
int H_Nozzle = 6;

void setup () {
	Serial.begin(9600);
	pinMode(H_Left, OUTPUT);
	pinMode(H_Right, OUTPUT);
	pinMode(H_Nozzle, OUTPUT);

}

void loop () {
	
	double T_BL = analogToTemp(averageRead(0)); //temp of back-left thermistor
	double T_FL = analogToTemp(averageRead(1));
	double T_BR = analogToTemp(averageRead(2));
	double T_FR = analogToTemp(averageRead(3));
	double T_NZ = analogToTemp(averageRead(4)); //temp of nozzle block

	String buf;
	buf += F("\nT_BL = ");
	buf += String(T_BL, 4);
	
	buf += F("\nT_FL = ");
	buf += String(T_FL, 4);

	buf += F("\nT_BR = ");
	buf += String(T_BR, 4);

	buf += F("\nT_FR = ");
	buf += String(T_FR, 4);

	buf += F("\nT_NZ = ");
	buf += String(T_NZ, 4);

	Serial.println(buf);

	if (max(T_BL, T_FL) >= setTemp) {
		analogWrite(H_Left, 0);
		Serial.println("H_Left OFF");
	}
	if (max(T_BL, T_FL) < setTemp) {
		analogWrite(H_Left, 120);
		Serial.println("H_Left ON");
	}

	if (max(T_BR, T_FR) >= setTemp) {
		analogWrite(H_Right, 0);
		Serial.println("H_Right OFF");
	}
	if (max(T_BR, T_FR) < setTemp) {
		analogWrite(H_Right, 120);
		Serial.println("H_Right ON");
	}

	if (T_NZ >= setTemp) {
		analogWrite(H_Nozzle, 0);
		Serial.println("H_Nozzle OFF");
	}
	if (T_NZ < setTemp) {
		analogWrite(H_Nozzle, 120);
		Serial.println("H_Nozzle ON");
	}

	delay(500);
}

double averageRead(int pin){
	int samples = 10;
	double accumulator = 0;
	for (int i=1; i<=samples; i++){
		accumulator += analogRead(pin);
	}
        double average = accumulator/samples;
	return average;	
}

double analogToTemp(long pinLevel){
	double voltage = pinLevel * (5.0 / 1024);
	long beta = 3950;
	long R0 = 100000;
	long Rd = 1800;
	double Rinf = R0 * exp(-1*beta/(25+273.15));

	double T = beta/(log(Rd*((5/voltage)-1)/Rinf)) - 273.15;
	return T;
}



