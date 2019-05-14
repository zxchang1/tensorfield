int pwmPin = 3;
int analogPin = 6;
int supplyVolts = 48;
int motorVolts = 24;
double val = 0;    //48V supply; 0 to 255 PWM. 64 gives 12V to motors, 128 gives 24V
double speed = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  int analogRaw = analogRead(analogPin);
  // Serial.println(analogRaw);
  val = (double(motorVolts) / supplyVolts) * (double(analogRaw) / 1024) * 255;
  Serial.println("Motor Voltage= ");
  Serial.println(supplyVolts*val/255);
  Serial.println("PWM Value =");
  Serial.println(int(val));
  // The motor decoupling capacitors are charging up and powering the motors with > than pwm value. We need to include 0.16 fudge factor to calibrate.
  analogWrite( pwmPin, 0.16*int(val));
  
  delay(1000);



}
