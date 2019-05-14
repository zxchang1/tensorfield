int solPin = 3;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(solPin,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i=1; i<=100; i++){
    digitalWrite(solPin,HIGH);
    delay(10);
    digitalWrite(solPin,LOW);
    delay(20);
  }
  Serial.println("Test Cycle Wait...");
  delay(1000);
  

}
