const int EN_A=3;
const int IN1_A=22;
const int IN2_A=23;

void setup() {
  pinMode(EN_A, OUTPUT);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
}

void loop(){
  analogWrite(EN_A, 25);

  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, HIGH);
  delay(1500);

  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, LOW);
  delay(100000000);
}
