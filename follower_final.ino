
// Define pin connections
const int trigPin2 = 2;  // Trig pin of sensor 1
#define echoPin2 A3   // Echo pin of sensor 1
const int trigPin1 = 4;  // Trig pin of sensor 2
const int echoPin1 = 5;  // Echo pin of sensor 2
// Define motor control pins for Motor 1 and Motor 2
#define motor1PinB 3
#define motor1PinA 9
const int motor2PinB = 11;
const int motor2PinA = 10;

const int whiteled1 = 13;
const int whiteled2 = 12;

#define enable1 A1
#define enable2 A2 

// Initialize PID parameters for Motor 1 and Motor 2
double Kp1 = 15.0;  // Proportional constant for Motor 1
double Ki1 = 0.0;   // Integral constant for Motor 1
double Kd1 = 5.0;   // Derivative constant for Motor 1
double Kp2 = 15.0;  // Proportional constant for Motor 2
double Ki2 = 0.0;   // Integral constant for Motor 2
double Kd2 = 5.0;   // Derivative constant for Motor 2
int targetDistance1 = 20;  // Target distance for Motor 1
int targetDistance2 = 20;  // Target distance for Motor 2

int scalePid1;
int scalePid2;

// Define variables
long duration1, distance1, duration2, distance2; // Variables to store sensor readings
double error1, last_error1 = 0, integral1 = 0, derivative1, pid_output1;
double error2, last_error2 = 0, integral2 = 0, derivative2, pid_output2;

void setup() {
  // Setup code, initialize motor pins, etc.
  Serial.begin(9600);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(motor1PinA, OUTPUT);
  pinMode(motor1PinB, OUTPUT);
  pinMode(motor2PinA, OUTPUT);
  pinMode(motor2PinB, OUTPUT);

  pinMode(whiteled1, OUTPUT);
  pinMode(whiteled2, OUTPUT);

  pinMode(enable1, OUTPUT);
  pinMode(enable2, OUTPUT);

  // Set the motor pins to LOW (stopped state)
  digitalWrite(motor1PinA, LOW);
  digitalWrite(motor1PinB, LOW);
  digitalWrite(motor2PinA, LOW);
  digitalWrite(motor2PinB, LOW);

  // Start PWM with 0% duty cycle (stopped state)
  analogWrite(motor1PinA, 0);
  analogWrite(motor2PinA, 0);
  digitalWrite(enable1,HIGH);
  digitalWrite(enable2,HIGH);
}

void loop() {
  // try{
  //digitalWrite(whiteled, HIGH);

  // Read sensor data for sensor 1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = duration1 * 0.034 / 2;

  // Read sensor data for sensor 2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2 * 0.034 / 2;

  // Compute PID output for Motor 1
  error1 = targetDistance1 - distance1;
  error1*=-1;
  integral1 += error1;
  derivative1 = error1 - last_error1;
  pid_output1 = Kp1 * error1 + Kd1 * derivative1;
  last_error1 = error1;
  if (pid_output1 < -200){
    pid_output1 = -200;
  }
  if (pid_output1 > 200){
    pid_output1 = 200;
  }
  scalePid1 = map(pid_output1, -200, 200, -255, 255);
 // digitalWrite (whiteled1, abs(scalePid1));

 // Compute PID output for Motor 2
  error2 = targetDistance2 - distance2;
  error2*=-1;
  integral2 += error2;
  derivative2 = error2 - last_error2;
  pid_output2 = Kp2 * error2 + Kd2 * derivative2;
  last_error2 = error2;
  if (pid_output2 < -200){
    pid_output2 = -200;
  }
  if (pid_output2 > 200){
    pid_output2 = 200;
  }
  scalePid2 = map(pid_output2, -200, 200, -255, 255);
 // digitalWrite (whiteled2, abs(scalePid2));

  analogWrite(enable1, 128);
  analogWrite(enable2, 128);

if((error1<-15 || error2<-15)||(error1>15 || error2>15)){
  digitalWrite(motor1PinA,LOW);
  digitalWrite(motor1PinB,LOW);
  digitalWrite(motor2PinA,LOW);
  digitalWrite(motor2PinB,LOW);

  digitalWrite(whiteled1, LOW);
  digitalWrite(whiteled2, LOW);
  }
 else{
  // Control Motor 1 using the PID output
  // Adjust motor speed based on pid_output1 using PWM
  if (scalePid1 > 0) {
    //digitalWrite(motor1PinA, HIGH);
    //digitalWrite(motor1PinB, LOW);
    analogWrite(motor1PinA, scalePid1);//multiplied by a scaling factor
    //analogWrite(motor1PinB, 0);
    digitalWrite(motor1PinB, LOW);
    digitalWrite (whiteled1, abs(scalePid1));
  } else {
    //digitalWrite(motor1PinA, LOW);
    //digitalWrite(motor1PinB, HIGH);
    analogWrite(motor1PinB, (-scalePid1));
    //analogWrite(motor1PinA, 0);
    digitalWrite(motor1PinA, LOW);
    digitalWrite (whiteled1, abs(scalePid1));
  }
//digitalWrite(whiteled,LOW);

  // Control Motor 2 using the PID output
  // Adjust motor speed based on pid_output2 using PWM
  if (scalePid2 > 0) {
    //digitalWrite(motor2PinA, HIGH);
    //digitalWrite(motor2PinB, LOW);
    analogWrite(motor2PinA, scalePid2);//multiplied by a scaling factor
    //analogWrite(motor2PinB, 0);
    digitalWrite(motor2PinB, LOW);
    digitalWrite (whiteled2, abs(scalePid2));
  } else {
    //digitalWrite(motor2PinA, LOW);
    //digitalWrite(motor2PinB, HIGH);
    analogWrite(motor2PinB, -scalePid2);
    //analogWrite(motor2PinA, 0);
    digitalWrite(motor2PinA, LOW);
    digitalWrite (whiteled2, abs(scalePid2));
  }
 }
Serial.println(error1);
 // Serial.println();
Serial.println(error2);
  //Serial.println();
  //Serial.println(scalePid1);
  //Serial.println();
  //Serial.println(scalePid2);
  //Serial.println();

  // Delay or use millis() to control the update rate of the PID loop
  delay(20);

  }
  // catch(...)
  // {
  //   aux();
  // }
// }

void aux()
{
  //digitalWrite(whiteled, HIGH);

  // Read sensor data for sensor 1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = duration1 * 0.034 / 2;

  // Read sensor data for sensor 2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2 * 0.034 / 2;

  // Compute PID output for Motor 1
  error1 = targetDistance1 - distance1;
  error1*=-1;
  integral1 += error1;
  derivative1 = error1 - last_error1;
  pid_output1 = Kp1 * error1 + Kd1 * derivative1;
  last_error1 = error1;
  if (pid_output1 < -200){
    pid_output1 = -200;
  }
  if (pid_output1 > 200){
    pid_output1 = 200;
  }
  scalePid1 = map(pid_output1, -200, 200, -255, 255);

  // Compute PID output for Motor 2
  error2 = targetDistance2 - distance2;
  error2*=-1;
  integral2 += error2;
  derivative2 = error2 - last_error2;
  pid_output2 = Kp2 * error2 + Kd2 * derivative2;
  last_error2 = error2;
  if (pid_output2 < -200){
    pid_output2 = -200;
  }
  if (pid_output2 > 200){
    pid_output2 = 200;
  }
  scalePid2 = map(pid_output2, -200, 200, -255, 255);

  analogWrite(enable1, 128);
  analogWrite(enable2, 128);

if((error1<-15 || error2<-15)||(error1>10 || error2>10)){
  digitalWrite(motor1PinA,LOW);
  digitalWrite(motor1PinB,LOW);
  digitalWrite(motor2PinA,LOW);
  digitalWrite(motor2PinB,LOW);
  }
 else{
  // Control Motor 1 using the PID output
  // Adjust motor speed based on pid_output1 using PWM
  if (scalePid1 > 0) {
    //digitalWrite(motor1PinA, HIGH);
    //digitalWrite(motor1PinB, LOW);
    analogWrite(motor1PinA, scalePid1);//multiplied by a scaling factor
    //analogWrite(motor1PinB, 0);
    digitalWrite(motor1PinB, LOW);
  } else {
    //digitalWrite(motor1PinA, LOW);
    //digitalWrite(motor1PinB, HIGH);
    analogWrite(motor1PinB, (-scalePid1));
    //analogWrite(motor1PinA, 0);
    digitalWrite(motor1PinA, LOW);
  }

  // Control Motor 2 using the PID output
  // Adjust motor speed based on pid_output2 using PWM
  if (scalePid2 > 0) {
    //digitalWrite(motor2PinA, HIGH);
    //digitalWrite(motor2PinB, LOW);
    analogWrite(motor2PinA, scalePid2);//multiplied by a scaling factor
    //analogWrite(motor2PinB, 0);
    digitalWrite(motor2PinB, LOW);
  } else {
    //digitalWrite(motor2PinA, LOW);
    //digitalWrite(motor2PinB, HIGH);
    analogWrite(motor2PinB, -scalePid2);
    //analogWrite(motor2PinA, 0);
    digitalWrite(motor2PinA, LOW);
  }
 }
Serial.println(error1);
 // Serial.println();
Serial.println(error2);
  //Serial.println();
  //Serial.println(scalePid1);
  //Serial.println();
  //Serial.println(scalePid2);
  //Serial.println();

  // Delay or use millis() to control the update rate of the PID loop
  delay(50);

}
