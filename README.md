# Arduino Ultrasonic Sensor and DC Motor Control for Mars Task-3

This Arduino project demonstrates how to integrate an ultrasonic sensor with a DC motor using the MD10C motor driver. The project includes code for controlling the motor based on distance measurements from the ultrasonic sensor.

## Components Used

- Arduino board 
- HC-SR04 Ultrasonic Sensor
- MD10C  R3 Motor Driver
- RMCS Motor
- Jumper wires
- Lipo Battery

## Pin Diagram
![Ultrasonic Sensor](https://cdn.shopify.com/s/files/1/0559/1970/6265/files/circuit_diagram_for_interface_the_Ultrasonic_sensor_with_Arduino_UNO_3e1034c9-e321-4055-8cd3-3cf66ff56a26_480x480.png?v=1653473277)

## Board Layout and Specification:
![MD10C Motor Driver](https://github.com/rajrajeshwergupta/Mars-Task-3/assets/70447865/566d3c99-5afe-4e76-8898-9beeb6af584d)

A. Terminal Block (GREEN) - For Vmotor, should be connected to battery power source. Please ensure the voltage polarity
(Positive and Negative) is correctly connected before applying power. MD10C does not come with Reverse Polarity Protection.

| Pin No.                  | Pin name                            | Description   |
|--------------------------|-------------------------------------|---------------|
| 1                        |  V motor Power +                    |Positive Supply (5V to 30VDC)|
| 2                        | V motor Power -                     |Negative Supply or Ground terminal|


B. Terminal Block (Black) - Connect to DC brushed motor, or load.
| Pin No.                  | Pin name                            | Description   |
|--------------------------|-------------------------------------|---------------|
| 3                        |  Motor Output A                   |Connect to motor terminal A|
| 4                        |  Motor Output B                |Connect to motor terminal B|

C. Red LED A – Turns on when the output A is high and output B is low. Indicates the
current flows from output A to B.

D. Red LED B – Turns on when the output A is low and output B is high. Indicates the
current flows from output B to A.

E. Test Button A – When this button is pressed, current flows from output A to B and motor
will turn CW (or CCW depending on the motor connection).

F. Test Button B – When this button is pressed, current flows from output B to A and
motor will turn CCW (or CW depending on the connection).

G. Control Input Signal Pin
| Pin No.                  | Pin name                            | Description   |
|--------------------------|-------------------------------------|---------------|
| 1                        |  GND                                |Logic ground|
| 2                        |  PWM                                |PWM input for speed control|
| 3                        |  DIR                                |Direction control|

H. Green LED – Power LED

## Code
```C++
// Define pins for ultrasonic sensor
const int trigPin = 5; // Trig pin of ultrasonic sensor connected to Arduino pin 9
const int echoPin = 6; // Echo pin of ultrasonic sensor connected to Arduino pin 10

// Define pins for motor control
const int motorPWM = 9; // PWM pin connected to PWM input of MD10C motor driver
const int motorDir = 7; // Direction  pin connected to MD10C motor driver


// Define variables
long duration; // Time taken for the ultrasonic pulse to return
int distance; // Distance measured by the ultrasonic sensor
int pwmSpeed; // PWM speed for the motor

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(trigPin, OUTPUT); // Set trigPin as an output
  pinMode(echoPin, INPUT); // Set echoPin as an input
  
  // Motor control pins as outputs
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);

}

void loop() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send a 10 microsecond pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the duration of the pulse from the echoPin
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance in centimeters
  distance = duration * 0.034 / 2;
  
  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm\tPWM Speed: ");
  Serial.print(pwmSpeed);
  Serial.println("%");
  
  // Motor control based on distance
  if (distance < 5) {
    // Move in direction 1 with 25% speed
    pwmSpeed = 25;
    analogWrite(motorPWM, 64); // 25% of maximum PWM value
    digitalWrite(motorDir, HIGH);
  } else if (distance >= 5 && distance < 10) {
    // Stop the motor
    pwmSpeed = 0;
    analogWrite(motorPWM, 0); // 0% speed
  } else if (distance >= 10 && distance < 20) {
    // Move in direction 2 with 25% speed
    pwmSpeed = 25;
    analogWrite(motorPWM, 64); // 25% of maximum PWM value
    digitalWrite(motorDir, LOW);
  } else if (distance >= 20 && distance < 50) {
    // Move in direction 1 with 50% speed
    pwmSpeed = 50;
    analogWrite(motorPWM, 128); // 50% of maximum PWM value
    digitalWrite(motorDir, HIGH);
  } else if (distance >= 50 && distance < 100) {
    // Move in direction 1 with 100% speed
    pwmSpeed = 100;
    analogWrite(motorPWM, 255); // 100% of maximum PWM value
    digitalWrite(motorDir, HIGH);
  }
  
  // Wait a short delay before the next measurement
  delay(1000);
}

```

## Connections

1. Connect the components according to the provided circuit diagram and board specifications
2. Uploaded the code to Arduino board.
3. Open the serial monitor to view distance measurements and PWM speed.

## Usage

- The ultrasonic sensor measures the distance to an object in front of it.
- Based on the distance measured, the Arduino controls the DC motor using the MD10C motor driver according to the following logic:
  - If distance < 5cm, move in anticlockwise direction with 25% speed.
  - If 5cm < distance < 10cm, stop the motor.
  - If 10cm < distance < 20cm, move in clockwise direction with 25% speed.
  - If 20cm < distance < 50cm, move in anticlockwise direction with 50% speed.
  - If 50cm < distance < 100cm, move in anticlockwise direction with 100% speed.

## Reference

[DOC](https://www.arduino.cc/](https://images-na.ssl-images-amazon.com/images/I/A1TemgvjKjL.pdf)https://images-na.ssl-images-amazon.com/images/I/A1TemgvjKjL.pdf)
[Link](https://robocraze.com/blogs/post/arduino-interfacing-with-ultrasonic-sensor)

