# Arduino Ultrasonic Sensor and DC Motor Control for Mars Task-3

This Arduino project demonstrates how to integrate an ultrasonic sensor with a DC motor using the MD10C motor driver. The project includes code for controlling the motor based on distance measurements from the ultrasonic sensor.

## Components Used

- Arduino board 
- HC-SR04 Ultrasonic Sensor
- MD10C  R3 Motor Driver
- RMCS Motor
- Jumper wires

## Circuit Diagram

![Circuit Diagram](![image](https://github.com/rajrajeshwergupta/Mars-Task-3/assets/70447865/566d3c99-5afe-4e76-8898-9beeb6af584d)
)

## Connections

1. Connect the components according to the provided circuit diagram.
2. Upload the provided Arduino sketch (`motor_control_ultrasonic.ino`) to your Arduino board.
3. Open the serial monitor to view distance measurements and PWM speed.

## Usage

- The ultrasonic sensor measures the distance to an object in front of it.
- Based on the distance measured, the Arduino controls the DC motor using the MD10C motor driver according to the following logic:
  - If distance < 5cm, move in anticlockwise direction with 25% speed.
  - If 5cm < distance < 10cm, stop the motor.
  - If 10cm < distance < 20cm, move in clockwise direction with 25% speed.
  - If 20cm < distance < 50cm, move in anticlockwise direction with 50% speed.
  - If 50cm < distance < 100cm, move in anticlockwise direction with 100% speed.

## Contributing

Contributions are welcome! If you have any ideas, suggestions, or improvements, feel free to open an issue or create a pull request.

## License

This project is licensed under the [MIT License](LICENSE).
