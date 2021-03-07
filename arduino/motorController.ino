// constantss

// MAXPWM gives us 100% duty cycle on the default setting of the nodeMCU, but I get short circuits when I run the turn_test() function, so I subtract a number
// from maxpwm and this new pwm didn't give me any short circuts, this pwm subtraction can add/remove shortcircuits in my case. the value was found experimatnaly
// // the lower the pwm the more you can hear it humming, both motors had different minimun PWM
// any lower than this value then one of them will hum and won't spin ( using L298n motor drive board)
const int MAXPWM = 1023 - 70;
const int MINPWM = 350;

// pwm doubles for the enable pin
const int left_pwm = 15;
const int right_pwm = 13;

// Motor A connections (left wheel)
const int in1 = 12;
const int in2 = 14;
// Motor B connections (right wheel)
const int in3 = 5;
const int in4 = 4;

// without this delay the control pins short circuit the power source when they flipping flip direction (when you try to go left and right)
// also you can't switch going from forward to back (or opposite) without waiting for motor to stop, otherwise u get a short circuit in the source
const int turn_delay = 0;
const int spin_direction_change_delay = 0;
const int debug_delay = 1000;

void setup() {
  Serial.begin(115200);
  // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors, Initial state
  turn_off_motors_force();
  //direction_test();
}

void loop() {
  test_motor_drive();

}

void test_motor_drive() {
  // testing crtieria: must perform all tests without shortcircuiting the source, it is recommended that you
  // use shortcircuit protection, if you encounter shortcircuit then increase the number that subtracts from MAXPWM variable

  printf("\n Motor driver test begins \n");
  turn_test();
  direction_test();
  speed_test();
  delay(4000);
  printf("\n Finished motor driver test \n");
}

void direction_test() {
  // test direction sping of each wheel

  printf("\nnew direction test\n");
  const int move_delay = debug_delay * 3;

  // forward
  printf("right wheel forward\n");
  move_right_wheel_forward(MAXPWM);
  delay(move_delay );
  turn_off_motors();
  delay(move_delay);

  printf("left wheel forward\n");
  move_left_wheel_forward(MAXPWM);
  delay(move_delay );
  turn_off_motors();
  delay(move_delay);

  //  reverse
  printf("right wheel backward\n");
  move_right_wheel_backward(MAXPWM);
  delay(move_delay );
  turn_off_motors();
  delay(move_delay);

  printf("left wheel backward\n");
  move_left_wheel_backward(MAXPWM);
  delay(move_delay );
  turn_off_motors();
  delay(move_delay);

  // Turn right
  printf("Turn right\n");
  move_right(MAXPWM);
  delay(move_delay);
  turn_off_motors();
  delay(move_delay);

  // Turn left
  printf("Turn left\n");
  move_left(MAXPWM);
  delay(move_delay);
  turn_off_motors();
  delay(move_delay);
}


void speed_test() {
  // test speed of the motors
  printf("\n new speed test\n");
  // Accelerate from min speed to max speed
  printf("Accelerating \n");
  for (int i = MINPWM; i < MAXPWM; i++) {
    move_forward(i);
    delay(20);
  }
  printf("Max speed \n");
  delay(debug_delay * 2);

  // Decelerate from max speed to min speed
  printf("Deceleratting \n");
  for (int i = MAXPWM; i >= 500; --i) {
    move_forward(i);
    delay(20);
  }
  printf("Min speed \n");
  delay(debug_delay * 2);
}

void turn_test() {
  // tests the ability of the robot to turn from right to left, and then from left to right. it should do so without short circuiting the power source.
  printf("\nnew turn_test \n \n");
  printf("To the right\n");
  set_right_wheel_spin_backward();
  set_left_wheel_spin_forward();
  printf("Finished setting control pins, will set pwm now\n");
  delay(debug_delay);
  set_right_pwm(MAXPWM);
  set_left_pwm(MAXPWM);
  delay(debug_delay * 3);
  turn_off_motors_force();
  printf("Finished right\n");
  delay(debug_delay * 2);

  printf("To the left\n");
  set_left_wheel_spin_backward();
  set_right_wheel_spin_forward();
  printf("Finished setting control pins, will set pwm now\n");
  delay(debug_delay);
  set_right_pwm(MAXPWM);
  set_left_pwm(MAXPWM);
  delay(debug_delay * 3);
  turn_off_motors_force();
  printf("Finished left\n");
  delay(debug_delay * 2);
}

///// Motor control functions

// whole car control

void move_forward(int pwm_val) {
  move_right_wheel_forward(pwm_val);
  move_left_wheel_forward(pwm_val);

}

void move_backward(int pwm_val) {
  move_right_wheel_backward(pwm_val);
  move_left_wheel_backward(pwm_val);
}

void move_right(int pwm_val) {
  move_right_wheel_backward(pwm_val);
  delay(turn_delay);
  move_left_wheel_forward(pwm_val);
}


void move_left(int pwm_val) {
  move_right_wheel_forward(pwm_val);
  delay(turn_delay);
  move_left_wheel_backward(pwm_val);
}

// parts control

void move_right_wheel_forward(int pwm_val) {
  set_right_wheel_spin_forward();
  set_right_pwm(pwm_val);
}

void move_left_wheel_forward(int pwm_val) {
  set_left_wheel_spin_forward();
  set_left_pwm(pwm_val);
}

void move_right_wheel_backward(int pwm_val) {
  set_right_wheel_spin_backward();
  set_right_pwm(pwm_val);
}

void move_left_wheel_backward(int pwm_val) {
  set_left_wheel_spin_backward();
  set_left_pwm(pwm_val);
}

void set_left_pwm(int pwm_val) {
  analogWrite(left_pwm, pwm_val);
}

void set_right_pwm(int pwm_val) {
  analogWrite(right_pwm, pwm_val);
}

void set_reverse_spin() {
  // changes the wheel spin direction
  printf("reverse \n");
  set_left_wheel_spin_backward();
  set_right_wheel_spin_backward();
}

void set_left_wheel_spin_backward() {
  // changes the wheel spin direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void set_right_wheel_spin_backward() {
  // changes the wheel spin direction
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void set_forward_spin() {
  // changes the wheel spin direction
  printf("forward \n");
  set_left_wheel_spin_forward();
  set_right_wheel_spin_forward();
}

void set_left_wheel_spin_forward() {
  // changes the wheel spin direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void set_right_wheel_spin_forward() {
  // changes the wheel spin direction
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turn_off_motors() {
  // turns off by motor control pins, but not by PWM
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void turn_off_motors_force() {
  // turns off pwm power, and turns off the control pins.
  turn_off_motors();
  set_left_pwm(0);
  set_right_pwm(0);
}
