/**
   Mohammed Al-Ameen

   NodeMCU ESP8266EX motor driver code using L298n driver board and two JGA25-370 motors that come with gearbox and hall sensor for motor encoding.
   In my case only the yellow encoder was working for both motors, and so only accounted for the yellow encoder in the code. The program can
   detect forward and reverse spin and update odometry accordingly, however, it can't detect spin direction when there is
   manual spinning of the wheel since there's only one hall sensor working.

   No warranties or guarantees whatsoever, use at your own risk. tip, make sure you have short circuit protection at your power source.
*/
#include <Ticker.h>

// constants

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
const int lencoder = 3; // this is the yellow wire

// Motor B connections (right wheel)
const int in3 = 5;
const int in4 = 4;
const int rencoder = 0;

//  odometry values
volatile long int lencoder_pos = 0;
volatile long int rencoder_pos = 0;

// these 4 flags help us determing odomotry direction with just one hall sensor instead of two
// when all of them are false then both motors are stopped.
// can never have both left forward and left reverse as true at the same time, if so, then there's something wrong
bool is_left_forward = false;
bool is_right_forward = false;
bool is_left_reverse = false;
bool is_right_reverse = false;

// controller params
String cmd;
const int  gear_ratio = 21;
const int ticks_rev =  11; // encoder ticks per wheel rev
const int pulses_per_rev = gear_ratio * ticks_rev;
const long wheel_diameter = 65000; // wheel diameter in micrometers, so 6.5 cm
const long radius = wheel_diameter / 2;
const long distance_per_rev = 204000; //20.4 cm convered per wheel reveloution put in micrometers
const long distance_per_pulse = distance_per_rev / pulses_per_rev; // every encoder tick we conver this distance
// const long timer_dt = 2500000; // this translates to 0.5sec aborted tmr1 interferes with pwm generation
//const double dt = 0.5; // for easier calc, dont change this alone, u have to set the timer val above first
volatile unsigned long previous_time = 0;
volatile int ledState = LOW;
volatile unsigned long next;
Ticker timer;
const int interval = 200; // 200ms or 0.2sec

// optional parameters
const int turn_delay = 0;
const int spin_direction_change_delay = 0;
const int debug_delay = 100;
const bool enable_debug_msgs = false;
const bool enable_debug_odometer_msgs = false;
bool unknown_right_spin_dir = false;
bool unknown_left_spin_dir = false;
const long approx_time = 8388606; // 80MHZ assuming prescaler so 80cpu beats per 1us, so that num will give us 0.1sec approx if prescaler assumption is good.
const long offset = 9500;


/* interrupts ISR */
void ICACHE_RAM_ATTR send_feedback_to_PID() {
  /* sends encoder feedback to the dragonboard, which sends it to the PID ros node*/
  if (enable_debug_msgs){Serial.println("In the ISR baby");}
  unsigned long current_time = millis();
  if (current_time - previous_time >= interval) {
    long dt = current_time - previous_time;
    led_control();
    send_feedback(dt);
    previous_time = current_time;
    if (enable_debug_msgs){Serial.print("delta t us:");Serial.println(dt);}
  }
  // arm the next interrupt
  next = ESP.getCycleCount() + approx_time;
  timer0_write(next+offset);
}

ICACHE_RAM_ATTR void update_lencoder() {
  if (is_left_motor_going_forward()) {
    lencoder_pos++ ;
    unknown_left_spin_dir = false;
  }
  else if (is_left_motor_going_backward()) {
    lencoder_pos--;
    unknown_left_spin_dir = false;
  }
  else {
    //unknown_left_spin_dir = true; // false positives happen on brake
    lencoder_pos++ ;
  }

  if (enable_debug_msgs && enable_debug_odometer_msgs) {
    Serial.print(" Left odometer ");
    Serial.println(lencoder_pos);
    if (unknown_left_spin_dir) {
      Serial.println("Warning only 1 hall sensor can't determine spin direction on manuel spin, plz use software commands to spin the wheels");
    }
  }
}


ICACHE_RAM_ATTR void update_rencoder() {
  if (is_right_motor_going_forward()) {
    rencoder_pos++ ;
    unknown_right_spin_dir = false;
  }
  else if (is_right_motor_going_backward()) {
    rencoder_pos--;
    unknown_right_spin_dir = false;
  }
  else {
    // unknown_right_spin_dir = true; // false positives happen on brake
    rencoder_pos++ ;
  }
  if (enable_debug_msgs && enable_debug_odometer_msgs) {
    Serial.print(" Right odometer ");
    Serial.println(rencoder_pos);
    if (unknown_right_spin_dir) {
      Serial.println("Warning only 1 hall sensor can't determine spin direction on manuel spin, plz use software commands to spin the wheels");
    }
  }
}

void setup() {
  Serial.begin(115200);
  // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(lencoder, INPUT);
  pinMode(rencoder, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Turn off motors, Initial state
  turn_off_motors_force();
  //direction_test();

  attachInterrupt(digitalPinToInterrupt(lencoder), update_lencoder, RISING);
  attachInterrupt(digitalPinToInterrupt(rencoder), update_rencoder, RISING);

  // aborted using timer1 as it interferes with PWM genration will use timer0
  // timer1 stuff, set it to 0.5s https://www.visualmicro.com/page/Timer-Interrupts-Explained.aspx
  //  timer1_attachInterrupt(send_feedback_to_PID); // Add ISR Function
  //  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  //  timer1_write(timer_dt);

  // timer 0 stuff example from here https://hackaday.io/project/12818-not-grbl/log/47580-esp8266-timer0-and-isr
  // btw timer0 stuff is hacky, it is an internal timer that is not ment to be configed, that's why i gotta count clock cycles, i think at least.
  
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(send_feedback_to_PID);
  next = ESP.getCycleCount() + approx_time;
  timer0_write(next+offset);
  interrupts();
  

}

void loop() {
 // printf("in the loop \n");
 // Serial.println("HEYYYY ");
//  /set_right_pwm(MAXPWM);
  //led_control();
  test_motor_drive();// enocder feedback interrupt doesnt work when pwm is going cuz they use the same timer
  //send_feedback();
  /*
    String myString = "7300,2250\n";
    myString.trim();
    int commaIndex = myString.indexOf(',');
    int vr = myString.substring(0, commaIndex).toInt();
    int vl = myString.substring(commaIndex + 1).toInt();
    vr = map(vr, 0, 10000, MINPWM, MAXPWM);
    vl = map(vl, 0, 10000, MINPWM, MAXPWM);
    Serial.println(vr);
    Serial.println(vl);
  */
}

void receive_velocity_cmd() {
  // takes velocity command from db and updates pwm for motors
  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();
    int commaIndex = cmd.indexOf(',');
    int vr = cmd.substring(0, commaIndex).toInt();
    int vl = cmd.substring(commaIndex + 1).toInt();
    Serial.println(vr);
    Serial.println(vl);
  }
}

void send_feedback(long dt) {
  // sends feedback to the dragonboard with serial
  long distance_right_wheel = rencoder_pos * distance_per_pulse;
  long distance_left_wheel = lencoder_pos * distance_per_pulse;
  long vel_right = distance_right_wheel / dt;
  long vel_left = distance_left_wheel / dt;
  if (enable_debug_msgs) {
    Serial.print("wheels covered distance in micrometers, right then left: \t");
    Serial.print(distance_right_wheel); Serial.print("\t");
    Serial.print(distance_left_wheel); Serial.print("\n");
  
    Serial.print("wheels vel micrometer/mili_sec, right then left: \t");
    Serial.print(vel_right); Serial.print("\t");
    Serial.print(vel_left); Serial.print("\n");
  } else { 
    // this print below is for talking with dragonboard, not for debugging
    Serial.print("dx_vel_r_l: ");
    Serial.print(distance_right_wheel); Serial.print(" ");
    Serial.print(distance_left_wheel); Serial.print(" ");
  
    //Serial.print("vel:");
    Serial.print(vel_right); Serial.print(" ");
    Serial.print(vel_left); Serial.print("\n");
    
    }

  
  
  reset_encoder();
}

void reset_encoder() {
  lencoder_pos = 0;
  rencoder_pos = 0;
}

void send_encoder_ticks() {
  Serial.print(lencoder_pos); Serial.print("\t");
  Serial.print(rencoder_pos); Serial.print("\n");
}

void led_control() {
  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
  digitalWrite(LED_BUILTIN, ledState);
}



void test_motor_drive() {
  // testing crtieria: must perform all tests without shortcircuiting the source, it is recommended that you
  // use shortcircuit protection, if you encounter shortcircuit then increase the number that subtracts from MAXPWM variable

  if (enable_debug_msgs) {
    printf("\n Motor driver test begins \n");
  }
  turn_test();
  direction_test();
  speed_test();
  delay(4000);
  if (enable_debug_msgs) {
    printf("\n Finished motor driver test \n");
  }
}

void direction_test() {
  // test direction sping of each wheel

  if (enable_debug_msgs) {
    printf("\nnew direction test\n");
  }
  const int move_delay = debug_delay * 3;

  // forward
  if (enable_debug_msgs) {
    printf("right wheel forward\n");
  }
  move_right_wheel_forward(MAXPWM);
  delay(move_delay );
  turn_off_motors();
  delay(move_delay);

  if (enable_debug_msgs) {
    printf("left wheel forward\n");
  }
  move_left_wheel_forward(MAXPWM);
  delay(move_delay );
  turn_off_motors();
  delay(move_delay);

  //  reverse
  if (enable_debug_msgs) {
    printf("right wheel backward\n");
  }
  move_right_wheel_backward(MAXPWM);
  delay(move_delay );
  turn_off_motors();
  delay(move_delay);

  if (enable_debug_msgs) {
    printf("left wheel backward\n");
  }
  move_left_wheel_backward(MAXPWM);
  delay(move_delay );
  turn_off_motors();
  delay(move_delay);

  // Turn right
  if (enable_debug_msgs) {
    printf("Turn right\n");
  }
  move_right(MAXPWM);
  delay(move_delay);
  turn_off_motors();
  delay(move_delay);

  // Turn left
  if (enable_debug_msgs) {
    printf("Turn left\n");
  }
  move_left(MAXPWM);
  delay(move_delay);
  turn_off_motors();
  delay(move_delay);
}


void speed_test() {
  // test speed of the motors
  if (enable_debug_msgs) {
    printf("\n new speed test\n");
  }
  // Accelerate from min speed to max speed
  if (enable_debug_msgs) {
    printf("Accelerating \n");
  }
  for (int i = MINPWM; i < MAXPWM; i++) {
    move_forward(i);
    delay(20);
  }
  if (enable_debug_msgs) {
    printf("Max speed \n");
  }
  delay(debug_delay * 2);

  // Decelerate from max speed to min speed
  if (enable_debug_msgs) {
    printf("Deceleratting \n");
  }
  for (int i = MAXPWM; i >= 500; --i) {
    move_forward(i);
    delay(20);
  }
  if (enable_debug_msgs) {
    printf("Min speed \n");
  }
  delay(debug_delay * 2);
}

void turn_test() {
  // tests the ability of the robot to turn from right to left, and then from left to right. it should do so without short circuiting the power source.
  if (enable_debug_msgs) {
    printf("\nnew turn_test \n \n");
  }
  if (enable_debug_msgs) {
    printf("To the right\n");
  }
  set_right_wheel_spin_backward();
  set_left_wheel_spin_forward();
  if (enable_debug_msgs) {
    printf("Finished setting control pins, will set pwm now\n");
  }
  delay(debug_delay);
  set_right_pwm(MAXPWM);
  set_left_pwm(MAXPWM);
  delay(debug_delay * 3);
  turn_off_motors_force();
  if (enable_debug_msgs) {
    printf("Finished right\n");
  }
  delay(debug_delay * 2);

  if (enable_debug_msgs) {
    printf("To the left\n");
  }
  set_left_wheel_spin_backward();
  set_right_wheel_spin_forward();
  if (enable_debug_msgs) {
    printf("Finished setting control pins, will set pwm now\n");
  }
  delay(debug_delay);
  set_right_pwm(MAXPWM);
  set_left_pwm(MAXPWM);
  delay(debug_delay * 3);
  turn_off_motors_force();
  if (enable_debug_msgs) {
    printf("Finished left\n");
  }
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
  is_right_forward = true;
  is_right_reverse = false;
  set_right_pwm(pwm_val);
}

void move_left_wheel_forward(int pwm_val) {
  set_left_wheel_spin_forward();
  is_left_forward = true;
  is_left_reverse = false;
  set_left_pwm(pwm_val);
}

void move_right_wheel_backward(int pwm_val) {
  set_right_wheel_spin_backward();
  is_right_forward = false;
  is_right_reverse = true;
  set_right_pwm(pwm_val);
}

void move_left_wheel_backward(int pwm_val) {
  set_left_wheel_spin_backward();
  is_left_forward = false;
  is_left_reverse = true;
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
  if (enable_debug_msgs) {
    printf("reverse \n");
  }
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
  if (enable_debug_msgs) {
    printf("forward \n");
  }
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

bool is_right_motor_going_forward() {
  // checks only the software config for motor, doesn't take feedback from hardware sensors
  // so doesnt check of the motor is actually spinning or not, or if there's a hardware break.
  if (is_right_forward && !is_right_reverse) {
    return true;
  }
  return false;

}

bool is_right_motor_going_backward() {
  // checks only the software config for motor, doesn't take feedback from hardware sensors
  // so doesnt check of the motor is actually spinning or not, or if there's a hardware break.
  if (!is_right_forward && is_right_reverse) {
    return true;
  }
  return false;

}

bool is_left_motor_going_forward() {
  // checks only the software config for motor, doesn't take feedback from hardware sensors
  // so doesnt check of the motor is actually spinning or not, or if there's a hardware break.
  if (is_left_forward && !is_left_reverse) {
    return true;
  }
  return false;

}

bool is_left_motor_going_backward() {
  // checks only the software config for motor, doesn't take feedback from hardware sensors
  // so doesnt check of the motor is actually spinning or not, or if there's a hardware break.
  if (!is_left_forward && is_left_reverse) {
    return true;
  }
  return false;

}

void turn_off_motors() {
  // turns off by motor control pins, but not by PWM
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  is_left_forward = false;
  is_right_forward = false;
  is_left_reverse = false;
  is_right_reverse = false;
}

void turn_off_motors_force() {
  // turns off pwm power, and turns off the control pins.
  turn_off_motors();
  set_left_pwm(0);
  set_right_pwm(0);
}