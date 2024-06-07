// DC Motor Variables
#define pwm_Pin 7
#define dir_Pin 6
int pwm_value;
int mtr_dir_cmnd;

// Encoder Variables
#define outputA1 20
#define outputB1 21
#define outputA2 18
#define outputB2 19
volatile int counter1 = 0;
volatile int counter2 = -600;

// Swing Up Variables
volatile int counter2_bottom;
volatile int previous_counter2_bottom = 0;
volatile int previous_previous_counter2_bottom = 0;
volatile int n_counter2_bottom;
volatile int n_minus1_counter2_bottom = 0;
volatile int n_minus2_counter2_bottom = 0;
bool left_swing = 1;
unsigned long PID_angle_start_time;
unsigned long PID_angle_running_time = 0;
const float K_P_theta_Swing = 115.0;
const float K_I_theta_Swing = 0.8;
const float K_D_theta_Swing = 3.0;

// Control Variables
float theta;
float cart_pos;
const float counter1_to_pos = 20.0 * .02 / 1200.0;  // counter1 to position in meters
const float counter2_to_theta = 360.0 / 1200.0;     // counter2 to theta in degrees
const float setpoint_theta = 0;           // top dead center rotational position in degrees (180 degrees above still since counter1 starts at -180 degrees on calibration)
const float setpoint_position = 0; // center of gantry defined as intial linear position on startup
float error_theta = 0;
float integral_error_theta;
float derivative_error_theta;
float error_position = 0;
float integral_error_position;
float derivative_error_position;
float output_theta = 0;
float output_position = 0;
float output = 0;
unsigned long current_time;
unsigned long previous_time;
float previous_error_theta;
float previous_error_position;
const float K_P_theta = 70.0;//115.0;  //80.0;//80.0; //
const float K_I_theta = 0.9;//0.5; //0; //
const float K_D_theta = 4.8;//2.0; //8.73;//8.73; //
const float K_P_position = 585.0; //600.0;//300.0;  //50.0; // 100
const float K_I_position = 55.0; //70.0; //0.0;
const float K_D_position = 80.0; //135.0;  //10.0; //20
float dt;

// Limit Switches
#define right_switches_Pin 3
#define left_switches_Pin 2
volatile bool any_switch_triggered = false;
//volatile bool right_switches_triggered = false;
//volatile bool left_switches_triggered = false;


void setup() {
  // DC Motor Setup
  pinMode(pwm_Pin, OUTPUT);
  pinMode(dir_Pin, OUTPUT);

  // Encoders setup
  Serial.begin (57600);
  pinMode (outputA1, INPUT);
  pinMode (outputB1, INPUT);
  pinMode (outputA2, INPUT);
  pinMode (outputB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(outputA1), pulseA1, RISING);
  attachInterrupt(digitalPinToInterrupt(outputB1), pulseB1, RISING);
  attachInterrupt(digitalPinToInterrupt(outputA2), pulseA2, RISING);
  attachInterrupt(digitalPinToInterrupt(outputB2), pulseB2, RISING);

  // Limit Switch Setup
  pinMode (right_switches_Pin, INPUT);
  pinMode (left_switches_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_switches_Pin), trigger_right, FALLING);
  attachInterrupt(digitalPinToInterrupt(left_switches_Pin), trigger_left, FALLING);
  any_switch_triggered = false;

  // Delay
  delay(3000);

  // Swing Up (LOW moves right, HIGH moves left)
  while (any_switch_triggered == false && counter1 > -1000) {
    move_DC_motor(255, HIGH);
  }
  move_DC_motor(0, LOW);
  while ((counter2 + 600) < 0) {
    delay(1);
  }
  while (any_switch_triggered == false && counter2 < -200) {
    counter2_bottom = counter2 + 600;

    if (previous_counter2_bottom == counter2_bottom) {
      n_minus2_counter2_bottom = n_minus2_counter2_bottom;
      n_minus1_counter2_bottom = n_minus1_counter2_bottom;
      n_counter2_bottom = counter2_bottom;
    } else {
      n_minus2_counter2_bottom = n_minus1_counter2_bottom;
      n_minus1_counter2_bottom = previous_counter2_bottom;
      n_counter2_bottom = counter2_bottom;
    }

    if ( (abs(n_counter2_bottom) < abs(n_minus1_counter2_bottom)) && (abs(n_minus1_counter2_bottom) > abs(n_minus2_counter2_bottom)) && counter2_bottom < 0 && left_swing == 0) {
      move_DC_motor(255, HIGH);
      left_swing = 1;
      delay(183);
      move_DC_motor(0, LOW);
    }
    if ( (abs(n_counter2_bottom) < abs(n_minus1_counter2_bottom)) && (abs(n_minus1_counter2_bottom) > abs(n_minus2_counter2_bottom)) && counter2_bottom > 0 && left_swing == 1) {
      move_DC_motor(255, LOW);
      left_swing = 0;
      delay(183);
      move_DC_motor(0, LOW);
    }

    previous_previous_counter2_bottom = previous_counter2_bottom;
    previous_counter2_bottom = counter2_bottom;

  }

  // START OF JUST ANGLE PID FOR CATCHING SWING UP
  delay(70); // 90
  while (any_switch_triggered == false && counter2 < -150) {
    move_DC_motor(255, LOW);
  }
  //  move_DC_motor(255, LOW);
  //  delay(65);
  PID_angle_start_time = micros();
  while (any_switch_triggered == false && PID_angle_running_time < 300000) {
    // Control Calculations Using Pendulum Encoder Theta (counter2 encoder, clockwise positive) + Cart Position (counter1 encoder, left positive)
    // Time
    current_time = micros();
    dt = (current_time - previous_time) / 1000000.0;
    // Theta
    theta = counter2 * counter2_to_theta;
    error_theta = (setpoint_theta - theta);
    integral_error_theta += ( ((error_theta + previous_error_theta) / 2.0)  * dt);
    derivative_error_theta = ( (error_theta - previous_error_theta) / dt );
    output_theta = K_P_theta_Swing * error_theta + K_I_theta_Swing * integral_error_theta + K_D_theta_Swing * derivative_error_theta;
    // Combining PID control calculations for Pendulum Theta and Cart Position
    output = output_theta;
    // Assigning current PID variables for next loop
    previous_error_theta = error_theta;
    previous_time = current_time;

    // Run Motor Programs
    pwm_value = abs(output);
    if (pwm_value < 0) {
      pwm_value = 0;
    } else if (pwm_value > 255) {
      pwm_value = 255;
    }

    // motor direction logic
    if (output > 0) {
      mtr_dir_cmnd = HIGH;      // HIGH moves left
    } else if (output <= 0) {
      mtr_dir_cmnd = LOW;       // LOW moves right, arbitrarily included zero. note that if output/pwm value is zero motor won't move anyways.
    }

    // motor function logic
    if (counter2 < 200 && counter2 > -200) {       // < 0 counter2 pendlulum is left of top dead center, > 0 counter2 is right of top dead center
      move_DC_motor(pwm_value, mtr_dir_cmnd);
    } else {
      move_DC_motor(0, LOW);
    }

    // Time delay
    delayMicroseconds(1000);
    // PID Angle Running Time
    PID_angle_running_time = current_time - PID_angle_start_time;
  }
  /// END OF JUST ANGLE PID FOR CATCHING SWING UP
  counter1 = 0;
}

void loop() {

  while (any_switch_triggered == false) {
    //Serial.println("ENTERED MAIN CONTROL LOOP");
    // Control Calculations Using Pendulum Encoder Theta (counter2 encoder, clockwise positive) + Cart Position (counter1 encoder, left positive)
    // Time
    current_time = micros();
    dt = (current_time - previous_time) / 1000000.0;
    // Theta Pendulum Angle
    theta = counter2 * counter2_to_theta;
    error_theta = (setpoint_theta - theta);
    integral_error_theta += ( ((error_theta + previous_error_theta) / 2.0)  * dt);
    derivative_error_theta = ( (error_theta - previous_error_theta) / dt );
    output_theta = K_P_theta * error_theta + K_I_theta * integral_error_theta + K_D_theta * derivative_error_theta;
    // Cart Position
    cart_pos = counter1 * counter1_to_pos;
    error_position = (setpoint_position - cart_pos);
    integral_error_position += ( ((error_position + previous_error_position) / 2.0)  * dt);
    derivative_error_position = ( (error_position - previous_error_position) / dt );
    output_position = K_P_position * error_position + K_I_position * integral_error_position + K_D_position * derivative_error_position;
    // Combining PID control calculations for Pendulum Theta and Cart Position
    output = output_theta + output_position;
    // Assigning current PID variables for next loop
    previous_error_theta = error_theta;
    previous_error_position = error_position;
    previous_time = current_time;

    // Run Motor Programs
    pwm_value = abs(output);
    if (pwm_value < 0) {
      pwm_value = 0;
    } else if (pwm_value > 255) {
      pwm_value = 255;
    }

    // motor direction logic
    if (output > 0) {
      mtr_dir_cmnd = HIGH;      // HIGH moves left
    } else if (output <= 0) {
      mtr_dir_cmnd = LOW;       // LOW moves right, arbitrarily included zero. note that if output/pwm value is zero motor won't move anyways.
    }

    // motor function logic
    if (counter2 < 200 && counter2 > -200) {       // < 0 counter2 pendlulum is left of top dead center, > 0 counter2 is right of top dead center
      move_DC_motor(pwm_value, mtr_dir_cmnd);
    } else {
      move_DC_motor(0, LOW);
    }

    // Time delay
    //delay(2);
    delayMicroseconds(1000);
    //Serial.println(output);
    //    Serial.println(any_switch_triggered);

    // Serial Monitor (remember to uncomment Serial.begin line in setup)
    //    Serial.print(counter1);
    //    Serial.print("\t");
    //    Serial.print(counter2);
    //    Serial.print("\t");
    //    Serial.print(theta);
    //    Serial.print("\t");
    //    Serial.print(error_theta);
    //    Serial.print("\t");
    //    Serial.print(integral_error_theta);
    //    Serial.print("\t");
    //    Serial.print(derivative_error_theta);
    //    Serial.print("\t");
    //    Serial.print(error_position);
    //    Serial.print("\t");
    ////    Serial.print(integral_error_position);
    ////    Serial.print("\t");
    ////    Serial.print(derivative_error_position);
    ////    Serial.print("\t");
    //    Serial.print(output_theta);
    //    Serial.print("\t");
    //    Serial.print(output_position);
    //    Serial.print("\t");
    //    Serial.print(pwm_value);
    //    Serial.print("\t");
    ////    Serial.print(previous_time);
    ////    Serial.print("\t");
    ////    Serial.print(current_time);
    ////    Serial.print("\t");
    //    Serial.println(dt);

  }

  while (any_switch_triggered == true) {
    //stay in loop
    move_DC_motor(0, LOW);
    //    Serial.println("stuck in SWITCH TRIGGER  loop");
    //    Serial.println(any_switch_triggered);
    //Serial.println(counter2);
  }


}

// Move DC Motor (LOW moves right, HIGH moves left)
void move_DC_motor(float pwm_value, int motor_direction) {
  digitalWrite(dir_Pin, motor_direction);
  analogWrite(pwm_Pin, pwm_value);
}

// Encoders Interrupt pulses
void pulseA1() {
  if (digitalRead(outputB1) == LOW) {
    counter1 --;
  } else {
    counter1 ++;
  }
}
void pulseB1() {
  if (digitalRead(outputA1) == LOW) {
    counter1 ++;
  } else {
    counter1 --;
  }
}

void pulseA2() {
  if (digitalRead(outputB2) == LOW) {
    counter2 ++;
  } else {
    counter2 --;
  }
}
void pulseB2() {
  if (digitalRead(outputA2) == LOW) {
    counter2 --;
  } else {
    counter2 ++;
  }
}

// Limit Switch Interrupt
void trigger_right() {
  detachInterrupt(right_switches_Pin);
  any_switch_triggered = true;
}

void trigger_left() {
  detachInterrupt(left_switches_Pin);
  any_switch_triggered = true;
}
