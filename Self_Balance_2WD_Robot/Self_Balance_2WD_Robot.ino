#include <Wire.h>
#include <SoftwareSerial.h>

// connect the TX of wirelss module to Arduino pin 11 RX;
// connect the RX of wirelss module to Arduino pin 12 TX;
SoftwareSerial serial_SW_wireless_module(11, 12); // (RX, TX)

#define LEFT_MOTOR_REVERSE false
#define RIGHT_MOTOR_REVERSE true

int gyro_address = 0x68;
int acc_calibration_value;                            //Enter the accelerometer calibration value

//Various settings
float pid_p_gain = 38;
float pid_i_gain = 0.5;
float pid_d_gain = 36;
float turning_speed = 100;                                    //Turning speed (900)
float max_target_speed = 1000;                                //Max target speed (1500)

byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer, stable_time;
const int stable_time_period = 1000;

int receive_counter=0;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float prev_angle_gyro = 0, angle_adjusted_filtered;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;
int speed_m = 1000; //max 2500
float pickup = 0.009; 

byte error, MPU_6050_found, nunchuck_found, lowByte, highByte;
int address;
int nDevices;


//Serial with ESPCAM variables
char lastReceivedChar = '\0'; // Variable to store the last received character
unsigned long previousMillis = 0; // Store the last time data was checked
const long interval = 100; // Interval at which to check for new data (milliseconds)


void set_gyro_registers();

void setup() {
  serial_SW_wireless_module.begin(115200); 
  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;

  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS21);
  OCR2A = 39;
  TCCR2A |= (1 << WGM21);

/*******************************************************************/
  Serial.println("Scanning I2C bus...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)Serial.print("0");
      Serial.println(address,HEX);
      nDevices++;
      if(address == 0x68 || address == 0x69){
        Serial.println("This could be a MPU-6050");
        Wire.beginTransmission(address);
        Wire.write(0x75);
        Wire.endTransmission();
        Serial.println("Send Who am I request...");
        Wire.requestFrom(address, 1);
        while(Wire.available() < 1);
        lowByte = Wire.read();
        if(lowByte == 0x68){
          Serial.print("Who Am I responce is ok: 0x");
          Serial.println(lowByte, HEX);
        }
        else{
          Serial.print("Wrong Who Am I responce: 0x");
          if (lowByte<16)Serial.print("0");
          Serial.println(lowByte, HEX);
        }
        if(lowByte == 0x68 && address == 0x68){
          MPU_6050_found = 1;
          Serial.println("Starting Gyro....");
          set_gyro_registers();
        }
      }
      if(address == 0x52){
        Serial.println("This could be a Nunchuck");
        Serial.println("Trying to initialise the device...");
        Wire.beginTransmission(0x52);
        Wire.write(0xF0);
        Wire.write(0x55);
        Wire.endTransmission();
        delay(20);
        Wire.beginTransmission(0x52);
        Wire.write(0xFB);
        Wire.write(0x00);
        Wire.endTransmission();
        delay(20);
        Serial.println("Sending joystick data request...");
        Wire.beginTransmission(0x52);
        Wire.write(0x00);
        Wire.endTransmission();
        Wire.requestFrom(0x52,1);
        while(Wire.available() < 1);
        lowByte = Wire.read();
        if(lowByte > 100 && lowByte < 160){
          Serial.print("Data response normal: ");
          Serial.println(lowByte);
          nunchuck_found = 1;
        }
        else{
          Serial.print("Data response is not normal: ");
          Serial.println(lowByte);
        }
      }
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  if(MPU_6050_found){
    Serial.print("Balance value: ");
    Wire.beginTransmission(0x68);
    Wire.write(0x3F);
    Wire.endTransmission();
    Wire.requestFrom(0x68,2);
    acc_calibration_value = (Wire.read()<<8|Wire.read())*-1;
    Serial.println(acc_calibration_value);
    delay(20);
    Serial.println("Printing raw gyro values");
    for(address = 0; address < 20; address++ ){
      Wire.beginTransmission(0x68);
      Wire.write(0x43);
      Wire.endTransmission();
      Wire.requestFrom(0x68,6);
      while(Wire.available() < 6);
      Serial.print("Gyro X = "); 
      Serial.print(Wire.read()<<8|Wire.read());
      Serial.print(" Gyro Y = "); 
      Serial.print(Wire.read()<<8|Wire.read());
      Serial.print(" Gyro Z = "); 
      Serial.println(Wire.read()<<8|Wire.read());
    }
    Serial.println("");
  }
  else Serial.println("No MPU-6050 device found at address 0x68");

  if(nunchuck_found){
    Serial.println("Printing raw Nunchuck values");
    for(address = 0; address < 20; address++ ){ 
      Wire.beginTransmission(0x52);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(0x52,2);
      while(Wire.available() < 2);
      Serial.print("Joystick X = "); 
      Serial.print(Wire.read());
      Serial.print(" Joystick y = ");
      Serial.println(Wire.read());
      delay(100);
    }
  }
  else Serial.println("No Nunchuck device found at address 0x52");
/************************************************************************/

  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  // internally CNC shiled pins are connected to 2,3,5,6 for step and dir
  // Set pins 2 to 7 as output using DDRD
  DDRD |= 0b11111100;  // Set bits 2 to 7 as output (11111100)

  // Set pin 8 as output using DDRB
  DDRB |= (1 << DDB0); // Set bit 0 (pin 8) of DDRB as output

    // Set pin 8 as output using DDRB
  DDRB |= (1 << DDB0); // Set bit 0 (pin 8) of DDRB as output

  for (receive_counter = 0; receive_counter < 500; receive_counter++) {     //Create 500 loops
    if (receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));       //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();           //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read();         //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
/*   if (Serial.available()) {                                                 //If there is serial data available
    received_byte = Serial.read();                                                //Reset the receive_counter variable
  Serial.println(received_byte);
  } */
  // use the SW serial to talk to the wireless modeul, so that the HW 
  // serial can be used for debug prints';
  if (serial_SW_wireless_module.available()) {                              //If there is serial data available
    received_byte = serial_SW_wireless_module.read();                       //Load the received serial data in the received_byte variable
    receive_counter = 0;                                                    //Reset the receive_counter variable
  }
  if(receive_counter <= 25)receive_counter ++;                              //The received byte will be valid for 25 program loops (100 milliseconds)
  else received_byte = 0;                                                //After 100 milliseconds the received byte is deleted

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read() << 8 | Wire.read();                  //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if (accelerometer_data_raw > 8200) {
    accelerometer_data_raw = 8200;          //Prevent division by zero by limiting the acc data to +/-8200;
  }
  if (accelerometer_data_raw < -8200) {
    accelerometer_data_raw = -8200;        //Prevent division by zero by limiting the acc data to +/-8200;
  }

  angle_acc = asin((float)accelerometer_data_raw / 8200.0) * 57.296;        //Calculate the current angle according to the accelerometer

  if (start == 0 && angle_acc > -0.5 && angle_acc < 0.5) {                  //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }

  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = Wire.read() << 8 | Wire.read();                       //Combine the two bytes to make one integer
  gyro_pitch_data_raw = Wire.read() << 8 | Wire.read();                     //Combine the two bytes to make one integer

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  angle_gyro -= gyro_yaw_data_raw * 0.0000003;                          //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle

  //Apply filter
  angle_adjusted_filtered = prev_angle_gyro * 0.99 + angle_gyro * 0.01;
  prev_angle_gyro = angle_gyro;
  
  pid_error_temp = angle_adjusted_filtered - self_balance_pid_setpoint - pid_setpoint;
  if (pid_output > 10 || pid_output < -10) {
    pid_error_temp += pid_output * 0.015 ;
  }

  pid_i_mem += pid_i_gain * pid_error_temp;
  if (pid_i_mem > speed_m) {
    pid_i_mem = speed_m;
  }
  else if (pid_i_mem < -speed_m) {
    pid_i_mem = -speed_m;
  }
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if (pid_output > speed_m) {
    pid_output = speed_m;
  }
  else if (pid_output < -speed_m) {
    pid_output = -speed_m;
  }

  pid_last_d_error = pid_error_temp;

  if (pid_output < 5 && pid_output > -5) {
    pid_output = 0;
  }

  if (angle_gyro > 40 || angle_gyro < -40 || start == 0 /* || low_bat == 1 */) {
    pid_output = 0;
    pid_i_mem = 0;
    start = 0;
    self_balance_pid_setpoint = 0;
  }

  pid_output_left = pid_output;
  pid_output_right = pid_output;

//Serial.println((char)received_byte); // debug;

/*if ((char)received_byte == '4') {  // Turn right
  pid_output_left += turning_speed;
  pid_output_right -= turning_speed;
}

if ((char)received_byte == '3') {  // Turn left
  pid_output_left -= turning_speed;
  pid_output_right += turning_speed;
}

if ((char)received_byte == '1') {  // Move backward
  if (pid_setpoint > -2.5 || pid_output > max_target_speed * -1) {
    pid_setpoint -= pickup;
  }
}

if ((char)received_byte == '2') {  // Move forward
  if (pid_setpoint < 2.5 || pid_output < max_target_speed) {
    pid_setpoint += pickup;
  }
}

  if ((char)received_byte == 'A') {
    if (pid_setpoint > 0.5) {
      pid_setpoint -= pickup;
    }
    else if (pid_setpoint < -0.5) {
      pid_setpoint += pickup;
    }
    else pid_setpoint = 0;
  }*/
  if (pid_setpoint == 0) {
    if (pid_output < 0) {
      self_balance_pid_setpoint += 0.0015;
    }
    if (pid_output > 0) {
      self_balance_pid_setpoint -= 0.0015;
    }
  }
  if (pid_output_left > 0) {
    pid_output_left = speed_m - (1 / (pid_output_left + 9)) * 5500;
  }
  else if (pid_output_left < 0) {
    pid_output_left = -speed_m - (1 / (pid_output_left - 9)) * 5500;
  }

  if (pid_output_right > 0) {
    pid_output_right = speed_m - (1 / (pid_output_right + 9)) * 5500;
  }
  else if (pid_output_right < 0) {
    pid_output_right = -speed_m - (1 / (pid_output_right - 9)) * 5500;
  }

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if (pid_output_left > 0) {
    left_motor = speed_m - pid_output_left;
  }
  else if (pid_output_left < 0) {
    left_motor = -speed_m - pid_output_left;
  }
  else {
    left_motor = 0;
  }

  if (pid_output_right > 0) {
    right_motor = speed_m - pid_output_right;
  }
  else if (pid_output_right < 0) {
    right_motor = -speed_m - pid_output_right;
  }
  else {
    right_motor = 0;
  }
  //Serial.println(left_motor);
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;
  while (loop_timer > micros());
  loop_timer += 4000;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Inoterrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect) {
  // ===================== Left motor pulse calculations =====================
  throttle_counter_left_motor++;  // Increment the throttle counter for the left motor

  if (throttle_counter_left_motor > throttle_left_motor_memory) {
    throttle_counter_left_motor = 0;  // Reset the counter
    throttle_left_motor_memory = throttle_left_motor;  // Update the throttle memory

    // Set direction for the left motor based on the reverse flag and throttle value
    if (throttle_left_motor_memory < 0) {
      throttle_left_motor_memory *= -1;  // Make the throttle value positive for step counting
      if (LEFT_MOTOR_REVERSE) {
        PORTD |= (1 << PD6);  // In reverse mode, set pin 6 high for forward movement
      } else {
        PORTD &= ~(1 << PD6);  // In normal mode, set pin 6 low for reverse direction
      }
    } else {
      if (LEFT_MOTOR_REVERSE) {
        PORTD &= ~(1 << PD6);  // In reverse mode, set pin 6 low for reverse movement
      } else {
        PORTD |= (1 << PD6);  // In normal mode, set pin 6 high for forward movement
      }
    }
  }

  // Create step pulse for the left motor (pin 2)
  if (throttle_counter_left_motor == 1) {
    PORTD |= (1 << PD2);  // Set pin 2 high to create the step pulse
  } else if (throttle_counter_left_motor == 2) {
    PORTD &= ~(1 << PD2);  // Set pin 2 low to end the step pulse
  }

  // ===================== Right motor pulse calculations =====================
  throttle_counter_right_motor++;  // Increment the throttle counter for the right motor

  if (throttle_counter_right_motor > throttle_right_motor_memory) {
    throttle_counter_right_motor = 0;  // Reset the counter
    throttle_right_motor_memory = throttle_right_motor;  // Update the throttle memory

    // Set direction for the right motor based on the reverse flag and throttle value
    if (throttle_right_motor_memory < 0) {
      throttle_right_motor_memory *= -1;  // Make the throttle value positive for step counting
      if (RIGHT_MOTOR_REVERSE) {
        PORTD |= (1 << PD5);  // In reverse mode, set pin 5 high for forward movement
      } else {
        PORTD &= ~(1 << PD5);  // In normal mode, set pin 5 low for reverse direction
      }
    } else {
      if (RIGHT_MOTOR_REVERSE) {
        PORTD &= ~(1 << PD5);  // In reverse mode, set pin 5 low for reverse movement
      } else {
        PORTD |= (1 << PD5);  // In normal mode, set pin 5 high for forward movement
      }
    }
  }

  // Create step pulse for the right motor (pin 3)
  if (throttle_counter_right_motor == 1) {
    PORTD |= (1 << PD3);  // Set pin 3 high to create the step pulse
  } else if (throttle_counter_right_motor == 2) {
    PORTD &= ~(1 << PD3);  // Set pin 3 low to end the step pulse
  }
}


void set_gyro_registers(){
  //Setup the MPU-6050
  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  
  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  Wire.beginTransmission(0x68);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 
}
