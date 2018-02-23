#include <QTRSensors.h>
#include <Wire.h>
#include <math.h>
#include <digcomp.h>//<--filtering

#define NUM_SENSORS   6     // number of sensors used
#define TIMEOUT       2500//2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN// used to be 2     // emitter is controlled by digital pin 2

const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
double roll, pitch;
double gyroXrate;
double gyroYrate;

double AcY_offset;
uint32_t timer; //it's a timer, saved as a large, unsigned int.  We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.

double compAngleX, compAngleY; //These are the angles in the complementary filter
#define degconvert 57.2957786 //there are like 57 degrees in a radian.

//int ax_offset=0;
int ay_offset=0;
double accel_frac=0.01;
int center_pos = NUM_SENSORS/2*1000;
unsigned int position;
double e;
double e_prev;
int e_enc;
double e_diff;
double e_dot;
double e_int;
int diff;
int nom = 0;
int nom2 = 0;
int min_nom =0;
float max_nom = 175.0;
float temp_nom;
float kp = 0.05;
float kp_enc1 = 0.1;
float kp_enc2 = 0.1;
float kp_tilt = 0.1;
float kd_tilt = 0.01;
float e_enc1;
float e_enc2;
float kd = 0.1;
float ki = 0.0;
int control_case=0;
int ol_width=0;
int ol_amp=0;

float slope = 0.1;
float f_fixed;
int des_pos;
float f_ss;
float des_theta;
float des_tilt;
float tilt_amp;

int posbreak1=0;
int negbreak1=0;
int posbreak2=0;
int negbreak2=0;

// LP filter
float nom_filt;

// 30 Hz, fs=200 Hz
//float b_lp[2] = {0.32030073, 0.32030073};
//float a_lp[2] = {1.00000000, -0.72848950};

// 10 Hz, fs=200 Hz
//float b_lp[2] = {0.13575525, 0.13575525};
//float a_lp[2] = {1.00000000, -0.72848950};

// 5 Hz, fs=200Hz
float b_lp[2] = {0.07282051, 0.07282051};
float a_lp[2] = {1.00000000, -0.85435899};


float lp_in[2];
float lp_out[2];

dig_comp lpfilt(b_lp, a_lp, lp_in, lp_out, 2,2);


// HP filter
float comp_filt;

float b_hp[2] = {0.99843167, -0.99843167};
float a_hp[2] = {1.00000000, -0.99686333};

float hp_in[2];
float hp_out[2];

dig_comp hpfilt(b_hp, a_hp, hp_in, hp_out, 2,2);



int inv_deadband(int speed, int posbreak, int negbreak){
  int out;
  out = speed;
  
  if ( out > 0){
    out += posbreak;
  }
  else if (out < 0){
    out += negbreak;
  }
  
  if (out < -255){
    out = -255;
  }
  else if (out > 255){
    out = 255;
  }

  return out;
}

/* int mysat(int vin){ */
/*   int mymax = 255; */
/*   int mymin = -255; */
/*   int vout; */
  
/*   if ( vin > mymax ){ */
/*     vout = mymax; */
/*   } */
/*   else if ( vin < mymin ){ */
/*     vout = mymin; */
/*   } */
/*   else{ */
/*     vout = vin; */
/*   } */

/*   return(vout); */
/* } */

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
//{23, 25, 27, 29, 31, 33, 35, 37},
QTRSensorsRC qtrrc((unsigned char[]) {A0,A1,A2,A3,A4,A5},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];


unsigned char getsecondbyte(int input){
  unsigned char output;
  output = (unsigned char)(input >> 8);
  return output;
}

int reassemblebytes(unsigned char msb, unsigned char lsb){
  int output;
  output = (int)(msb << 8);
  output += lsb;
  return output;
}

void mynewline(){
  Serial.print('\n');
}

unsigned long t1;
unsigned long t2;

// Encoder pins
// Verify that these are the same pins your encoder
// A and B channels are connected to
#define enc1A 2
#define enc1B 11

#define enc2A 3
#define enc2B 12

// The pwm, in1, and in2 connections are internal to the
// H-bridge shield
int pwm_pin1 = 9;
int m1in1 = 6;
int m1in2 = 4;

//pwm_pin2, m2in1, m2in2
int pwm_pin2 = 10;//<-- buy batteries and check these
int m2in1 = 7;//<-- buy batteries and check these
int m2in2 = 8;//<-- buy batteries and check these

//int isr_pin = A0;
//int trig1 = 24;
//int echo1 = 22;
//int trig2 = 28;
//int echo2 = 26;

//  encoder
volatile bool _EncBSet1;
volatile long enc_count1 = 0;

volatile bool _EncBSet2;
volatile long enc_count2 = 0;
long prev_enc2 = 0;
float enc2_dot;

volatile int enc_pend = 0;

volatile bool freqresp=false;//true;

int n_loop, stop_n, n_delay, nISR;
long raw_loop_count=0;

const byte mask = B11111000;
int prescale = 1;

int fresh;
int ISRstate;


int inByte;
int outByte;

int extra_param;
int amp;
int amp1;
int amp2;
float freq;
float w;
float theta_d;
float dt_ms;
float dt1;
float dt2;
float dt_sec;
float a;
int pwm1;
int pwm2;
int width;
int curspeed;

int state;
int print_case;

char testbuf[10];
unsigned long t;
unsigned long prevt;
unsigned long t0;
unsigned long tempt;
float t_ms;
float t_sec;
int mydelay;
int prevenc;


int v;
int preve;
float edot; 

int average_ay(){
  long ay_sum=0;
  int offset=0;
  int num=100;
  int k=0;

  for (k=0; k < num; k++){
        //Collect raw data from the sensor.
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
      AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
      AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
      ay_sum = ay_sum + AcY;

      delay(10);
  }

  offset = ay_sum/num;
  return(offset);
}


void setup()
{
  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif


  Serial.begin(115200);
  Serial.print("krauss balancing v. 1.1.0");
  Serial.print('\n');

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  print_case = 0;
  
  pinMode(pwm_pin1, OUTPUT);
  pinMode(m1in1, OUTPUT);
  pinMode(m1in2, OUTPUT);

  pinMode(pwm_pin2, OUTPUT);
  pinMode(m2in1, OUTPUT);
  pinMode(m2in2, OUTPUT);


  //pinModeFast(isr_pin, OUTPUT);
  //digitalWrite(isr_pin, LOW);

  // encoder
  pinMode(enc1A, INPUT); 
  pinMode(enc1B, INPUT); 
  // turn on pullup resistors
  digitalWrite(enc1A, HIGH);
  digitalWrite(enc1B, HIGH);

  pinMode(enc2A, INPUT); 
  pinMode(enc2B, INPUT); 
  // turn on pullup resistors
  digitalWrite(enc2A, HIGH);
  digitalWrite(enc2B, HIGH);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(enc1A), doEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(enc2A), doEncoder2, RISING);  

  n_loop = 10000;//must be higher than stop_n to start with 
  amp = 17;
  width = 0;
  stop_n = 200;//3000;//5000
  n_delay = 10;

  state = 0;
  pwm1 = 0;
  pwm2 = 0;
  
  posbreak1 = 0;
  posbreak2 = 0;
  negbreak1 = 0;
  negbreak2 = 0;

  preve = 0;

  //=======================================================
  // set up the Timer3 interrupt
  //=======================================================
  /* cli();          // disable global interrupts */
  /* TCCR3A = 0;     // set entire TCCR1A register to 0 */
  /* TCCR3B = 0;     // same for TCCR1B */

  /* // set compare match register to desired timer count: */
  /* //OCR1A = 15624; */
  /* OCR3A = 155;//100 Hz */
  /* // OCR1A = 100;//150ish - seems to work */
  /* //OCR1A = 77;//200 Hz <-- seems very borderline (might be 184 Hz) */
  /* //OCR1A = 30;//500 Hz */
  /* //OCR1A = 15;//1000 Hz */
  /* //OCR1A = 7;//2000 Hz */


  /* // turn on CTC mode: */
  /* TCCR3B |= (1 << WGM12); */

  /* // Set CS10 and CS12 bits for 1024 prescaler: */
  /* TCCR3B |= (1 << CS10); */
  /* TCCR3B |= (1 << CS12); */
 
  /* // enable timer compare interrupt: */
  /* TIMSK3 |= (1 << OCIE3A); */

  /* sei(); */
  //=======================================================
}

void command_motor(int speed, int pwm_pin, int in1, int in2){
  if (speed > 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm_pin, speed);
  }
  else if (speed < 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, abs(speed));
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, 0);
  }
}

void command_motor1(int speed){
  command_motor(speed, pwm_pin1, m1in1, m1in2);
}

void command_motor2(int speed){
  command_motor(speed, pwm_pin2, m2in1, m2in2);
}

void cal_command_motor(int speed, int cal_case){
    if (cal_case == 1){
      command_motor1(speed);
    }
    else if (cal_case == 2){
      command_motor2(speed);
    }
}

void set_breakaways_case(int posbreak, int negbreak, int cal_case){
  if (cal_case == 1){
    posbreak1 = posbreak;
    negbreak1 = negbreak;
  }
  else if (cal_case == 2){
    posbreak2 = posbreak;
    negbreak2 = negbreak;
  }
}

void calibrate_deadband(int cal_case, volatile long *enc_count){
  int posbreak = 0;
  int negbreak = 0;
  prevenc = *enc_count;
  for (int a=0; a<200; a++){
    cal_command_motor(a, cal_case);
    delay(50);
    if (*enc_count != prevenc){
      Serial.print("posbreak amp = ");
      Serial.println(a);
      posbreak = a;
      break;
    }
  }
  cal_command_motor(0, cal_case);

  delay(100);
  prevenc = *enc_count;
  for (int a=0; a>-200; a--){
    cal_command_motor(a, cal_case);
    delay(50);
    if (*enc_count != prevenc){
      Serial.print("negbreak amp = ");
      Serial.println(a);
      negbreak = a;
      break;
    }
  }
  cal_command_motor(0, cal_case);

  set_breakaways_case(posbreak, negbreak, cal_case);
}

void preptest(){
  Serial.println("#====================");
  Serial.print("#kp=");
  Serial.println(kp,4);
  Serial.print("#kd=");
  Serial.println(kd,4);
  if (freqresp){
    Serial.println("#n_loop, t_ms, des_pos, e, diff, position");
  }
  else{
    //Serial.println("#raw_loop_count,n_loop, t_ms, e, pwm1, pwm2, position");
    Serial.println("#n_loop, t_ms, compAngleY, comp_filt, e_dot, nom");
  }
  delay(50);
  enc_count1 = 0;
  enc_count2 = 0;  
  n_loop = 0;
  nISR = 0;
  ISRstate = 0;
  fresh = 0;
  t0 = micros();
  prevt = t0;
  t = t0;
  dt2 = 0;
  print_case = 2;
}

int get_int(){
  int out_int;
  out_int = 0;
  while (out_int == 0){
    while (Serial.available() == 0){
      delay(10);
    }
    out_int = Serial.parseInt();
  }
  return(out_int);
  
}

float get_float(){
  float out_float;

  out_float = 0;
  while (out_float == 0){
    while (Serial.available() == 0){
      delay(10);
    }
    out_float = Serial.parseFloat();
  }
  return(out_float);
  
}

int read_one_byte(){
  int outbyte;

  while (Serial.available() == 0){
    delay(10);
  }
  outbyte = Serial.read();

  return(outbyte);
}

int get_valid_byte(){
  int outbyte;
  outbyte = read_one_byte();
  
  if (outbyte == 10 || outbyte == 13){
    //try again
    outbyte = read_one_byte();
  }
  
  return(outbyte);
}

void print_line_serial(){
  //labels: n_loop, dt_ms, t_ms, pwm1, pwm2, enc_w1, enc_w2, enc_pend
  // balancing data: t, compAngleY, comp_filt, nom
      Serial.print(n_loop);Serial.print(",");    
      //Serial.print(dt_ms);//2
      //Serial.print(",");
      Serial.print(t_ms);Serial.print(",");    
      //---------------------------------
      // the balancing control is based on
      // e = compAngleX-des_tilt; where des_tilt=0 for now
      //    AcY_offset = AcY - ay_offset;
      //    roll = atan2(AcY_offset, AcZ)*degconvert;
      //    compAngleX = 0.99 * (compAngleX + gyroXrate * dt_sec) + 0.01 * roll;
      Serial.print(AcY);Serial.print(",");
      Serial.print(AcY_offset);Serial.print(",");
      Serial.print(AcZ);Serial.print(",");
      Serial.print(roll);Serial.print(",");
      Serial.print(gyroXrate);Serial.print(",");
      Serial.print(compAngleX);Serial.print(",");
      Serial.print(e);Serial.print(",");
      Serial.print(e_dot);
      //---------------------------------
      Serial.print('\n');
}

void print_line_freq_resp(){
  //labels: n_loop, dt_ms, t_ms, pwm1, pwm2, enc_w1, enc_w2, enc_pend
      Serial.print(n_loop);//1
      Serial.print(",");    
      //Serial.print(dt_ms);//2
      //Serial.print(",");
      Serial.print(t_ms);//3
      Serial.print(",");
      Serial.print(des_pos);
      Serial.print(",");
      Serial.print(e);//3
      Serial.print(",");    
      Serial.print(diff);//4
      Serial.print(",");    
      //Serial.print(enc_count1);//6
      //Serial.print(",");    
      //Serial.print(enc_count2);//7
      Serial.print(position);
      //Serial.print(enc_pend);
      Serial.print('\n');
}

int two_digits(float floatin){
  float float_part;
  int out_digits;
  float_part = floatin - (int)floatin;
  out_digits = (int)(100*float_part);
  return out_digits;
}

int three_digits(float floatin){
  float float_part;
  int out_digits;
  float_part = floatin - (int)floatin;
  out_digits = (int)(1000*float_part);
  return out_digits;
}

void print_line_sprintf(int pwm_out){
  char buffer[70];
  sprintf(buffer, "%d,%d.%0.3d,%d.%0.2d,%d,%d",
	  n_loop, (int)dt_ms, three_digits(dt_ms),
	  (int)t_ms, two_digits(t_ms),
	  enc_count1, pwm_out);
  Serial.println(buffer);
}

/* unsigned char getsecondbyte(int input){ */
/*     unsigned char output; */
/*     output = (unsigned char)(input >> 8); */
/*     return output; */
/* } */

 

/* int reassemblebytes(unsigned char msb, unsigned char lsb){ */
/*     int output; */
/*     output = (int)(msb << 8); */
/*     output += lsb; */
/*     return output; */
/* } */

int readtwobytes(void){
    unsigned char msb, lsb;
    int output;
    int iter = 0;
    while (Serial.available() <2){
      iter++;
      if (iter > 1e5){
	break;
      }
    }
    msb = Serial.read();
    lsb = Serial.read();
    output = reassemblebytes(msb, lsb);
    return output;
}

void SendTwoByteInt(int intin){
    unsigned char lsb, msb;
    lsb = (unsigned char)intin;
    msb = getsecondbyte(intin);
    Serial.write(msb);
    Serial.write(lsb);
}

void calibrate(){
  Serial.println("getting ready to calibrate");
  delay(500);
  //pinMode(13, OUTPUT);
  //digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  int dir=1;
  int amp=50;
  int check;
  
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    check = i % 50;
    if ( check == 0 ){
      dir *= -1;
      command_motor1(dir*amp);
      command_motor2(dir*amp);
    } 
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  //digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  command_motor1(0);
  command_motor2(0);

  // print the calibration minimum values measured when emitters were on

  Serial.println("calibration results:");
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(2000);
}

void cal_inv_db_both(){
      calibrate_deadband(1, &enc_count1);
      Serial.print("posbreak1: ");
      Serial.print(posbreak1);
      Serial.print('\n');
      Serial.print("negbreak1: ");
      Serial.print(negbreak1);
      Serial.print('\n');

      calibrate_deadband(2, &enc_count2);
      Serial.print("posbreak2: ");
      Serial.print(posbreak2);
      Serial.print('\n');
      Serial.print("negbreak2: ");
      Serial.print(negbreak2);
      Serial.print('\n');
}

void menu(){
  Serial.println("input test case");
  mynewline();
  state = 0;// If something goes wrong, set code to return to menu

  while (Serial.available() == 0){
      delay(10);
  }

  inByte = Serial.read();

  if ( inByte == '1' ){
    //run test
    state = 1;
    n_loop = 0;
    e_int = 0;
    ay_offset = average_ay();
    compAngleX = 0;
    e_int = 0;
    preptest();
  }
  else if ( inByte == 'c' ){
    calibrate();
  }
  else if ( inByte == 'k' ){
    kp = Serial.parseFloat();
    Serial.print("kp = ");
    Serial.println(kp,4);
  }
  else if ( inByte == 'd' ){
    kd = Serial.parseFloat();
    Serial.print("kd = ");
    Serial.println(kd,4);
  }
  else if (inByte == '2' ){
    nom2 = Serial.parseInt();
    Serial.print("nom2 = ");
    Serial.println(nom2);
  }
  else if (inByte == 'm' ){
    temp_nom = Serial.parseFloat();
    max_nom = (float)(temp_nom);
    Serial.print("max_nom = ");
    Serial.println(max_nom);
  }
  else if (inByte == '0' ){
    min_nom = Serial.parseInt();
    Serial.print("min_nom = ");
    Serial.println(min_nom);
  }  
  else if (inByte == 's'){
    stop_n = Serial.parseInt();
    Serial.print("stop_n = ");
    Serial.println(stop_n);
  }
  else if (inByte == 'a'){
    amp = Serial.parseInt();
    Serial.print("amp = ");
    Serial.println(amp);
  }
  else if (inByte == 'i'){
    cal_inv_db_both();
  }
  else if (inByte == '3'){
    // select control case
    // 0 = normal
    // 1 = freq_resp
    // 2 = OL pulse
    control_case = Serial.parseInt();
    Serial.print("control_case = ");
    Serial.println(control_case);
    if (control_case == 1){
      freqresp = true;
    }
    else{
      freqresp = false;
    }
  }
  else if (inByte == 'o'){
    ol_amp = Serial.parseInt();
    Serial.print("ol_amp = ");
    Serial.println(ol_amp);
  }
  else if (inByte == 'w'){
    ol_width = Serial.parseInt();
    Serial.print("ol_width = ");
    Serial.println(ol_width);
  }
  else if (inByte == 'M'){
    slope = Serial.parseFloat();
    Serial.print("slope = ");
    Serial.println(slope);    
  }
  else if (inByte == 'F'){
    f_fixed = Serial.parseFloat();
    Serial.print("f_fixed = ");
    Serial.println(f_fixed);    
  }
  else if (inByte == 'T'){
    tilt_amp = Serial.parseFloat();
    Serial.print("tilt_amp = ");
    Serial.println(tilt_amp);    
  }
  else if (inByte == 'I'){
    ki = Serial.parseFloat();
    Serial.print("ki = ");
    Serial.println(ki);    
  }
  else if (inByte == 'z'){
    ay_offset = average_ay();
    compAngleX = 0;
    Serial.println("zeroing done");
    Serial.print("ay_offset = ");Serial.println(ay_offset);
  }
  else if (inByte == '!'){
    kp_enc1 = Serial.parseFloat();
    Serial.print("kp_enc1 = ");
    Serial.println(kp_enc1,4);    
  }
  else if (inByte == '@'){
    kp_enc2 = Serial.parseFloat();
    Serial.print("kp_enc2 = ");
    Serial.println(kp_enc2,4);    
  }
  else if (inByte == 't'){
    kp_tilt = Serial.parseFloat();
    Serial.print("kp_tilt = ");
    Serial.println(kp_tilt,4);    
  }
  else if (inByte == 'D'){
    kd_tilt = Serial.parseFloat();
    Serial.print("kd_tilt = ");
    Serial.println(kd_tilt,4);    
  }
  else if (inByte == '/'){
    accel_frac = Serial.parseFloat();
    Serial.print("accel_frac = ");
    Serial.println(accel_frac,6);    
  }
}

void loop()
{
  if ( state == 0){
      menu();
  }
      
  else{
    if (n_loop == 0){
      t0 = micros();
    }

    prevt = t;//moved from in between the two lines below 03/29/17 11:50AM
    t = micros();
    n_loop++;
    dt_ms = (t-prevt)/1000.0;
    dt_sec = dt_ms/1000.0;
    t_ms = (t-t0)/1000.0;
    t_sec = t_ms/1000.0;

    //---------------------------
    //
    // Read Tilt Sensor
    //
    //---------------------------
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    AcY_offset = AcY - ay_offset;

    roll = atan2(AcY_offset, AcZ)*degconvert;
    pitch = atan2(-AcX, AcZ)*degconvert;

    
    //The gyroscope outputs angular velocities.  To convert these velocities from the raw data to deg/second, divide by 131.  
    //Notice, we're dividing by a double "131.0" instead of the int 131.
    //double gyroXrate = GyX/131.0;
    //double gyroYrate = GyY/131.0;
    gyroXrate = GyX/131.0;
    gyroYrate = GyY/131.0;


  
    //THE COMPLEMENTARY FILTER
    // This filter calculates the angle based MOSTLY on integrating
    // the angular velocity to an angular displacement.
    //
    // dt, recall, is the time between gathering data from the MPU6050.
    // We'll pretend that the angular velocity has remained constant
    // over the time dt, and multiply angular velocity by 
    // time to get displacement.
    // The filter then adds a small correcting factor from
    // the accelerometer ("roll" or "pitch"), so the gyroscope
    // knows which way is down.
    
    // Calculate the angle using a Complimentary filter
    //compAngleX = 0.99 * (compAngleX + gyroXrate * dt_sec) + 0.01 * roll;
    compAngleX = (1-accel_frac) * (compAngleX + gyroXrate * dt_sec) + accel_frac * roll;
    compAngleY = 0.98 * (compAngleY + gyroYrate * dt_sec) + 0.02 * pitch;
    //comp_filt = hpfilt.calc_out(compAngleX);
    //compAngleY = 0.97 * (compAngleY + gyroYrate * dt) + 0.03 * pitch ;

    /* f_ss = t_sec*slope; */
    /* des_tilt = tilt_amp*sin(2*PI*t_sec*f_ss); */

    
    //enc2_dot = (enc_count2-prev_enc2)/dt_sec;
    //prev_enc2 = enc_count2;
    //des_tilt = -kp_tilt*enc_count2 - kd_tilt*enc2_dot;

    des_tilt = 0;
    
    e_prev = e;
    e = compAngleX-des_tilt;
    //e = comp_filt;
    e_diff = (float)(e-e_prev);
    e_dot = e_diff/dt_sec;
    e_int += e*dt_sec;
    // what do I do here?      
    //nom = kp*e + kd*e_dot +ki*e_int;//change to comp_filt later
    nom = kp*e + kd*gyroXrate + ki*e_int;//change to comp_filt later
    nom_filt = lpfilt.calc_out(nom);

    // drift and spin control
    //e_enc1 = -enc_count1;
    //e_enc2 = -enc_count2;
    //pwm1 = -nom + kp_enc1*e_enc1;
    //pwm2 = nom + kp_enc2*e_enc2;
    pwm1 = -nom;
    pwm2 = nom;

    //compAngleX, e, e_dot, nom, pwm1, pwm2
    // --> Also consider a step response in the stable configuration <--
    //f_ss = t_sec*slope;
    //des_theta = amp*sin(2*PI*t_sec*f_ss);
    //des_theta = amp*sin(2*PI*t_sec*f_fixed);
    //e_enc = des_theta - enc_count2;
    pwm1 = inv_deadband(pwm1, posbreak1, negbreak1);//note inv_deadband includes sat
    pwm2 = inv_deadband(pwm2, posbreak2, negbreak2);
    //command_motor1(pwm1);
    //command_motor2(pwm2);
    command_motor1(0);
    command_motor2(0);

    
    /* if ((n_loop > 10) && (n_loop < (stop_n-100))){ */
    /*   nom = kp*e_enc; */
    /*   pwm1 = inv_deadband(-nom, posbreak1, negbreak1);//note inv_deadband includes sat */
    /*   pwm2 = inv_deadband(nom, posbreak2, negbreak2); */
    /*   command_motor1(pwm1); */
    /*   command_motor2(pwm2); */
    /* } */
    /* else{ */
    /*   nom = 0; */
    /*   command_motor1(0); */
    /*   command_motor2(0); */
    /* } */
    // what do I want to print?
    print_line_serial();//t, compAngleY, comp_filt, nom
    
    /* if (freqresp){ */
    /*   print_line_freq_resp(); */
    /* } */
    /* else{ */
    /*   print_line_serial(); */
    /* } */

    delayMicroseconds(750);
    //delay(3);

   if ( n_loop > stop_n){
     state = 0;
     command_motor1(0);
     command_motor2(0);
   }
 }
}  
// Interrupt service routines for the right motor's quadrature encoder
void doEncoder1()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  //n++;

  _EncBSet1 = digitalRead(enc1B);   // read the input pin
  
  // and adjust counter + if A leads B
  if (_EncBSet1){
    enc_count1 ++;
  }
  else {
    enc_count1 --;
  }
}

void doEncoder2()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  //n++;

  _EncBSet2 = digitalRead(enc2B);   // read the input pin
  
  // and adjust counter + if A leads B
  if (_EncBSet2){
    enc_count2 ++;
  }
  else {
    enc_count2 --;
  }
}

/* ISR(TIMER3_COMPA_vect) */
/* { */
/*   nISR++; */
/*   fresh = 1; */
/*   /\* if (ISRstate == 1){ *\/ */
/*   /\*   ISRstate = 0; *\/ */
/*   /\*   digitalWrite(sw_pin, LOW); *\/ */
/*   /\* } *\/ */
/*   /\* else{ *\/ */
/*   /\*   ISRstate = 1; *\/ */
/*   /\*   digitalWrite(sw_pin, HIGH); *\/ */
/*   /\* } *\/ */
/* } */

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
/* void requestEvent() { */
/*   //Wire.write(50); // respond with message of 6 bytes */
/*   //Wire.write(17); */
/*   SendTwoByteInt_i2c(encoder_count); */
/* } */


//void receiveEvent(int howMany) {
//  enc_pend = readtwobytes_i2c();
//  nISR++;
//  fresh = 1;
//  //Serial.println(x);         // print the integer
//}