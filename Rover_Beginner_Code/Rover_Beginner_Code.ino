#include <ECE3.h>
//SPEED
const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const int LED_RF = 41;
const int LED_RR = 57;

//SENSORS
uint16_t sensorValues[8];
int error = 0;
int value = 0;
int min_vals[8] = {481, 550, 528, 528, 529, 597, 368, 506};
int max_vals[8] = {1471,1377,1447,805,1090,879,1369,1707};
int weights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};

int t_i;

void setup()
{
  ECE3_Init();

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);

  pinMode(LED_RF, OUTPUT);

  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);

  t_i = millis();
}

void Move(int left, int right) {
    if(left>0){
      digitalWrite(left_dir_pin,LOW);
    }else
    {
      digitalWrite(left_dir_pin,HIGH);
    }
    if(right>0){
      digitalWrite(right_dir_pin,LOW);
    }else
    {
      digitalWrite(right_dir_pin,HIGH);
    }
  analogWrite(abs(left_pwm_pin), int(abs(double(left))/1000*255));
  analogWrite(abs(right_pwm_pin), int(abs(double(right))/1000*255));
}
//
//void Ramp(int leftI, int leftF, int rightI, int rightF, int t)
//{
//  int n = 500;
//  int leftCur = leftI;
//  int rightCur = rightI;
//  int leftStep = (leftF-leftI)/n;
//  int rightStep = (rightF-rightI)/n;
//  while(signbit(leftStep)==signbit(leftF-leftCur)&&signbit(rightStep)==signbit(rightF-rightCur)){
//    Serial.print(signbit(leftStep)==signbit(leftF-leftCur)&&signbit(rightStep)==signbit(rightF-rightCur));
//      leftCur += leftStep;
//      rightCur += rightStep;
//      Move(leftCur, rightCur);
//      delay(t/n);
//    }
//}

int prev_error = 0;

//const double kp = 0.0006;
//const double kd = 0.0005; //-0.00315

const int sample_n = 4;
const int sample_size = sample_n * 17;
const int sample_int = sample_n * 250;
int samples [sample_size];
int prev_t = 0;
int i=0;
double u;

double getkp(int u){
  if(u>700){
      return 0.00007;
    }
  else if(u>500){
    return 0.000145;
    }
  else if(u>300){
    return 0.00042;
    }
  return 0.0006;
    
  }

double getkd(int u){
  if(u>700){
      return 0.000011;
    }
  else if(u>500){
    return 0.00003;
    }
  else if(u>300){
    return 0.00030;
    }
  return 0.0005;
    
  }

  double getPID(int error){
  double kp = getkp(u);
  double kd = getkd(u);
  double de = error - prev_error;
  double op = error * kp + kd * de;
  prev_error = error;
  return op;
  }

void track_line(double error, boolean go){
  double pid = getPID(error);
  if(i>=sample_size){
    i=0;
  }
  if(abs(error) > samples[i]){
    for(int j=0; j<11;j++){
        samples[i] = abs(error);
        i++;
      }
    }
  samples[i] = abs(error);
  int mean = 0;
  for(int j=0; j<sample_size; j++){
    mean += samples[j];
  }
  mean /= sample_size;
  u = mean/5000.0 * 800 * 3;
  u =  900-u;
  if(u<100){
    u=100;
    }

  if(u > 600){
    u=600;
    }
//  Serial.println(String(error) + "  " + String(mean) +"  " + String(u) + "  " +String(int(u*(1.0-pid))) );
//  u = 301;
  int t = millis() - t_i;
//
//  if(t<1000){
//      u = 200;
//    }
// else{
//  u = 0;
//  }
  
  Move(int(u*(1.0-pid)), int(u*(1.0+pid)));
  
  
//  Serial.println(String(error) + "  " + String(pid));
  i++;
  return;
  }

boolean go = false;
int s_prev =0;
void loop()
{

  int s=0;
// read raw sensor values
ECE3_read_IR(sensorValues);
for (unsigned char i = 0; i < 8; i++)
{
value = sensorValues[i];
s+= value;
value = weights[i] * (value - min_vals[i] * 1000 / max_vals[i]);
error = error+value;
//Serial.print(String(sensorValues[i]) + "  ");
}
Serial.println(s);
//Serial.println();

error = error/4;

error -= 101.0;

if(millis() >= t_i+ sample_int*2 ){
  go = true;
  }
//go = true;

track_line(error, go);

if (s >= 19500 && s_prev >= 19500){
  Move(800,-800);
  delay (370);
  Move(0,0);
}

s_prev = s;

}
