#include<Wire.h>
#include <Arduino.h>

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;

int x;
int y;
int z;

int xx = 0;
int yy = 0;
int zz = 0;

int prev_x = 0;
int prev_y = 0;
int prev_z = 0;

#define pwm1 13   
#define pwm2 12 
#define pwm3 11   
#define pwm4 10 
#define pwm5 9    
#define pwm6 8  

#define en1_1 53  
#define en1_2 52  
#define en2_1 51  
#define en2_2 50
#define en3_1 49  
#define en3_2 48  
#define en4_1 47  
#define en4_2 46
#define en5_1 45  
#define en5_2 44  
#define en6_1 43  
#define en6_2 42

#define pos_in_1 A1   
#define pos_in_2 A2    
#define pos_in_3 A3
#define pos_in_4 A4 
#define pos_in_5 A5
#define pos_in_6 A6

double errorlast1;  double errorlast2;  double errorlast3;  double errorlast4;  double errorlast5;  double errorlast6;
double error1;      double error2;      double error3;      double error4;      double error5;      double error6;
  
double dt;          double des_pos = 0;
double K_p = 1.0;   double K_i = 0;  double K_d = 0.05;
//double K_p = 0.45;   double K_i = 0.005;  double K_d = 0.0125;

double output1;  double integral1 = 0;  double derivative1 = 0;
double output2;  double integral2 = 0;  double derivative2 = 0;
double output3;  double integral3 = 0;  double derivative3 = 0;
double output4;  double integral4 = 0;  double derivative4 = 0;
double output5;  double integral5 = 0;  double derivative5 = 0;
double output6;  double integral6 = 0;  double derivative6 = 0;




void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  

  pinMode(en1_1, OUTPUT); pinMode(en1_2, OUTPUT); pinMode(pwm1, OUTPUT); pinMode(pos_in_1, INPUT);
  pinMode(en2_1, OUTPUT); pinMode(en2_2, OUTPUT); pinMode(pwm2, OUTPUT); pinMode(pos_in_2, INPUT);
  pinMode(en3_1, OUTPUT); pinMode(en3_2, OUTPUT); pinMode(pwm3, OUTPUT); pinMode(pos_in_3, INPUT);
  pinMode(en4_1, OUTPUT); pinMode(en4_2, OUTPUT); pinMode(pwm4, OUTPUT); pinMode(pos_in_4, INPUT);
  pinMode(en5_1, OUTPUT); pinMode(en5_2, OUTPUT); pinMode(pwm5, OUTPUT); pinMode(pos_in_5, INPUT);
  pinMode(en6_1, OUTPUT); pinMode(en6_2, OUTPUT); pinMode(pwm6, OUTPUT); pinMode(pos_in_6, INPUT);
  
}
int iii = 0;
double pwmspeed = 150;
double errorTol = 120;
double initialHeight = 167;
void loop() {
  x = 0;
  y = 0;
  z = 0;
  double* finalLength;
  double* finalLength2;
  double* finalLength3;
  
  // speed & error 
  if (iii == 0) {
      // start from the initial position - no angle, 0 height
      move6actuatorsTo(40, pwmspeed, 0, 0, 0, 0, 0, 0);
      delay(1000);
      // no angle, initial height
      move6actuatorsTo(errorTol, pwmspeed, initialHeight, initialHeight, initialHeight, initialHeight, initialHeight, initialHeight);
      delay(50);
    }
    iii++;
    // if h = 167, limit yaw +-40, pitch +-15 roll +-15
  // Code for IMU - input Euler angle
  
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    
    GyX=Wire.read()<<8|Wire.read();
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read();
    
    int xAng = map(GyX,minVal,maxVal,-90,90);
    int yAng = map(GyY,minVal,maxVal,-90,90);
    int zAng = map(GyZ,minVal,maxVal,-90,90);

 
    x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); // pitch
    y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI); // roll
    z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI); // ignore
    z = x;
    
  Serial.print("\n");
  Serial.print("***before**xyz \n");
    Serial.print("x = ");
  Serial.print(x);
  Serial.print("      ");
      Serial.print("y = ");
  Serial.print(y);
  Serial.print("    ");
  if (x > 90) {
     x = x -360; // pitch
    }
  if (y > 90) {
     y = y -360; // pitch
    }

     
    
    int yawLimit = 10; // yaw
    int pitchLlimit = 10; //pitch
    int rollLimit = 10; // roll


 


  Serial.print("\n");
  Serial.print("**after*xyz \n");
     Serial.print("x = ");
  Serial.print(x);
  Serial.print("      ");
      Serial.print("y = ");
  Serial.print(y);
  Serial.print("    ");

    // yaw
    if (x > yawLimit) {
      x = yawLimit;
    }
    if (x < -yawLimit) {
      x = -yawLimit;
    }

    // pitch
    if (y > pitchLlimit) {
      y = pitchLlimit;
    }
    if (y < -pitchLlimit) {
      y = -pitchLlimit;
    }

    // roll
    if (z > rollLimit) {
      z = rollLimit;
    }
    if (z < -rollLimit) {
      z = -rollLimit;
    }



  finalLength2 = positionTo(0,x,y, initialHeight);
   move6actuatorsTo(errorTol, pwmspeed, mmTopotConverter(finalLength2[0]), mmTopotConverter(finalLength2[1]), mmTopotConverter(finalLength2[2]), 
    mmTopotConverter(finalLength2[3]), mmTopotConverter(finalLength2[4]), mmTopotConverter(finalLength2[5]));
      
}

double* positionTo(double psi, double theta, double phi, double height) {
 psi = psi * 0.01745329252;
 theta = theta * 0.01745329252;
 phi = phi * 0.01745329252;
  
 double rotationMatrix[3][3] = { { cos(psi)*cos(theta), -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi)},
    {sin(psi)*cos(theta), cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi), -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi) },
    { -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi) } };

  static double points_platform[6][3] = {{ -9.92, 56.28, 0}, {9.93, 56.28, 0}, {53.7, -19.55, 0},
    {43.78, -36.74, 0}, { -43.78, -36.74, 0}, { -53.7, -19.55, 0}};
    
  static double result1[6][3];
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        result1[i][j] += points_platform[i][k] * rotationMatrix[k][j];
      }
    }
  }
  
  for (int i = 0; i < 6; i++) {
    result1[i][2] = result1[i][2] + height;
  }
  
  static double points_base[6][3] = {{ -53.51, 44.9, 0}, {53.51, 44.9, 0}, {65.64, 23.89, 0},
    {12.13, -68.79, 0}, { -12.13, -68.79, 0}, {-65.54, 23.89, 0} };
    
  // Calculate the final length
  static double final_length[6];
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      final_length[i] += (abs(points_base[i][j] - result1[i][j])) * (abs(points_base[i][j] - result1[i][j]));
      if (j == 2)
        final_length[i] = sqrt(final_length[i]);
    }
  }
  double *p = final_length;

  // initialize the array - result1
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
        result1[i][j] = 0;
    }
  }

  return p;
}

double mmTopotConverter(double mm) {
    double pot = (1/4.647590e-02)*mm - (157.3/4.647590e-02);
    //double pot = (1/4.647590e-02)*mm - (162.5/4.647590e-02);
    return pot;
    //1.729699e+02 initial length
    }

void move6actuatorsTo(double err_tolerance, double speed_control, double position1, double position2, double position3, double position4, double position5, double position6) {
  digitalWrite(en1_1, LOW); digitalWrite(en1_2, LOW); analogWrite(pwm1, 0);
  digitalWrite(en2_1, LOW); digitalWrite(en2_2, LOW); analogWrite(pwm2, 0);
  digitalWrite(en3_1, LOW); digitalWrite(en3_2, LOW); analogWrite(pwm3, 0);
  digitalWrite(en4_1, LOW); digitalWrite(en4_2, LOW); analogWrite(pwm4, 0);
  digitalWrite(en5_1, LOW); digitalWrite(en5_2, LOW); analogWrite(pwm5, 0);
  digitalWrite(en6_1, LOW); digitalWrite(en6_2, LOW); analogWrite(pwm6, 0);
  boolean good = false;
  
  error1 = 0; error2 = 0; error3 = 0; error4 = 0; error5 = 0; error6 = 0;
  
  integral1 = 0; derivative1 = 0;
  integral2 = 0; derivative2 = 0;
  integral3 = 0; derivative3 = 0;
  integral4 = 0; derivative4 = 0;
  integral5 = 0; derivative5 = 0;
  integral6 = 0; derivative6 = 0;
  
  double prevtime = 0; double currtime = 0;
  
  while (!good) {
    //Serial.print(analogRead(pos_in));
    dt = 0.005;

    errorlast1 = error1;  errorlast2 = error2;  errorlast3 = error3;
    errorlast4 = error4;  errorlast5 = error5;  errorlast6 = error6;
    
    error1 = position1 - analogRead(pos_in_1);
    error2 = position2 - analogRead(pos_in_2);
    error3 = position3 - analogRead(pos_in_3);
    error4 = position4 - analogRead(pos_in_4);
    error5 = position5 - analogRead(pos_in_5);
    error6 = position6 - analogRead(pos_in_6);
    if (abs(error1) <= err_tolerance && abs(error2) <= err_tolerance && abs(error3) <= err_tolerance && abs(error4) <= err_tolerance && abs(error5) <= err_tolerance  && abs(error6) <= err_tolerance) {
      
      output1 = 0;  output2 = 0;  output3 = 0;  output4 = 0;  output5 = 0;  output6 = 0;
      error1 = 0;   error2 = 0;   error3 = 0;   error4 = 0;   error5 = 0;   error6 = 0;
      
      good = true;
      
      analogWrite(pwm1, 0);   analogWrite(pwm2, 0);   analogWrite(pwm3, 0);   
      analogWrite(pwm4, 0);   analogWrite(pwm5, 0);   analogWrite(pwm6, 0);
      
    } else {
      output1 = ((K_p * error1) + (K_i * integral1)) + (K_d * derivative1);
      output2 = ((K_p * error2) + (K_i * integral2)) + (K_d * derivative2);
      output3 = ((K_p * error3) + (K_i * integral3)) + (K_d * derivative3);
      output4 = ((K_p * error4) + (K_i * integral4)) + (K_d * derivative4);
      output5 = ((K_p * error5) + (K_i * integral5)) + (K_d * derivative5);
      output6 = ((K_p * error6) + (K_i * integral6)) + (K_d * derivative6);
      
    }
    if (output1 >= speed_control) {
      output1 = speed_control;
    }
    if (output1 <= -speed_control) {
      output1 = -speed_control;
    }
    if (output2 >= speed_control) {
      output2 = speed_control;
    }
    if (output2 <= -speed_control) {
      output2 = -speed_control;
    }
    if (output3 >= speed_control) {
      output3 = speed_control;
    }
    if (output3 <= -speed_control) {
      output3 = -speed_control;
    }
    if (output4 >= speed_control) {
      output4 = speed_control;
    }
    if (output4 <= -speed_control) {
      output4 = -speed_control;
    }
    if (output5 >= speed_control) {
      output5 = speed_control;
    }
    if (output5 <= -speed_control) {
      output5 = -speed_control;
    }
    if (output6 >= speed_control) {
      output6 = speed_control;
    }
    if (output6 <= -speed_control) {
      output6 = -speed_control;
    }
    
    integral1 += error1 * dt;
    integral2 += error2 * dt;
    integral3 += error3 * dt;
    integral4 += error4 * dt;
    integral5 += error5 * dt;
    integral6 += error6 * dt;
    
    derivative1 = (error1 - errorlast1) / dt;
    derivative2 = (error2 - errorlast2) / dt;
    derivative3 = (error3 - errorlast3) / dt;
    derivative4 = (error4 - errorlast4) / dt;
    derivative5 = (error5 - errorlast5) / dt;
    derivative6 = (error6 - errorlast6) / dt;
    
    if (output1 >= 0) {
      digitalWrite(en1_1, HIGH); digitalWrite(en1_2, LOW); analogWrite(pwm1, abs(output1));
    }
    if (output1 < 0) {
      digitalWrite(en1_1, LOW); digitalWrite(en1_2, HIGH);  analogWrite(pwm1, abs(output1));
    }
    if (output2 >= 0) {
      digitalWrite(en2_1, HIGH); digitalWrite(en2_2, LOW); analogWrite(pwm2, abs(output2));
    }
    if (output2 < 0) {
      digitalWrite(en2_1, LOW); digitalWrite(en2_2, HIGH);  analogWrite(pwm2, abs(output2));
    }
    if (output3 >= 0) {
      digitalWrite(en3_1, HIGH); digitalWrite(en3_2, LOW); analogWrite(pwm3, abs(output3));
    }
    if (output3 < 0) {
      digitalWrite(en3_1, LOW); digitalWrite(en3_2, HIGH);  analogWrite(pwm3, abs(output3));
    }
    if (output4 >= 0) {
      digitalWrite(en4_1, HIGH); digitalWrite(en4_2, LOW); analogWrite(pwm4, abs(output4));
    }
    if (output4 < 0) {
      digitalWrite(en4_1, LOW); digitalWrite(en4_2, HIGH);  analogWrite(pwm4, abs(output4));
    }
    if (output5 >= 0) {
      digitalWrite(en5_1, HIGH); digitalWrite(en5_2, LOW); analogWrite(pwm5, abs(output5));
    }
    if (output5 < 0) {
      digitalWrite(en5_1, LOW); digitalWrite(en5_2, HIGH);  analogWrite(pwm5, abs(output5));
    }
    if (output6 >= 0) {
      digitalWrite(en6_1, HIGH); digitalWrite(en6_2, LOW); analogWrite(pwm6, abs(output6));
    }
    if (output6 < 0) {
      digitalWrite(en6_1, LOW); digitalWrite(en6_2, HIGH);  analogWrite(pwm6, abs(output6));
    }
  }
}
