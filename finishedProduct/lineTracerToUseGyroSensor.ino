    #include <Wire.h>
#define mpu_add 0x68
#define INTERRUPT 2

int IN1 = 9;    // DC모터 pin
int IN2 = 8;    // DC모터 pin
int IN3 = 7;    // DC모터 pin
int IN4 = 6;    // DC모터 pin
int ENA = 10;   // 모터 제어
int ENB = 5;    // 모터 제어
int speed = 255;// 모터 속도

int leftLineSensor = 13;
int rightLineSensor = 12;

int redPin = 11;    // 빨간색 led pin 
int greenPin = 3;   // 초록색 led pin 
int bluePin = 4;    // 파란색 led pin 

int LeftLineSensor;
int RightLineSensor;

void setColor (int red, int green, int blue)    // led 설정
{        
          analogWrite (redPin, red);
          analogWrite (greenPin, green);
          analogWrite (bluePin, blue);      
}

void forward ()    // 전진
{     
          digitalWrite (IN1, HIGH);
          digitalWrite (IN2, LOW);
          analogWrite (ENA, 230);
          digitalWrite (IN3, HIGH);
          digitalWrite (IN4, LOW);
          analogWrite (ENB, 230);
          digitalWrite (redPin, HIGH);
          digitalWrite (greenPin, LOW);
          digitalWrite (bluePin, HIGH);   
}

void left ()     // 좌회전
{
          digitalWrite (IN1, LOW);
          digitalWrite (IN2, LOW);
          analogWrite (ENA, 200);
          digitalWrite (IN3, HIGH);
          digitalWrite (IN4, LOW);
          analogWrite (ENB, 200);
          digitalWrite (redPin, HIGH);
          digitalWrite (greenPin, LOW);
          digitalWrite (bluePin, HIGH);        
}

void right ()     // 우회전
{          
          digitalWrite (IN1, HIGH);
          digitalWrite (IN2, LOW);
          analogWrite (ENA, 200);
          digitalWrite (IN3, LOW);
          digitalWrite (IN4, LOW);
          analogWrite (ENB, 200);                         
          digitalWrite (redPin, HIGH);
          digitalWrite (greenPin, LOW);
          digitalWrite (bluePin, HIGH);         
}

void stop ()     // 정지
{  
          digitalWrite (IN1, LOW);
          digitalWrite (IN2, LOW);
          analogWrite (ENA, 0);
          digitalWrite (IN3, LOW);
          digitalWrite (IN4, LOW);
          analogWrite (ENB, 0);
          digitalWrite (redPin, LOW);
          digitalWrite (greenPin, HIGH);
          digitalWrite (bluePin, HIGH);          
}

class kalman    //자이로 센서 값 보정을 위한 칼만필터
{ 
    public:
        double getkalman (double acc, double gyro, double dt) {
          angle += dt * (gyro - bias);

          P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
          P[0][1] -= dt * P[1][1];
          P[1][0] -= dt * P[1][1];
          P[1][1] += Q_gyro * dt;
          
        double S = P[0][0] + R_measure;
          K[0] = P[0][0] / S;
          K[1] = P[1][0] / S;

        double y = acc - angle;
          angle += K[0] * y;
          bias += K[1] * y;

        double P_temp[2] = {P[0][0], P[0][1]};
          P[0][0] -= K[0] * P_temp[0];
          P[0][1] -= K[0] * P_temp[1];
          P[1][0] -= K[1] * P_temp[0];
          P[1][1] -= K[1] * P_temp[1];

          return angle;
      };
      
    void init (double angle, double gyro, double measure) {
          Q_angle = angle;
          Q_gyro = gyro;
          R_measure = measure;

          angle = 0;
          bias = 0;

          P[0][0] = 0;
          P[0][1] = 0;
          P[1][0] = 0;
          P[1][1] = 0;
        };
        
    double getvar (int num) {
        switch (num) {
          case 0:
            return Q_angle;
            break;
          case 1:
            return Q_gyro;
            break;
          case 2:
            return R_measure;
            break;
        }
      };
      
    private:
      double Q_angle, Q_gyro, R_measure;
      double angle, bias;
      double P[2][2], K[2];
 };

kalman kal;

long ac_x, ac_y, ac_z, gy_x, gy_y, gy_z;

double deg, dgy_y;
double dt;
uint32_t pasttime;

void setup () 
{ 
        Serial.begin (9600);
        Wire.begin ();
        Wire.beginTransmission (mpu_add);
        Wire.write (0x6B);
        Wire.write (0);
        Wire.endTransmission (true);
        
        kal.init (0.001, 0.003, 0.03);
        
        Serial.println ();
        Serial.print ("parameter");
        Serial.print ("\t");
        Serial.print (kal.getvar(0), 4);
        Serial.print ("\t");
        Serial.print (kal.getvar(1), 4);
        Serial.print ("\t");
        Serial.println (kal.getvar(2), 4);
        
        pinMode (IN1, OUTPUT);
        pinMode (IN2, OUTPUT);
        pinMode (IN3, OUTPUT);
        pinMode (IN4, OUTPUT);
        
        pinMode (leftLineSensor, INPUT);
        pinMode (rightLineSensor, INPUT);
        
        pinMode (INTERRUPT, INPUT);
        pinMode (redPin, OUTPUT);
        pinMode (greenPin, OUTPUT);
        pinMode (bluePin, OUTPUT);
        
        analogWrite (ENA, 0);
        analogWrite (ENB, 0);      
}

void loop () 
{  
        Wire.beginTransmission (mpu_add);  //자이로 센서 사용
        Wire.write (0x3B);
        Wire.endTransmission (false);
        Wire.requestFrom (mpu_add, 6, true);
        ac_x = Wire.read () << 8 | Wire.read ();
        ac_y = Wire.read () << 8 | Wire.read ();
        ac_z = Wire.read () << 8 | Wire.read ();

        Wire.beginTransmission (mpu_add); 
        Wire.write (0x43);
        Wire.endTransmission (false);
        Wire.requestFrom (mpu_add, 6, true);
        gy_x = Wire.read () << 8 | Wire.read ();
        gy_y = Wire.read () << 8 | Wire.read ();
        gy_z = Wire.read () << 8 | Wire.read ();

        deg = atan2(ac_x, ac_z) * 180 / PI;
        dgy_y = gy_y / 131.;

        dt = (double) (micros () - pasttime) / 1000000;
        pasttime = micros ();

        double val = kal.getkalman (deg, dgy_y, dt);  //자이로 센서 칼만필터 적용

        Serial.print ("kalman degree");
        Serial.print ("\t");
        Serial.println (val);
        LeftLineSensor = digitalRead (leftLineSensor);
        RightLineSensor = digitalRead (rightLineSensor);

        if (LeftLineSensor == HIGH && RightLineSensor == HIGH) {  // 좌우측 적외선 LineTrace 센서가 모두 감지 되었을 때 
          forward ();
          if (val > 8) {
            digitalWrite (IN1, HIGH);
            digitalWrite (IN2, LOW);    
            analogWrite (ENA, speed);
            
            digitalWrite (IN3, HIGH);
            digitalWrite (IN4, LOW);
            analogWrite (ENB, speed);
            
            digitalWrite (redPin, HIGH);
            digitalWrite (greenPin, HIGH);
            digitalWrite (bluePin, LOW);
          }
          if (val < -5) {         
            digitalWrite (IN1, HIGH);
            digitalWrite (IN2, LOW);
            analogWrite (ENA, 130);
            
            digitalWrite (IN3, HIGH);
            digitalWrite (IN4, LOW);
            analogWrite (ENB, 130);
            
            setColor (0,200,255);
          }
        }

        else if (LeftLineSensor == HIGH && RightLineSensor == LOW) {  // 좌측 적외선 LineTrace 센서만 감지 되었을 때 
          left ();
          if (val > 8) {
            digitalWrite (IN1, LOW);
            digitalWrite (IN2, LOW);
            analogWrite (ENA, 0);
            
            digitalWrite (IN3, HIGH);
            digitalWrite (IN4, LOW);
            analogWrite (ENB, speed);
            
            digitalWrite (redPin, HIGH);
            digitalWrite (greenPin, HIGH);
            digitalWrite (bluePin, LOW);
          }
          if (val < -5) {
            digitalWrite (IN1, LOW);
            digitalWrite (IN2, LOW);
            analogWrite (ENA, 0);
            
            digitalWrite (IN3, HIGH);
            digitalWrite (IN4, LOW);
            analogWrite (ENB, 130);
            
            setColor (0,200,255);
          }
         }

         else if (LeftLineSensor == LOW && RightLineSensor == HIGH) {  // 우측 적외선 LineTrace 센서만 감지 되었을 때 
           right ();
           if (val > 8) {
             digitalWrite (IN1, HIGH);
             digitalWrite (IN2, LOW);
             analogWrite (ENA, speed);
             
             digitalWrite (IN3, LOW);
             digitalWrite (IN4, LOW);
             analogWrite (ENB, 0);
             
             digitalWrite (redPin, HIGH);
             digitalWrite (greenPin, HIGH);
             digitalWrite (bluePin, LOW);
          }
          if (val < -5) {
             digitalWrite (IN1, HIGH);
             digitalWrite (IN2, LOW);
             analogWrite (ENA, 130);
             digitalWrite (IN3, LOW);
             digitalWrite (IN4, LOW);
             analogWrite (ENB, 0);
             setColor (0,200,255);
          }
        }

        else if (LeftLineSensor == LOW && RightLineSensor == LOW) {  // 좌우측 적외선 LineTrace 센서가 모두 감지되지 않았을 때 
          stop ();
        }
}
