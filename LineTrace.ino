 
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
int leftLineSensor =  13;      // 라인트레이서 왼쪽 센서
int rightLineSensor = 12;      // 라인트레이서 오른쪽 센서

void forward() {    // 전진 : 모터 두 개를 모두 전진
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
}
void back() {    // 후진 : 모터 두 개를 모두 후진
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
}
void left() {    // 좌회전 : 오른쪽 모터만 전진
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
}
void right() {   // 우회전 : 왼쪽 모터만 전진
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);  
}
void stop() {    // 정지 : 2개의 모터 정지
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
}
void setup() {
  pinMode(IN1, OUTPUT);     // A 모터 1
  pinMode(IN2, OUTPUT);     // A 모터 2
  pinMode(IN3, OUTPUT);     // B 모터 1
  pinMode(IN4, OUTPUT);     // B 모터 2
  pinMode(leftLineSensor, INPUT); 
  pinMode(rightLineSensor, INPUT);
}

void loop() {  
   //양쪽 Line Tracking sensor 감지  : 전진
  if (!digitalRead(leftLineSensor) && !digitalRead(rightLineSensor)) {
    //모터A, B 전진
  forward();
    //왼쪽 Line Tracking sensor 감지 : 좌회전
  } else if (!digitalRead(leftLineSensor) && digitalRead(rightLineSensor)) {
  left();
    //오른쪽 Line Tracking sensor 감지 : 우회전
  } else if (digitalRead(leftLineSensor) && !digitalRead(rightLineSensor)) {
  right();
    // 양쪽 Line Tracking sensor 미감지 :  정지
  } else if (digitalRead(leftLineSensor) && digitalRead(rightLineSensor)) {
  stop();
  }
}
