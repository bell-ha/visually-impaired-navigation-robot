const int buttonPin1 = 18;  // 기존 버튼
const int buttonPin2 = 19;  // 추가된 버튼 (I019)
const int pressurePin = 39;

int lastButtonState1 = LOW;
int lastButtonState2 = LOW;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT); // 19번 핀을 입력으로 설정
}

void loop() {
  int state1 = digitalRead(buttonPin1);
  int state2 = digitalRead(buttonPin2);
  int pressureVal = analogRead(pressurePin);

  // 근거: 어느 버튼이 눌렸는지 구분하기 위해 TRIG 번호를 부여합니다.
  if (state1 == HIGH && lastButtonState1 == LOW) {
    Serial.print("TRIG,1,"); // 18번 버튼 트리거
    Serial.println(pressureVal); 
  } 
  else if (state2 == HIGH && lastButtonState2 == LOW) {
    Serial.print("TRIG,2,"); // 19번 버튼 트리거
    Serial.println(pressureVal); 
  }
  else {
    // 일반 데이터 전송 시 두 버튼의 상태를 모두 보냅니다.
    Serial.print("DATA,");
    Serial.print(state1);
    Serial.print(",");
    Serial.print(state2);
    Serial.print(",");
    Serial.println(pressureVal);
  }

  lastButtonState1 = state1;
  lastButtonState2 = state2;
  
  delay(10); 
}