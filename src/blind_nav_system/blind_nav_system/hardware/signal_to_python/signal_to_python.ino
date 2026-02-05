const int buttonPin = 18;
const int pressurePin = 39;
int lastButtonState = LOW;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT);
}

void loop() {
  int currentButtonState = digitalRead(buttonPin);
  int pressureVal = analogRead(pressurePin);

  // 근거: 트리거가 발생하면 Serial.flush() 효과를 내기 위해 
  // 다른 데이터보다 우선적으로 즉시 전송합니다.
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    Serial.print("TRIG,1,");
    Serial.println(pressureVal); 
  } else {
    Serial.print("DATA,");
    Serial.print(currentButtonState);
    Serial.print(",");
    Serial.println(pressureVal);
  }

  lastButtonState = currentButtonState;
  
  // 근거: delay가 너무 길면 버튼 반응이 둔해집니다. 10ms로 줄여 반응성을 높입니다.
  delay(10); 
}