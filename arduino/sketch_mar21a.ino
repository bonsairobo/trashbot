int pinnum = 13;
int enrpwm = 2;
int enlpwm = 9;

//m2 goes in reverse
int m2in1l = 3;
int m2in2l = 4;
int m1in1r = 11;
int m1in2r = 10;

void setup() {
    Serial.begin(9600);
    pinMode(pinnum, OUTPUT);
    pinMode(enrpwm, OUTPUT);
    pinMode(enlpwm, OUTPUT);
    pinMode(m2in1l, OUTPUT);
    pinMode(m2in2l, OUTPUT);
    pinMode(m1in1r, OUTPUT);
    pinMode(m1in2r, OUTPUT);
}

void loop() {
  
      while (Serial.available() > 0) {
        char inChar = Serial.read();
        Serial.print(inChar);
        if(inChar == 'w')
        {
        digitalWrite(pinnum, HIGH);
      
        digitalWrite(m1in1r, HIGH);
        digitalWrite(m1in2r, LOW);

        digitalWrite(m2in1l, HIGH);
        digitalWrite(m2in2l, LOW);
        digitalWrite(enrpwm, HIGH);
        digitalWrite(enlpwm, HIGH);
        delay(50);
        }
        if(inChar == 's')
        {
        digitalWrite(pinnum, LOW);
   
        digitalWrite(m1in1r, LOW);
        digitalWrite(m1in2r, HIGH);
       
        digitalWrite(m2in1l, LOW);
        digitalWrite(m2in2l, HIGH);
        digitalWrite(enrpwm, HIGH);
        digitalWrite(enlpwm, HIGH);
        delay(50);
        }
        if(inChar == 'a')
        {
        digitalWrite(pinnum, HIGH);
    
        digitalWrite(m1in1r, HIGH);
        digitalWrite(m1in2r, LOW);
     
        digitalWrite(m2in1l, LOW);
        digitalWrite(m2in2l, HIGH);
        digitalWrite(enlpwm, HIGH);
        digitalWrite(enrpwm, HIGH);
        delay(50);
        }
        if(inChar == 'd')
        {
        digitalWrite(pinnum, LOW);
  
        digitalWrite(m1in1r, LOW);
        digitalWrite(m1in2r, HIGH);
 
        digitalWrite(m2in1l, HIGH);
        digitalWrite(m2in2l, LOW);
        digitalWrite(enrpwm, HIGH);
        digitalWrite(enlpwm, HIGH);
        delay(50);
        }
        

      }
        digitalWrite(pinnum, LOW);
        digitalWrite(enrpwm, LOW);
        digitalWrite(enlpwm, LOW);
    
    
    // digitalWrite(enrpwm, HIGH);
    // digitalWrite(m1in1r, HIGH);
    // digitalWrite(m1in2r, LOW);
    // digitalWrite(enlpwm, LOW);
    // digitalWrite(m2in1l, HIGH);
    // digitalWrite(m2in2l, LOW);
    // delay(1000);
    // digitalWrite(enrpwm, LOW);
    // digitalWrite(m1in1r, HIGH);
    // digitalWrite(m1in2r, LOW);
    // digitalWrite(enlpwm, HIGH);
    // digitalWrite(m2in1l, HIGH);
    // digitalWrite(m2in2l, LOW);
    // delay(1000);
}

