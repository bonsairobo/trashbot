int pinnum = 13; // LED
int enrpwm = 2;
int enlpwm = 9;

// m2 goes in reverse
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
    int ticks_high = 1;
    int ticks_low = 1;
    while (Serial.available() > 0) {
        char inChar = Serial.read();
        Serial.print(inChar);
        if (inChar == 'w') {
            digitalWrite(pinnum, HIGH);

            digitalWrite(m1in1r, HIGH);
            digitalWrite(m1in2r, LOW);
            digitalWrite(m2in1l, HIGH);
            digitalWrite(m2in2l, LOW);
        } else if (inChar == 's') {
            digitalWrite(pinnum, LOW);

            digitalWrite(m1in1r, LOW);
            digitalWrite(m1in2r, HIGH);
            digitalWrite(m2in1l, LOW);
            digitalWrite(m2in2l, HIGH);
        } else if (inChar == 'a') {
            digitalWrite(pinnum, HIGH);

            digitalWrite(m1in1r, HIGH);
            digitalWrite(m1in2r, LOW);
            digitalWrite(m2in1l, LOW);
            digitalWrite(m2in2l, HIGH);
        } else if (inChar == 'd') {
            digitalWrite(pinnum, LOW);

            digitalWrite(m1in1r, LOW);
            digitalWrite(m1in2r, HIGH);
            digitalWrite(m2in1l, HIGH);
            digitalWrite(m2in2l, LOW);
        }

        unsigned long start_ms = millis();
        while (millis() - start_ms < 50) {
            digitalWrite(enrpwm, HIGH);
            digitalWrite(enlpwm, HIGH);
            digitalWrite(enrpwm, LOW);
            digitalWrite(enlpwm, LOW);
        }
    }
    digitalWrite(pinnum, LOW);
    digitalWrite(enrpwm, LOW);
    digitalWrite(enlpwm, LOW);
}
