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

unsigned char left_motor_byte = 0;
unsigned char right_motor_byte = 0;

void pwm() {
    // Use byte range [0,255] as the duty cycle range over (255 / 20) ms.
    unsigned char div = 25;
    digitalWrite(enrpwm, HIGH);
    digitalWrite(enlpwm, HIGH);
    unsigned char total_delay = min(left_motor_byte, right_motor_byte) / div;
    delay(total_delay);
    unsigned char overlap_delay = 0;
    if (left_motor_byte > right_motor_byte) {
        digitalWrite(enrpwm, LOW);
        overlap_delay = (left_motor_byte - right_motor_byte) / div;
        delay(overlap_delay);
    } else {
        digitalWrite(enlpwm, LOW);
        overlap_delay = (right_motor_byte - left_motor_byte) / div;
        delay(overlap_delay);
    }
    total_delay += overlap_delay;
    digitalWrite(enrpwm, LOW);
    digitalWrite(enlpwm, LOW);
    delay(255 / div - total_delay);
}

void loop() {
    while (Serial.available() >= 6) {
        char c = 0;
        while (c != 'L') {
            c = Serial.read();
        }
        char left_sgn = Serial.read();
        left_motor_byte = Serial.read();
        char r = Serial.read();
        char right_sgn = Serial.read();
        right_motor_byte = Serial.read();

        // Verify packet.
        if (r != 'R') {
            continue;
        }
        if (left_sgn == '+') {
            digitalWrite(m2in1l, HIGH);
            digitalWrite(m2in2l, LOW);
        } else if (left_sgn == '-') {
            digitalWrite(m2in1l, LOW);
            digitalWrite(m2in2l, HIGH);
        } else {
            continue;
        }
        if (right_sgn == '+') {
            digitalWrite(m1in1r, HIGH);
            digitalWrite(m1in2r, LOW);
        } else if (right_sgn == '-') {
            digitalWrite(m1in1r, LOW);
            digitalWrite(m1in2r, HIGH);
        } else {
            continue;
        }

        if (left_motor_byte != 0 or right_motor_byte != 0) {
            pwm();
        }
    }

    if (left_motor_byte != 0 or right_motor_byte != 0) {
        pwm();
    }
}
