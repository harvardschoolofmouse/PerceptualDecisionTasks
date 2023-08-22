
// output pins
#define PIN_SPEAKER A21 // output to piezo speaker
// input pins
#define PIN_INPUT 2 // input from larger arduino 
// set maxmimum and minimum frequencies that noise generator can produce

void setup() {
    pinMode(PIN_INPUT, INPUT);
    pinMode(PIN_SPEAKER, OUTPUT);
}
void loop() {
    // if button is pressed, generate white noise
    if (digitalRead(PIN_INPUT) == HIGH) {     
        analogWrite(PIN_SPEAKER, random(0,255));
    }
    if (digitalRead(PIN_INPUT) == LOW) {
        digitalWrite(PIN_SPEAKER, 0);
    }
}