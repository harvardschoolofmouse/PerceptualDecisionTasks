// Volume Library 1KHz Sketch (c) 2016 Connor Nishijima
// Released under the GPLv3 license

#include "Volume.h" // Include the Volume library

Volume vol(6); // Plug your speaker into Pin 6

// the setup routine runs once when you press reset:
void setup() {
  vol.begin();
}

// the loop routine runs over and over again forever:
void loop() {
  float mv = constrain(analogRead(A0) / 10, 0, 100) / 100.00;
  vol.tone(1000, 255*mv);
}
