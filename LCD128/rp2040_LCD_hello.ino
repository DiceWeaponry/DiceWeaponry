#include <SPI.h>
#include <TFT.h>            // Arduino TFT library

#define cs   10
#define dc   9
#define rst  8

TFT screen = TFT(cs, dc, rst);

void setup() {
  // initialize the screen
  screen.begin();

  // make the background black
  screen.background(0,0,0);

  // set the text color to white
  screen.stroke(255,255,255);

  // write text to the screen in the top left corner
  screen.text("Testing!", 0, 0);
}

void loop() {

}
