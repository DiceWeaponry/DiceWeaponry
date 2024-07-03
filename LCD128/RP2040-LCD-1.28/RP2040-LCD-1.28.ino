#include "LCD_Test.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

// Global variables
UWORD *BlackImage; // Declare BlackImage globally

void setup() {
  // put your setup code here, to run once:
  if(DEV_Module_Init() != 0)
    Serial.println("GPIO Init Fail!");
  else
    Serial.println("GPIO Init successful!");

  LCD_1IN28_Init(HORIZONTAL);
  DEV_SET_PWM(50);
  LCD_1IN28_Clear(WHITE);
  UDOUBLE Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
  
  // Allocate memory for BlackImage
  if ((BlackImage = (UWORD *)malloc(Imagesize)) == NULL)
  {
      Serial.println("Failed to apply for black memory...");
      exit(0);
  }
  
  Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28_WIDTH, LCD_1IN28_HEIGHT, 0, WHITE);
  Paint_SetScale(65);
  Paint_Clear(WHITE);
  Paint_SetRotate(ROTATE_0);
  Paint_Clear(WHITE);

  // Initialize random seed
  srand(time(NULL));
}

void loop() {
    Paint_Clear(WHITE);
    int randomColor = rand() % 65536; // Generate a random color
    Paint_DrawString_EN(50, 100, "DiceWeaponry", &Font20, randomColor, WHITE);
    LCD_1IN28_Display(BlackImage);
    DEV_Delay_ms(1000); // Delay for 1 second (adjust as needed)
}
