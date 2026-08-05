#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_RA8875.h"

#define RA8875_CS     8
#define RA8875_RESET  9
#define RA8875_INT    10
#define RA8875_DARKGREY 0x4208

#define X_MIN 58
#define X_MAX 960
#define Y_MIN 150
#define Y_MAX 916

#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 480

Adafruit_RA8875 tft = Adafruit_RA8875(RA8875_CS, RA8875_RESET);

// Component names
const char* parts[] = {
  "Wire Set",
  "1k Resistor",
  "1nF Capacitor",
  "10k Resistor",
  "10nF Capacitor",
  "Red LED",
  "1N4148 Diode"
};

const uint8_t NUM_PARTS = 7;

// Quantity selected for each part
int quantities[NUM_PARTS] = {0};

// Only these parts are currently enabled
const bool partEnabled[NUM_PARTS] = {
    true,   // Wire Set
    true,   // 1k Resistor
    true,   // 1nF Capacitor
    false,  // 10k Resistor
    false,  // 10nF Capacitor
    false,  // Red LED
    false   // 1N4148 Diode
};

// Number of kits selected
int numKits = 0;

const int startY = 68;
const int rowSpacing = 42;

const int minusX = 315;
const int quantityX = 375;
const int plusX = 450;

const int buttonYOffset = 0;

const int buttonW = 42;
const int buttonH = 34;

const int kitMinusX = 565;
const int kitPlusX  = 700;
const int kitButtonY = 140;

bool pointInRect(int px, int py, int x, int y, int w, int h)
{
    return (px >= x && px <= (x + w) &&
            py >= y && py <= (y + h));
}

void redrawQuantity(int index)
{
    int rowY = startY + index * rowSpacing;
    drawQuantityBox(quantityX, rowY - 3, quantities[index]);
}

void redrawKitQuantity()
{
    drawQuantityBox(625, 140, numKits);
}

void showDispenseList()
{
    Serial.println();
    Serial.println("Dispensing:");

    if (quantities[0] > 0)
    {
        Serial.print("Wire Set (Red, White, Black) x ");
        Serial.println(quantities[0] * numKits);
    }

    if (quantities[1] > 0)
    {
        Serial.print("1k Resistor x ");
        Serial.println(quantities[1] * numKits);
    }

    if (quantities[2] > 0)
    {
        Serial.print("1nF Capacitor x ");
        Serial.println(quantities[2] * numKits);
    }

    Serial.print("Number of Kits: ");
    Serial.println(numKits);
    Serial.println("----------------------");
}

void touchToScreen(uint16_t rawX, uint16_t rawY, int &x, int &y)
{
    // Map X
    x = (int)((rawX - X_MIN) * (float)SCREEN_WIDTH / (X_MAX - X_MIN));

    // Map Y
    y = (int)((rawY - Y_MIN) * (float)SCREEN_HEIGHT / (Y_MAX - Y_MIN));

    // Clamp safety
    if (x < 0) x = 0;
    if (x > SCREEN_WIDTH) x = SCREEN_WIDTH;

    if (y < 0) y = 0;
    if (y > SCREEN_HEIGHT) y = SCREEN_HEIGHT;
}

//=========================================================
// Draw centered text inside a rectangle
//=========================================================
void drawCenteredText(int x, int y, int w, int h,
                      const char *text,
                      uint16_t textColor,
                      uint16_t bgColor,
                      uint8_t size)
{
    int charWidth = 8 * (size + 1);
    int charHeight = 16 * (size + 1);

    int textWidth = strlen(text) * charWidth;

    int tx = x + (w - textWidth) / 2;
    int ty = y + (h - charHeight) / 2;

    tft.textMode();
    tft.textColor(textColor, bgColor);
    tft.textEnlarge(size);
    tft.textSetCursor(tx, ty);
    tft.textWrite(text);
    tft.graphicsMode();
}

void drawButton(int x, int y, int w, int h,
                uint16_t fillColor,
                uint16_t textColor,
                const char* label)
{
    tft.fillRoundRect(x, y, w, h, 8, fillColor);
    tft.drawRoundRect(x, y, w, h, 8, RA8875_WHITE);

    drawCenteredText(x, y, w, h,
                     label,
                     textColor,
                     fillColor,
                     1);
}

void drawQuantityBox(int x, int y, int value)
{
    char buffer[6];
    sprintf(buffer, "%d", value);

    tft.fillRect(x, y, 60, 40, RA8875_WHITE);
    tft.drawRect(x, y, 60, 40, RA8875_BLACK);

    drawCenteredText(x, y, 60, 40,
                     buffer,
                     RA8875_BLACK,
                     RA8875_WHITE,
                     1);
}

void drawInterface()
{
  tft.fillScreen(RA8875_BLACK);

  //=================================================
  // Title Bar
  //=================================================
  tft.fillRect(0, 0, 800, 50, RA8875_RED);

  tft.textMode();
  tft.textColor(RA8875_WHITE, RA8875_RED);
  drawCenteredText(
    0, 0, 800, 50,
    "ENSC120 Lab Component Selector",
    RA8875_WHITE,
    RA8875_RED,
    2
  );
  tft.graphicsMode();

  //=================================================
  // Component rows
  //=================================================
  int startY = 68;
  const int rowSpacing = 42;

  for(int i = 0; i < NUM_PARTS; i++)
  {
    int rowY = startY + i * rowSpacing;

    // Component name
    tft.textMode();
    tft.textColor(RA8875_WHITE, RA8875_BLACK);
    tft.textEnlarge(1);
    tft.textSetCursor(20, rowY + 9);
    tft.textWrite(parts[i]);
    tft.graphicsMode();

  drawButton(315, rowY, 42, 34,
           RA8875_RED,
           RA8875_WHITE,
           "-");

  drawQuantityBox(375, rowY - 3,
                quantities[i]);

  drawButton(450, rowY, 42, 34,
           RA8875_GREEN,
           RA8875_WHITE,
           "+");
  }

  //=================================================
  // Number of Kits section
  //=================================================
  tft.fillRect(550, 80, 220, 120, RA8875_DARKGREY);

  tft.textMode();
  tft.textColor(RA8875_WHITE, RA8875_DARKGREY);
  drawCenteredText(
    550,
    85,
    220,
    40,
    "# of Kits",
    RA8875_WHITE,
    RA8875_DARKGREY,
    1
  );
  tft.graphicsMode();

  drawButton(565, 140, 45, 40,
           RA8875_RED,
           RA8875_WHITE,
           "-");

  drawQuantityBox(625, 140,
                numKits);

  drawButton(700, 140, 45, 40,
           RA8875_GREEN,
           RA8875_WHITE,
           "+");

  //=================================================
  // Dispense Button
  //=================================================
  drawButton(
    550,
    300,
    220,
    90,
    RA8875_GREEN,
    RA8875_WHITE,
    "DISPENSE"
);
}

void setup()
{
    Serial.begin(115200);

    if (!tft.begin(RA8875_800x480))
    {
        Serial.println("RA8875 not found");
        while (1);
    }

    tft.displayOn(true);
    tft.GPIOX(true);

    tft.PWM1config(true, RA8875_PWM_CLK_DIV1024);
    tft.PWM1out(255);

    // Enable the touch controller
    tft.touchEnable(true);

    drawInterface();
}

void loop()
{
    uint16_t rawX, rawY;
    int x, y;

    static bool wasTouched = false;

    // Nothing touching — reset for the next press
    if (!tft.touched())
    {
        wasTouched = false;
        return;
    }

    // Always read to clear the RA8875 touch interrupt flag
    bool gotPoint = tft.touchRead(&rawX, &rawY);

    // Already handled this press — wait for release
    if (wasTouched)
        return;

    if (!gotPoint)
        return;

    // Lock this press until the finger lifts
    wasTouched = true;

    // Convert to screen coordinates
    touchToScreen(rawX, rawY, x, y);

    //-----------------------------------------
    // Component buttons
    //-----------------------------------------
    for (int i = 0; i < NUM_PARTS; i++)
    {
        int rowY = startY + i * rowSpacing;

        // Minus button
        if (pointInRect(x, y, minusX, rowY, buttonW, buttonH))
        {
            if (partEnabled[i] && quantities[i] > 0)
            {
                quantities[i]--;
                redrawQuantity(i);
            }
            return;
        }

        // Plus button
        if (pointInRect(x, y, plusX, rowY, buttonW, buttonH))
        {
            if (partEnabled[i] && quantities[i] < 9)
            {
                quantities[i]++;
                redrawQuantity(i);
            }
            return;
        }
    }

    //-----------------------------------------
    // Number of kits
    //-----------------------------------------
    if (pointInRect(x, y, kitMinusX, kitButtonY, 45, 40))
    {
        if (numKits > 0)
        {
            numKits--;
            redrawKitQuantity();
        }
        return;
    }

    if (pointInRect(x, y, kitPlusX, kitButtonY, 45, 40))
    {
        if (numKits < 9)
        {
            numKits++;
            redrawKitQuantity();
        }
        return;
    }

    //-----------------------------------------
    // Dispense button
    //-----------------------------------------
    if (pointInRect(x, y, 550, 300, 220, 90))
    {
        showDispenseList();
        return;
    }
}