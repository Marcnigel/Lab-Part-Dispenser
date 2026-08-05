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

#define COLOR_BG        RA8875_BLACK

#define COLOR_TAB_ACTIVE   0xFBE0    // vibrant orange
#define COLOR_TAB_INACTIVE 0xCB20    // vibrant deep orange

#define COLOR_PLUS      0xFBE0
#define COLOR_MINUS     0xCB20

#define COLOR_WHITE     RA8875_WHITE
#define COLOR_TEXT      RA8875_BLACK
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_LOADED    COLOR_MINUS  // dark orange, used for the LOADED reload-button state

//=========================================================
// Page state
//=========================================================
enum Page { PAGE_DISPENSE, PAGE_RELOAD };
Page currentPage = PAGE_DISPENSE;

// Warning popup — when visible, it captures all touches until closed
bool popupVisible = false;

//=========================================================
// Component names — grid layout:
//   Col1: Wire 1 / Capacitor 1 / Capacitor 2 / Capacitor 3
//   Col2: Wire 2
//   Col3: Wire 3 / Resistor 1 / Resistor 2 / Resistor 3
//=========================================================
const char* parts[] = {
  "Wire 1(cm)",
  "Wire 2(cm)",
  "Wire 3(cm)",
  "Capacitor 1",
  "Capacitor 2",
  "Capacitor 3",
  "Resistor 1",
  "Resistor 2",
  "Resistor 3"
};

// Wire labels for the Load page — no "cm" suffix there, since the Load
// page is about the dispenser's stock/load state, not a length selection
const char* wireLoadLabel[3] = {
  "Wire 1",
  "Wire 2",
  "Wire 3"
};

const uint8_t NUM_PARTS = 9;

// Which grid column (0,1,2) and row (0-3) each part lives in
const uint8_t partGridCol[NUM_PARTS] = {0, 1, 2, 0, 0, 0, 2, 2, 2};
const uint8_t partGridRow[NUM_PARTS] = {0, 0, 0, 1, 2, 3, 1, 2, 3};

// Quantity selected for each part
int quantities[NUM_PARTS] = {0};

// Enable/disable individual parts (all enabled by default)
const bool partEnabled[NUM_PARTS] = {
    true, true, true,
    true, true, true,
    true, true, true
};

// Component type per part — wire is measured in cm, capacitor/resistor by count
enum PartType { TYPE_WIRE, TYPE_CAPACITOR, TYPE_RESISTOR };

const PartType partType[NUM_PARTS] = {
    TYPE_WIRE,      TYPE_WIRE,      TYPE_WIRE,       // Wire 1/2/3
    TYPE_CAPACITOR, TYPE_CAPACITOR, TYPE_CAPACITOR,  // Capacitor 1/2/3
    TYPE_RESISTOR,  TYPE_RESISTOR,  TYPE_RESISTOR    // Resistor 1/2/3
};

// Reload-page status for the resistor/capacitor dispensers.
// (Wire dispensers don't use this — their Reload button always behaves normally.)
enum DispenserFlag { FLAG_EMPTY, FLAG_LOADED, FLAG_REMOVE };

DispenserFlag dispenserFlag[NUM_PARTS] = {
    FLAG_EMPTY, FLAG_EMPTY, FLAG_EMPTY, // Wire 1/2/3 — unused
    FLAG_EMPTY, FLAG_EMPTY, FLAG_EMPTY, // Capacitor 1/2/3
    FLAG_EMPTY, FLAG_EMPTY, FLAG_EMPTY  // Resistor 1/2/3
};

// Wire dispensers only track two states, shown as a small status dot
// next to each wire option on the Reload page.
enum WireStock { WIRE_EMPTY, WIRE_INSTOCK };

WireStock wireStock[NUM_PARTS] = {
    WIRE_EMPTY, WIRE_EMPTY, WIRE_EMPTY, // Wire 1/2/3
    WIRE_EMPTY, WIRE_EMPTY, WIRE_EMPTY, // unused (Capacitor)
    WIRE_EMPTY, WIRE_EMPTY, WIRE_EMPTY  // unused (Resistor)
};

// Number of kits selected
int numKits = 0;

//=========================================================
// Layout constants
//=========================================================
const int TAB_H = 50;

// Grid geometry
const int colX[3] = {20, 190, 360};
const int rowY[4] = {70, 170, 270, 370};

const int BTN_W = 42;
const int BTN_H = 54;
const int QTY_W = 60;
const int QTY_H = 40;      // used by the "# of Kits" quantity box
const int GRID_QTY_H = 60; // taller quantity box used in the component grid
const int GAP   = 6;

const int LABEL_H = 34; // height reserved for the component label text
const int LABEL_TO_BUTTON_OFFSET = 36; // vertical gap between label and button row

// Kits panel
const int kitPanelX = 560;
const int kitPanelY = 70;
const int kitPanelW = 220;
const int kitPanelH = 125;

const int KIT_BTN_W = 45;
const int KIT_BTN_H = BTN_H; // match the component grid's button height

// Evenly spaced using the same GAP as the component grid, centered in the panel
const int kitMinusX = kitPanelX + (kitPanelW - (KIT_BTN_W + GAP + QTY_W + GAP + KIT_BTN_W)) / 2;
const int kitQtyX   = kitMinusX + KIT_BTN_W + GAP;
const int kitPlusX  = kitQtyX + QTY_W + GAP;
const int kitButtonY = 130;

// Dispense button
const int dispenseBtnX = 560;
const int dispenseBtnY = 210;
const int dispenseBtnW = 230;
const int dispenseBtnH = 200;

// Reload page — single wide "Reload" button per part, same footprint as
// the minus/qty/plus block it replaces
const int RELOAD_BTN_W = BTN_W + GAP + QTY_W + GAP + BTN_W; // 156
const int RELOAD_BTN_H = 50;

// Reload All button — reuses the Dispense button's slot on the right panel
const int reloadAllBtnX = dispenseBtnX;
const int reloadAllBtnY = dispenseBtnY;
const int reloadAllBtnW = dispenseBtnW;
const int reloadAllBtnH = dispenseBtnH;

// Wire in-stock/empty status dot — sits in the top-right corner of each
// wire option's label, on the Reload page only.
const int WIRE_DOT_RADIUS = 6;
const int WIRE_DOT_MARGIN = 10; // distance from the right edge of the label block

// Stock legend — occupies the same right-panel slot the "# of Kits" box
// uses on the Dispense page, since Reload has nothing there.
const int legendX = kitPanelX;
const int legendY = kitPanelY;
const int legendW = kitPanelW;
const int legendH = kitPanelH;

const int legendDotX    = legendX + 24;
const int legendRow1Y   = legendY + 50;
const int legendRow2Y   = legendY + 95;
const int legendTextX   = legendDotX + 18;

// Warning popup
const int POPUP_W = 420;
const int POPUP_H = 200;
const int popupX = (SCREEN_WIDTH - POPUP_W) / 2;
const int popupY = (SCREEN_HEIGHT - POPUP_H) / 2;

const int POPUP_CLOSE_SIZE = 32; // tappable red X, top-right corner of the popup
const int popupCloseX = popupX + POPUP_W - POPUP_CLOSE_SIZE - 10;
const int popupCloseY = popupY + 10;

//=========================================================
// Helpers for computing per-part button positions
//=========================================================
int partMinusX(int i) { return colX[partGridCol[i]]; }
int partQtyX(int i)   { return partMinusX(i) + BTN_W + GAP; }
int partPlusX(int i)  { return partQtyX(i) + QTY_W + GAP; }
int partLabelY(int i) { return rowY[partGridRow[i]]; }
int partButtonY(int i){ return partLabelY(i) + LABEL_TO_BUTTON_OFFSET; }
int partReloadBtnX(int i) { return colX[partGridCol[i]]; }

bool pointInRect(int px, int py, int x, int y, int w, int h)
{
    return (px >= x && px <= (x + w) &&
            py >= y && py <= (y + h));
}

void redrawQuantity(int index)
{
    drawQuantityBox(partQtyX(index), partButtonY(index) - 3, QTY_W, GRID_QTY_H, quantities[index]);
}

void redrawKitQuantity()
{
    drawQuantityBox(kitQtyX, kitButtonY, QTY_W, GRID_QTY_H, numKits);
}

void showDispenseList()
{
    Serial.println();
    Serial.println("Dispensing:");

    for (int i = 0; i < NUM_PARTS; i++)
    {
        if (quantities[i] > 0)
        {
            Serial.print(parts[i]);
            if (partType[i] == TYPE_WIRE)
            {
                Serial.print(": ");
                Serial.print(quantities[i] * numKits);
                Serial.println(" cm");
            }
            else
            {
                Serial.print(" x ");
                Serial.println(quantities[i] * numKits);
            }
        }
    }

    Serial.print("Number of Kits: ");
    Serial.println(numKits);
    Serial.println("----------------------");
}

//=========================================================
// Reload actions — placeholders only.
// TODO: each dispenser type reloads differently — fill in the
// actual motor-driving code for that type in its own function below.
//=========================================================
void reloadWire(int index)
{
    // TODO: activate the wire dispenser motor for this slot
}

void reloadCapacitor(int index)
{
    // TODO: activate the capacitor dispenser motor for this slot
}

void reloadResistor(int index)
{
    // TODO: activate the resistor dispenser motor for this slot
}

void reloadPart(int index)
{
    Serial.print("Load requested for: ");
    Serial.println(parts[index]);

    switch (partType[index])
    {
        case TYPE_WIRE:      reloadWire(index);      break;
        case TYPE_CAPACITOR: reloadCapacitor(index); break;
        case TYPE_RESISTOR:  reloadResistor(index);  break;
    }
}

// Called when a resistor/capacitor dispenser button is tapped while flagged REMOVE
void removeExtra(int index)
{
    Serial.print("Remove Extra requested for: ");
    Serial.println(parts[index]);
    // TODO: activate the motor/mechanism that ejects the excess part
}

void loadResistorsAndCapacitors()
{
    Serial.println("Load Resistors/Capacitors requested");
    for (int i = 0; i < NUM_PARTS; i++)
    {
        if (partType[i] != TYPE_WIRE)
        {
            reloadPart(i);
        }
    }
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

// Left-aligned text, vertically centered on y (used next to the legend dots)
void drawLeftText(int x, int y,
                  const char *text,
                  uint16_t textColor,
                  uint16_t bgColor,
                  uint8_t size)
{
    int charHeight = 16 * (size + 1);

    tft.textMode();
    tft.textColor(textColor, bgColor);
    tft.textEnlarge(size);
    tft.textSetCursor(x, y - charHeight / 2);
    tft.textWrite(text);
    tft.graphicsMode();
}

void drawButton(int x, int y, int w, int h,
                uint16_t fillColor,
                uint16_t textColor,
                const char* label,
                uint8_t textSize = 1)
{
    tft.fillRoundRect(x, y, w, h, 8, fillColor);
    tft.drawRoundRect(x, y, w, h, 8, RA8875_WHITE);

    drawCenteredText(x, y, w, h,
                     label,
                     textColor,
                     fillColor,
                     textSize);
}

void drawButtonTwoLine(int x, int y, int w, int h,
                       uint16_t fillColor,
                       uint16_t textColor,
                       const char* line1,
                       const char* line2,
                       uint8_t size)
{
    tft.fillRoundRect(x, y, w, h, 8, fillColor);
    tft.drawRoundRect(x, y, w, h, 8, RA8875_WHITE);

    int lineHeight = 16 * (size + 1);
    int lineGap = 6;
    int blockH = lineHeight * 2 + lineGap;
    int startY = y + (h - blockH) / 2;

    drawCenteredText(x, startY, w, lineHeight, line1, textColor, fillColor, size);
    drawCenteredText(x, startY + lineHeight + lineGap, w, lineHeight, line2, textColor, fillColor, size);
}

void drawQuantityBox(int x, int y, int w, int h, int value)
{
    char buffer[6];
    sprintf(buffer, "%d", value);

    tft.fillRect(x, y, w, h, RA8875_WHITE);
    tft.drawRect(x, y, w, h, RA8875_BLACK);

    drawCenteredText(x, y, w, h,
                     buffer,
                     RA8875_BLACK,
                     RA8875_WHITE,
                     1);
}

//=========================================================
// Top tab bar — toggles between Dispense / Reload
//=========================================================
void drawTabBar()
{
    uint16_t dispenseColor = (currentPage == PAGE_DISPENSE) ? COLOR_TAB_ACTIVE : COLOR_TAB_INACTIVE;
    uint16_t reloadColor   = (currentPage == PAGE_RELOAD)   ? COLOR_TAB_ACTIVE : COLOR_TAB_INACTIVE;

    tft.fillRect(0, 0, SCREEN_WIDTH / 2, TAB_H, dispenseColor);
    tft.fillRect(SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, TAB_H, reloadColor);

    drawCenteredText(0, 0, SCREEN_WIDTH / 2, TAB_H,
                     "DISPENSE",
                     COLOR_WHITE,
                     dispenseColor,
                     2);

    drawCenteredText(SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, TAB_H,
                     "LOAD",
                     COLOR_WHITE,
                     reloadColor,
                     2);
}

//=========================================================
// Dispense page
//=========================================================
void drawDispensePage()
{
    // Component grid
    for (int i = 0; i < NUM_PARTS; i++)
    {
        int labelY = partLabelY(i);
        int minusX = partMinusX(i);
        int qtyX   = partQtyX(i);
        int plusX  = partPlusX(i);
        int buttonY = partButtonY(i);
        int blockW = (plusX + BTN_W) - minusX;

        drawCenteredText(minusX, labelY, blockW, LABEL_H,
                         parts[i],
                         RA8875_WHITE,
                         RA8875_BLACK,
                         1);

        drawButton(minusX, buttonY, BTN_W, BTN_H,
                   COLOR_MINUS, RA8875_WHITE, "-");

        drawQuantityBox(qtyX, buttonY - 3, QTY_W, GRID_QTY_H, quantities[i]);

        drawButton(plusX, buttonY, BTN_W, BTN_H,
                   COLOR_PLUS, RA8875_WHITE, "+");
    }

    //=================================================
    // Number of Kits section
    //=================================================
    tft.fillRect(kitPanelX, kitPanelY, kitPanelW, kitPanelH, RA8875_DARKGREY);

    drawCenteredText(kitPanelX, kitPanelY + 5, kitPanelW, 40,
                     "# of Kits",
                     RA8875_WHITE,
                     RA8875_DARKGREY,
                     1);

    drawButton(kitMinusX, kitButtonY, KIT_BTN_W, KIT_BTN_H,
               COLOR_MINUS, RA8875_WHITE, "-");

    drawQuantityBox(kitQtyX, kitButtonY, QTY_W, GRID_QTY_H, numKits);

    drawButton(kitPlusX, kitButtonY, KIT_BTN_W, KIT_BTN_H,
               COLOR_PLUS, RA8875_WHITE, "+");

    //=================================================
    // Dispense Button
    //=================================================
    drawButton(dispenseBtnX, dispenseBtnY, dispenseBtnW, dispenseBtnH,
               COLOR_PLUS, RA8875_WHITE, "DISPENSE");
}

//=========================================================
// Reload page
//=========================================================
void drawReloadPage()
{
    // One "Load" button per part, same grid position as the Dispense page
    for (int i = 0; i < NUM_PARTS; i++)
    {
        int labelY = partLabelY(i);
        int btnX = partReloadBtnX(i);
        int buttonY = partButtonY(i);

        const char* labelText = (partType[i] == TYPE_WIRE) ? wireLoadLabel[i] : parts[i];

        drawCenteredText(btnX, labelY, RELOAD_BTN_W, LABEL_H,
                         labelText,
                         RA8875_WHITE,
                         RA8875_BLACK,
                         1);

        // Wire stock status dot — green (in stock) / red (empty)
        if (partType[i] == TYPE_WIRE)
        {
            int dotX = btnX + RELOAD_BTN_W - WIRE_DOT_MARGIN;
            int dotY = labelY + LABEL_H / 2;
            uint16_t dotColor = (wireStock[i] == WIRE_INSTOCK) ? COLOR_GREEN : COLOR_RED;

            tft.fillCircle(dotX, dotY, WIRE_DOT_RADIUS, dotColor);
            tft.drawCircle(dotX, dotY, WIRE_DOT_RADIUS, RA8875_WHITE);
        }

        // Wire dispensers always show a normal, pressable Load button.
        // Resistor/capacitor dispensers depend on their current flag:
        //   EMPTY  -> normal orange "Load" button, pressable
        //   LOADED -> dark orange "LOADED" button, not pressable
        //   REMOVE -> red "Remove Extra" button, pressable
        uint16_t fillColor = COLOR_PLUS;
        const char* label = "Load";
        uint8_t labelSize = 1;

        if (partType[i] != TYPE_WIRE)
        {
            switch (dispenserFlag[i])
            {
                case FLAG_EMPTY:
                    fillColor = COLOR_PLUS;
                    label = "Load";
                    labelSize = 1;
                    break;
                case FLAG_LOADED:
                    fillColor = COLOR_LOADED;
                    label = "LOADED";
                    labelSize = 1;
                    break;
                case FLAG_REMOVE:
                    fillColor = COLOR_RED;
                    label = "Remove Extra";
                    labelSize = 0; // longer label needs the smaller text size to fit
                    break;
            }
        }

        // NOTE: this is where the motor-drive call for this dispenser
        // slot gets triggered — see reloadPart(i) / removeExtra(i) in the touch handler.
        drawButton(btnX, buttonY, RELOAD_BTN_W, RELOAD_BTN_H,
                   fillColor, RA8875_WHITE, label, labelSize);
    }

    //=================================================
    // Wire stock legend
    //=================================================
    tft.fillRect(legendX, legendY, legendW, legendH, RA8875_DARKGREY);

    drawCenteredText(legendX, legendY + 5, legendW, 30,
                     "Wire Status",
                     RA8875_WHITE,
                     RA8875_DARKGREY,
                     1);

    tft.fillCircle(legendDotX, legendRow1Y, WIRE_DOT_RADIUS, COLOR_GREEN);
    tft.drawCircle(legendDotX, legendRow1Y, WIRE_DOT_RADIUS, RA8875_WHITE);
    drawLeftText(legendTextX, legendRow1Y, "In Stock", RA8875_WHITE, RA8875_DARKGREY, 1);

    tft.fillCircle(legendDotX, legendRow2Y, WIRE_DOT_RADIUS, COLOR_RED);
    tft.drawCircle(legendDotX, legendRow2Y, WIRE_DOT_RADIUS, RA8875_WHITE);
    drawLeftText(legendTextX, legendRow2Y, "Empty", RA8875_WHITE, RA8875_DARKGREY, 1);

    //=================================================
    // Load Resistors/Capacitors button
    //=================================================
    // NOTE: this is where the "load every resistor/capacitor dispenser"
    // motor sequence gets triggered — see reloadAllParts() in the touch handler.
    drawButtonTwoLine(reloadAllBtnX, reloadAllBtnY, reloadAllBtnW, reloadAllBtnH,
                      COLOR_PLUS, RA8875_WHITE, "Load Resistors", "and Capacitors", 1);
}

void drawInterface()
{
    tft.fillScreen(RA8875_BLACK);

    drawTabBar();

    if (currentPage == PAGE_DISPENSE)
        drawDispensePage();
    else
        drawReloadPage();
}

//=========================================================
// Warning popup — a white box with a message, closed via the
// red X in its top-right corner. Call showWarningPopup(msg) to
// display it; it blocks other touches until closed.
//=========================================================
void showWarningPopup(const char* message)
{
    popupVisible = true;

    tft.fillRect(popupX, popupY, POPUP_W, POPUP_H, RA8875_WHITE);
    tft.drawRect(popupX, popupY, POPUP_W, POPUP_H, RA8875_BLACK);

    // Message area (leaves room for the close button at the top-right)
    drawCenteredText(popupX + 10, popupY + 10, POPUP_W - 20, POPUP_H - 20,
                     message,
                     RA8875_BLACK,
                     RA8875_WHITE,
                     1);

    // Red X close button
    drawCenteredText(popupCloseX, popupCloseY, POPUP_CLOSE_SIZE, POPUP_CLOSE_SIZE,
                     "X",
                     COLOR_RED,
                     RA8875_WHITE,
                     1);
}

void closeWarningPopup()
{
    popupVisible = false;
    drawInterface();
}

void setup()
{
    Serial.begin(9600);

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

    static unsigned long lastTouchTime = 0; // Tracks the time of the last valid press

    if (!tft.touched())
        return;

    if (!tft.touchRead(&rawX, &rawY))
        return;

    // Reject the touch if it happened too quickly after the last registered click
    if (millis() - lastTouchTime < 300)
        return;

    // Successfully registered a real press, update the timestamp
    lastTouchTime = millis();

    // Convert to screen coordinates
    touchToScreen(rawX, rawY, x, y);

    //-----------------------------------------
    // Warning popup — captures all touches while visible
    //-----------------------------------------
    if (popupVisible)
    {
        if (pointInRect(x, y, popupCloseX, popupCloseY, POPUP_CLOSE_SIZE, POPUP_CLOSE_SIZE))
        {
            closeWarningPopup();
        }
        return; // swallow all other touches until the popup is closed
    }

    //-----------------------------------------
    // Tab bar (always active, on any page)
    //-----------------------------------------
    if (pointInRect(x, y, 0, 0, SCREEN_WIDTH / 2, TAB_H))
    {
        if (currentPage != PAGE_DISPENSE)
        {
            currentPage = PAGE_DISPENSE;
            drawInterface();
        }
        return;
    }

    if (pointInRect(x, y, SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, TAB_H))
    {
        if (currentPage != PAGE_RELOAD)
        {
            currentPage = PAGE_RELOAD;
            drawInterface();
        }
        return;
    }

    //-----------------------------------------
    // Reload page
    //-----------------------------------------
    if (currentPage == PAGE_RELOAD)
    {
        for (int i = 0; i < NUM_PARTS; i++)
        {
            if (pointInRect(x, y, partReloadBtnX(i), partButtonY(i), RELOAD_BTN_W, RELOAD_BTN_H))
            {
                if (partType[i] == TYPE_WIRE)
                {
                    reloadPart(i);
                }
                else if (dispenserFlag[i] == FLAG_EMPTY)
                {
                    reloadPart(i);
                }
                else if (dispenserFlag[i] == FLAG_REMOVE)
                {
                    removeExtra(i);
                }
                // FLAG_LOADED: button is not pressable, ignore the tap
                return;
            }
        }

        if (pointInRect(x, y, reloadAllBtnX, reloadAllBtnY, reloadAllBtnW, reloadAllBtnH))
        {
            loadResistorsAndCapacitors();
            return;
        }

        return;
    }

    //-----------------------------------------
    // Component buttons
    //-----------------------------------------
    for (int i = 0; i < NUM_PARTS; i++)
    {
        int buttonY = partButtonY(i);

        // Minus button
        if (pointInRect(x, y, partMinusX(i), buttonY, BTN_W, BTN_H))
        {
            if (partEnabled[i] && quantities[i] > 0)
            {
                quantities[i]--;
                redrawQuantity(i);
            }
            return;
        }

        // Plus button
        if (pointInRect(x, y, partPlusX(i), buttonY, BTN_W, BTN_H))
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
    if (pointInRect(x, y, kitMinusX, kitButtonY, KIT_BTN_W, KIT_BTN_H))
    {
        if (numKits > 0)
        {
            numKits--;
            redrawKitQuantity();
        }
        return;
    }

    if (pointInRect(x, y, kitPlusX, kitButtonY, KIT_BTN_W, KIT_BTN_H))
    {
        if (numKits < 10)
        {
            numKits++;
            redrawKitQuantity();
        }
        return;
    }

    //-----------------------------------------
    // Dispense button
    //-----------------------------------------
    if (pointInRect(x, y, dispenseBtnX, dispenseBtnY, dispenseBtnW, dispenseBtnH))
    {
        showDispenseList();
        return;
    }
}