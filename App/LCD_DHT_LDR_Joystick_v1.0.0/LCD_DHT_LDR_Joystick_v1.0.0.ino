// Environmental Data Collector with LCD Screen & User-defined LED Indicator(s)
// Version: 1.0.1
// By: Cole B. Prather
// Date: April 1, 2021
//
// This Arduino code is written to collect data from a DHT11 Temperature & Humidity sensor and an LDR
// as well as take input from a joystick module to control the display of an LCD and an LED.
//
// The design of the main "menu" is based on a matrix-like structure that is slightly modified from the 
// classical definition of a matrix. Matrices are typically defined as Aij where i is the index of the
// row and j is the index of the column. Spatially, shifting through the i (row) index is like shifting
// in the y-direction (moving up and down through the column). Likewise, shifting through the j (column)
// index is like shfting in the x-direction (moving left and right through the row). 
//      
//          [A00 A01 A02 ... A0j]
//          [A10 A11 A12 ... A1j]
//    Aij = [A20 A21 A22 ... A2j]
//          [ :   :   :   :   : ]
//          [Ai0 Ai1 Ai2 ... Aij]
//
// Which could reasonably be expressed as:
//
//    Aij = Ayx ; where i is the row index, the y-position, and j is the column index, the x-position.
//
// To make movement through this matrix more natural (or to simply bend my brain a bit), the matrix is 
// rotated 90 degrees CCW to produce the following:
//
//          [A0i A1i A2i ... Aji]
//          [ :   :   :   :   : ]
//    Aji = [A02 A12 A22 ... Aj2] 
//          [A01 A11 A21 ... Aj1]
//          [A00 A10 A20 ... Aj0]
//
// Now, the matrix can be expressed as:
// 
//    Aji = Axy ; where j is the column index, the x-position, and i is the row index, the y-position. 
//
// This is the notation that will be used in the following program.

#include <LiquidCrystal.h>    // Include LCD library
#include <DHT.h>              // Include DHT library

#define DHTPIN A2             // Define DHT analog pin
#define DHTTYPE DHT11         // Define DHT type

const int LDR = A1;           // Initialize LDR analog pin
const int VRx = A3;           // Initialize joystick-x analog pin
const int VRy = A4;           // Initialize joystick-y analog pin
const int SW = 8;             // Initialize joystick switch digital pin
const int bluePin = 9;        // Initialize LED blue leg digital pin
const int greenPin = 10;      // Initialize LED green leg digital pin
const int redPin = 11;        // Initialize LED red leg digital pin
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // Initialize LCD digital pins
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                // Initialize LCD function from library
DHT dht(DHTPIN, DHTTYPE);     // Initialize DHT function from library

bool button = false;          // Initialize button as a boolean
bool toggle = false;          // Initialize toggle as a boolean
bool hold = false;            // Initialize hold as a boolean

byte Check[] = {              // Initialize Check as a byte
  B00000,
  B00001,
  B00011,
  B10110,
  B11100,
  B01000,
  B00000,
  B00000
};
byte Lock[] = {               // Initialize Lock as a byte
  B01110,
  B10001,
  B10001,
  B11111,
  B11011,
  B11011,
  B11111,
  B00000
};
byte Arrow[] = {              // Initialize Arrow as a byte
  B00000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B00000,
  B00000
};

float h;                      // Initialize measured humidity
float t;                      // Initialize measured temperature in degrees C
float f;                      // Initialize measured temperature in degrees F
float hif;                    // Initialize measured temperature in degrees F
float hic;                    // Initialize measured temperature in degrees C
float value = 0;              // Initialize user-defined value
float savedvalue = 0;         // Initialize saved user-defined value
float usertOpt = 0;           // Initialize user-defined optimal temperature
float userhOpt = 0;           // Initialize user-defined optimal humidity
float usertSMax = 0;          // Initialize user-defined soft-maximum temperature
float usertHMax = 0;          // Initialize user-defined hard-maximum temperature
float usertHMin = 0;          // Initialize user-defined hard-minimum temperature
float usertSMin = 0;          // Initialize user-defined soft-minimum temperature
float userhSMax = 0;          // Initialize user-defined soft-maximum humidity
float userhHMax = 0;          // Initialize user-defined hard-maximum humidity
float userhHMin = 0;          // Initialize user-defined hard-minimum humidity
float userhSMin = 0;          // Initialize user-defined soft-minimum humidity

int brightness = 0;           // Initialize brightness variable for LDR
int joythresh = -250;         // Initialize joystick position threshold (defined as negative)
int xPosition = 0;            // Initialize joystick-x position
int yPosition = 0;            // Initialize joystick-y position
int mapX = 0;                 // Initialize mapped x position
int mapY = 0;                 // Initialize mapped y position
int calX = 0;                 // Initialize calibrated x position
int calY = 0;                 // Initialize calibrated y position
int mainrownum = 2;           // Initialize number of rows in the main display matrix [here, 3 (# = n+1)]
int maincolnum = 2;           // Initialize number of columns in the main display matrix [here, 3 (# = n+1)]
int mainrowpos = 0;           // Initialize user row position in the main display matrix
int maincolpos = 0;           // Initialize user column position in the main display matrix
int numrownum = 9;            // Initialize number of rows in the number matrix [here, 10 (# = n+1)]
int numcolnum = 4; // needed? // Initialize number of columns in the number matrix [here, 5 (# = n+1)]
int numrowpos = 0;            // Initialize user row position in the number matrix
int numcolpos = 0;            // Initialize user column position in the number matrix
int optrownum = 1;            // Initialize number of rows in the main options matrix [here, 2 (# = n+1)]
int optcolnum = 7;            // Initialize number of columns in the options matrix [here, 8 (# = n+1)]
int optrowpos = 0;            // Initialize user row position in the options matrix
int optcolpos = 0;            // Initialize user column position in the options matrix
int opt2rownum = 4;           // Initialize number of rows for the embedded options [here, 5 (# = n+1)]
int opt2rowpos = 0;           // Initialize user row position in the embedded options 
int tensdigit = 0;            // Initialize value in the tens digit
int onesdigit = 0;            // Initialize value in the ones digit
int tenthsdigit = 0;          // Initialize value in the tenths digit
int hundthdigit = 0;          // Initialize value in the hundredths digit
int prevcolpos = 0;           // Initialize user previous column position in the options matrix
int varrowpos = 0;            // Initialize user selected row position in the options matrix
int var2rowpos = 0;           // Initialize user selected row position in the embedded options

const long jsPositionDelay = 300;         // joystick position response delay in milliseconds
const long toggleDelay = 300;             // toggle position response delay in milliseconds
const long cursorDelay = 50;              // cursor blink delay in milliseconds
unsigned long previousMillis = 0;         // last time the joystick position was updated
unsigned long currentMillis = 0;          // time the joystick position is updated
unsigned long lastButtonDownTime = 0;     // last time the button was pushed
unsigned long lastButtonUpTime = 0;       // last time the button was released
unsigned long toggleTime = 0;             // time the joystick is held to trigger a toggle

String Screen = "Void";       // Initialize string for default/void screen [Temperature(F)/Humidity]
String Screen00 = "Screen00"; // Initialize string for main display screen [Temperature(F)/Humidity]
String Screen10 = "Screen10"; // Initialize string for main display screen [Temperature(C)/Humidity]
String Screen20 = "Screen20"; // Initialize string for main display screen [Temperature(K)/Humidity]
String Screen01 = "Screen01"; // Initialize string for main display screen [Heat Index(F)]
String Screen11 = "Screen11"; // Initialize string for main display screen [Heat Index(C)]
String Screen21 = "Screen21"; // Initialize string for main display screen [Heat Index(K)]
String Screen02 = "Screen02"; // Initialize string for main display screen [Brightness]
String Screen12 = "Screen12"; // Initialize string for main display screen [Brightness]
String Screen22 = "Screen22"; // Initialize string for main display screen [Brightness]

void setup() {
  Serial.begin(9600);
  dht.begin();
  lcd.begin(16, 2);
  lcd.createChar(1, Check);
  lcd.createChar(2, Lock);
  lcd.createChar(3, Arrow);
  pinMode(VRx, INPUT);
  pinMode(VRy, INPUT);
  pinMode(SW, INPUT_PULLUP);
  pinMode(LDR, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  Screen = String("Void");
  Screen00 = String("Screen00");
  Screen10 = String("Screen10");
  Screen20 = String("Screen20");
  Screen01 = String("Screen01");
  Screen11 = String("Screen11");
  Screen21 = String("Screen21");
  Screen02 = String("Screen02");
  Screen12 = String("Screen12");
  Screen22 = String("Screen22");
}

void loop() {
  sensorData();            // Data collection
  joystickData();          // Input detection
  UIV();                   // User-Input-Values logic
  printData();             // Print data and input
  LEDcontroller();         // LED logic
  LCDcontroller();         // LCD logic
}


///// FUNCTIONS /////


// Get Sensor Data
void sensorData()
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature(false);
  // Read temperature as Fahrenheit (isFahrenheit = true)
  f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again)
  if (isnan(h) || isnan(t) || isnan(f))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  hic = dht.computeHeatIndex(t, h, false);

  // Record brightness data
  brightness = analogRead(LDR);
}


// Get Joystick Data
void joystickData()
{
  xPosition = analogRead(VRx);
  yPosition = analogRead(VRy);
  button = digitalRead(SW);
  mapX = map(xPosition, 0, 1023, -512, 512);
  mapY = map(yPosition, 0, 1023, -512, 512);
  calX = mapX + 10;
  calY = mapY - 14;

  // Check vertical joystick position
  if (calY <= joythresh)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= jsPositionDelay)
    {
      previousMillis = currentMillis;
      // increment row index for main menu
      if (mainrowpos < mainrownum)
      {
        mainrowpos = mainrowpos + 1;
      }
      else
      {
        mainrowpos = 0;
      }
      // increment row index for number menu
      if (numrowpos < numrownum)
      {
        numrowpos = numrowpos + 1;
      }
      else
      {
        numrowpos = 0;
      }
      // increment row index for options menu
      if (optrowpos < optrownum)
      {
        optrowpos = optrowpos + 1;
      }
      else
      {
        optrowpos = 0;
      }
      // increment row index for options menu 2
      if (opt2rowpos < opt2rownum)
      {
        opt2rowpos = opt2rowpos + 1;
      }
      else
      {
        opt2rowpos = 0;
      }
    }
  }
  if (calY >= abs(joythresh))
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= jsPositionDelay)
    {
      previousMillis = currentMillis;
      // decrement row index
      if (mainrowpos <= mainrownum && mainrowpos > 0)
      {
        mainrowpos = mainrowpos - 1;
      }
      else
      {
        mainrowpos = mainrownum;
      }
      // decrement row index for number menu
      if (numrowpos <= numrownum && numrowpos > 0)
      {
        numrowpos = numrowpos - 1;
      }
      else
      {
        numrowpos = numrownum;
      }
      // decrement row index for options menu
      if (optrowpos <= optrownum && optrowpos > 0)
      {
        optrowpos = optrowpos - 1;
      }
      else
      {
        optrowpos = optrownum;
      }

      // decrement row index for options menu 2
      if (opt2rowpos <= opt2rownum && opt2rowpos > 0)
      {
        opt2rowpos = opt2rowpos - 1;
      }
      else
      {
        opt2rowpos = opt2rownum;
      }
    }
  }

  // Check horizontal joystick position
  if (calX >= abs(joythresh))
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= jsPositionDelay)
    {
      previousMillis = currentMillis;
      // increment column index
      if (maincolpos < maincolnum)
      {
        maincolpos = maincolpos + 1;
      }
      else
      {
        maincolpos = 0;
      }
      // increment column index for number menu
      if (optcolpos < optcolnum)
      {
        optcolpos = optcolpos + 1;
      }
      else
      {
        optcolpos = 0;
      }
    }
  }
  if (calX <= joythresh)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= jsPositionDelay)
    {
      previousMillis = currentMillis;
      // decrement column index
      if (maincolpos <= maincolnum && maincolpos > 0)
      {
        maincolpos = maincolpos - 1;
      }
      else
      {
        maincolpos = maincolnum;
      }
      // decrement column index for number menu
      if (optcolpos <= optcolnum && optcolpos > 0)
      {
        optcolpos = optcolpos - 1;
      }
      else
      {
        optcolpos = 0; // Disabled left-bound cycling (was optcolpos = optcolnum;)
      }
    }
  }

  // Check joystick button position
  // Check for toggle
  if (button == 0) // if button is held down
  {
    toggleTime = millis();
    if (toggleTime - lastButtonDownTime > toggleDelay)
    {
      toggle = !toggle;
      lastButtonDownTime = millis();
    }
  }
}


// LED Controller
void LEDcontroller()
{
  // Set LED for humidity settings
  if (h > userhSMin && h < userhSMax)
  {
    analogWrite(redPin, 0);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 0);
  }
  if ((h > userhHMin && h <= userhSMin) || (h >= userhSMax && h < userhHMax))
  {
    analogWrite(redPin, 255);
    analogWrite(greenPin, 22);
    analogWrite(bluePin, 0);
  }
  if (h < userhHMin || h > userhHMax)
  {
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0);
  }
  else
  {
  }
}


// LCD Screen Controller
void LCDcontroller()
{
  // Main Menu
  if (toggle == 0)
  {
    if (mainrowpos == 0)
    {
      if (maincolpos == 0)
      {
        Screen = "Screen00";
        lcd.setCursor(0, 0);
        lcd.print("Temp:    ");
        lcd.print(f);
        lcd.print((char)223);
        lcd.print("F");
        lcd.setCursor(0, 1);
        lcd.print("Humidity: ");
        lcd.print(h);
        lcd.print("%");
      }
      if (maincolpos == 1)
      {
        Screen = "Screen10";
        lcd.setCursor(0, 0);
        lcd.print("Temp:    ");
        lcd.print(t);
        lcd.print((char)223);
        lcd.print("C");
        lcd.setCursor(0, 1);
        lcd.print("Humidity: ");
        lcd.print(h);
        lcd.print("%");
      }
      if (maincolpos == 2)
      {
        Screen = "Screen20";
        lcd.setCursor(0, 0);
        lcd.print("Temp:    ");
        lcd.print(t+273.15);
        lcd.print("K");
        lcd.setCursor(0, 1);
        lcd.print("Humidity: ");
        lcd.print(h);
        lcd.print("%");
      }
    }
    else if (mainrowpos == 1)
    {
      if (maincolpos == 0)
      {
        Screen = "Screen01";
        lcd.setCursor(0, 0);
        lcd.print("Heat Index:     ");
        lcd.setCursor(0, 1);
        lcd.print(hif);
        lcd.print((char)223);
        lcd.print("F         ");
      }
      if (maincolpos == 1)
      {
        Screen = "Screen11";
        lcd.setCursor(0, 0);
        lcd.print("Heat Index:     ");
        lcd.setCursor(0, 1);
        lcd.print(hic);
        lcd.print((char)223);
        lcd.print("C         ");
      }
      if (maincolpos == 2)
      {
        Screen = "Screen21";
        lcd.setCursor(0, 0);
        lcd.print("Heat Index:     ");
        lcd.setCursor(0, 1);
        lcd.print(hic);
        lcd.print("K          ");
      }
    }
    else if (mainrowpos == 2)
    {
      if (maincolpos == 0)
      {
        Screen = "Screen02";
        lcd.setCursor(0, 0);
        lcd.print("Brightness:     ");
        lcd.setCursor(13, 0);
        lcd.print(brightness);
        lcd.setCursor(0, 1);
        lcd.print("                ");
      }
      if (maincolpos == 1)
      {
        Screen = "Screen12";
        lcd.setCursor(0, 0);
        lcd.print("Brightness:     ");
        lcd.setCursor(13, 0);
        lcd.print(brightness);
        lcd.setCursor(0, 1);
        lcd.print("                ");
      }
      if (maincolpos == 2)
      {
        Screen = "Screen22";
        lcd.setCursor(0, 0);
        lcd.print("Brightness:     ");
        lcd.setCursor(13, 0);
        lcd.print(brightness);
        lcd.setCursor(0, 1);
        lcd.print("                ");
      }
    }
    // numrowpos = 0; // this can most likely be deleted
    optcolpos = 0; // Reset column position of options menu so the UVI starts at 0
    tensdigit = 0; // Reset digits
    onesdigit = 0;
    tenthsdigit = 0;
    hundthdigit = 0;
  }
}

float UIV()
{
  if (toggle == 1)
  {
    if (optcolpos == 0)
    {
      if (optrowpos == 0)
      {
        lcd.setCursor(0, 0);
        lcd.print("Enter #'s for:  ");
        lcd.setCursor(0, 1);
        lcd.print("Temperature     ");
        numrowpos = 0; // to avoid storing the row position of the selection as the first digit of the value
      }
      if (optrowpos == 1)
      {
        lcd.setCursor(0, 0);
        lcd.print("Enter #'s for:  ");
        lcd.setCursor(0, 1);
        lcd.print("Humidity        ");
        numrowpos = 0;
      }
      varrowpos = optrowpos;
    }
    if (optcolpos == 1)
    {
      if (varrowpos == 0)
      {
        if (opt2rowpos == 0)
        {
          lcd.setCursor(0, 0);
          lcd.print("Temperature val:");
          lcd.setCursor(0, 1);
          lcd.print("Optimal Temp    ");
          numrowpos = 0; // to avoid storing the row position of the selection as the first digit of the value
        }
        if (opt2rowpos == 1)
        {
          lcd.setCursor(0, 0);
          lcd.print("Temperature val:");
          lcd.setCursor(0, 1);
          lcd.print("Soft Max Temp   ");
          numrowpos = 0;
        }
        if (opt2rowpos == 2)
        {
          lcd.setCursor(0, 0);
          lcd.print("Temperature val:");
          lcd.setCursor(0, 1);
          lcd.print("Hard Max Temp   ");
          numrowpos = 0;
        }
        if (opt2rowpos == 3)
        {
          lcd.setCursor(0, 0);
          lcd.print("Temperature val:");
          lcd.setCursor(0, 1);
          lcd.print("Hard Min Temp   ");
          numrowpos = 0;
        }
        if (opt2rowpos == 4)
        {
          lcd.setCursor(0, 0);
          lcd.print("Temperature val:");
          lcd.setCursor(0, 1);
          lcd.print("Soft Min Temp   ");
          numrowpos = 0;
        }
        var2rowpos = opt2rowpos;
      }
      if (varrowpos == 1)
      {
        if (opt2rowpos == 0)
        {
          lcd.setCursor(0, 0);
          lcd.print("Humidity value: ");
          lcd.setCursor(0, 1);
          lcd.print("Optimal Humidity");
          numrowpos = 0; // to avoid storing the row position of the selection as the first digit of the value
        }
        if (opt2rowpos == 1)
        {
          lcd.setCursor(0, 0);
          lcd.print("Humidity value: ");
          lcd.setCursor(0, 1);
          lcd.print("Soft Max Humid  ");
          numrowpos = 0;
        }
        if (opt2rowpos == 2)
        {
          lcd.setCursor(0, 0);
          lcd.print("Humidity value: ");
          lcd.setCursor(0, 1);
          lcd.print("Hard Max Humid  ");
          numrowpos = 0;
        }
        if (opt2rowpos == 3)
        {
          lcd.setCursor(0, 0);
          lcd.print("Humidity value: ");
          lcd.setCursor(0, 1);
          lcd.print("Hard Min Humid  ");
          numrowpos = 0;
        }
        if (opt2rowpos == 4)
        {
          lcd.setCursor(0, 0);
          lcd.print("Humidity value: ");
          lcd.setCursor(0, 1);
          lcd.print("Soft Min Humid  ");
          numrowpos = 0;
        }
        var2rowpos = opt2rowpos;
      }
    }
    if (optcolpos == 2)
    {  
      if (optcolpos != prevcolpos) // when digits place is shifted by user...
      { // set digit to last known value (to avoid resetting value each time digits are shifted)
        numrowpos = tensdigit; // this used to read "numrowpos = 0;" but reset each digit to zero upon digit shift
      }
      if (numrowpos == 0)
      {
        tensdigit = 0;
      }
      if (numrowpos == 1)
      {
        tensdigit = 1;
      }
      if (numrowpos == 2)
      {
        tensdigit = 2;
      }
      if (numrowpos == 3)
      {
        tensdigit = 3;
      }
      if (numrowpos == 4)
      {
        tensdigit = 4;
      }
      if (numrowpos == 5)
      {
        tensdigit = 5;
      }
      if (numrowpos == 6)
      {
        tensdigit = 6;
      }
      if (numrowpos == 7)
      {
        tensdigit = 7;
      }
      if (numrowpos == 8)
      {
        tensdigit = 8;
      }
      if (numrowpos == 9)
      {
        tensdigit = 9;
      }
      lcd.setCursor(0, 0);
      lcd.print("Enter value:    ");
      lcd.setCursor(0, 1);
      lcd.print("          ");
      lcd.print(tensdigit);
      lcd.print(onesdigit);
      lcd.print(".");
      lcd.print(tenthsdigit);
      lcd.print(hundthdigit);
      lcd.print("%");
      lcd.setCursor(10, 1);
      lcd.cursor();
      delay(cursorDelay);
      prevcolpos = optcolpos;
    }
    if (optcolpos == 3)
    { 
      if (optcolpos != prevcolpos)
      {
        numrowpos = onesdigit;
      }
      if (numrowpos == 0)
      {
        onesdigit = 0;
      }
      if (numrowpos == 1)
      {
        onesdigit = 1;
      }
      if (numrowpos == 2)
      {
        onesdigit = 2;
      }
      if (numrowpos == 3)
      {
        onesdigit = 3;
      }
      if (numrowpos == 4)
      {
        onesdigit = 4;
      }
      if (numrowpos == 5)
      {
        onesdigit = 5;
      }
      if (numrowpos == 6)
      {
        onesdigit = 6;
      }
      if (numrowpos == 7)
      {
        onesdigit = 7;
      }
      if (numrowpos == 8)
      {
        onesdigit = 8;
      }
      if (numrowpos == 9)
      {
        onesdigit = 9;
      }
      lcd.setCursor(0, 0);
      lcd.print("Enter value:    ");
      lcd.setCursor(0, 1);
      lcd.print("          ");
      lcd.print(tensdigit);
      lcd.print(onesdigit);
      lcd.print(".");
      lcd.print(tenthsdigit);
      lcd.print(hundthdigit);
      lcd.print("%");
      lcd.setCursor(11, 1);
      lcd.cursor();
      delay(cursorDelay);
      prevcolpos = optcolpos;
    }
    if (optcolpos == 4)
    {
      if (optcolpos != prevcolpos)
      {
        numrowpos = tenthsdigit;
      }
      if (numrowpos == 0)
      {
        tenthsdigit = 0;
      }
      if (numrowpos == 1)
      {
        tenthsdigit = 1;
      }
      if (numrowpos == 2)
      {
        tenthsdigit = 2;
      }
      if (numrowpos == 3)
      {
        tenthsdigit = 3;
      }
      if (numrowpos == 4)
      {
        tenthsdigit = 4;
      }
      if (numrowpos == 5)
      {
        tenthsdigit = 5;
      }
      if (numrowpos == 6)
      {
        tenthsdigit = 6;
      }
      if (numrowpos == 7)
      {
        tenthsdigit = 7;
      }
      if (numrowpos == 8)
      {
        tenthsdigit = 8;
      }
      if (numrowpos == 9)
      {
        tenthsdigit = 9;
      }
      lcd.setCursor(0, 0);
      lcd.print("Enter value:    ");
      lcd.setCursor(0, 1);
      lcd.print("          ");
      lcd.print(tensdigit);
      lcd.print(onesdigit);
      lcd.print(".");
      lcd.print(tenthsdigit);
      lcd.print(hundthdigit);
      lcd.print("%");
      lcd.setCursor(13, 1);
      lcd.cursor();
      delay(cursorDelay);
      prevcolpos = optcolpos;
    }
    if (optcolpos == 5)
    {
      if (optcolpos != prevcolpos)
      {
        numrowpos = hundthdigit;
      }
      if (numrowpos == 0)
      {
        hundthdigit = 0;
      }
      if (numrowpos == 1)
      {
        hundthdigit = 1;
      }
      if (numrowpos == 2)
      {
        hundthdigit = 2;
      }
      if (numrowpos == 3)
      {
        hundthdigit = 3;
      }
      if (numrowpos == 4)
      {
        hundthdigit = 4;
      }
      if (numrowpos == 5)
      {
        hundthdigit = 5;
      }
      if (numrowpos == 6)
      {
        hundthdigit = 6;
      }
      if (numrowpos == 7)
      {
        hundthdigit = 7;
      }
      if (numrowpos == 8)
      {
        hundthdigit = 8;
      }
      if (numrowpos == 9)
      {
        hundthdigit = 9;
      }
      lcd.setCursor(0, 0);
      lcd.print("Enter value:    ");
      lcd.setCursor(0, 1);
      lcd.print("          ");
      lcd.print(tensdigit);
      lcd.print(onesdigit);
      lcd.print(".");
      lcd.print(tenthsdigit);
      lcd.print(hundthdigit);
      lcd.print("%");
      lcd.setCursor(14, 1);
      lcd.cursor();
      delay(cursorDelay);
      prevcolpos = optcolpos;
    }
    if (optcolpos == 6)
    {
      value = tensdigit*10 + onesdigit + (tenthsdigit*0.1) + (hundthdigit*0.01);
      if (varrowpos == 0) // store values as temperatures
      {
        if (var2rowpos == 0) // store value as optimal temperature
        {
          if (usertOpt != 0)
          {
            if (value < 10)
            {
              lcd.setCursor(0, 0);
              lcd.print("Overwrite value?");
              lcd.setCursor(0, 1);
              lcd.write(byte(2));
              lcd.print(usertOpt);
              lcd.print((char)223);
              lcd.print("F");
              lcd.write(byte(3));
              lcd.setCursor(10, 1);
              lcd.print(value);
              lcd.print((char)223);
              lcd.print("F");
            }
            else
            {
              lcd.setCursor(0, 0);
              lcd.print("Overwrite value?");
              lcd.setCursor(0, 1);
              lcd.write(byte(2));
              lcd.print(usertOpt);
              lcd.print((char)223);
              lcd.print("F");
              lcd.write(byte(3));
              lcd.setCursor(9, 1);
              lcd.print(value);
              lcd.print((char)223);
              lcd.print("F");
            }
          }
          else
          {
            checkSave();
          }
        }
        if (var2rowpos == 1)
        {
          if (usertSMax != 0)
          {
            checkOverwrite();
          }
          else
          {
            checkSave();
          }
        }
        if (var2rowpos == 2)
        {
          if (usertHMax != 0)
          {
            checkOverwrite();
          }
          else
          {
            checkSave();
          }
        }
        if (var2rowpos == 3)
        {
          if (usertHMin != 0)
          {
            checkOverwrite();
          }
          else
          {
            checkSave();
          }
        }
        if (var2rowpos == 4)
        {
          if (usertSMin != 0)
          {
            checkOverwrite();
          }
          else
          {
            checkSave();
          }
        }
      }
      if (varrowpos == 1) // store values as humidities
      {
        if (var2rowpos == 0) // store value as optimal temperature
        {
          if (userhOpt != 0)
          {
            checkOverwrite();
          }
          else
          {
            checkSave();
          }
        }
        if (var2rowpos == 1)
        {
          if (userhSMax != 0)
          {
            checkOverwrite();
          }
          else
          {
            checkSave();
          }
        }
        if (var2rowpos == 2)
        {
          if (userhHMax != 0)
          {
            checkOverwrite();
          }
          else
          {
            checkSave();
          }
        }
        if (var2rowpos == 3)
        {
          if (userhHMin != 0)
          {
            checkOverwrite();
          }
          else
          {
            checkSave();
          }
        }
        if (var2rowpos == 4)
        {
          if (userhSMin != 0)
          {
            checkOverwrite();
          }
          else
          {
            checkSave();
          }
        }
      }   
    }
    if (optcolpos == 7)
    {
      savedvalue = value;
      if (savedvalue < 10)
      {
        lcd.setCursor(0, 0);
        lcd.print("Saved value:     ");
        lcd.setCursor(0, 1);
        lcd.print("         ");
        lcd.write(byte(1));
        lcd.setCursor(11, 1);
        lcd.print(savedvalue);
        lcd.print("%");
      }
      else
      {
        lcd.setCursor(0, 0);
        lcd.print("Saved value:     ");
        lcd.setCursor(0, 1);
        lcd.print("         ");
        lcd.write(byte(1));
        lcd.setCursor(10, 1);
        lcd.print(savedvalue);
        lcd.print("%");
      }
      if (varrowpos == 0) // store values as temperatures
      {
        if (var2rowpos == 0) // store value as optimal temperature
        {
          usertOpt = savedvalue;
        }
        if (var2rowpos == 1)
        {
          usertSMax = savedvalue;
        }
        if (var2rowpos == 2)
        {
          usertHMax = savedvalue;
        }
        if (var2rowpos == 3)
        {
          usertHMin = savedvalue;
        }
        if (var2rowpos == 4)
        {
          usertSMin = savedvalue;
        }
      }
      if (varrowpos == 1) // store values as humidities
      {
        if (var2rowpos == 0) // store value as optimal temperature
        {
          userhOpt = savedvalue;
        }
        if (var2rowpos == 1)
        {
          userhSMax = savedvalue;
        }
        if (var2rowpos == 2)
        {
          userhHMax = savedvalue;
        }
        if (var2rowpos == 3)
        {
          userhHMin = savedvalue;
        }
        if (var2rowpos == 4)
        {
          userhSMin = savedvalue;
        }
      }
      delay(1500); // Reset values upon exiting UVI Menu
      toggle = 0;
      mainrowpos = 0;
      maincolpos = 0;
      numrowpos = 0;
      optcolpos = 0;
      optrowpos = 0;
      tensdigit = 0;
      onesdigit = 0;
      tenthsdigit = 0;
      hundthdigit = 0;
    }
    else
    {
      return 0;
    }
  }
}

void checkOverwrite()
{
  if (value < 10)
  {
    lcd.setCursor(0, 0);
    lcd.print("Overwrite value?");
    lcd.setCursor(0, 1);
    lcd.print("         ");
    lcd.write(byte(2));
    lcd.setCursor(11, 1);
    lcd.print(value);
    lcd.print("%");
  }
  else
  {
    lcd.setCursor(0, 0);
    lcd.print("Overwrite value?");
    lcd.setCursor(0, 1);
    lcd.print("         ");
    lcd.write(byte(2));
    lcd.setCursor(10, 1);
    lcd.print(value);
    lcd.print("%");
  }
}

void checkSave()
{
  if (value < 10)
  {
    lcd.setCursor(0, 0);
    lcd.print("Save value?     ");
    lcd.setCursor(0, 1);
    lcd.print("          ");
    lcd.setCursor(11, 1);
    lcd.print(value);
    lcd.print("%");
  }
  else
  {
    lcd.setCursor(0, 0);
    lcd.print("Save value?     ");
    lcd.setCursor(0, 1);
    lcd.print("          ");
    lcd.setCursor(10, 1);
    lcd.print(value);
    lcd.print("%");
  }
}

// Print Data on Serial Monitor
void printData()
{
//    Serial.print(F("Humidity: "));
//    Serial.print(h);
//    Serial.print(F("%  Temperature: "));
//    Serial.print(t);
//    Serial.print(F("°C "));
//    Serial.print(f);
//    Serial.print(F("°F  Heat index: "));
//    Serial.print(hic);
//    Serial.print(F("°C "));
//    Serial.print(hif);
//    Serial.println(F("°F"));
//    Serial.print("Brightness: ");
//    Serial.println(brightness);
//    Serial.print(" | ");
//    Serial.print("Button: ");
//    Serial.print(button);
//    Serial.print(" | ");
//    Serial.print("Toggle: ");
//    Serial.print(toggle);
//    Serial.print(" | ");
//    Serial.print("Maincolpos: ");
//    Serial.print(maincolpos);
//    Serial.print(" | ");
//    Serial.print("Mainrowpos: ");
//    Serial.print(mainrowpos);
//    Serial.print(" | ");
//    Serial.print(Screen);
//    Serial.print(" | ");
//    Serial.print("ocp: ");
//    Serial.print(optcolpos);
//    Serial.print(" | ");
//    Serial.print("orp: ");
//    Serial.print(optrowpos);
//    Serial.print(" | ");
//    Serial.print("o2rp: ");
//    Serial.print(opt2rowpos);
//    Serial.print(" | ");
//    Serial.print("vrp: ");
//    Serial.print(varrowpos);
//    Serial.print(" | ");
//    Serial.print("v2rp: ");
//    Serial.print(var2rowpos);
//    Serial.print(" | ");
//    Serial.print("nrp: ");
//    Serial.print(numrowpos);
//    Serial.print(" | ");
//    Serial.print("V: ");
//    Serial.print(value);
//    Serial.print(" | ");
//    Serial.print("sV: ");
//    Serial.print(savedvalue);
//    Serial.print(" | ");
  Serial.print("tHx: ");
  Serial.print(usertHMax);
  Serial.print(" | ");
  Serial.print("tSx: ");
  Serial.print(usertSMax);
  Serial.print(" | ");
  Serial.print("t: ");
  Serial.print(usertOpt);
  Serial.print(" | ");
  Serial.print("tSn: ");
  Serial.print(usertSMin);
  Serial.print(" | ");
  Serial.print("tHn: ");
  Serial.print(usertHMin);
  Serial.print(" | ");
  Serial.print("hHx: ");
  Serial.print(userhHMax);
  Serial.print(" | ");
  Serial.print("hSx: ");
  Serial.print(userhSMax);
  Serial.print(" | ");
  Serial.print("h: ");
  Serial.print(userhOpt);
  Serial.print(" | ");
  Serial.print("hSn: ");
  Serial.print(userhSMin);
  Serial.print(" | ");
  Serial.print("hHn: ");
  Serial.print(userhHMin);
  Serial.print("\n");
}
