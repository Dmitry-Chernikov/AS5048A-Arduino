#include <AS5048A.h>

// include the library code:
#include <Arduino.h>
//#include <Time.h>  
#include <TimeLib.h>
#include <DS1307RTC.h> 

//#include <Wire.h>
//#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <AS5048A.h>
//#include <SPI.h>
//#include <math.h>
//#include <LCD_1602_RUS.h>


// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
AS5048A angleSensor(SS);
// These #defines make it easy to set the backlight color
#define buzzerPin 2
int frequency = 0;
int duration = 0;

#define   CMD_SET_TIME       "Set "
#define   CMD_READ_TIME      "Read"
#define   OPERATION_ERROR    "Failure"
#define   OPERATION_SUCCESS  "OK"
#define   CMD_LENGTH         4
#define   DATE_ITEMS_COUNT   7

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

//word registerAddress = 0x3FFF ;
int x = 0;
float ValTek, ValPred, AbsUgol = 0;
//float PereReikiZaObort = (PI * 17 * 3) / 360;
int time;
String SubString;

void setup() {
  // Debugging output
  Serial.begin(19200);

  while (!Serial) ; // wait until Arduino Serial Monitor opens
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if(timeStatus()!= timeSet) 
     Serial.println("Unable to sync with the RTC (Невозможно синхронизировать с RTC)");
  else
     Serial.println("RTC has set the system time (RTC установил системное время)");      
         
  angleSensor.init();
  ValPred = ValTek = angleSensor.RotationRawToAngle(angleSensor.getRawRotation(false));
   
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD. We track how long it takes since
  // this library has been optimized a bit and we're proud of it :)
  time = millis();
  //lcd.print("Robotdyn test!");
  //time = millis() - time;
  Serial.print("Took "); Serial.print(time); Serial.println(" ms");
  lcd.setBacklight(WHITE);
}

uint8_t i = 0;

void loop() {
  
  //delay(500);
  //for (int i = 1; i <= 100; i++) {
      ValTek = angleSensor.RotationRawToAngle(angleSensor.getRawRotation(true));
      angleSensor.AbsoluteAngleRotation(&AbsUgol, &ValTek, &ValPred);

      lcd.clear();

      lcd.setCursor(0, 0);
      lcd.print(angleSensor.LinearDisplacementRack(AbsUgol, 3, 17), 4);
      lcd.print(" mm");
      
      lcd.setCursor(0, 1);
      //lcd.print(val, DEC);

      //lcd.print(angleSensor.RotationRawToRadian(angleSensor.getRawRotation(true)), DEC);
      lcd.print(int(AbsUgol), DEC); //lcd.print(millis()/1000);
      lcd.print(char(223));
      
      lcd.print(int (angleSensor.GetAngularMinutes(AbsUgol)), DEC);//lcd.print(millis()/1000);
      lcd.print(char(34));

      lcd.print(int (angleSensor.GetAngularSeconds(AbsUgol)), DEC);
      lcd.print(char(39)); 
      lcd.print("  ");

      


  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):


  while (Serial.available() > 0)
  {
    String NewString = Serial.readString();
    SubString = NewString.substring(0, CMD_LENGTH);

    //Serial.println(NewString);
    //Serial.println(SubString);
    
    NewString.remove(0, CMD_LENGTH + 1);
    //Serial.println(NewString);

    lcd.setCursor(0, 0);
    lcd.print(SubString);

    if (SubString == CMD_SET_TIME)
    {
      int DateItems[DATE_ITEMS_COUNT];
      for (int Item = 0; Item < DATE_ITEMS_COUNT; Item++)
      {      
        SubString.remove(0);
      
        for(int CharIndex = 0; CharIndex < NewString.length();  CharIndex++)
        {
          if (NewString[CharIndex] ==  ' ')
          {
            NewString.remove(0, CharIndex + 1);
            break;
          }
          SubString += NewString[CharIndex];
        }

        DateItems[Item] = SubString.toInt();
        Serial.println(DateItems[Item]);
      }

      tmElements_t tm;
      tm.Second = DateItems[0];
      tm.Minute = DateItems[1];
      tm.Hour = DateItems[2];      
      tm.Wday = DateItems[3];  
      tm.Day = DateItems[4];
      tm.Month = DateItems[5];
      tm.Year = CalendarYrToTm(DateItems[6]);

      if (RTC.write(tm))
        Serial.println(OPERATION_SUCCESS);
      else
        Serial.println(OPERATION_ERROR);
    }else if (SubString == CMD_READ_TIME)
    {
      tmElements_t tm;

      if (RTC.read(tm))
      {
        SubString.remove(0);
        SubString = String(tm.Hour) + ":" + String(tm.Minute) + ":" + String(tm.Second) + ", " +
                    String(tm.Day) + "." + String(tm.Month) + "." + String(tmYearToCalendar(tm.Year));
        
        Serial.println(SubString);
      }

      else
        Serial.println(OPERATION_ERROR);
    }
  }

    

  //Serial.print("Got rotation of:");
  //Serial.print(val, DEC);
  //Serial.println("°");
  //Serial.print("State: ");
  
    //lcd.print(angleSensor.error(), DEC);
    
  angleSensor.printState();      
  angleSensor.printErrors();
  // Serial.println(" ");
  //Serial.print("Errors: ");
  //Serial.println(angleSensor.getErrors());

  uint8_t buttons = lcd.readButtons();

  if (buttons) {
    lcd.clear();
    lcd.setCursor(0, 0);
    if (buttons & BUTTON_UP) {
      lcd.write(char(x));
      if (x < 255 ) {
        x++;
      }
      lcd.print(" ");
      lcd.print(x);
      //lcd.print("UP ");
      lcd.setBacklight(RED);
      delay(1000);
    }
    if (buttons & BUTTON_DOWN) {
      lcd.write(char(x));
      if (x < 255 ) {
        x--;
      }
      lcd.print(" ");
      lcd.print(x);
      //lcd.print("UP ");
      lcd.setBacklight(RED);
      // lcd.print("DOWN ");
      lcd.setBacklight(BLUE);
      delay(1000);
    }
    if (buttons & BUTTON_LEFT) {
      lcd.print("LEFT ");
      if (timeStatus() == timeSet) {
        digitalClockDisplay();
      } else {
        Serial.println("The time has not been set.  Please run the Time");  //  "Время задано не было. Пожалуйста, запустите"
        Serial.println(" пример «Установка RTC-времени».");
        Serial.println();
        delay(4000);
      }
        delay(1000);
      
      lcd.setBacklight(GREEN);
    }
    if (buttons & BUTTON_RIGHT) {
      lcd.print("RIGHT ");
      lcd.setBacklight(TEAL);
    }
    if (buttons & BUTTON_SELECT) {
      lcd.print("SELECT ");
      lcd.setBacklight(VIOLET);
      buzzer();
      AbsUgol = 0;
    }
  }
}

void digitalClockDisplay(){
  // digital clock display of the time
  tmElements_t tm;
  RTC.read(tm);
  
  Serial.print(tm.Hour);
  printDigits(tm.Minute);
  printDigits(tm.Second);
  Serial.print(" ");
  Serial.print(tm.Day);
  Serial.print(".");
  Serial.print(tm.Month);
  Serial.print(".");
  Serial.print(tm.Year); 
  Serial.println(); 
  
  //delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(String(tm.Hour)+":"+ String(tm.Minute)+":"+String(tm.Second));
  lcd.setCursor(0, 1);
  lcd.print(String(tm.Day)+"."+String(tm.Month)+"."+String(tmYearToCalendar(tm.Year)));
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void buzzer() {
  for (frequency = 1800; frequency < 2000; frequency++)
  {
    duration = 10;
    tone (buzzerPin, frequency, duration);
  }
  noTone (buzzerPin);
}



