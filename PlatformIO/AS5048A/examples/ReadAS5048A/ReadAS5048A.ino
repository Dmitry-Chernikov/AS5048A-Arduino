#include <AS5048A.h>

//AS5048A angleSensor(10);
//выход на Arduino SS = PIN_SPI_SS (53), MOSI = PIN_SPI_MOSI (51), MISO = PIN_SPI_MISO (50), SCK = PIN_SPI_SCK (52)
AS5048A angleSensor(SS);

//Переменные энкодера
float angleCurrent, anglePrevious, absoluteAngle = 0;
float normalModule = 3;      //Модуль нормальны
float numberGearTeeth = 17;  //Число зубьев колеса или число заходов червяка

void setup() {
  Serial.begin(19200);
  /////////////Инициализация энкодера/////////////
  angleSensor.init();
  //AnglePrevious = AngleCurrent = angleSensor.RotationRawToAngle(angleSensor.getRawRotation());
  absoluteAngle = 0;
  // 16 количество массива измеренных значения для медианного среднего
  // false выключение медианного усреднения
  // true включение углового среднего
  anglePrevious = angleCurrent = angleSensor.rotationRawToAngle(angleSensor.getRawRotation(16, false, true));
}

void loop() {
  // 16 количество массива измеренных значения для медианного среднего
  // false выключение медианного усреднения
  // true включение углового среднего
  //word val = angleSensor.getRawRotation(16, false, true); // угловое среднее
  //word val = angleSensor.getRawRotation(16, true, false); // медианное среднее
  word val = angleSensor.getRawRotation(16, true, true); // угловое медианное среднее
  float angle = angleSensor.rotationRawToAngle(val);

  Serial.print("Got rotation of: 0x");
  Serial.print(val, HEX);
  Serial.print("\t");
  Serial.print(val, DEC);
  Serial.print("\t");

  Serial.print( static_cast<int>(angle) );
  Serial.print("˚");
  Serial.print( static_cast<int>(angleSensor.getAngularMinutes(angle)) );
  Serial.print("\"");
  Serial.print( static_cast<int>(angleSensor.getAngularSeconds(angle)) );
  Serial.println("\'");

  Serial.print("State: ");
  angleSensor.printState();

  Serial.print("Errors: ");
  Serial.println(angleSensor.getErrors());

  Serial.print("Linear ");
  Serial.print(getLinearMotion(), 4);
  Serial.println(" mm");
  Serial.println("");

  delay(1000);
}

// Возвращает перемещение прямозубой зубчатой рейки в мм
float getLinearMotion() {
  return angleSensor.linearDisplacementRack(angleSensor.absoluteAngleRotation(&absoluteAngle, angleSensor.rotationRawToAngle(angleSensor.getRawRotation(16, false, true)), &anglePrevious), normalModule, numberGearTeeth);
}
