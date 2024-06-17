#include <AS5048A.h>


//AS5048A angleSensor(10);
//выход на Arduino SS = PIN_SPI_SS (53), MOSI = PIN_SPI_MOSI (51), MISO = PIN_SPI_MISO (50), SCK = PIN_SPI_SCK (52)
AS5048A angleSensor(SS);

//Переменные энкодера
float AngleCurrent, AnglePrevious, AbsoluteAngle = 0;
float NormalModule = 3;      //Модуль нормальны
float NumberGearTeeth = 17;  //Число зубьев колеса или число заходов червяка

void setup() {
  Serial.begin(19200);
  /////////////Инициализация энкодера/////////////
  angleSensor.init();
  //AnglePrevious = AngleCurrent = angleSensor.RotationRawToAngle(angleSensor.getRawRotation(true));
  AbsoluteAngle = 0;
  // true включение медианного среднего
  // 64 количество массива измеренных значения для медианного среднего
  AnglePrevious = AngleCurrent = angleSensor.RotationRawToAngle(angleSensor.getRawRotation(true, 64));
}

void loop() {
  delay(1000);

  word val = angleSensor.getRawRotation();

  Serial.print("Got rotation of: 0x");
  Serial.println(val, HEX);
  Serial.print("State: ");
  angleSensor.printState();
  Serial.print("Errors: ");
  Serial.println(angleSensor.getErrors());

  Serial.print("Linear");
  Serial.print(getLinearMotion(), 4);
  Serial.println(" mm");
}

// Возвращает перемещение прямозубой зубчатой рейки в мм
float getLinearMotion() {
  return angleSensor.LinearDisplacementRack(angleSensor.AbsoluteAngleRotation(&AbsoluteAngle, angleSensor.RotationRawToAngle(angleSensor.getRawRotation(true, 64)), &AnglePrevious), NormalModule, NumberGearTeeth);
}
