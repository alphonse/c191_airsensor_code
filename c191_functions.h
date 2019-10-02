
#include <TimeLib.h>
#include <Snooze.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <hpma115s0.h>

SnoozeAlarm alarm;                            // Hibernate wake up alarm
SnoozeSPI sdCard;                             // SD Card Hibernate
SnoozeBlock teensySleep(alarm, sdCard);       // Hibernate
Adafruit_BME680 bme;                          // I2C
HPMA115S0 pm(Serial1);                        // PM Sensor Serial Port
Sd2Card  card;

bool pm_status;

char dataFileName[16];
const int chipSelect = BUILTIN_SDCARD;

void setBMESamplingParameters() {
    if (!bme.begin(0x76)) {
        return;
    }
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
}

String fillDigits(int digits) {
  if (digits < 10) {
    String digitsout = "0" + String(digits);
    return digitsout;
  }
  else return digits;
}

String printTime(int h, int m, int s) {
  return fillDigits(h) + ":" + fillDigits(m) + ":" + fillDigits(s);
}

String printDate(int y, int m, int d) {
  return String(y) + "-" + fillDigits(m) + "-" + fillDigits(d);
}

String printDateTime(String d, String t) {
  return d + ' ' + t + '\t';
}

String printData(float t, float h, float p, float g, float pm25, float pm10) {
  return String(t) + '\t' + String(h) + '\t' + String(p/100.0) + '\t' + String(g/1000.0)+ '\t' + String(pm25)+ '\t' + String(pm10);
}

String createFileName(int y, int m, int d) {
    return String(fillDigits(y)) + String(fillDigits(m)) + String(fillDigits(d)) + ".txt";
}

void initializeSensors() {
    pinMode(33, OUTPUT);
    digitalWrite(33, HIGH);
    pinMode(LED_BUILTIN, OUTPUT);
    setBMESamplingParameters();

    Serial1.begin(9600);
    while (!Serial1) {
        ; // wait for serial port to connect. Needed for native USB
    }
    pm_status = pm.stop_autosend();
    if (pm_status == 1) {
        for (int i = 1; i < 6; i++) {
            digitalWriteFast(LED_BUILTIN, HIGH);
            delay(250);
            digitalWriteFast(LED_BUILTIN, LOW);
            delay(250);
        }
        delay(5000);
    }
    else {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(5000);
        digitalWrite(LED_BUILTIN, LOW);
    }
}

void initializeSDCard() {
    sdCard.spiClockPin(BUILTIN_SDCARD);
    if (!SD.begin(chipSelect)) {
        return;
    }
    if (!card.init(SPI_HALF_SPEED, chipSelect)) {
        // don't do anything more:
        while (1) {
          digitalWriteFast(LED_BUILTIN, HIGH);
          delay(250);
          digitalWriteFast(LED_BUILTIN, LOW);
          delay(250);
        }
    }
    for (int i = 1; i < 3; i++){
          digitalWriteFast(LED_BUILTIN, HIGH);
          delay(250);
          digitalWriteFast(LED_BUILTIN, LOW);
          delay(250);
    }
}


time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void wakeSensors() {
    digitalWrite(33, HIGH);
    pm_status = pm.start_measurement();
    if (pm_status == 1){
      delay(500);
    }
    else{
      return;
    }
  }

String makeMeasurement(int n) {
    float p25avg = 0.0;
    float p25sum = 0.0;
    float p10avg = 0.0;
    float p10sum = 0.0;
    float p25;
    float p10;
    for (int i=1; i < (n + 1); i++) {
      pm_status = pm.read(&p25, &p10);

      if (i > 15) {
        p25sum = p25sum + p25;
        p25avg = (p25sum)/(float(i)-15.0);
        p10sum = p10sum + p10;
        p10avg = (p10sum)/(float(i)-15.0);
      }
      delay(1000);
      return printData(bme.temperature, bme.humidity, bme.pressure, bme.gas_resistance, p25avg, p10avg);
    }
}

void writeFile(char filename[16], String dataString) {
    File dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        dataFile.print(deviceID); dataFile.print('\t');
        dataFile.print(printDateTime(printDate(year(), month(), day()), printTime(hour(), minute(), second())));
        dataFile.print(dataString);
        delay(500);
        dataFile.close();
     }
     else return;
}

void turnOffSensors() {
  pm_status = pm.stop_measurement();
  delay(500);
  digitalWrite(33, LOW);
}
