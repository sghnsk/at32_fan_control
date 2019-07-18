#include <Arduino.h>

#include <FanController.h>
#include <OneWire.h>

// fan switch temperatures
#define TEMP_MIN 40 // C°
#define TEMP_MAX 100 // C°

#define FT_START 5000 // msec
#define DELAY_TIMEOUT	1000
#define PWM_MIN_1 5 // минимальное значение pwm
#define PWM_MAX_1 100 // максимальное значение pwm
#define PWM_MIN_2 5 // минимальное значение pwm
#define PWM_MAX_2 100 // максимальное значение pwm

// Sensor wire is plugged into port 2 on the Arduino.
// For a list of available pins on your board,
// please refer to: https://www.arduino.cc/en/Reference/AttachInterrupt
#define SENSOR_PIN_1 2
#define SENSOR_PIN_2 3

// Choose a threshold in milliseconds between readings.
// A smaller value will give more updated results,
// while a higher value will give more accurate and smooth readings
#define SENSOR_THRESHOLD 1000

// PWM pin (4th on 4 pin fans)
#define PWM_PIN_1 5
#define PWM_PIN_2 6

#define DS_1 7
#define DS_2 8

FanController fan1 = FanController(SENSOR_PIN_1, SENSOR_THRESHOLD, PWM_PIN_1);
FanController fan2 = FanController(SENSOR_PIN_2, SENSOR_THRESHOLD, PWM_PIN_2);

OneWire ds1(DS_1);  // Dallas one wire data buss pin, a 4.7K resistor pullup is needed
OneWire ds2(DS_2);  // Dallas one wire data buss pin, a 4.7K resistor pullup is needed

int16_t GetMaxTemperature(OneWire *Sensor) {
    byte data[12];
    int16_t raw = 0;
    int16_t celsius = 0;
    int16_t maxValue = 0;

    byte addr[8];

    Sensor->reset_search();

    while(Sensor->search(addr)) {
        Sensor->reset();       // reset one wire buss
        Sensor->select(addr);
        Sensor->write(0x44);   // start conversion
        Sensor->reset();
        Sensor->select(addr);
        Sensor->write(0xBE);   // Read Scratchpad
        for (byte i = 0; i < 9; i++) {       // 9 bytes
            data[i] = Sensor->read();
        } 
        // Convert the data to actual temperature
        raw = (data[1] << 8) | data[0];
        celsius = raw / 16;
        // обновить максимальное значение
        if (celsius > maxValue) {
            maxValue = celsius;
        }
    }
    Sensor->reset_search();
    return maxValue;
}

void SetFanSpeedWithTemp(FanController *fan, int temp, byte pwmMin, byte pwmMax) {
    byte dutyCycle = pwmMin;

    if(temp < TEMP_MIN) {
        fan->setDutyCycle(dutyCycle);
    } 
    if((temp >= TEMP_MIN) && (temp <= TEMP_MAX)) {
        dutyCycle = map(temp, TEMP_MIN, TEMP_MAX, pwmMin, pwmMax);
    } 
    if(temp > TEMP_MAX) {
        dutyCycle = pwmMax;
    }
    fan->setDutyCycle(dutyCycle);

    Serial.print(temp);
    Serial.print(";C;");
    Serial.print(fan->getDutyCycle(), DEC);
    Serial.print(";%;");
    Serial.print(fan->getSpeed());
    Serial.println(F(";rpm"));
}

void setup(void) {
    // start serial port
    Serial.begin(9600);

    // Start up the library
    fan1.begin();
    fan2.begin();

    // расскрутить после включения на максимум на время FT_START и после выставить минимальную скорость
    fan1.setDutyCycle(PWM_MAX_1);
    fan2.setDutyCycle(PWM_MAX_2);
    delay(FT_START);
    fan1.setDutyCycle(PWM_MIN_1);
    fan2.setDutyCycle(PWM_MIN_2);
}

void loop() {
    delay(DELAY_TIMEOUT);
    Serial.print(F("ch1;"));
    int temp1 = GetMaxTemperature(&ds1);
    SetFanSpeedWithTemp(&fan1, temp1, PWM_MIN_1, PWM_MAX_1);
    Serial.print(F("ch2;"));
    int temp2 = GetMaxTemperature(&ds2);
    SetFanSpeedWithTemp(&fan2, temp2, PWM_MIN_2, PWM_MAX_2);
}
