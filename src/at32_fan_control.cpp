#include <Arduino.h>
// Include the library
#include <FanController.h>
#include <OneWire.h>

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

// Initialize library
FanController fan1 = FanController(SENSOR_PIN_1, SENSOR_THRESHOLD, PWM_PIN_1);
FanController fan2 = FanController(SENSOR_PIN_2, SENSOR_THRESHOLD, PWM_PIN_2);

OneWire ds1(DS_1);  // Dallas one wire data buss pin, a 4.7K resistor pullup is needed
OneWire ds2(DS_2);  // Dallas one wire data buss pin, a 4.7K resistor pullup is needed

void setup(void) {
    // start serial port
    Serial.begin(9600);

    // Start up the library
    fan1.begin();
    fan2.begin();
}

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
		// delay(1000);                     // wait for the conversion 
		Sensor->reset();
		// Sensor->skip();
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

void loop(void) {
    Serial.print("ds1: ");
    int temp = GetMaxTemperature(&ds1);
    Serial.print(temp);
    Serial.println(" C");
	Serial.print("ds2: ");
	temp = GetMaxTemperature(&ds2);
    Serial.print(temp);
    Serial.println(" C");
    
    // Call fan.getSpeed() to get fan RPM.
    Serial.print("Current speed - 1: ");
    unsigned int rpms = fan1.getSpeed(); // Send the command to get RPM
    Serial.print(rpms);
    Serial.println("RPM");
    Serial.print("Current speed - 2: ");
    rpms = fan2.getSpeed(); // Send the command to get RPM
    Serial.print(rpms);
    Serial.println("RPM");

    // Get new speed from Serial (0-100%)
    if (Serial.available() > 0) {
        // Parse speed
        int input = Serial.parseInt();

        // Constrain a 0-100 range
        byte target = max(min(input, 100), 0);

        // Print obtained value
        Serial.print("Setting duty cycle: ");
        Serial.println(target, DEC);

        // Set fan duty cycle
        fan1.setDutyCycle(target);
        fan2.setDutyCycle(target);

        // Get duty cycle
        byte dutyCycle = fan1.getDutyCycle();
        Serial.print("Duty cycle - 1: ");
        Serial.println(dutyCycle, DEC);
        dutyCycle = fan2.getDutyCycle();
        Serial.print("Duty cycle - 2: ");
        Serial.println(dutyCycle, DEC);
    }

    // Not really needed, just avoiding spamming the monitor,
    // readings will be performed no faster than once every THRESHOLD ms anyway
    delay(250);
}