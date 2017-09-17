#define MY_BAUD_RATE 9600
// #define MY_DEBUG_VERBOSE_RF24
// #define static ID
#define MY_NODE_ID 11
// Enable debug prints to serial monitor
// #define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enable repeater functionality for this node
// #define MY_REPEATER_FEATURE
#include <BME280I2C.h>
#include <Wire.h>
#include <MySensors.h>

#define CHILD_ID_LIGHT 7
#define LIGHT_SENSOR_ANALOG_PIN 2

#define RELAY_1  3  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 1 // Total number of attached relays
#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay
#define RELAY_PULSE 10 // 0 for standard Relay >0 minimum Pilse Time in ms for latching Relay

#define BARO_CHILD 2
#define TEMP_CHILD 3
#define HUM_CHILD 4
#define ALTITUDE_CHILD 5
#define DEWPOINT_CHILD 6

unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)
unsigned long LAST_MEASURE=0; // Marker for last Measure (in milliseconds)
/* ==== BME Global Variables ==== */
BME280I2C bme(0x1,0x1,0x1,B11,B101,B000,false,0x77); // Default : forced mode, standby time = 1000 ms
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool metric = true;
/* ==== END BME Global Variables ==== */

MyMessage light_msg(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage temp_msg(TEMP_CHILD,V_TEMP);
MyMessage hum_msg(HUM_CHILD,V_HUM);
MyMessage pres_msg(BARO_CHILD,V_PRESSURE);
MyMessage alt_msg(ALTITUDE_CHILD,V_CUSTOM);
MyMessage dew_msg(DEWPOINT_CHILD,V_CUSTOM);

int lastLightLevel;

void before()
{
	for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
		// Then set relay pins in output mode
		pinMode(pin, OUTPUT);
		// Set relay to last known state (using eeprom storage)
		digitalWrite(pin, loadState(sensor)?RELAY_ON:RELAY_OFF);
    }
    
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("MyTempHumLightRelais", "1.0");

    for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
		// Register all sensors to gw (they will be created as child devices)
		present(sensor, S_BINARY);
	}

	// Register all sensors to gateway (they will be created as child devices)
    present(BARO_CHILD,V_PRESSURE);
    present(TEMP_CHILD,V_TEMP);
    present(HUM_CHILD,V_HUM);
    present(ALTITUDE_CHILD,V_CUSTOM);
    present(DEWPOINT_CHILD,V_CUSTOM);
    present(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
}

void setup(){
    while(!bme.begin()){
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
    }

}

void loop()
{
    if((millis()-LAST_MEASURE)>SLEEP_TIME){
        // Read lightLevel
        int16_t lightLevel = (analogRead(LIGHT_SENSOR_ANALOG_PIN))/10.23;
        Serial.print("Light: ");
        Serial.println(lightLevel);
        if (lightLevel != lastLightLevel) {
            send(light_msg.set(lightLevel));
            lastLightLevel = lightLevel;
        }

        // Read Environment
        float temp(NAN), hum(NAN), pres(NAN);
        uint8_t pressureUnit(3);                                           // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
        bme.read(pres, temp, hum, metric, pressureUnit);                   // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
        float alt = bme.alt(metric);
        float dew = bme.dew(metric);
        Serial.print("Temp: ");
        Serial.println(temp);
        send(temp_msg,temp);
        Serial.print("Humidity: ");
        Serial.println(hum);
        send(hum_msg,hum);
        Serial.print("Pressure: ");
        Serial.println(pres);
        send(pres_msg,pres);
        Serial.print("Altitude: ");
        Serial.println(alt);
        send(alt_msg,alt);
        Serial.print("Dewpoint: ");
        Serial.println(dew);
        send(dew_msg,dew);
        LAST_MEASURE=millis();
    }
    
   
}

void receive(const MyMessage &message)
{
	// We only expect one type of message from controller. But we better check anyway.
	if (message.type==V_STATUS) {
		// Change relay state
        if (RELAY_PULSE==0){
            digitalWrite(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
        }else{
            digitalWrite(message.sensor-1+RELAY_1,RELAY_ON);
            delay(RELAY_PULSE);
            digitalWrite(message.sensor-1+RELAY_1,RELAY_OFF);
        }
		// Store state in eeprom
		saveState(message.sensor, message.getBool());
		// Write some debug info
		Serial.print("Incoming change for sensor:");
		Serial.print(message.sensor);
		Serial.print(", New status: ");
		Serial.println(message.getBool());
	}
}



