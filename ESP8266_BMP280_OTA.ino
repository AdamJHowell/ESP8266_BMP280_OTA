/**
 * This sketch will use a BMP280 sensor to show temperature, pressure, and estimated altitude.
 * The BMP280 uses the I2C bus to communicate with the microcontroller.
 * The ESP8266/ESP-12E SCL pin is D1 (GPIO5), and SDA is D2 (GPIO4).
 * @copyright   Copyright © 2022 Adam Howell
 * @licence     The MIT License (MIT)
 */
#include <ESP8266WiFi.h>						// This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <Adafruit_BMP280.h>		// The Adafruit library for BMP280 sensor.
#include <Adafruit_Sensor.h>		// The Adafruit sensor library.
#include "privateInfo.h"			// I use this file to hide my network information from random people browsing my GitHub repo.
#include <ThingSpeak.h>				// https://github.com/mathworks/thingspeak-arduino
//#include <Arduino_JSON.h>			// https://github.com/arduino-libraries/Arduino_JSON
#include <ArduinoJson.h>			// https://arduinojson.org/

#define BMP280_I2C_ADDRESS 0x76	// Confirmed working I2C address as of 2021-08-21, for the GY-BM model https://smile.amazon.com/gp/product/B07S98QBTQ/.

/**
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 * If you do not want to use that file, you can set them here instead.
 */
//const char* wifiSsid = "your WiFi SSID";
//const char* wifiPassword = "your WiFi password";
//const char* mqttBroker = "your broker address";
//const int mqttPort = 1883;
const char* mqttTopic = "espWeather";
const char* sketchName = "ESP8266BMP280ThingSpeak";
const char* notes = "Lolin ESP8266 with BMP280";
const int LED_PIN = 2;											// This LED for the Lolin devkit is on the ESP8266 module itself (next to the antenna).
const char* espControlTopic = "espControl";				// This is a topic we subscribe to, to get updates.  Updates may change publishDelay, seaLevelPressure, or request an immediate poll of the sensors.

char ipAddress[16];
char macAddress[18];
unsigned int loopCount = 0;									// This is a counter for how many loops have happened since power-on (or overflow).
unsigned long publishDelay = 60000;							// This is the loop delay in miliseconds.
int mqttReconnectDelay = 5000;								// How long to wait (in milliseconds) between MQTT connection attempts.
unsigned long lastPublish = 0;								// This is used to determine the time since last MQTT publish.
float seaLevelPressure = 1014.5;								// Adjust this to the sea level pressure (in hectopascals) for your local weather conditions.
float bmp280TPA[3];												// This holds the temperature, pressure, and altitude.
unsigned long bootTime;
// Provo Airport: https://forecast.weather.gov/data/obhistory/KPVU.html
// ThingSpeak variables
unsigned long myChannelNumber = 1;
//const char* ThingSpeakWriteKey = "yourWriteKey";

// Create class objects.
Adafruit_BMP280 bmp280;
WiFiClient espClient;
PubSubClient mqttClient( espClient );


void onReceiveCallback( char* topic, byte* payload, unsigned int length )
{
	char str[length + 1];
	Serial.print( "Message arrived [" );
	Serial.print( topic );
	Serial.print( "] " );
	int i=0;
	for( i = 0; i < length; i++ )
	{
		Serial.print( ( char ) payload[i] );
		str[i] = ( char )payload[i];
	}
	Serial.println();
	str[i] = 0; // Null termination
	StaticJsonDocument <256> doc;
	deserializeJson( doc, str );

	// The command can be: publishTelemetry, changeTelemetryInterval, changeSeaLevelPressure, or publishStatus.
	const char* command = doc["command"];
	if( strcmp( command, "publishTelemetry") == 0 )
	{
		Serial.println( "Reading and publishing sensor values." );
		// Poll the sensor and immediately publish the readings.
		readTelemetry();
		// Publish the sensor readings.
		readTelemetry();
		Serial.println( "Readings have been published." );
	}
	else if( strcmp( command, "changeTelemetryInterval") == 0 )
	{
		Serial.println( "Changing the publish interval." );
		unsigned long tempValue = doc["value"];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds mixup.
		if( tempValue > 4000 )
			publishDelay = tempValue;
		Serial.print( "MQTT publish interval has been updated to " );
		Serial.println( publishDelay );
		lastPublish = 0;
	}
	else if( strcmp( command, "changeSeaLevelPressure") == 0 )
	{
		Serial.println( "Changing the sea-level pressure." );
		seaLevelPressure = doc["value"];
		Serial.print( "Sea-level pressure has been updated to " );
		Serial.println( seaLevelPressure );
	}
	else if( strcmp( command, "publishStatus") == 0 )
	{
		Serial.println( "publishStatus is not yet implemented." );
	}
	else
	{
		Serial.print( "Unknown command: " );
		Serial.println( command );
	}
} // End of onReceiveCallback() function.


void wifiConnect( int maxAttempts )
{
	// Announce WiFi parameters.
	String logString = "WiFi connecting to SSID: ";
	logString += wifiSsid;
	Serial.println( logString );

	// Connect to the WiFi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : " - Failed!" );
	WiFi.begin( wifiSsid, wifiPassword );

	int i = 1;
	/*
     WiFi.status() return values:
     0 : WL_IDLE_STATUS when WiFi is in process of changing between statuses
     1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
     3 : WL_CONNECTED after successful connection is established
     4 : WL_CONNECT_FAILED if wifiPassword is incorrect
     6 : WL_DISCONNECTED if module is not configured in station mode
  */
	// Loop until WiFi has connected.
	while( WiFi.status() != WL_CONNECTED && i < maxAttempts )
	{
		digitalWrite( LED_PIN, LOW ); // Turn the LED off.
		delay( 1000 );
		Serial.println( "Waiting for a connection..." );
		Serial.print( "WiFi status: " );
		Serial.println( WiFi.status() );
		Serial.print( i++ );
		Serial.println( " seconds" );
	}

	WiFi.setAutoReconnect( true );
	WiFi.persistent( true );

	// Print that WiFi has connected.
	Serial.println( '\n' );
	Serial.println( "WiFi connection established!" );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );
	Serial.print( "IP address: " );
	snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
	Serial.println( ipAddress );

	digitalWrite( LED_PIN, 0 );	// Turn the WiFi LED on.
} // End of wifiConnect() function.


// mqttConnect() will attempt to (re)connect the MQTT client.
void mqttConnect( int maxAttempts )
{
	digitalWrite( LED_PIN, LOW ); // Turn the LED off.
	Serial.print( "Attempting to connect to the MQTT broker up to " );
	Serial.print( maxAttempts );
	Serial.println( " times." );

	int i = 0;
	// Loop until MQTT has connected.
	while( !mqttClient.connected() && i < maxAttempts )
	{
		Serial.print( "Attempt # " );
		Serial.print( i + 1 );
		Serial.print( "..." );
		// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
		if( mqttClient.connect( macAddress ) )
		{
			Serial.println( " connected!" );
			digitalWrite( LED_PIN, HIGH ); // Turn the LED on.
		}
		else
		{
			Serial.print( " failed!  Return code: " );
			Serial.print( mqttClient.state() );
			Serial.print( ".  Trying again in " );
			Serial.print( mqttReconnectDelay / 1000 );
			Serial.println( " seconds." );
			digitalWrite( LED_PIN, HIGH ); // Turn the LED on.
			delay( mqttReconnectDelay / 2 );
			digitalWrite( LED_PIN, LOW ); // Turn the LED off.
			delay( mqttReconnectDelay / 2 );
		}
		i++;
	}
	mqttClient.setBufferSize( 512 );
	char mqttString[512];
	snprintf( mqttString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\"\n}", sketchName, macAddress, ipAddress );
	mqttClient.publish( "espConnect", mqttString );

	Serial.println( "Function mqttConnect() has completed." );
} // End of mqttConnect() function.


void setup()
{
	pinMode( LED_PIN, OUTPUT );			// Initialize digital pin WiFi LED as an output.
	digitalWrite( LED_PIN, HIGH );	  // Turn the LED on.
	delay( 500 );
	// Start the Serial communication to send messages to the computer.
	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );
	Serial.println( '\n' );
	Serial.print( sketchName );
	Serial.println( " is beginning its setup()." );
	Serial.println( __FILE__ );

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );
	mqttClient.setCallback( onReceiveCallback );				 // Assign the onReceiveCallback() function to handle MQTT callbacks.

	Serial.println( "Attempting to connect to the BMP280..." );
	if( !bmp280.begin( BMP280_I2C_ADDRESS ) )
	{
		Serial.println( "Could not find a valid BMP280 sensor, Check wiring!" );
		while( 1 )
			;
	}
	Serial.println( "Connected to the BMP280!\n" );

	// Set the MAC address variable to its value.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );

	wifiConnect( 20 );
	ThingSpeak.begin( espClient );  // Initialize ThingSpeak

	printUptime();
} // End of setup() function.


void printUptime()
{
	Serial.print( "Uptime in " );
	long seconds = ( millis() - bootTime ) / 1000;
	long minutes = seconds / 60;
	long hours = minutes / 60;
	if( seconds < 601 )
	{
		Serial.print( "seconds: " );
		Serial.println( seconds );
	}
	else if( minutes < 121 )
	{
		Serial.print( "minutes: " );
		Serial.println( minutes );
	}
	else
	{
		Serial.print( "hours: " );
		Serial.println( hours );
	}
}


void readTelemetry()
{
	// Get temperature, pressure and altitude from the Adafruit BMP280 library.
	// Temperature is always a floating point in Centigrade units. Pressure is a 32 bit integer in Pascal units.
	bmp280TPA[0] = bmp280.readTemperature();	 				// Get temperature.
	bmp280TPA[1] = bmp280.readPressure();			 				// Get pressure.
	bmp280TPA[2] = bmp280.readAltitude( seaLevelPressure );	// Get altitude based on the sea level pressure for your location. Note that "altitude" is a keyword, hence the underscore.
}


void publishTelemetry()
{
	// Print the signal strength:
	long rssi = WiFi.RSSI();
	Serial.print( "WiFi RSSI: " );
	Serial.println( rssi );
	// Prepare a String to hold the JSON.
	char mqttString[512];
	// Write the readings to the String in JSON format.
	snprintf( mqttString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"tempC\": %.2f,\n\t\"pressure\": %.1f,\n\t\"altitude\": %.1f,\n\t\"seaLevelPressure\": %.1f,\n\t\"rssi\": %ld,\n\t\"loopCount\": %d,\n\t\"notes\": \"%s\"\n}", sketchName, macAddress, ipAddress, bmp280TPA[0], bmp280TPA[1], bmp280TPA[2], seaLevelPressure, rssi, loopCount, notes );
	// Publish the JSON to the MQTT broker.
	bool success = mqttClient.publish( mqttTopic, mqttString, false );
	if( success )
		Serial.println( "Successfully published this to the broker:" );
	else
		Serial.println( "MQTT publish failed!  Attempted to publish this to the broker:" );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );
	lastPublish = millis();
}


void publishThingSpeak()
{
	// Set the ThingSpeak fields.
	ThingSpeak.setField( 1, bmp280TPA[0] );
	ThingSpeak.setField( 2, bmp280TPA[1] );
	ThingSpeak.setField( 3, bmp280TPA[2] );
	int x = ThingSpeak.writeFields( myChannelNumber, ThingSpeakWriteKey );
	if( x == 200 )
		Serial.println( "Thingspeak update successful." );
	else
		Serial.println( "Problem updating channel. HTTP error code " + String( x ) );
}


void loop()
{
	// Reconnect to WiFi if necessary.
	if( WiFi.status() != WL_CONNECTED )
		wifiConnect( 10 );
	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
		mqttConnect( 10 );
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	unsigned long time = millis();
	// When time is less than publishDelay, subtracting publishDelay from time causes an overlow which results in a very large number.
	if( lastPublish == 0 || ( ( time > publishDelay ) && ( time - publishDelay ) > lastPublish ) )
	{
		loopCount++;
		// These next 3 lines act as a "heartbeat", to give local users a visual indication that the system is working.
		digitalWrite( LED_PIN, 1 );	// Turn the WiFi LED off to alert the user that a reading is about to take place.
		delay( 1000 );						// Wait for one second.
		digitalWrite( LED_PIN, 0 );	// Turn the WiFi LED on.

		Serial.println( sketchName );
		Serial.print( "Connected to broker at \"" );
		Serial.print( mqttBroker );
		Serial.print( ":" );
		Serial.print( mqttPort );
		Serial.println( "\"" );
		Serial.print( "Listening for control messages on topic \"" );
		Serial.print( espControlTopic );
		Serial.println( "\"." );
		printUptime();

		readTelemetry();
		publishTelemetry();
		publishThingSpeak();

	  	Serial.print( "Next publish in " );
		Serial.print( publishDelay / 1000 );
		Serial.println( " seconds.\n" );
	}
} // End of loop() function.
