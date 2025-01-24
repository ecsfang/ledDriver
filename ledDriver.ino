/*
  <========Arduino Sketch for Adafruit Huzzah ESP8266=========>
  Used as a 'smart' LED PWM dimmer for 12V leds
  Interfaces: encoder with button function, connected to pins 0 (button, 5 and 4 (encoder) and WiFi connection to MQTT
  Output 16 is the PWM signal for the Power MOSFET (IRF540)
  Output 2 is the blue LED on the huzzah board, to signal connection to the Wifi network
  Input 0 is the button, connected to the encoder button; it is also connected to the red led on the huzzah board.
  The button has multiple functions:
  - Short press (< 450ms) toggles light between last brightness and off
  - Long press either saves last brightness (when the led was on) and turns it off or sets brightness to 100% (when the led was off)
  The dimmer uses MQTT to listen for commands:
  topic_in for incoming commands (OFF, ON or brightness)
  topic_out for state (ON, OFF) and startup state
  topic_out_brightness for brightness value (0..MAXRANGE)

*/
#include <ESP8266WiFi.h>
#include "secrets.h"
#include "PubSubClient.h"
#include <EasyButton.h>
#include <Encoder.h>

#define MINRANGE  100   // min pwm value
#define MAXRANGE  1023  // max pwm value
#define POWER_OFF 0
#define STEP1     40
#define STEP2     80
#define STEP3     120

#define LED_ON    LOW
#define LED_OFF   HIGH
#define BUTT_ON   HIGH
#define BUTT_OFF  LOW

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;

String hostname = CLIENT_ID;

bool debug = true;

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "mossa/esp/in";

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "mossa/esp/out";
const char* topic_out_brightness = "mossa/esp/out/brightness";

char messageBuffer[100];
char topicBuffer[100];
String ip = "";

WiFiClient espClient;
PubSubClient mqttClient;

void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}

// For ESP8266
#define LED_GREEN   D5  // pin for green led: LOW is on
#define LED_BLUE    D6  // pin for blue led: LOW is on
#define LED_RED     D7  // pin for red led: LOW is on
#define LED_PWM_PIN D0  // pin for driving MOSFET

#define EN_PIN_A    D2
#define EN_PIN_B    D1
#define BTN_PIN     D8  // pin for button: HIGH is on

Encoder myEnc(EN_PIN_A, EN_PIN_B);
//unsigned char encoder_A;
//unsigned char encoder_B;
//unsigned char last_encoder_A;
//unsigned char last_encoder_B;

int led_power = 0;
int last_led_power = MAXRANGE;
int power_step = 1;
//long loop_time;

// Button parameters
//int NumberOfButtons = 1;
//int ButtonPins[] = {BTN_PIN};
#define DEBOUNCE_DELAY 100
#define LONGPRESS_TIME 450

void updateLeds(void)
{
  analogWrite(LED_PWM_PIN, led_power);            // Drive the MOSFET
  analogWrite(LED_GREEN, 1023-led_power);         // ... and the encoder LED
}

// Instance of the button.
EasyButton button(BTN_PIN, DEBOUNCE_DELAY, false, false); // pin, debounce_time, pullup_enable, active_low

void buttonPressed()
{
  if ( led_power == POWER_OFF ) {                 // If light is off, set brightness to last saved value
    led_power = last_led_power;
  } else {                                        // if light is on, turn it off (and don't save brightness)
    led_power = POWER_OFF;
  }
  ShowDebug( "Button pressed - L ==> " + String(led_power) );
  updateLeds();
  report_state();                                 // and report it back to MQTT
}
void buttonPressedLong()
{
  if ( led_power == POWER_OFF ) {                 // If light is off, set brightness to max
    led_power = MAXRANGE;
  } else {                                        // if light is on, save brightness and turn light off
    last_led_power = led_power;
    led_power = POWER_OFF;
  }
  ShowDebug( "Button long pressed - L ==> " + String(led_power) );
  updateLeds();
  report_state();
}
void buttonISR()
{
  // When button is being used through external interrupts, parameter INTERRUPT must be passed to read() function.
  button.read();
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    digitalWrite(LED_BLUE, LED_OFF);  // Blue led off: no connection yet..
    ShowDebug("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID, mqtt_user, mqtt_passw)) {
      ShowDebug("connected");
      //digitalWrite(LED_BLUE, LED_ON); // Blue led on: connected to MQTT
      // Once connected, publish an announcement...
      mqttClient.publish(topic_out, ip.c_str());
      mqttClient.publish(topic_out, "ESP8266 connected");
      // ... and resubscribe
      mqttClient.subscribe(topic_in);
    } else {
      ShowDebug("failed, rc=" + String(mqttClient.state()));
      ShowDebug(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void report_state()
{
  String messageString;
  ShowDebug("LED Power: " + String(led_power));
  if (led_power < MINRANGE) {
    messageString = "OFF";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out, messageBuffer);
  }
  else {
    messageString = "ON";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out, messageBuffer);
    messageString = String(led_power);
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_brightness, messageBuffer);
  }
}

void callback(char* topic, byte * payload, unsigned int length) {
  char msgBuffer[20];
  // I am only using one ascii character as command, so do not need to take an entire word as payload
  // However, if you want to send full word commands, uncomment the next line and use for string comparison
  payload[length] = '\0'; // terminate string with 0
  String strPayload = String((char*)payload);  // convert to string

  ShowDebug("Message arrived");
  ShowDebug(topic);
  ShowDebug(strPayload);

  if (strPayload == "OFF")  {
    if( led_power )
      last_led_power = led_power;   // Save last value if ON
    led_power = POWER_OFF;
    updateLeds();
    report_state();
  }
  else if (strPayload == "ON")  {
    led_power = last_led_power;
    updateLeds();
    report_state();
  }
  else if (strPayload == "STAT")  {
    report_state();
  }
  else if (strPayload == "IP")  {
    // 'Show IP' commando
    mqttClient.publish(topic_out, ip.c_str()); // publish IP nr
    mqttClient.publish(topic_out, hostname.c_str()); // publish hostname
  }
  else {
    if ((strPayload.substring(0).toInt() == 0) and (strPayload != "0")) {
      mqttClient.publish(topic_out, "Unknown Command"); // unknown command
    }
    else {
      led_power = strPayload.substring(0).toInt();
      if (led_power > MAXRANGE)
        led_power = MAXRANGE;
      if (led_power < MINRANGE)
        led_power = POWER_OFF;
      updateLeds();
      report_state();
    }
  }
}


void setup() {

  // set up PWM range
  analogWriteRange(MAXRANGE);

  // set up serial comms
  Serial.begin( 115200 );

  // wait a bit
  delay(100);

  // and send some output
  ShowDebug("LED PWM dimmer START");
  ShowDebug("Connecting to ");
  ShowDebug(ssid);

  // Set gpio pins
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(EN_PIN_A, INPUT_PULLUP);
  pinMode(EN_PIN_B, INPUT_PULLUP);
  pinMode(BTN_PIN, INPUT_PULLUP);

  // Set brightness to 0
  analogWrite(LED_PWM_PIN, POWER_OFF);
  // and enable encoder LEDs
  digitalWrite(LED_GREEN, LED_OFF);
  digitalWrite(LED_BLUE, LED_OFF);
  digitalWrite(LED_RED, LED_OFF);

  // Connect to Wifi
  int led = true;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_RED, led ? LED_ON : LED_OFF);
    led = !led;
    delay(500);
    Serial.print(".");
  }
  digitalWrite(LED_RED, LED_OFF);

  //convert ip Array into String
  ip = String (WiFi.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[3]);

  ShowDebug("WiFi connected");
  //ShowDebug("IP address: " + String(WiFi.localIP()));
  // ShowDebug("Netmask: " + String(WiFi.subnetMask()));
  // ShowDebug("Gateway: " + String(WiFi.gatewayIP()));

  // setup mqtt client
  mqttClient.setClient(espClient);
  mqttClient.setServer( MQTTSERVER, MQTTPORT ); // or local broker
  ShowDebug(F("MQTT client configured"));
  mqttClient.setCallback(callback);

  // Initialize the button.
  button.begin();

  button.onPressed(buttonPressed);
  button.onPressedFor(LONGPRESS_TIME, buttonPressedLong);

  if (button.supportsInterrupt())
  {
    button.enableInterrupt(buttonISR);
    ShowDebug("Button will be used through interrupts");
  }

}

void loop() {
  if (!mqttClient.connected()) {          // check MQTT connection status and reconnect if connection lost
    ShowDebug("Not Connected!");
    reconnect();
  }
  mqttClient.loop();

  button.update();

  int steps = myEnc.readAndReset();
  // If on and switch has rotated
  if( led_power && steps ) {
    if ( led_power <= STEP1 ) {             // use different steps for different values
      power_step = 1;
    } else if ( led_power <= STEP2 ) {
      power_step = 2;
    } else if ( led_power <= STEP3 ) {
      power_step = 5;
    } else {
      power_step = 10;
    }
    power_step *= steps;
    if ( power_step < 0 ) {
      // if encoder has moved CCW, decrease led brightness
      Serial.println( "L: " + String(led_power) + " - " + String(-power_step));
    } else {
      // if encoder has moved CW, inrease led brightness
      Serial.println( "R: " + String(led_power) + " + " + String(power_step));
    }
    led_power += power_step;
    if( led_power < MINRANGE) led_power = MINRANGE;
    if( led_power > MAXRANGE) led_power = MAXRANGE;
    updateLeds();
    report_state();                       // report back on the MQTT topic
  }
}
