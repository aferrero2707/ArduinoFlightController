#define STATUS_LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define ERROR_LED_PIN 12 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
unsigned long loop_timer, loop_timer_stop, loop_count = 0;

// ESC variables
unsigned long esc_loop_timer;
unsigned long escLength, escTimer;

#define HB_TIMEOUT 60
void heartBeat()
{
  loop_count += 1;

  if(loop_count == HB_TIMEOUT) {
    // blink LED to indicate activity
    digitalWrite(STATUS_LED_PIN, HIGH);
  }

  if(loop_count == (HB_TIMEOUT+5)) {
    // blink LED to indicate activity
    digitalWrite(STATUS_LED_PIN, LOW);
    loop_count = 0;
  }
}


void escSignalStart()
{ 
  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  escTimer = escLength + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
}

void escSignalWait()
{
#ifdef BENCHMARK
  esc_loop_timer = micros();                                              //Read the current time.
  if (escTimer <= esc_loop_timer) {
        Serial.println("ESC timeout exceeded!!!");
  }
#endif

  while(PORTD >= 16) {                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(escTimer <= esc_loop_timer) PORTD &= B00001111;                //Set digital output 4 to low if the time is expired.
  }
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // configure LED pins
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, HIGH);

  // initialize serial communication
  Serial.begin(115200);
  loop_count = 0;

  // initialize ESC signal lengths to minimum
  escLength = 2000;

  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);
  blinkState = LOW;
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
  byte data;
  if(Serial.available() > 0) {
    data = Serial.read();                                       //Read the incomming byte.
    delayMicroseconds(1000);                                    //Wait for any other bytes to come in
    while(Serial.available() > 0) Serial.read(); //Empty the Serial buffer.

    if(data == '1') {
      escLength = 2000;
      Serial.println(escLength);
    }
    if(data == '2') {
      escLength = 1000;
      Serial.println(escLength);
    }
    if(data == '3') {
      escLength = 1200;
      Serial.println(escLength);
    }
  }

  heartBeat();
  
  loop_timer_stop = micros();
  if((loop_timer_stop - loop_timer) >= 4000) {
    Serial.print("Timeout exceeded! ");
    Serial.println(loop_timer_stop - loop_timer);
  }
  while((micros() - loop_timer) < 4000);

  loop_timer = micros();
  
  escSignalStart();
  escSignalWait();
}
