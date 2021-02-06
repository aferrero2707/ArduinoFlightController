#define STATUS_LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define ERROR_LED_PIN 12 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
unsigned long loop_timer, loop_timer_stop, loop_count = 0, elapsed, mpuLoopCount = 0;

// ESC variables
unsigned long esc_loop_timer;
unsigned long timer_esc_1, timer_esc_2, timer_esc_3, timer_esc_4;
unsigned long esc_1, esc_2, esc_3, esc_4;
unsigned long escFL, escFR, escBL, escBR, throttle;

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
  timer_esc_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_esc_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_esc_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_esc_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
}

void escSignalWait()
{
#ifdef BENCHMARK
  esc_loop_timer = micros();                                              //Read the current time.
  if ((timer_esc_1 <= esc_loop_timer) || (timer_esc_2 <= esc_loop_timer) || 
      (timer_esc_3 <= esc_loop_timer) || (timer_esc_4 <= esc_loop_timer)) {
        Serial.println("ESC timeout exceeded!!!");
  }
#endif

  while(PORTD >= 16) {                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_esc_1 <= esc_loop_timer) PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_esc_2 <= esc_loop_timer) PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_esc_3 <= esc_loop_timer) PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_esc_4 <= esc_loop_timer) PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
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
  esc_1 = 2000;
  esc_2 = 2000;
  esc_3 = 2000;
  esc_4 = 2000;

  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);
  blinkState = LOW;
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  if(Serial.available() > 0){
    data = Serial.read();                                                               //Read the incomming byte.
    delay(100);                                                                         //Wait for any other bytes to come in
    while(Serial.available() > 0)loop_counter = Serial.read();                          //Empty the Serial buffer.

  
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
