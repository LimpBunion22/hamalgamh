
#include "SAMD_PWM.h"
#include "Arduino.h"

#define _PWM_LOGLEVEL_       4
#if defined(__SAMD51__)
// Pin 5:TCC2_CH1, pin 7: TCC1_CH2, pin 11: TCC0_CH1, pin 25/MOSI: TC2_CH0
//uint32_t PWM_Pins[]   = { 5, 7, 11, 25 };     // Different timers
// Pin 10:TCC0_CH0, pin 11: TCC0_CH1, pin 12: TCC0_CH3, pin 13: TCC0_CH2
uint32_t PWM_Pins[]   = { 10, 11, 12, 13 };     // Same timers
#else
uint32_t PWM_Pins[]   = { 6, 9, 10, 11 };
#endif

#define NUM_OF_PINS       ( sizeof(PWM_Pins) / sizeof(uint32_t) )


typedef enum {
    ST_BOOTING,
    ST_WAIT,
    ST_MONITORING,
    ST_RUN_CHECK,
    ST_RUN_CHECK_SERVO,
    ST_RUN_CHECK_READER,
    ST_RUN,
    ST_ERROR
} state_t;

typedef struct {
    state_t s = ST_BOOTING;
    bool readerEnabled = false;
    bool servoEnabled = false;
    unsigned long     flankCounter = 0;
    unsigned long     millisCounter = 0;
} fsm_t;

volatile fsm_t fsm;

unsigned int interruptPin = 7;
void detectFlank()
{
  if(fsm.readerEnabled)
    fsm.flankCounter++;
}
float dutyCycle[] = { 2.0f, 30.0f, 5.0f, 8.0f };

// Must be same frequency for same channel
float frequency[] = { 50.0f, 50.0f, 50.0f, 50.0f }; // = 2000.0f;

//creates pwm instances
// SAMD_PWM* PWM_Instance[NUM_OF_PINS];
SAMD_PWM* PWM_Instance = nullptr;

char dashLine[] = "=====================================================================================";

void printPWMInfo(SAMD_PWM* PWM_Instance)
{
  Serial.print("\t\t[SERVO] Actual data: pin = ");
  Serial.print(PWM_Instance->getPin());
  Serial.print(", PWM DC = ");
  Serial.print(PWM_Instance->getActualDutyCycle());
  Serial.print(", PWMPeriod = ");
  Serial.print(PWM_Instance->getPWMPeriod());
  Serial.print(", PWM Freq (Hz) = ");
  Serial.println(PWM_Instance->getActualFreq(), 4);
}
unsigned long t0;
void setup()
{
  pinMode(interruptPin, INPUT);
  
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  delay(100);

  Serial.println(dashLine);
  Serial.print("[STATE = BOOTING] - Initializing System on Board: ");
  Serial.println(BOARD_NAME);
  Serial.println(dashLine);

  // t0 = millis();
  attachInterrupt(digitalPinToInterrupt(interruptPin), detectFlank, FALLING);
  Serial.println("\t[SYSTEM] - READER Configured on pin " + String(interruptPin));

  Serial.print(F("\n[SERVO] Starting PWM_MultiChannel on "));

  PWM_Instance = new SAMD_PWM(PWM_Pins[0], frequency[0], dutyCycle[0]);
  PWM_Instance->setPWM();
  Serial.println("\t[SYSTEM] - PWM Configured");

  printPWMInfo(PWM_Instance);
  fsm.s = ST_WAIT;
  Serial.println(dashLine);
  Serial.print("[STATE = WAIT] - System ready");
  Serial.println(dashLine);

}

void loop()
{
  switch (fsm.s) {
  case ST_WAIT:
    if(Serial.available() > 0){
    
      String entrada = Serial.readStringUntil('\n');
      entrada.trim();

      if (!(entrada.length() > 0)) {
        delay(100);
        break;
      }
      if(entrada.startsWith("[HELP]")){
        Serial.println("[HELP] - Available commands:");
        Serial.println("\t[CHECK] - Check the SYSTEM");
        Serial.println("\t[MONITORING] - Start MONITORING mode");
        Serial.println("\t[RUN][Position0,Delay0,...,PositionN,DelayN,FinalPos] - Start RUN mode");
        Serial.println("\t\tThe positions are especified in floats [5.0-11.0].");
        Serial.println("\t\tThe delays are especified in floats [1-5000][ms].");
        Serial.println("\t[EXIT] - Exit the program");
      }
      else if(entrada.startsWith("[CHECK]")){
        Serial.println("[SYSTEM] - Checking System...");
        fsm.s = ST_RUN_CHECK;
      }
      else if(entrada.startsWith("[MONITORING]")){
        Serial.println("[SYSTEM] - Starting MONITORING mode...");
        Serial.println(dashLine);
        Serial.print("[STATE = MONITORING] - ");
        Serial.println(dashLine);
        fsm.s = ST_MONITORING;
      }
      else if(entrada.startsWith("[RUN]")){
        Serial.println("[SYSTEM] - Starting RUN mode...");
        fsm.s = ST_RUN;
      }
      else if(entrada.startsWith("[EXIT]")){
        Serial.println("[SYSTEM] - Exiting program...");
        delete PWM_Instance;
        return;
    }
  }
    delay(100);
    break;
  case ST_RUN_CHECK:
      Serial.println("[CHECK] - Checking Servo...");
      Serial.println("[CHECK] -   Servo to position 0.0%");
      PWM_Instance->setPWM(6, 50.0f, 5.0f);
      delay(1000);
      Serial.println("[CHECK] -   Servo to position 50.0%");
      PWM_Instance->setPWM(6, 50.0f, 8.0f);
      delay(1000);
      Serial.println("[CHECK] -   Servo to position 100.0%");
      PWM_Instance->setPWM(6, 50.0f, 11.0f);
      delay(100);
      Serial.println("[CHECK] - Checking Monitoring system...");
      delay(100);
      Serial.println("[CHECK] - Checking complete.");
      fsm.s = ST_WAIT;
      Serial.println(dashLine);
      Serial.print("[STATE = WAIT] - System ready");
      Serial.println(dashLine);
    break;
  case ST_MONITORING:
    if(Serial.available() > 0){
    
      String entrada = Serial.readStringUntil('\n');
      entrada.trim();

      if (!(entrada.length() > 0)) {
        delay(100);
        break;
      }
    }
    break;
  
  default:
    break;
  }

  //Long delay has no effect on the operation of hardware-based PWM channels
  delay(2000);  
  Serial.print("[READER] - Flank counter = ");
  Serial.println(fsm.flankCounter);
  Serial.print("[READER] - Tiempo[s] = ");
  Serial.println((millis() - t0)/1000.0f);
  Serial.print("[READER] - Flancos/s = ");
  Serial.println(fsm.flankCounter*1000.0f/(millis() - t0));


  
  Serial.println("[SERVO] - Awaiting new test time (1.0 - 1000.0) for pin 6");

  if(!(Serial.available() > 0))
    return;  

  String entrada = Serial.readStringUntil('\n');
  entrada.trim(); // Elimina espacios y saltos de línea
  if (entrada.length() > 0) {
    float valor = entrada.toFloat();
    if(valor < 1.0f || valor > 1000.0f) {
      Serial.println("[SERVO][ERROR] - Valor fuera de rango. Debe ser entre 1.0 y 1000.0.");        
    }else {
      Serial.print("[SERVO] - Nuevo valor recibido: ");
      Serial.println(valor);

      for(float i=5; i<=11; i+=0.1f){
        PWM_Instance->setPWM(6, 50.0f, i);
        delay(valor);
      }
      for(float i=11; i>=5; i-=0.1f){
        PWM_Instance->setPWM(6, 50.0f, i);
        delay(valor);
      }      
    }
  }
  
  // Serial.println("[SERVO] - Awaiting new duty cycle value (5.0 - 11.0) for pin 6");

  // while (!(Serial.available() > 0)) {
  //   delay(1);
  // }
  // String entrada = Serial.readStringUntil('\n');
  // entrada.trim(); // Elimina espacios y saltos de línea
  // if (entrada.length() > 0) {
  //   float valor = entrada.toFloat();
  //   if(valor < 5.0f || valor > 11.0f) {
  //     Serial.println("[ERROR] - Valor fuera de rango. Debe ser entre 5.0 y 11.0.");        
  //   }else {
  //     Serial.print("[INFO] - Nuevo valor recibido: ");
  //     Serial.println(valor);

  //     PWM_Instance[0]->setPWM(6, 50.0f, valor);
  //   }
  // }
  
}