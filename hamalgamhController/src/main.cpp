
// #define _PWM_LOGLEVEL_       4
// #include "SAMD_PWM.h"
#include "Arduino.h"

#define MAX_POINTS 2048


#include <Arduino.h>
#include "wiring_private.h"  // g_APinDescription, etc.

// --------- Helpers de bajo nivel (SAMD21) ----------
static inline void waitSyncTCC(Tcc* tcc) {
  while (tcc->SYNCBUSY.reg) { }
}

static void setup_gclk_tcc() {
  // Alimenta TCC0/TCC1 con GCLK0 (48 MHz)
  // Habilita el reloj de perifericos
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1;

  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID(TCC0_GCLK_ID) |
      GCLK_CLKCTRL_GEN_GCLK0 |
      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY) {}

  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_ID(TCC1_GCLK_ID) |
      GCLK_CLKCTRL_GEN_GCLK0 |
      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY) {}
}

static void tcc_common_config(Tcc* tcc, uint32_t per) {
  // Reset
  tcc->CTRLA.bit.SWRST = 1;
  while (tcc->SYNCBUSY.bit.SWRST || tcc->CTRLA.bit.SWRST) {}

  // Prescaler 1, onda NPWM
  tcc->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV8;
  tcc->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  waitSyncTCC(tcc);

  // Periodo 24-bit (valor > 16 bits para verificar)
  tcc->PER.reg = per;
  waitSyncTCC(tcc);

  // CCx = 50%
  for (int ch = 0; ch < 4; ++ch) {
    if (ch < 2 || tcc == TCC0) { // TCC1 solo tiene CC0..1
      tcc->CC[ch].reg = per / 2;
    }
  }
  waitSyncTCC(tcc);

  // Enable
  tcc->CTRLA.bit.ENABLE = 1;
  waitSyncTCC(tcc);
}

static void tcc_set_duty_both(uint32_t per, float duty01) {
  uint32_t cc = (uint32_t)(per * duty01 + 0.5f);
  // TCC0: CC0..3
  for (int ch = 0; ch < 4; ++ch) {
    TCC0->CC[ch].reg = cc; 
  }
  waitSyncTCC(TCC0);

  // TCC1: CC0..1
  for (int ch = 0; ch < 2; ++ch) {
    TCC1->CC[ch].reg = cc;
  }
  waitSyncTCC(TCC1);
}

static void set_pin_mux(uint8_t pin, uint8_t func /*0..7 = A..H*/) {
  // Deshabilita control GPIO y selecciona periférico func (A..H).
  uint8_t port = g_APinDescription[pin].ulPort;
  uint8_t pinNum = g_APinDescription[pin].ulPin;

  // Habilitar PMUX en el pin
  PORT->Group[port].PINCFG[pinNum].bit.PMUXEN = 1;

  // PMUX es por pares: E/O
  uint8_t pmuxIndex = pinNum >> 1;
  bool odd = pinNum & 1;

  if (odd) {
    // PMUXO
    uint8_t val = PORT->Group[port].PMUX[pmuxIndex].bit.PMUXE;
    PORT->Group[port].PMUX[pmuxIndex].reg = (val & 0x0F) | (func << 4);
  } else {
    // PMUXE
    uint8_t val = PORT->Group[port].PMUX[pmuxIndex].bit.PMUXO;
    PORT->Group[port].PMUX[pmuxIndex].reg = (val << 4) | (func & 0x0F);
  }
}

static void release_pin_to_gpio(uint8_t pin) {
  // Vuelve a control GPIO como entrada con pull-down (silencio)
  pinMode(pin, INPUT_PULLDOWN);
  uint8_t port = g_APinDescription[pin].ulPort;
  uint8_t pinNum = g_APinDescription[pin].ulPin;
  PORT->Group[port].PINCFG[pinNum].bit.PMUXEN = 0; // quita periférico
}

// ----------------------------------------------------

const uint32_t PER_24 = 120000; // > 16 bits; 50 Hz con prescaler 8 (48 MHz / 8 / 120000)
const uint8_t funcs[2] = { 4, 5 }; // E=4, F=5


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

char dashLine[] = "=====================================================================================";
unsigned long t0;
void setup()
{
  pinMode(interruptPin, INPUT);
  
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  delay(100);

  Serial.println(dashLine);
  Serial.print("[STATE = BOOTING] - Initializing System on Board: ");
  Serial.println(dashLine);

  // t0 = millis();
  attachInterrupt(digitalPinToInterrupt(interruptPin), detectFlank, FALLING);
  Serial.println("\t[SYSTEM] - READER Configured on pin " + String(interruptPin));

  Serial.print(F("\n[SERVO] Starting PWM_MultiChannel on "));
  setup_gclk_tcc();
  tcc_common_config(TCC0, PER_24);
  tcc_common_config(TCC1, PER_24);
  release_pin_to_gpio(6);
  set_pin_mux(6, funcs[1]);
  tcc_set_duty_both(PER_24, 0.05);
  delay(10); // pequeño settle
  Serial.println("\t[SYSTEM] - PWM Configured");
  

  fsm.s = ST_WAIT;
  Serial.println(dashLine);
  Serial.print("[STATE = WAIT] - System ready");
  Serial.println(dashLine);

}

float positions[MAX_POINTS];
float times[MAX_POINTS];
int nPoints;
String operationName;

void loop()
{
  switch (fsm.s) {
    case ST_WAIT:{
      if(Serial.available() > 0){
      
        String entrada = Serial.readStringUntil('\n');
        entrada.trim();

        if (!(entrada.length() > 0)) {
          delay(100);
          break;
        }
        if(entrada.startsWith("[HELP]")){
          Serial.println("[HELP] - WELCOME to the HAMALGAMH Testing System");
          Serial.println("  Configure your arduino with the following pins:");
          Serial.println("\tReader PIN = 7");
          Serial.println("\tPWM    PIN = 6");
          Serial.println("  Available commands:");
          Serial.println("\t[CHECK] - Check the SYSTEM");
          Serial.println("\t[MONITORING] - Start MONITORING mode");
          Serial.println("\t[RUN]<operationName_string>;<numberOfPoints_Int>;<pos1_float>,<delay1_float>;...;<posN_float>,<delayN_float>- Start RUN mode");
          Serial.println("\t\tThe positions are especified in floats [5.0-11.0].");
          Serial.println("\t\tThe times are especified in floats [1-5000][ms].");
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
          t0 = millis();
          fsm.flankCounter = 0;
          fsm.readerEnabled = true;
        }
        else if(entrada.startsWith("[RUN]")){
          Serial.println("[SYSTEM] - Starting RUN mode...");
          fsm.s = ST_RUN;
        }
        else if(entrada.startsWith("[EXIT]")){
          Serial.println("[SYSTEM] - Exiting program...");
          // delete PWM_Instance;
          return;
        }
      }
      delay(100);
      break;
    }
    case ST_RUN_CHECK:{
        Serial.println("[CHECK] - Checking Servo...");
        Serial.println("[CHECK] -   Servo to position 0.0%");        
        tcc_set_duty_both(PER_24, 0.05);
        delay(1000);
        Serial.println("[CHECK] -   Servo to position 50.0%");
        tcc_set_duty_both(PER_24, 0.075);
        delay(1000);
        Serial.println("[CHECK] -   Servo to position 100.0%");
        tcc_set_duty_both(PER_24, 0.10);
        delay(100);

        Serial.println("[CHECK] - Checking Monitoring system...");
        t0 = millis();
        fsm.flankCounter = 0;
        fsm.readerEnabled = true;        
        delay(1000);
        Serial.print("[READER][");
        Serial.print(fsm.flankCounter*1000.0f/(millis() - t0));
        Serial.println("]");
        fsm.readerEnabled = false;  

        Serial.println("[CHECK] - Checking complete.");
        fsm.s = ST_WAIT;
        Serial.println(dashLine);
        Serial.print("[STATE = WAIT] - System ready");
        Serial.println(dashLine);
      break;
    }
    case ST_MONITORING:{
      Serial.print("[READER][");
      Serial.print(fsm.flankCounter*1000.0f/(millis() - t0));
      Serial.println("]");
      fsm.flankCounter = 0;
      t0 = millis();
      delay(1000);
      if(Serial.available() > 0){
      
        String entrada = Serial.readStringUntil('\n');
        entrada.trim();

        if (!(entrada.length() > 0)) {
          delay(100);
          break;
        }
        if(entrada.startsWith("[HELP]")){
          Serial.println("[HELP] - WELCOME to the HAMALGAMH Testing System");
          Serial.println("  Configure your arduino with the following pins:");
          Serial.println("\tReader PIN = 7");
          Serial.println("\tPWM    PIN = 6");
          Serial.println("  Available commands:");
          Serial.println("\t[CHECK] - Check the SYSTEM");
          Serial.println("\t[MONITORING] - Start MONITORING mode");
          Serial.println("\t[RUN]<operationName_string>;<numberOfPoints_Int>;<pos1_float>,<delay1_float>;...;<posN_float>,<delayN_float>- Start RUN mode");
          Serial.println("\t\tThe positions are especified in floats [5.0-11.0].");
          Serial.println("\t\tThe times are especified in floats [1-5000][ms].");
          Serial.println("\t[EXIT] - Exit the program");
        }
        else if(entrada.startsWith("[CHECK]")){
          Serial.println("[SYSTEM] - Checking System...");
          fsm.s = ST_RUN_CHECK;
        }
        else if(entrada.startsWith("[RUN]")){
          Serial.println("[SYSTEM] - Starting RUN mode...");
          fsm.s = ST_RUN;
        }
        else if(entrada.startsWith("[EXIT]")){
          Serial.println("[SYSTEM] - Exiting program...");
          // delete PWM_Instance;
          return;
        }
      }
      break;
    }
    case ST_RUN:{
        Serial.println("[RUN] - Awaiting operation details");
        
        for(int i = 0; i<5; i++){
          if(Serial.available() > 0) break;
          delay(1000);
        }
        String entrada = Serial.readStringUntil('\n');
        entrada.trim();

        if (!(entrada.length() > 0)){
          Serial.println("[ERROR] - BAD FORMAT - Operation details");
          break;
        }

        int start = 0;
        int sep = entrada.indexOf(';', start);
        if (sep == -1){
          Serial.println("[ERROR] - BAD FORMAT - Operation name");
          break;
        }
        operationName = entrada.substring(start, sep);

        start = sep + 1;
        sep = entrada.indexOf(';', start);
        if (sep == -1) return;
        nPoints = entrada.substring(start, sep).toInt();
        if (nPoints > MAX_POINTS){
          nPoints = MAX_POINTS;
          Serial.println("[WARNING] - TOO MANY POINTS - Number of points set to MAX 2048");
        }

        for (int i = 0; i < nPoints; i++) {
          start = sep + 1;
          sep = entrada.indexOf(';', start);
          if (sep == -1) sep = entrada.length(); // último par
          String pair = entrada.substring(start, sep);

          int comma = pair.indexOf(',');
          if (comma != -1) {
            positions[i] = pair.substring(0, comma).toFloat();
            if(positions[i])
            times[i] = pair.substring(comma + 1).toFloat();
          }
        }

        Serial.print("[RUN] - Reading complete - Starting operation ");
        Serial.print(operationName);
        Serial.print(" with ");
        Serial.print(nPoints);
        Serial.println(" points.");

        // t0 = millis();
        const int TIME_STEP_MS = 50;
        fsm.flankCounter = 0;
        fsm.readerEnabled = true;
        int cnt = 0;
        float position = positions[0];
        float delta = (positions[cnt+1]-positions[cnt])/(times[cnt+1]-times[cnt])*TIME_STEP_MS;
        fsm.flankCounter = 0;
        unsigned long t2 = millis();
        unsigned long t3 = 0;
        tcc_set_duty_both(PER_24, position);
        t0 = millis();

        unsigned long t1 = millis();
        while(true) {
          delay(max(0,TIME_STEP_MS-(millis()-t1)));
          t1 = millis();

          bool update = false;
          while(times[cnt+1]<(t1-t0)){
            update = true;
            cnt += 1;
            position = positions[cnt];
            if(cnt == (nPoints-1))  break;
            else{
              delta = (positions[cnt+1]-positions[cnt])/(times[cnt+1]-times[cnt])*TIME_STEP_MS;
            }
          }          
          if(!update) position += delta;

          tcc_set_duty_both(PER_24, position);
          t3 = millis(); 
          Serial.println(t3 - t0);
          Serial.println(fsm.flankCounter);
          fsm.flankCounter = 0;
          Serial.println(position);
          if(update) if(cnt == (nPoints-1)) break;
        }
        Serial.print("[RUN] - Operation ");
        Serial.print(operationName);
        Serial.print(" - COMPLETED");
        fsm.s = ST_WAIT;
      break;
      }
    default:
      break;
  }

  //Long delay has no effect on the operation of hardware-based PWM channels
  // delay(2000);  
  // Serial.print("[READER] - Flank counter = ");
  // Serial.println(fsm.flankCounter);
  // Serial.print("[READER] - Tiempo[s] = ");
  // Serial.println((millis() - t0)/1000.0f);
  // Serial.print("[READER] - Flancos/s = ");
  // Serial.println(fsm.flankCounter*1000.0f/(millis() - t0));


  
  // Serial.println("[SERVO] - Awaiting new test time (1.0 - 1000.0) for pin 6");

  // if(!(Serial.available() > 0))
  //   return;  

  // String entrada = Serial.readStringUntil('\n');
  // entrada.trim(); // Elimina espacios y saltos de línea
  // if (entrada.length() > 0) {
  //   float valor = entrada.toFloat();
  //   if(valor < 1.0f || valor > 1000.0f) {
  //     Serial.println("[SERVO][ERROR] - Valor fuera de rango. Debe ser entre 1.0 y 1000.0.");        
  //   }else {
  //     Serial.print("[SERVO] - Nuevo valor recibido: ");
  //     Serial.println(valor);

  //     for(float i=5; i<=11; i+=0.1f){
  //       PWM_Instance->setPWM(6, 50.0f, i);
  //       delay(valor);
  //     }
  //     for(float i=11; i>=5; i-=0.1f){
  //       PWM_Instance->setPWM(6, 50.0f, i);
  //       delay(valor);
  //     }      
  //   }
  // }
  
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