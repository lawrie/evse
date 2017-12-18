// -*- C++ -*-
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <pins_arduino.h>
#include <Time.h>
#include "Arduino.h"

const char VERSTR[] PROGMEM = "Collinge v1";

//-- begin features

// serial port command line
#define SERIALCLI

//-- end features

//-- begin configuration

// n.b. DEFAULT_SERVICE_LEVEL is ignored if ADVPWR defined, since it's autodetected
#define DEFAULT_SERVICE_LEVEL 2 // 1=L1, 2=L2

// current capacity in amps
#define DEFAULT_CURRENT_CAPACITY_L1 12
#define DEFAULT_CURRENT_CAPACITY_L2 16

// minimum allowable current in amps
#define MIN_CURRENT_CAPACITY 6

// maximum allowable current in amps
#define MAX_CURRENT_CAPACITY_L1 16 // J1772 Max for L1 on a 20A circuit
#define MAX_CURRENT_CAPACITY_L2 80 // J1772 Max for L2

//J1772EVSEController
//#define CURRENT_PIN 0 // analog current reading pin A0
#define VOLT_PIN 1 // analog voltage reading pin A1
#define ACLINE1_PIN 3 // TEST PIN 1 for L1/L2, ground and stuck relay
#define ACLINE2_PIN 4 // TEST PIN 2 for L1/L2, ground and stuck relay
#define RED_LED_PIN 5 // Digital pin
#define CHARGING_PIN2 7 // digital Relay trigger pin for second relay
#define CHARGING_PIN 8 // digital Charging LED and Relay Trigger pin
#define PILOT_PIN 10 // n.b. PILOT_PIN *MUST* be digital 10 because SetPWM() assumes it
#define GREEN_LED_PIN 13 // Digital pin

#define SERIAL_BAUD 38400

// EEPROM offsets for settings
#define EOFS_CURRENT_CAPACITY_L1 0 // 1 byte
#define EOFS_CURRENT_CAPACITY_L2 1 // 1 byte
#define EOFS_FLAGS               2 // 1 byte


// must stay within thresh for this time in ms before switching states
#define DELAY_STATE_TRANSITION 250
// must transition to state A from contacts closed in < 100ms according to spec
// but Leaf sometimes bounces from 3->1 so we will debounce it a little anyway
#define DELAY_STATE_TRANSITION_A 25

//-- end configuration

//-- begin class definitions

#ifdef SERIALCLI
#define CLI_BUFLEN 20
class CLI {
  char m_CLIinstr[CLI_BUFLEN]; // CLI byte being read in
  int m_CLIstrCount; //CLI string counter
  char *m_strBuf;
  int m_strBufLen;

  void info();
public:
  CLI();
  void Init();
  void println(char *s) { 
    Serial.println(s); 
  }
  void println_P(const char *s);
  void print(char *s) { 
    Serial.print(s); 
  }
  void print_P(const char *s);
  void printlnn();
  void flush() { 
    Serial.flush(); 
  }
  void getInput();
  uint8_t getInt();
};
#endif // SERIALCLI

// Just sets LED in thiscut down version
class OnboardDisplay {
  
public:
  OnboardDisplay();
  void Init();
  void SetGreenLed(uint8_t state);
  void SetRedLed(uint8_t state);

  void Update();
};

typedef enum {
  PILOT_STATE_P12,PILOT_STATE_PWM,PILOT_STATE_N12} 
PILOT_STATE;
class J1772Pilot {
  uint8_t m_bit;
  uint8_t m_port;
  PILOT_STATE m_State;
public:
  J1772Pilot() {
  }
  void Init();
  void SetState(PILOT_STATE pstate); // P12/N12
  PILOT_STATE GetState() { 
    return m_State; 
  }
  int SetPWM(int amps); // 12V 1KHz PWM
};

// EVSE states for m_EvseState
#define EVSE_STATE_UNKNOWN 0x00
#define EVSE_STATE_A       0x01 // vehicle state A 12V - not connected
#define EVSE_STATE_B       0x02 // vehicle state B 9V - connected, ready
#define EVSE_STATE_C       0x03 // vehicle state C 6V - charging
#define EVSE_STATE_D       0x04 // vehicle state D 3V - vent required
#define EVSE_STATE_DIODE_CHK_FAILED 0x05 // diode check failed
#define EVSE_STATE_GFCI_FAULT 0x06       // GFCI fault
#define EVSE_STATE_NO_GROUND 0x07 //bad ground
#define EVSE_STATE_STUCK_RELAY 0x08 //stuck relay
#define EVSE_STATE_DISABLED 0xff // disabled

typedef struct threshdata {
  uint16_t m_ThreshAB; // state A -> B
  uint16_t m_ThreshBC; // state B -> C
  uint16_t m_ThreshCD; // state C -> D
  uint16_t m_ThreshD;  // state D
  uint16_t m_ThreshDS; // diode short
} THRESH_DATA,*PTHRESH_DATA;

typedef struct calibdata {
  uint16_t m_pMax;
  uint16_t m_pAvg;
  uint16_t m_pMin;
  uint16_t m_nMax;
  uint16_t m_nAvg;
  uint16_t m_nMin;
} CALIB_DATA,*PCALIB_DATA;

// J1772EVSEController m_bFlags bits - saved to EEPROM
#define ECF_L2                 0x01 // service level 2
#define ECF_DIODE_CHK_DISABLED 0x02 // no diode check
#define ECF_VENT_REQ_DISABLED  0x04 // no vent required state
#define ECF_GND_CHK_DISABLED   0x08 // no chk for ground fault
#define ECF_STUCK_RELAY_CHK_DISABLED 0x10 // no chk for stuck relay
#define ECF_AUTO_SVC_LEVEL_DISABLED  0x20 // auto detect svc level - requires ADVPWR
// Ability set the EVSE for manual button press to start charging - GoldServe
#define ECF_AUTO_START_DISABLED 0x40  // no auto start charging
#define ECF_SERIAL_DBG         0x80 // enable debugging messages via serial
#define ECF_DEFAULT            0x00

// J1772EVSEController volatile m_bVFlags bits - not saved to EEPROM
#define ECVF_NOGND_TRIPPED      0x20 // no ground has tripped at least once
#define ECVF_CHARGING_ON        0x40 // charging relay is closed
#define ECVF_GFI_TRIPPED        0x80 // gfi has tripped at least once
#define ECVF_DEFAULT            0x00

class J1772EVSEController {
  J1772Pilot m_Pilot;

  uint8_t m_bFlags; // ECF_xxx
  uint8_t m_bVFlags; // ECVF_xxx
  THRESH_DATA m_ThreshData;
  uint8_t m_EvseState;
  uint8_t m_PrevEvseState;
  uint8_t m_TmpEvseState;
  unsigned long m_TmpEvseStateStart;
  uint8_t m_CurrentCapacity; // max amps we can output
  unsigned long m_ChargeStartTimeMS;
  unsigned long m_ChargeOffTimeMS;
  time_t m_ChargeStartTime;
  time_t m_ChargeOffTime;
  time_t m_ElapsedChargeTime;
  time_t m_ElapsedChargeTimePrev;

  void chargingOn();
  void chargingOff();
  uint8_t chargingIsOn() { return m_bVFlags & ECVF_CHARGING_ON; }
  void setFlags(uint8_t flags) { 
    m_bFlags |= flags; 
  }
  void clrFlags(uint8_t flags) { 
    m_bFlags &= ~flags; 
  }

public:
  J1772EVSEController();
  void Init();
  void Update(); // read sensors
  void Enable();
  void Disable();
  void LoadThresholds();

  uint8_t GetFlags() { return m_bFlags; }
  uint8_t GetState() { 
    return m_EvseState; 
  }
  uint8_t GetPrevState() { 
    return m_PrevEvseState; 
  }
  int StateTransition() { 
    return (m_EvseState != m_PrevEvseState) ? 1 : 0; 
  }
  uint8_t GetCurrentCapacity() { 
    return m_CurrentCapacity; 
  }
  int SetCurrentCapacity(uint8_t amps,uint8_t updatepwm=0);
  //int GetCurrentReading() { return m_CurrentReading; }
  //float GetCurrentAmps();
  time_t GetElapsedChargeTime() { 
    return m_ElapsedChargeTime; 
  }
  time_t GetElapsedChargeTimePrev() { 
    return m_ElapsedChargeTimePrev; 
  }
  time_t GetChargeOffTime() { 
    return m_ChargeOffTime; 
  }
  void Calibrate(PCALIB_DATA pcd);
  uint8_t GetCurSvcLevel() { 
    return (m_bFlags & ECF_L2) ? 2 : 1; 
  }
  void SetSvcLevel(uint8_t svclvl);
  PTHRESH_DATA GetThreshData() { 
    return &m_ThreshData; 
  }
  uint8_t DiodeCheckEnabled() { 
    return (m_bFlags & ECF_DIODE_CHK_DISABLED) ? 0 : 1;
  }
  void EnableDiodeCheck(uint8_t tf);
  uint8_t VentReqEnabled() { 
    return (m_bFlags & ECF_VENT_REQ_DISABLED) ? 0 : 1;
  }
  void EnableVentReq(uint8_t tf);

  uint8_t SerDbgEnabled() { 
    return (m_bFlags & ECF_SERIAL_DBG) ? 1 : 0;
  }
  // Function to suppport Auto Start feature - GoldServe
  void EnableSerDbg(uint8_t tf);
};

// -- end class definitions

//-- begin global variables

char g_sTmp[64];

THRESH_DATA g_DefaultThreshData = {875,780,690,0,260};
J1772EVSEController g_EvseController;
OnboardDisplay g_OBD;

// Instantiate RTC and Delay Timer - GoldServe

#ifdef SERIALCLI
CLI g_CLI;

const char g_psEnabled[] PROGMEM = "enabled";
const char g_psDisabled[] PROGMEM = "disabled";
#endif // SERIALCLI

//-- end global variables

void EvseReset();

void SaveSettings()
{
  // n.b. should we add dirty bits so we only write the changed values? or should we just write them on the fly when necessary?
  EEPROM.write((g_EvseController.GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2,(byte)g_EvseController.GetCurrentCapacity());
  EEPROM.write(EOFS_FLAGS,g_EvseController.GetFlags());
}

#ifdef SERIALCLI
CLI::CLI()
{
  m_CLIstrCount = 0; 
  m_strBuf = g_sTmp;
  m_strBufLen = sizeof(g_sTmp);
}

void CLI::info()
{
  println_P(PSTR("OpenEVSE")); // CLI print prompt when serial is ready
  print_P(PSTR("Software - Open EVSE Ver ")); //CLI info
  println_P(VERSTR);
  printlnn();
}

void CLI::Init()
{
  info();
  println_P(PSTR("type help for command list"));
  print_P(PSTR("OpenEVSE> ")); // CLI Prompt
  flush();

}

uint8_t CLI::getInt()
{
  uint8_t c;
  uint8_t num = 0;

  do {
    c = Serial.read(); // read the byte
    if ((c >= '0') && (c <= '9')) {
      num = (num * 10) + c - '0';
    }
  } while (c != 13);
  return num;
}

void CLI::printlnn()
{
  println("");
}

const char g_pson[] PROGMEM = "on";
void CLI::getInput()
{
  int currentreading;
  uint8_t amp;
  if (Serial.available()) { // if byte(s) are available to be read
    char inbyte = (char) Serial.read(); // read the byte
    Serial.print(inbyte);
    if (inbyte != 13) { // CR
      if (((inbyte >= 'a') && (inbyte <= 'z')) || ((inbyte >= '0') && (inbyte <= '@') || (inbyte == ' ')) ) { //sar - allow ?
  m_CLIinstr[m_CLIstrCount] = inbyte;
  m_CLIstrCount++;
      }
      else if (m_CLIstrCount && ((inbyte == 8) || (inbyte == 127))) {
  m_CLIstrCount--;
      }
    }

    if ((inbyte == 13) || (m_CLIstrCount == CLI_BUFLEN-1)) { // if enter was pressed or max chars reached
      m_CLIinstr[m_CLIstrCount] = '\0';
      printlnn(); // print a newline
      
      if (strcmp_P(m_CLIinstr, PSTR("show")) == 0){ //if match SHOW 
        info();
        
        println_P(PSTR("Settings"));
        print_P(PSTR("Service level: "));
        Serial.println((int)g_EvseController.GetCurSvcLevel()); 
        print_P(PSTR("Current capacity (Amps): "));
        Serial.println((int)g_EvseController.GetCurrentCapacity()); 
        print_P(PSTR("Min Current Capacity: "));
        Serial.println(MIN_CURRENT_CAPACITY);
        print_P(PSTR("Max Current Capacity: "));
        Serial.println(MAX_CURRENT_CAPACITY_L2);
        print_P(PSTR("Vent Required: "));
        println_P(g_EvseController.VentReqEnabled() ? g_psEnabled : g_psDisabled);
        print_P(PSTR("Diode Check: "));
        println_P(g_EvseController.DiodeCheckEnabled() ? g_psEnabled : g_psDisabled);
      
        // Option to disable auto start - GoldServe
        // Start Delay Timer feature - GoldServe
      } 
      else if ((strcmp_P(m_CLIinstr, PSTR("help")) == 0) || (strcmp_P(m_CLIinstr, PSTR("?")) == 0)){ // string compare
        println_P(PSTR("Help Commands"));
        printlnn();
        println_P(PSTR("help - Display commands")); // print to the terminal
        println_P(PSTR("set  - Change settings"));
        println_P(PSTR("show - Display settings/values"));
        println_P(PSTR("save - Write to EEPROM"));
        // Start Delay Timer feature - GoldServe
        // End Delay Timer feature - GoldServe
      } 
      else if (strcmp_P(m_CLIinstr, PSTR("set")) == 0) { // string compare
        println_P(PSTR("Set Commands - Usage: set amp"));
        printlnn();
        println_P(PSTR("amp  - Set EVSE Current Capacity")); // print to the terminal
        println_P(PSTR("vntreq on/off - enable/disable vent required state"));
        println_P(PSTR("diochk on/off - enable/disable diode check"));
        println_P(PSTR("sdbg on/off - turn serial debugging on/off"));
      }
      else if (strncmp_P(m_CLIinstr, PSTR("set "),4) == 0) {
        char *p = m_CLIinstr + 4;
        if (!strncmp_P(p,PSTR("sdbg "),5)) {
          p += 5;
          print_P(PSTR("serial debugging "));
          if (!strcmp_P(p,g_pson)) {
            g_EvseController.EnableSerDbg(1);
            println_P(g_psEnabled);
          }
          else {
            g_EvseController.EnableSerDbg(0);
            println_P(g_psDisabled);
          }
        }
        else if (!strncmp_P(p,PSTR("vntreq "),7)) {
          p += 7;
          print_P(PSTR("vent required "));
          if (!strcmp_P(p,g_pson)) {
            g_EvseController.EnableVentReq(1);
            println_P(g_psEnabled);
          }
          else {
            g_EvseController.EnableVentReq(0);
            println_P(g_psDisabled);
          }
        }
        else if (!strncmp_P(p,PSTR("diochk "),7)) {
          p += 7;
          print_P(PSTR("diode check "));
          if (!strcmp_P(p,g_pson)) {
            g_EvseController.EnableDiodeCheck(1);
            println_P(g_psEnabled);
          }
          else {
            g_EvseController.EnableDiodeCheck(0);
            println_P(g_psDisabled);
          }
        }
        else if (!strcmp_P(p,PSTR("amp"))){ // string compare
          println_P(PSTR("WARNING - Do not set higher than 80% of breaker value"));
          printlnn();
          print_P(PSTR("Enter amps ("));
          Serial.print(MIN_CURRENT_CAPACITY);
          print_P(PSTR("-"));
          Serial.print((g_EvseController.GetCurSvcLevel()  == 1) ? MAX_CURRENT_CAPACITY_L1 : MAX_CURRENT_CAPACITY_L2);
          print_P(PSTR("): "));
          amp = getInt();
          Serial.println((int)amp);
          if(g_EvseController.SetCurrentCapacity(amp,1)) {
            println_P(PSTR("Invalid Setting"));
          }
    
          print_P(PSTR("Current now: ")); // print to the terminal
          Serial.print((int)g_EvseController.GetCurrentCapacity());
          print_P(PSTR("A"));
        } 
        else {
          goto unknown;
        }
      }
      else if (strcmp_P(m_CLIinstr, PSTR("save")) == 0){ // string compare
        println_P(PSTR("Saving to EEPROM")); // print to the terminal
        SaveSettings();
      } 
      // Start Delay Timer feature - GoldServe
      // End Delay Timer feature - GoldServe
      else { // if the input text doesn't match any defined above
      unknown:
        println_P(PSTR("Unknown Command -- type help for command list")); // echo back to the terminal
      } 
      printlnn();
//      printlnn();
      print_P(PSTR("OpenEVSE> "));
      g_CLI.flush();
      m_CLIstrCount = 0; // get ready for new input... reset strCount
      m_CLIinstr[0] = '\0'; // set to null to erase it
    }
  }
}

void CLI::println_P(const char *s)
{
  strncpy_P(m_strBuf,s,m_strBufLen);
  println(m_strBuf);
}

void CLI::print_P(const char *s)
{
  strncpy_P(m_strBuf,s,m_strBufLen);
  print(m_strBuf);
}

#endif // SERIALCLI

OnboardDisplay::OnboardDisplay()
{
} 

void OnboardDisplay::Init()
{
  pinMode (GREEN_LED_PIN, OUTPUT);
  pinMode (RED_LED_PIN, OUTPUT);

  SetGreenLed(LOW);
  SetRedLed(LOW); 
}

void OnboardDisplay::SetGreenLed(uint8_t state)
{
  digitalWrite(GREEN_LED_PIN,state);
}

void OnboardDisplay::SetRedLed(uint8_t state)
{
  digitalWrite(RED_LED_PIN,state);
}

void OnboardDisplay::Update()
{
  uint8_t curstate = g_EvseController.GetState();
  uint8_t svclvl = g_EvseController.GetCurSvcLevel();
  int i;

  if (g_EvseController.StateTransition()) {
    switch(curstate) {
    case EVSE_STATE_A: // not connected
      SetGreenLed(HIGH);
      SetRedLed(LOW);
      // n.b. blue LED is off
      break;
    case EVSE_STATE_B: // connected/not charging
      SetGreenLed(HIGH);
      SetRedLed(HIGH);
      // n.b. blue LED is off
      break;
    case EVSE_STATE_C: // charging
      SetGreenLed(LOW);
      SetRedLed(LOW);
      // n.b. blue LED is on
      break;
    case EVSE_STATE_D: // vent required
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      // n.b. blue LED is off
      break;
    case EVSE_STATE_DIODE_CHK_FAILED:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      // n.b. blue LED is off
      break;
    case EVSE_STATE_GFCI_FAULT:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      // n.b. blue LED is off
      break;
     case EVSE_STATE_NO_GROUND:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      // n.b. blue LED is off
      break;
     case EVSE_STATE_STUCK_RELAY:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      // n.b. blue LED is off
      break;
    case EVSE_STATE_DISABLED:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      break;
    default:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      // n.b. blue LED is off
    }
  }
}

//-- begin J1772Pilot

void J1772Pilot::Init()
{
  pinMode(PILOT_PIN,OUTPUT);
  m_bit = digitalPinToBitMask(PILOT_PIN);
  m_port = digitalPinToPort(PILOT_PIN);

  SetState(PILOT_STATE_P12); // turns the pilot on 12V steady state
}

// no PWM pilot signal - steady state
// PILOT_STATE_P12 = steady +12V (EVSE_STATE_A - VEHICLE NOT CONNECTED)
// PILOT_STATE_N12 = steady -12V (EVSE_STATE_F - FAULT) 
void J1772Pilot::SetState(PILOT_STATE state)
{
  volatile uint8_t *out = portOutputRegister(m_port);

  uint8_t oldSREG = SREG;
  cli();
  TCCR1A = 0; //disable pwm by turning off COM1A1,COM1A0,COM1B1,COM1B0
  if (state == PILOT_STATE_P12) {
    *out |= m_bit;  // set pin high
  }
  else {
    *out &= ~m_bit;  // set pin low
  }
  SREG = oldSREG;

  m_State = state;
}

// set EVSE current capacity in Amperes
// duty cycle 
// outputting a 1KHz square wave to digital pin 10 via Timer 1
//
int J1772Pilot::SetPWM(int amps)
{
  uint8_t ocr1b = 0;
  if ((amps >= 6) && (amps <= 51)) {
    ocr1b = 25 * amps / 6 - 1;  // J1772 states "Available current = (duty cycle %) X 0.6"
   } else if ((amps > 51) && (amps <= 80)) {
     ocr1b = amps + 159;  // J1772 states "Available current = (duty cycle % - 64) X 2.5"
   }
  else {
    return 1; // error
  }

  if (ocr1b) {
    // Timer1 initialization:
    // 16MHz / 64 / (OCR1A+1) / 2 on digital 9
    // 16MHz / 64 / (OCR1A+1) on digital 10
    // 1KHz variable duty cycle on digital 10, 500Hz fixed 50% on digital 9
    // pin 10 duty cycle = (OCR1B+1)/(OCR1A+1)
    uint8_t oldSREG = SREG;
    cli();

    TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
    OCR1A = 249;

    // 10% = 24 , 96% = 239
    OCR1B = ocr1b;

    SREG = oldSREG;

    m_State = PILOT_STATE_PWM;
    return 0;
  }
  else { // !duty
    // invalid amps
    return 1;
  }
}

//-- end J1772Pilot

//-- begin J1772EVSEController

J1772EVSEController::J1772EVSEController()
{
}

void J1772EVSEController::chargingOn()
{  // turn on charging current
  Serial.println("Charging on");
  digitalWrite(CHARGING_PIN,HIGH); 
  digitalWrite(CHARGING_PIN2,HIGH); 
  m_bVFlags |= ECVF_CHARGING_ON;
  
  m_ChargeStartTime = now();
  m_ChargeStartTimeMS = millis();
}

void J1772EVSEController::chargingOff()
{ // turn off charging current
  Serial.println("Charging off");
  digitalWrite(CHARGING_PIN,LOW); 
  digitalWrite(CHARGING_PIN2,LOW); 
  m_bVFlags &= ~ECVF_CHARGING_ON;

  m_ChargeOffTime = now();
  m_ChargeOffTimeMS = millis();
} 

void J1772EVSEController::EnableDiodeCheck(uint8_t tf)
{
  if (tf) {
    m_bFlags &= ~ECF_DIODE_CHK_DISABLED;
  }
  else {
    m_bFlags |= ECF_DIODE_CHK_DISABLED;
  }
}

void J1772EVSEController::EnableVentReq(uint8_t tf)
{
  if (tf) {
    m_bFlags &= ~ECF_VENT_REQ_DISABLED;
  }
  else {
    m_bFlags |= ECF_VENT_REQ_DISABLED;
  }
}

void J1772EVSEController::EnableSerDbg(uint8_t tf)
{
  if (tf) {
    m_bFlags |= ECF_SERIAL_DBG;
  }
  else {
    m_bFlags &= ~ECF_SERIAL_DBG;
  }
}

void J1772EVSEController::Enable()
{
  m_PrevEvseState = EVSE_STATE_DISABLED;
  m_EvseState = EVSE_STATE_UNKNOWN;
  m_Pilot.SetState(PILOT_STATE_P12);
}

void J1772EVSEController::Disable()
{
  m_EvseState = EVSE_STATE_DISABLED;
  chargingOff();
  m_Pilot.SetState(PILOT_STATE_N12);
  g_OBD.Update();
  // cancel state transition so g_OBD doesn't keep updating
  m_PrevEvseState = EVSE_STATE_DISABLED;
}

void J1772EVSEController::LoadThresholds()
{
    memcpy(&m_ThreshData,&g_DefaultThreshData,sizeof(m_ThreshData));
}

void J1772EVSEController::SetSvcLevel(uint8_t svclvl)
{
#ifdef SERIALCLI
  if (SerDbgEnabled()) {
    g_CLI.printlnn();
    g_CLI.print_P(PSTR("SetSvcLevel: "));Serial.println((int)svclvl);
  }
#endif //#ifdef SERIALCLI
  if (svclvl == 2) {
    m_bFlags |= ECF_L2; // set to Level 2
  }
  else {
    svclvl = 1;
    m_bFlags &= ~ECF_L2; // set to Level 1
  }

  uint8_t ampacity =  EEPROM.read((svclvl == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2);

  if ((ampacity == 0xff) || (ampacity == 0)) {
    ampacity = (svclvl == 1) ? DEFAULT_CURRENT_CAPACITY_L1 : DEFAULT_CURRENT_CAPACITY_L2;
  }
  
  if (ampacity < MIN_CURRENT_CAPACITY) {
    ampacity = MIN_CURRENT_CAPACITY;
  }
  else {
    if (svclvl == 1) { // L1
      if (ampacity > MAX_CURRENT_CAPACITY_L1) {
        ampacity = MAX_CURRENT_CAPACITY_L1;
      }
    }
    else {
      if (ampacity > MAX_CURRENT_CAPACITY_L2) {
        ampacity = MAX_CURRENT_CAPACITY_L2;
      }
    }
  }

  LoadThresholds();

  SetCurrentCapacity(ampacity);
}

void J1772EVSEController::Init()
{
  pinMode(CHARGING_PIN,OUTPUT);
  pinMode(CHARGING_PIN2,OUTPUT);

  chargingOff();

  m_Pilot.Init(); // init the pilot

  uint8_t svclvl = (uint8_t)DEFAULT_SERVICE_LEVEL;

  uint8_t rflgs = EEPROM.read(EOFS_FLAGS);
  if (rflgs == 0xff) { // uninitialized EEPROM
    m_bFlags = ECF_DEFAULT;
  }
  else {
    m_bFlags = rflgs;
    svclvl = GetCurSvcLevel();
  }

  m_bVFlags = ECVF_DEFAULT;

  m_EvseState = EVSE_STATE_UNKNOWN;
  m_PrevEvseState = EVSE_STATE_UNKNOWN;  

  SetSvcLevel(svclvl);

  g_OBD.SetGreenLed(LOW);
}

//TABLE A1 - PILOT LINE VOLTAGE RANGES (recommended.. adjust as necessary
//                           Minimum Nominal Maximum 
//Positive Voltage, State A  11.40 12.00 12.60 
//Positive Voltage, State B  8.36 9.00 9.56 
//Positive Voltage, State C  5.48 6.00 6.49 
//Positive Voltage, State D  2.62 3.00 3.25 
//Negative Voltage - States B, C, D, and F -11.40 -12.00 -12.60 
void J1772EVSEController::Update()
{
  if (m_EvseState == EVSE_STATE_DISABLED) {
    // n.b. don't know why, but if we don't have the delay below
    // then doEnableEvse will always have packetBuf[2] == 0
    // so we can't re-enable the EVSE
    delay(5);
    return;
  }

  uint8_t prevevsestate = m_EvseState;
  uint8_t tmpevsestate = EVSE_STATE_UNKNOWN;
  uint8_t nofault = 1;

  int plow;
  int phigh;

   if (nofault) {
    if ((prevevsestate == EVSE_STATE_GFCI_FAULT) ||
        (prevevsestate == EVSE_STATE_NO_GROUND) ||
  (prevevsestate == EVSE_STATE_STUCK_RELAY)) {
      // just got out of GFCI fault state - pilot back on
      m_Pilot.SetState(PILOT_STATE_P12);
      prevevsestate = EVSE_STATE_UNKNOWN;
      m_EvseState = EVSE_STATE_UNKNOWN;
    }
    // Begin Sensor readings
    int reading;
    plow = 1023;
    phigh = 0;
    //  digitalWrite(3,HIGH);
    // 1x = 114us 20x = 2.3ms 100x = 11.3ms
    for (int i=0;i < 100;i++) {
      reading = analogRead(VOLT_PIN);  // measures pilot voltage
      if (reading > phigh) {
        phigh = reading;
      }
      else if (reading < plow) {
        plow = reading;
      }
    }
    
    if (DiodeCheckEnabled() && (m_Pilot.GetState() == PILOT_STATE_PWM) && (plow >= m_ThreshData.m_ThreshDS)) {
      // diode check failed
      tmpevsestate = EVSE_STATE_DIODE_CHK_FAILED;
    }
    else if (phigh >= m_ThreshData.m_ThreshAB) {
      // 12V EV not connected
      tmpevsestate = EVSE_STATE_A;
    }
    else if (phigh >= m_ThreshData.m_ThreshBC) {
      // 9V EV connected, waiting for ready to charge
      tmpevsestate = EVSE_STATE_B;
    }
    else if ((phigh  >= m_ThreshData.m_ThreshCD) ||
       (!VentReqEnabled() && (phigh > m_ThreshData.m_ThreshD))) {
      // 6V ready to charge
      tmpevsestate = EVSE_STATE_C;
    }
    else if (phigh > m_ThreshData.m_ThreshD) {
      // 3V ready to charge vent required
      tmpevsestate = EVSE_STATE_D;
    }
    else {
      tmpevsestate = EVSE_STATE_UNKNOWN;
    }

    // debounce state transitions
    if (tmpevsestate != prevevsestate) {
      if (tmpevsestate != m_TmpEvseState) {
        m_TmpEvseStateStart = millis();
      }
      else if ((millis()-m_TmpEvseStateStart) >= ((tmpevsestate == EVSE_STATE_A) ? DELAY_STATE_TRANSITION_A : DELAY_STATE_TRANSITION)) {
        m_EvseState = tmpevsestate;
      }
    }
  }

  m_TmpEvseState = tmpevsestate;

  //Serial.print("State: ");
  //Serial.println(m_TmpEvseState);
  
  // state transition
  if (m_EvseState != prevevsestate) {
    if (m_EvseState == EVSE_STATE_A) { // connected, not ready to charge
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_P12);
    }
    else if (m_EvseState == EVSE_STATE_B) { // connected 
      chargingOff(); // turn off charging current
      m_Pilot.SetPWM(m_CurrentCapacity);
    }
    else if (m_EvseState == EVSE_STATE_C) {
      m_Pilot.SetPWM(m_CurrentCapacity);
      chargingOn(); // turn on charging current
    }
    else if (m_EvseState == EVSE_STATE_D) {
      // vent required not supported
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_P12);
    }
    else if (m_EvseState == EVSE_STATE_GFCI_FAULT) {
      // vehicle state F
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_N12);
    }
    else if (m_EvseState == EVSE_STATE_DIODE_CHK_FAILED) {
      chargingOff(); // turn off charging current
      // must leave pilot on so we can keep checking
      m_Pilot.SetPWM(m_CurrentCapacity);
    }
    else if (m_EvseState == EVSE_STATE_NO_GROUND) {
      // Ground not detected
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_N12);
    }
    else if (m_EvseState == EVSE_STATE_STUCK_RELAY) {
      // Stuck relay detected
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_N12);
    }
    else {
      m_Pilot.SetState(PILOT_STATE_P12);
      chargingOff(); // turn off charging current
    }
#ifdef SERIALCLI
    if (SerDbgEnabled()) {
      g_CLI.print_P(PSTR("state: "));
      Serial.print((int)prevevsestate);
      g_CLI.print_P(PSTR("->"));
      Serial.print((int)m_EvseState);
      g_CLI.print_P(PSTR(" p "));
      Serial.print(plow);
      g_CLI.print_P(PSTR(" "));
      Serial.println(phigh);
    }
#endif //#ifdef SERIALCLI
  }

  m_PrevEvseState = prevevsestate;

  // End Sensor Readings

  if (m_EvseState == EVSE_STATE_C) {
    m_ElapsedChargeTimePrev = m_ElapsedChargeTime;
    m_ElapsedChargeTime = now() - m_ChargeStartTime;
  }
}

// read ADC values and get min/max/avg for pilot steady high/low states
void J1772EVSEController::Calibrate(PCALIB_DATA pcd)
{
  uint16_t pmax,pmin,pavg,nmax,nmin,navg;

  for (int l=0;l < 2;l++) {
    int reading;
    uint32_t tot = 0;
    uint16_t plow = 1023;
    uint16_t phigh = 0;
    uint16_t avg = 0;
    m_Pilot.SetState(l ? PILOT_STATE_N12 : PILOT_STATE_P12);

    delay(250); // wait for stabilization

    // 1x = 114us 20x = 2.3ms 100x = 11.3ms
    uint8_t pin = PILOT_PIN;
    int i;
    for (i=0;i < 1000;i++) {
      reading = analogRead(VOLT_PIN);  // measures pilot voltage

      if (reading > phigh) {
        phigh = reading;
      }
      else if (reading < plow) {
        plow = reading;
      }

      tot += reading;
    }
    avg = tot / i;

    if (l) {
      nmax = phigh;
      nmin = plow;
      navg = avg;
    }
    else {
      pmax = phigh;
      pmin = plow;
      pavg = avg;
    }
  }
  pcd->m_pMax = pmax;
  pcd->m_pAvg = pavg;
  pcd->m_pMin = pmin;
  pcd->m_nMax = nmax;
  pcd->m_nAvg = navg;
  pcd->m_nMin = nmin;
}

int J1772EVSEController::SetCurrentCapacity(uint8_t amps,uint8_t updatepwm)
{
  int rc = 0;
  if ((amps >= MIN_CURRENT_CAPACITY) && (amps <= ((GetCurSvcLevel() == 1) ? MAX_CURRENT_CAPACITY_L1 : MAX_CURRENT_CAPACITY_L2))) {
    m_CurrentCapacity = amps;
  }
  else {
    m_CurrentCapacity = MIN_CURRENT_CAPACITY;
    rc = 1;
  }

  if (updatepwm && (m_Pilot.GetState() == PILOT_STATE_PWM)) {
    m_Pilot.SetPWM(m_CurrentCapacity);
  }
  return rc;
}

//-- end J1772EVSEController

void EvseReset()
{

#ifdef SERIALCLI
  g_CLI.Init();
#endif // SERIALCLI

  g_OBD.Init();
  g_EvseController.Init();
}

void setup()
{
  wdt_disable();
  
  Serial.begin(SERIAL_BAUD);
  EvseReset();
} 

void loop()
{ 

#ifdef SERIALCLI
  g_CLI.getInput();
#endif // SERIALCLI

  g_EvseController.Update();
  g_OBD.Update();
}

