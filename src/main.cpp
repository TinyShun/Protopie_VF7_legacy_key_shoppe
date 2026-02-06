#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#endif

#include <ACAN2517FD.h>
#include <SPI.h>
#include <string.h>
#include <lin_frame.h>
#include <can_defs.h>

// Map pin for MCP2515
static const byte MCP2517_SCK  = 18;
static const byte MCP2517_MOSI = 23;
static const byte MCP2517_MISO = 19;
static const byte MCP2517_CS  = 5;
static const byte MCP2517_INT = 21;

// ACAN2517FD Driver object
ACAN2517FD acan (MCP2517_CS, SPI, MCP2517_INT) ;

CANFDMessage g_rxCanMsg;
volatile uint32_t gCanRxCount = 0;
volatile uint32_t gCanTxCount = 0;

unsigned long gLastCanPrintMs = 0;

#define CAN_PRINT_INTERVAL_MS 1000 

enum IgnState {
    IGN_OFF = 0,
    IGN_ACC = 1,
    IGN_ON  = 2
};

IgnState gIgnState = IGN_OFF;

// Handle BLE Key Commands
enum KeyCommand {
    KEY_NONE = 0,
    KEY_LOCK = 1,
    KEY_UNLOCK = 2,
    KEY_TRUNK = 3
};

KeyCommand gLastKeyCommand = KEY_NONE;
bool gKeyCommandProcessed = true;
#define PIN_LOCK_BUTTON 25
#define PIN_UNLOCK_BUTTON 33
#define PIN_DOOR_SWITCH  14
bool gDoorOpened = false;

// Handle UART1
#define UART1_RX_PIN  26
#define UART1_TX_PIN  27

//Handle G29 Pedals/Gas
#define PIN_BRAKE_ADC   34
#define PIN_GAS_ADC     35
#define BRAKE_RAW_MIN 2250 //2000
#define BRAKE_RAW_MAX 3650 //3900
#define GAS_RAW_MIN   440 //190
#define GAS_RAW_MAX   3650 //3900
uint8_t brakePercent = 0;
uint8_t gasPercent   = 0;
uint8_t prevBrakePercent = 0xFF;
uint8_t prevGasPercent   = 0xFF;
#define PEDAL_PRINT_THRESHOLD  1
uint8_t brakeIndex = 0, gasIndex = 0;
bool brakeFilled = false, gasFilled = false;
uint8_t gSpeed = 0;


// Handle Gear
uint8_t gGearMapped = 0x00; // 0:P, 1:R, 2:N, 3:D


// Handle MA
#define MA_WINDOW_SIZE  20
uint16_t brakeBuf[MA_WINDOW_SIZE] = {0};
uint16_t gasBuf[MA_WINDOW_SIZE]   = {0};
uint32_t brakeSum = 0;
uint32_t gasSum   = 0;


// Handle wheel turn 
uint8_t prevWheelTurn = 0xFF;

// Handle ADC conversion
#define ADC_MAX           4095.0f
#define ADC_INTERVAL_MS  50

// Handle LIN
unsigned long lastAdcRead = 0;
unsigned long vrPressedStart = 0;
bool vrButtonPressed = false;
int vrState = 1;
uint8_t vrPressType = 0;
#define LIN_TARGET_ID  0x61

//enum
enum TurnState : uint8_t {
  TURN_NONE    = 0,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_OUTWARD,
  TURN_INWARD
};
//prevState
unsigned long gLastReq = 0;
uLIN_MSG gLinMsg;
TurnState gCurrentState = TURN_NONE;
TurnState gLastState    = TURN_NONE;

void printToProtopie(String message, String value )
{
    if (value.length() == 0) {
        Serial.println(message);
    } else {
        Serial.println(message + "||" + value);
    }
}

//Moving average filter
uint16_t movingAverage(uint16_t *buf, uint32_t &sum,
                       uint8_t &index, bool &filled,
                       uint16_t newVal)
{
    sum -= buf[index];
    buf[index] = newVal;
    sum += newVal;

    uint16_t avg = sum / (filled ? MA_WINDOW_SIZE : (index + 1));

    index++;
    if (index >= MA_WINDOW_SIZE) {
        index = 0;
        filled = true;
    }

    return avg;
}

// HandleADC Pedals
void handleAdcPedals()
{
    unsigned long now = millis();
    if (now - lastAdcRead < ADC_INTERVAL_MS)
        return;

    lastAdcRead = now;

    int rawBrake = analogRead(PIN_BRAKE_ADC);
    int rawGas   = analogRead(PIN_GAS_ADC);

    if (rawBrake < BRAKE_RAW_MIN) rawBrake = BRAKE_RAW_MIN;
    if (rawBrake > BRAKE_RAW_MAX) rawBrake = BRAKE_RAW_MAX;

    if (rawGas < GAS_RAW_MIN) rawGas = GAS_RAW_MIN;
    if (rawGas > GAS_RAW_MAX) rawGas = GAS_RAW_MAX;
    uint16_t brakeFilt = movingAverage( brakeBuf, brakeSum, brakeIndex, brakeFilled, rawBrake);
    uint16_t gasFilt = movingAverage(gasBuf, gasSum, gasIndex, gasFilled, rawGas);

    //Normalize filtered values
    float brakeNorm = (float)(BRAKE_RAW_MAX - brakeFilt) /
                    (float)(BRAKE_RAW_MAX - BRAKE_RAW_MIN);

    float gasNorm   = (float)(GAS_RAW_MAX - gasFilt) /
                    (float)(GAS_RAW_MAX - GAS_RAW_MIN);


    // Convert to percentage 
    brakePercent = (uint8_t)(brakeNorm * 100.0f + 0.5f);
    gasPercent   = (uint8_t)(gasNorm   * 100.0f + 0.5f);

    //Print only if change >= 2%
    if (abs((int)brakePercent - (int)prevBrakePercent) >= (PEDAL_PRINT_THRESHOLD)) {
        Serial.print("BRAKE_PERCENT||");
        Serial.println(brakePercent);
        prevBrakePercent = brakePercent;
    }

    if (abs((int)gasPercent - (int)prevGasPercent) >= PEDAL_PRINT_THRESHOLD) {
        Serial.print("GAS_PERCENT||");
        Serial.println(gasPercent);
        prevGasPercent = gasPercent;
    }
}

// Handle pressed
bool brakePressed()
{
    return brakePercent > 20;
}

// Door Handle
void handleDoor()
{
    bool doorNow = (digitalRead(PIN_DOOR_SWITCH) == LOW);

    if (doorNow != gDoorOpened) {
        gDoorOpened = doorNow;
        Serial.print("DOOR||");
        Serial.println(gDoorOpened ? "OPENED" : "CLOSED");
    }
}

// Handle change bit ign state
void setIgnStateToCan(uint8_t ign)
{
    /* BCM_Clamp_Stat – ID 0x112 – BYTE 4 */
    txTasks[TX_BCM_CLAMP_STAT].canMess.data[4] = ign;
}

// Handle CAN RX
void handleCanRx()
{
     while (acan.receive(g_rxCanMsg)) {
      gCanRxCount++;

        switch (g_rxCanMsg.id) {

            case 0x108: {  // Gear selector

                    if (brakePressed() && !gDoorOpened) {

                        static uint8_t prevGearMapped = 0xFF;  // lưu gear đã gửi trước đó

                        uint8_t gearRaw = g_rxCanMsg.data[2];
                        uint8_t gearMapped = 0xFF;

                    switch (gearRaw) {
                        case 0x00:
                                Serial.println("GEAR||P");
                                gearMapped = 00;
                                break;

                        case 0x20:
                                Serial.println("GEAR||R");
                                gearMapped = 01;
                                break;

                        case 0x40:
                                Serial.println("GEAR||N");
                                gearMapped = 02;
                                break;

                        case 0x60:
                                Serial.println("GEAR||D");
                                gearMapped = 03;
                                break;

                        default:
                                break;
                        }
                        if (gearMapped != 0xFF && gearMapped != prevGearMapped) {
                            txTasks[TX_VCU_HV_DRVSYS_STATUS].canMess.data[4] = gearMapped;
                            prevGearMapped = gearMapped;
                            gGearMapped = gearMapped;
                        }
                    }
                    break;
                }


            case 0x17E: {  // Steering angle sensor
                if (g_rxCanMsg.len < 7) break;
                    uint16_t rawAngle =
                    ((uint16_t)g_rxCanMsg.data[5] << 8) |
                    g_rxCanMsg.data[6];
                    uint8_t wheelTurn =
                    100 - ((uint32_t)rawAngle * 100 / 65535);

                if (wheelTurn != prevWheelTurn) {
                    Serial.print("WHEEL_TURN||");
                    Serial.println(wheelTurn);
                    prevWheelTurn = wheelTurn;
                }
                break;
            }
            default:
                break;
        }
    }
}

// Handle CAN debug
void candebug()
{
    unsigned long now = millis();
    if (now - gLastCanPrintMs >= CAN_PRINT_INTERVAL_MS) {
        gLastCanPrintMs = now;
    }
}



// Handle LIN and Debug
// Slave response: ID + 8 data + checksum
bool tryReceiveLin(uLIN_MSG &msg) {
  static uint8_t idx = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (idx == 0) {
      msg.frame.id = b;
    }
    else if (idx >= 1 && idx <= 8) {
      msg.array[idx] = b;   
    }
    else if (idx == 9) {
      msg.frame.checkSum = b;
      idx = 0;
      return true;          
    }

    idx++;
  }

  return false;
}

// Byte cần decode = msg.array[3]
TurnState decodeTurnState(uint8_t val) {
  if (val & 0x04 || val & 0x08) {
    return TURN_RIGHT;
  }
  if (val & 0x02 || val & 0x01) {
    return TURN_LEFT;
  }
  if (val & 0x10) {
    return TURN_OUTWARD;
  }
  if (val & 0x20) {
    return TURN_INWARD;
  }
  return TURN_NONE;
}

// Handle turn state print
void printTurnState(TurnState state) {
  switch (state) {
    case TURN_LEFT:
      Serial.println("STATE: TURN_LEFT");
      break;
    case TURN_RIGHT:
      Serial.println("STATE: TURN_RIGHT");
      break;
    case TURN_OUTWARD:
      Serial.println("STATE: OUTWARD");
      break;
    case TURN_INWARD:
      Serial.println("STATE: INWARD");
      break;
    default:
      Serial.println("STATE: NONE");
      break;
  }
}

// Handle turn state machine
void handleTurnStateMachine(const uLIN_MSG& msg) {
  // Byte sau 0x7C, trước ALV
  uint8_t val = msg.array[3];

  gCurrentState = decodeTurnState(val);

  if (gCurrentState != gLastState) {
    printTurnState(gCurrentState);
    gLastState = gCurrentState;
  }
}
//Handle uart send to protopie
void handleUart1Debug()
{
    static uint8_t prevGear = 0xFF;
    if (gGearMapped != prevGear) {
        prevGear = gGearMapped;
        Serial1.print("GEAR||");
        Serial1.println(gGearMapped); // Placeholder for brake percent
        Serial1.flush();
    }

}

// Handle speed from Serial USB
void handleSpeed()
{
    static char buffer[16];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buffer[idx] = '\0';

                uint16_t speedValue = atoi(buffer); 
                Serial1.print("SPEED||");
                Serial1.println(speedValue);
                gSpeed = (uint8_t)(speedValue & 0xFF);

                // Set speed to IDB_Status (0x20D) - bits 27-40 (14 bits)
                // Bit 27-40: byte 3 (bits 3-7), byte 4 (bits 0-7), byte 5 (bits 0-4)
                // speedValue is 14-bit value (0-16383)
                uint16_t speed14bit = speedValue & 0x3FFF;  // Mask to 14 bits
                // Distribute 14 bits across bytes 3, 4, 5
                txTasks[TX_IDB_STATUS].canMess.data[3] = (txTasks[TX_IDB_STATUS].canMess.data[3] & 0x07) | ((speed14bit & 0x1F) << 3);
                txTasks[TX_IDB_STATUS].canMess.data[4] = (speed14bit >> 5) & 0xFF;
                txTasks[TX_IDB_STATUS].canMess.data[5] = (txTasks[TX_IDB_STATUS].canMess.data[5] & 0xE0) | ((speed14bit >> 13) & 0x1F);
            }
            idx = 0;
        }
        else if (idx < sizeof(buffer) - 1) {
            buffer[idx++] = c;
        }
    }
}

// Handle keyfob
void handleKeyAction()
{
    static bool prevLock = HIGH;
    static bool prevUnlock = HIGH;

    bool lockNow = digitalRead(PIN_LOCK_BUTTON);
    bool unlockNow = digitalRead(PIN_UNLOCK_BUTTON);

    // UNLOCK: OFF -> ACC
    if (unlockNow == LOW && prevUnlock == HIGH) {
        if (gIgnState == IGN_OFF) {
            gIgnState = IGN_ACC;
            setIgnStateToCan(IGN_ACC);
            Serial1.println("IGN||ACC");
        }
    }

    // LOCK: ACC/ON -> OFF
    if (lockNow == LOW && prevLock == HIGH) {
        if (gIgnState == IGN_ACC || gIgnState == IGN_ON) {
            setIgnStateToCan(IGN_OFF);
            gIgnState = IGN_OFF;

            txTasks[TX_VCU_HV_DRVSYS_STATUS].canMess.data[4] = 0x00;
            gGearMapped = 0x00;

            Serial1.println("GEAR||P");
            Serial1.println("IGN||OFF");
        }
    }

    prevLock = lockNow;
    prevUnlock = unlockNow;

    // Brake: ACC -> ON
    if (gIgnState == IGN_ACC && brakePressed()) {
        gIgnState = IGN_ON;
        setIgnStateToCan(IGN_ON);
        Serial1.println("IGN||ON");
        Serial1.print("pedals_brake||");
        Serial1.println("50");}
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(19200, SERIAL_8N1, 16, 17);   // LIN
    Serial1.begin(115200, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN); //26 27
    // Serial.println("[BLE] SCANNING...");
    // Pin init
    pinMode(PIN_DOOR_SWITCH, INPUT_PULLUP);
    pinMode(PIN_LOCK_BUTTON, INPUT_PULLUP);
    pinMode(PIN_UNLOCK_BUTTON, INPUT_PULLUP);
    //pinMode(PIN_VR, INPUT_PULLUP);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    pinMode(PIN_BRAKE_ADC, INPUT);
    pinMode(PIN_GAS_ADC, INPUT);

    Serial.println("ESP32 main + CAN scheduler ready");
    //Begin SPI
    SPI.begin (MCP2517_SCK, MCP2517_MISO, MCP2517_MOSI) ;
// //--- Configure ACAN2517FD
//   Serial.print ("sizeof (ACAN2517FDSettings): ") ;
//   Serial.print (sizeof (ACAN2517FDSettings)) ;
//   Serial.println (" bytes") ;
//   Serial.println ("Configure ACAN2517FD") ;
//--- For version >= 2.1.0
  ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_40MHz, 500UL * 1000UL, DataBitRateFactor::x4) ;
//--- For version < 2.1.0
//  ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_4MHz10xPLL, 125 * 1000, ACAN2517FDSettings::DATA_BITRATE_x1) ;
  settings.mRequestedMode = ACAN2517FDSettings::NormalFD ; // Select loopback mode
//--- RAM Usage
  // Serial.print ("MCP2517FD RAM Usage: ") ;
  // Serial.print (settings.ramUsage ()) ;
  // Serial.println (" bytes") ;
//--- Begin
  const uint32_t errorCode = acan.begin (settings, [] { acan.isr () ; }) ;
  // if (errorCode == 0) {
  //   Serial.print ("Bit Rate prescaler: ") ;
  //   Serial.println (settings.mBitRatePrescaler) ;
  //   Serial.print ("Arbitration Phase segment 1: ") ;
  //   Serial.println (settings.mArbitrationPhaseSegment1) ;
  //   Serial.print ("Arbitration Phase segment 2: ") ;
  //   Serial.println (settings.mArbitrationPhaseSegment2) ;
  //   Serial.print ("Arbitration SJW:") ;
  //   Serial.println (settings.mArbitrationSJW) ;
  //   Serial.print ("Actual Arbitration Bit Rate: ") ;
  //   Serial.print (settings.actualArbitrationBitRate ()) ;
  //   Serial.println (" bit/s") ;
  //   Serial.print ("Exact Arbitration Bit Rate ? ") ;
  //   Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
  //   Serial.print ("Arbitration Sample point: ") ;
  //   Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
  //   Serial.println ("%") ;
  // }else{
  //   Serial.print ("Configuration error 0x") ;
  //   Serial.println (errorCode, HEX) ;
  // }
}


void loop(){

    handleAdcPedals();
    handleKeyAction();
    handleDoor();
    // Handle speed
    handleSpeed();
    // CAN TX TASKS
    processTxTasks(acan);
    // CAN RX HANDLER 
    handleCanRx();
    // CAN DEBUG PRINT
    candebug();
    // UART1 DEBUG
    handleUart1Debug();
    //LIN 
      unsigned long now = millis();
  if (now - gLastReq >= 100) {
    gLastReq = now;
    sendRequest(LIN_TARGET_ID);
  }

  // 2️⃣ Nhận LIN liên tục (non-blocking)
  if (tryReceiveLin(gLinMsg)) {
    if (gLinMsg.frame.id == LIN_TARGET_ID) {
      handleTurnStateMachine(gLinMsg);
    }
  }


        // uint8_t count = 0;
        // bool syncDetected = false;

        // while ((count < LIN_FRAME_SIZE + 1) && Serial2.available()) {
        //     uint8_t linByte = Serial2.read();
        //     if (linByte == SYN_FIELD)
        //         syncDetected = true;

        //     if (syncDetected) {
        //         delay(1);
        //         if (count == 1) {
        //             gLinMess.frame.id = linByte;
        //         } else if (count == 10) {
        //             gLinMess.frame.checkSum = linByte;
        //         } else if (count > 1) {
        //             gLinMess.array[count - 1] = linByte;
        //         }
        //         count++;
        //     }
        // }

        // handleLinMess(gLinMess);
        // gReqID = (gReqID == MFS_L) ? MFS_R : MFS_L;
    //}

    // VR BUTTON
    // int temp = digitalRead(PIN_VR);

    // if (vrButtonPressed) {
    //     unsigned long t = millis();
    //     if (t - vrPressedStart > 500) {
    //         if (vrPressType != 2) {
    //             vrPressType = 2;
    //             printToProtopie("MFS_R_VR", String(vrPressType));
    //         }
    //     }
    // }

    // if (temp != vrState) {
    //     if (temp == LOW) {
    //         vrButtonPressed = true;
    //         vrPressedStart = millis();
    //     } else {
    //         if (millis() - vrPressedStart < 500) {
    //             vrPressType = 1;
    //             printToProtopie("MFS_R_VR", String(vrPressType));
    //         }
    //         vrButtonPressed = false;
    //         vrPressType = 0;
    //         printToProtopie("MFS_R_VR", String(vrPressType));
    //     }
    //     vrState = temp;
    // }
}
