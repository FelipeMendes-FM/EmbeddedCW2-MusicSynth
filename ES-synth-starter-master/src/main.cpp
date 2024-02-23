#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

const uint32_t stepSizes[] = {
  51076057, 
  54113197, 
  57330935, 
  60740010, 
  64351799, 
  68178356, 
  72232452, 
  76527617, 
  81078186, 
  85899346, 
  91007187, 
  96418756
  };

volatile uint32_t currentStepSize;

const char noteNames[12][3] = {
  "C", "C#", "D", "D#", 
  "E", "F", "F#", "G", 
  "G#", "A", "A#", "B"
  };

struct {
  std::bitset<32> inputs;
  bool pressed = false;
  int lastPressed;
} sysState;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

std::bitset<4> readCols(){
  std::bitset<4> result;

  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;
}

void setRow(uint8_t rowIdx){

  digitalWrite(REN_PIN, LOW); // Disable row select enable to prevent glitches
  digitalWrite(RA0_PIN, rowIdx & 0x01); // Set RA0 based on the least significant bit of rowIdx
  digitalWrite(RA1_PIN, (rowIdx & 0x02) >> 1); // Set RA1 based on the second bit of rowIdx
  digitalWrite(RA2_PIN, (rowIdx & 0x04) >> 2); // Set RA2 based on the third bit of rowIdx
  digitalWrite(REN_PIN, HIGH); // Enable row select

}


void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> inputs;
    static std::bitset<32> prevInputs;
    static int firstKeyPressed, lastKeyPress;

    setRow(0);
    delayMicroseconds(3);
    std::bitset<4> col1 = readCols();
    setRow(1);
    delayMicroseconds(3);
    std::bitset<4> col2 = readCols();
    setRow(2);
    delayMicroseconds(3);
    std::bitset<4> col3 = readCols();

    for (int i = 0; i < 4; i++) {
        inputs[i] = col1[i];
        inputs[i + 4] = col2[i];
        inputs[i + 8] = col3[i];
    }

    if (prevInputs != inputs) {
      std::bitset<32> change = inputs ^ prevInputs;
      
      for (firstKeyPressed = 0; firstKeyPressed < 12; firstKeyPressed++) {
        if (change[0] == 1) {
          break;
        } else {
          change = change >> 1;
        }
      }
      lastKeyPress = firstKeyPressed;
      uint32_t newStepSize = stepSizes[lastKeyPress];
      __atomic_store_n(&currentStepSize, newStepSize, __ATOMIC_RELAXED);
    }

    std::bitset<32> mask = 0b00000000000000000000111111111111;
    std::bitset<12> last12 = std::bitset<12>((inputs & mask).to_ulong());
    if (!last12.all()) { 
      sysState.lastPressed = lastKeyPress;
      sysState.pressed = true;
    } else {
      currentStepSize = 0;
      sysState.pressed = false;
    }

    sysState.inputs = inputs;
    prevInputs = inputs;
  }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    //Update display
    u8g2.clearBuffer();         
    u8g2.setFont(u8g2_font_ncenB08_tr); 
    u8g2.drawStr(2,10,"Hello World!"); 
    u8g2.setCursor(2,20);
    u8g2.print(sysState.inputs.to_ulong(),HEX);
    
    if (sysState.pressed) {
      u8g2.setCursor(2,30);
      u8g2.print(noteNames[sysState.lastPressed]);
    }
    u8g2.sendBuffer();       
    digitalToggle(LED_BUILTIN);
  }
}


void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		    /* Function that implements the task */
  "scanKeys",		      /* Text name for the task */
  64,      		        /* Stack size in words, not bytes */
  NULL,			          /* Parameter passed into the task */
  2,			            /* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */
  
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		  /* Function that implements the task */
  "displayUpdate",		    /* Text name for the task */
  64,      		            /* Stack size in words, not bytes */
  NULL,			              /* Parameter passed into the task */
  1,			                /* Task priority */
  &displayUpdateHandle );	/* Pointer to store the task handle */
  
  vTaskStartScheduler();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}

void loop() {
  // put your main code here, to run repeatedly:
}

