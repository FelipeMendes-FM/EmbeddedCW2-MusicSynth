#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>

//Constants
const uint32_t interval = 100; //Display update interval
const uint32_t sampleRate = 22000; // Sample rate in Hz
const double baseFrequency = 440.0; // Frequency of A4 in Hz
const double twelfthRootOfTwo = pow(2.0, 1.0 / 12.0);

const double calculateFrequency(int n) {
    return baseFrequency * pow(twelfthRootOfTwo, n - 9);
}

// Calculate step size for each note
constexpr uint32_t calculateStepSize(double frequency) {
    return static_cast<uint32_t>((static_cast<uint64_t>(1) << 32) * frequency / sampleRate);
}

const uint32_t stepSizes[] = {
    calculateStepSize(calculateFrequency(0)),
    calculateStepSize(calculateFrequency(1)),
    calculateStepSize(calculateFrequency(2)),
    calculateStepSize(calculateFrequency(3)),
    calculateStepSize(calculateFrequency(4)),
    calculateStepSize(calculateFrequency(5)),
    calculateStepSize(calculateFrequency(6)),
    calculateStepSize(calculateFrequency(7)),
    calculateStepSize(calculateFrequency(8)),
    calculateStepSize(calculateFrequency(9)), // This is A at 440Hz
    calculateStepSize(calculateFrequency(10)),
    calculateStepSize(calculateFrequency(11)),
};

int convertToIndex(int row, int col) {
  return row * 4 + col;
}

const char* noteNames[] = {
  "C", "C#", "D", "D#",
  "E", "F", "F#", "G",
  "G#", "A", "A#", "B"
};

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

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

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

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;
  std::bitset<32> inputs;
  volatile uint32_t currentStepSize = 0;

  while (millis() < next); // Wait for next interval

  next += interval;

  for (uint8_t row = 0; row < 3; ++row) {
    setRow(row); // Set the current row
    auto colBits = readCols(); // Read the columns for the current row

    for (uint8_t col = 0; col < 4; ++col) {
      // Calculate the bit index for inputs and set it
      // Note: Adjust the formula according to your matrix wiring and logic
      inputs[row * 4 + col] = colBits[col];
      int index = convertToIndex(row, col);
    }
    
  }


      // Update display with the selected note
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setCursor(2,24);
      u8g2.print("Note: ");
      u8g2.print(index); // Convert index to string before printing
      u8g2.sendBuffer(); // Send buffer to display




  // You can toggle an LED or do other tasks below...
}
