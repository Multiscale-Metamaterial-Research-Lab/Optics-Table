#include <Arduino.h>
#include <SPI.h>

// =====================================================
// Board A (SPI0)
// =====================================================
static const int A_SCLK = 18;
static const int A_MOSI = 19;
static const int A_LE   = 20;
static const int A_CE   = 17;

// =====================================================
// Board B (SPI1)
// =====================================================
static const int B_SCLK = 10;
static const int B_MOSI = 11;
static const int B_LE   = 12;
static const int B_CE   = 13;

// =====================================================
// Unused pins for RP2040 SPI constructor
// =====================================================
static const int A_MISO_UNUSED = 16;
static const int A_CS_UNUSED   = 21;

static const int B_MISO_UNUSED = 14;
static const int B_CS_UNUSED   = 15;

// Arduino-Pico SPI objects
SPIClassRP2040 spiA(spi0, A_MISO_UNUSED, A_CS_UNUSED, A_SCLK, A_MOSI);
SPIClassRP2040 spiB(spi1, B_MISO_UNUSED, B_CS_UNUSED, B_SCLK, B_MOSI);

// =====================================================
// General settings
// =====================================================
#define extPin A0
#define delayVCO 300

String inputString = "";
bool stringComplete = false;
String commandString = "";

int msDelay = 100;
int hopCycles = 1;
unsigned int extIntMin = 100;
int extTime = 5000;

// =====================================================
// Separate register banks for each board
// =====================================================
uint32_t regsA[13]    = {0};
uint32_t regsHopA[13] = {0};

uint32_t regsB[13]    = {0};
uint32_t regsHopB[13] = {0};

// Active board selector
enum ActiveBoard
{
  BOARD_A,
  BOARD_B
};

ActiveBoard activeBoard = BOARD_A;

// =====================================================
// Helpers to access current board resources
// =====================================================
uint32_t* getRegs()
{
  return (activeBoard == BOARD_A) ? regsA : regsB;
}

uint32_t* getRegsHop()
{
  return (activeBoard == BOARD_A) ? regsHopA : regsHopB;
}

SPIClassRP2040* getActiveSPI()
{
  return (activeBoard == BOARD_A) ? &spiA : &spiB;
}

int getActiveLE()
{
  return (activeBoard == BOARD_A) ? A_LE : B_LE;
}

int getActiveCE()
{
  return (activeBoard == BOARD_A) ? A_CE : B_CE;
}

// =====================================================
// Forward declarations
// =====================================================
void getCommand();
void writeRegs(String regNum);
void writeRegsHop(String regNum);
void setADF5355();
void updateADF5355();
void hopUpdateADF5355();
void WriteRegister32(const uint32_t value);
int bitExtracted(int number, int k, int p);
void readSerialLine();

// =====================================================
// Setup
// =====================================================
void setup()
{
  Serial.begin(115200);

  pinMode(A_LE, OUTPUT);
  pinMode(A_CE, OUTPUT);
  pinMode(B_LE, OUTPUT);
  pinMode(B_CE, OUTPUT);

  digitalWrite(A_LE, HIGH);
  digitalWrite(B_LE, HIGH);

  digitalWrite(A_CE, HIGH);
  digitalWrite(B_CE, HIGH);

  spiA.begin();
  spiB.begin();

  inputString.reserve(200);
}

// =====================================================
// Main loop
// =====================================================
void loop()
{
  readSerialLine();

  if (stringComplete)
  {
    stringComplete = false;
    getCommand();

    if (commandString.equals("STAR"))
    {
      // startup blank
    }
    else if (commandString.equals("BRDA"))
    {
      activeBoard = BOARD_A;
      Serial.println("ACTIVE:A");
    }
    else if (commandString.equals("BRDB"))
    {
      activeBoard = BOARD_B;
      Serial.println("ACTIVE:B");
    }
    else if (commandString.equals("REGS"))
    {
      String regNum = inputString.substring(5, 7);
      writeRegs(regNum);
    }
    else if (commandString.equals("INIT"))
    {
      setADF5355();
    }
    else if (commandString.equals("FREQ"))
    {
      updateADF5355();
    }
    else if (commandString.equals("HOPS"))
    {
      String regNum = inputString.substring(5, 7);
      writeRegsHop(regNum);
    }
    else if (commandString.equals("HOPD"))
    {
      char charBuf[inputString.length() - 6];
      inputString.substring(5, inputString.length() - 1).toCharArray(charBuf, inputString.length() - 6);
      msDelay = strtoul(charBuf, NULL, 0);
    }
    else if (commandString.equals("HOPC"))
    {
      char charBuf[inputString.length() - 6];
      inputString.substring(5, inputString.length() - 1).toCharArray(charBuf, inputString.length() - 6);
      hopCycles = strtoul(charBuf, NULL, 0);
    }
    else if (commandString.equals("HOPB"))
    {
      for (int i = 0; i <= hopCycles; i++)
      {
        updateADF5355();
        delay(msDelay);
        hopUpdateADF5355();
        delay(msDelay);
      }

      for (int i = 0; i < 20; i++)
      {
        Serial.write("STPH");
      }
      Serial.println();
    }
    else if (commandString.equals("EXTI"))
    {
      char charBuf[inputString.length() - 6];
      inputString.substring(5, inputString.length() - 1).toCharArray(charBuf, inputString.length() - 6);
      extIntMin = (int)strtoul(charBuf, NULL, 0);
    }
    else if (commandString.equals("EXTT"))
    {
      char charBuf[inputString.length() - 6];
      inputString.substring(5, inputString.length() - 1).toCharArray(charBuf, inputString.length() - 6);
      extTime = strtoul(charBuf, NULL, 0);
    }
    else if (commandString.equals("EXTS"))
    {
      uint32_t* regs = getRegs();

      int extIntMax = bitExtracted(regs[0], 16, 5);
      int extIntRange = extIntMax - extIntMin;
      unsigned long startTime = millis();

      while ((millis() - startTime) < (unsigned long)extTime)
      {
        updateADF5355();

        int extVin = analogRead(extPin);
        double frac = (double)extVin / 1023.0;
        double prod = frac * (double)extIntRange;
        int updateInt = (int)prod + (int)extIntMin;

        regs[0] = ((uint32_t)updateInt << 4) | (regs[0] & 0xF);
        delay(10);
      }

      for (int i = 0; i < 20; i++)
      {
        Serial.write("STPH");
      }
      Serial.println();
    }

    inputString = "";
  }
}

// =====================================================
// Read serial manually instead of relying on serialEvent()
// =====================================================
void readSerialLine()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    inputString += inChar;

    if (inChar == '\n')
    {
      stringComplete = true;
    }
  }
}

void getCommand()
{
  if (inputString.length() > 0)
  {
    commandString = inputString.substring(1, 5);
  }
}

// =====================================================
// Register loading
// =====================================================
void writeRegs(String regNum)
{
  uint32_t* regs = getRegs();

  char charBuf[inputString.length() - 8];
  inputString.substring(7, inputString.length() - 1).toCharArray(charBuf, inputString.length() - 8);
  uint32_t val = strtoul(charBuf, NULL, 0);

  if      (regNum == "00") regs[0]  = val;
  else if (regNum == "01") regs[1]  = val;
  else if (regNum == "02") regs[2]  = val;
  else if (regNum == "03") regs[3]  = val;
  else if (regNum == "04") regs[4]  = val;
  else if (regNum == "05") regs[5]  = val;
  else if (regNum == "06") regs[6]  = val;
  else if (regNum == "07") regs[7]  = val;
  else if (regNum == "08") regs[8]  = val;
  else if (regNum == "09") regs[9]  = val;
  else if (regNum == "10") regs[10] = val;
  else if (regNum == "11") regs[11] = val;
  else if (regNum == "12") regs[12] = val;
}

void writeRegsHop(String regNum)
{
  uint32_t* regsHop = getRegsHop();

  char charBuf[inputString.length() - 8];
  inputString.substring(7, inputString.length() - 1).toCharArray(charBuf, inputString.length() - 8);
  uint32_t val = strtoul(charBuf, NULL, 0);

  if      (regNum == "00") regsHop[0]  = val;
  else if (regNum == "01") regsHop[1]  = val;
  else if (regNum == "02") regsHop[2]  = val;
  else if (regNum == "03") regsHop[3]  = val;
  else if (regNum == "04") regsHop[4]  = val;
  else if (regNum == "05") regsHop[5]  = val;
  else if (regNum == "06") regsHop[6]  = val;
  else if (regNum == "07") regsHop[7]  = val;
  else if (regNum == "08") regsHop[8]  = val;
  else if (regNum == "09") regsHop[9]  = val;
  else if (regNum == "10") regsHop[10] = val;
  else if (regNum == "11") regsHop[11] = val;
  else if (regNum == "12") regsHop[12] = val;
}

// =====================================================
// ADF5355 programming functions
// =====================================================
void setADF5355()
{
  uint32_t* regs = getRegs();

  for (int i = 12; i >= 1; i--)
  {
    WriteRegister32(regs[i]);
  }

  delayMicroseconds(delayVCO);
  WriteRegister32(regs[0]);
}

void updateADF5355()
{
  uint32_t* regs = getRegs();

  WriteRegister32(regs[10]);                // R10
  regs[4] |= (1UL << 4);
  WriteRegister32(regs[4]);                 // R4 DB4=1
  WriteRegister32(regs[2]);                 // R2
  WriteRegister32(regs[1]);                 // R1
  regs[0] &= ~(1UL << 21);
  WriteRegister32(regs[0]);                 // R0 DB21=0
  regs[4] &= ~(1UL << 4);
  WriteRegister32(regs[4]);                 // R4 DB4=0
  delayMicroseconds(delayVCO);
  regs[0] |= (1UL << 21);
  WriteRegister32(regs[0]);                 // R0 DB21=1
}

void hopUpdateADF5355()
{
  uint32_t* regsHop = getRegsHop();

  WriteRegister32(regsHop[10]);             // R10
  regsHop[4] |= (1UL << 4);
  WriteRegister32(regsHop[4]);              // R4 DB4=1
  WriteRegister32(regsHop[2]);              // R2
  WriteRegister32(regsHop[1]);              // R1
  regsHop[0] &= ~(1UL << 21);
  WriteRegister32(regsHop[0]);              // R0 DB21=0
  regsHop[4] &= ~(1UL << 4);
  WriteRegister32(regsHop[4]);              // R4 DB4=0
  delayMicroseconds(delayVCO);
  regsHop[0] |= (1UL << 21);
  WriteRegister32(regsHop[0]);              // R0 DB21=1
}

// =====================================================
// Bit helper
// =====================================================
int bitExtracted(int number, int k, int p)
{
  return (((1 << k) - 1) & (number >> (p - 1)));
}

// =====================================================
// Low level 32-bit write to currently selected board
// =====================================================
void WriteRegister32(const uint32_t value)
{
  SPIClassRP2040* spi = getActiveSPI();
  int lePin = getActiveLE();

  spi->beginTransaction(SPISettings(600000, MSBFIRST, SPI_MODE0));

  digitalWrite(lePin, LOW);
  for (int i = 3; i >= 0; i--)
  {
    spi->transfer((value >> (8 * i)) & 0xFF);
  }
  digitalWrite(lePin, HIGH);

  spi->endTransaction();
}
