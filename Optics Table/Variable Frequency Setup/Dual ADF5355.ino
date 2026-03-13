#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// =====================================================
// Board A (SPI0)
// =====================================================
static const int A_SCLK = 18;
static const int A_MOSI = 19;
static const int A_LE   = 20;
static const int A_CE   = 17;
static const int A_MUX  = 22;

static const int A_MISO_UNUSED = 16;
static const int A_CS_UNUSED   = 21;

// =====================================================
// Board B (SPI1)
// =====================================================
static const int B_SCLK = 10;
static const int B_MOSI = 11;
static const int B_LE   = 12;
static const int B_CE   = 13;
static const int B_MUX  = 26;

static const int B_MISO_UNUSED = 14;
static const int B_CS_UNUSED   = 15;

// RP2040 SPI objects
SPIClassRP2040 spiA(spi0, A_MISO_UNUSED, A_CS_UNUSED, A_SCLK, A_MOSI);
SPIClassRP2040 spiB(spi1, B_MISO_UNUSED, B_CS_UNUSED, B_SCLK, B_MOSI);

// =====================================================
// User settings
// =====================================================
static const double REF_HZ = 122880000.0;     // EV-ADF5355SD1Z onboard TCXO
static const int RF_PWR = 1;                  // 0=-4, 1=-1, 2=+2, 3=+5 dBm

static const double MIN_RFOUT_HZ = 54000000.0;
static const double MAX_RFOUT_HZ = 6800000000.0;

// =====================================================
// Board container
// =====================================================
struct ADFBoard
{
  const char* name;
  SPIClassRP2040* spi;
  int pinLE;
  int pinCE;
  int pinMUX;
  uint32_t reg[13];
};

ADFBoard boardA = { "A", &spiA, A_LE, A_CE, A_MUX, {0} };
ADFBoard boardB = { "B", &spiB, B_LE, B_CE, B_MUX, {0} };

String inputLine = "";

// =====================================================
// Low-level write
// =====================================================
void writeRegister32(ADFBoard& b, uint32_t value)
{
  b.spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  digitalWrite(b.pinLE, LOW);
  b.spi->transfer((value >> 24) & 0xFF);
  b.spi->transfer((value >> 16) & 0xFF);
  b.spi->transfer((value >> 8)  & 0xFF);
  b.spi->transfer((value >> 0)  & 0xFF);
  delayMicroseconds(2);
  digitalWrite(b.pinLE, HIGH);
  delayMicroseconds(2);

  b.spi->endTransaction();

  delayMicroseconds(10);
  digitalWrite(b.pinLE, LOW);
}

void programADF5355(ADFBoard& b)
{
  for (int i = 12; i >= 0; i--)
  {
    writeRegister32(b, b.reg[i]);
    delayMicroseconds(20);
  }
}

void dumpRegisters(const ADFBoard& b)
{
  Serial.print("Board ");
  Serial.print(b.name);
  Serial.println(" Registers:");

  for (int i = 12; i >= 0; i--)
  {
    Serial.print("R");
    Serial.print(i);
    Serial.print(" = 0x");
    if (b.reg[i] < 0x10000000UL) Serial.print("0");
    Serial.println(b.reg[i], HEX);
  }
}

// =====================================================
// Build registers from desired RFOUTA frequency
// =====================================================
bool buildADF5355Registers(ADFBoard& b, double rfoutHz)
{
  if (rfoutHz < MIN_RFOUT_HZ || rfoutHz > MAX_RFOUT_HZ)
  {
    return false;
  }

  // -----------------------------
  // Register 4 fields
  // -----------------------------
  int U1_CountRes  = 0;
  int U2_Cp3state  = 0;
  int U3_PwrDown   = 0;
  int U4_PDpola    = 1;
  int U5_MuxLog    = 1;   // 3.3 V logic on MUXOUT
  int U6_RefMode   = 0;   // recommended single-ended mode for refs <= 250 MHz
  int CP_ChgPump   = 9;
  int D1_DoublBuf  = 0;
  int R_Counter    = 1;
  int RD1_Rdiv2    = 0;
  int RD2refdoubl  = 0;
  int M_Muxout     = 6;   // digital lock detect

  // -----------------------------
  // Register 6 fields
  // -----------------------------
  int D_out_PWR    = RF_PWR;
  int D_RF_ena     = 1;   // RFOUTA enabled
  int D_RFoutB     = 1;   // RFOUTB disabled
  int D_MTLD       = 0;
  int CPBleed      = 126;
  int D_RfDivSel   = 0;
  int D_FeedBack   = 1;

  // -----------------------------
  // Choose divider so VCO is 3.4 to 6.8 GHz
  // -----------------------------
  int outdiv = 1;
  double vcoHz = rfoutHz;

  while (vcoHz < 3400000000.0 && outdiv < 64)
  {
    outdiv *= 2;
    vcoHz = rfoutHz * outdiv;
  }

  if (vcoHz < 3400000000.0 || vcoHz > 6800000000.0)
  {
    return false;
  }

  switch (outdiv)
  {
    case 1:  D_RfDivSel = 0; break;
    case 2:  D_RfDivSel = 1; break;
    case 4:  D_RfDivSel = 2; break;
    case 8:  D_RfDivSel = 3; break;
    case 16: D_RfDivSel = 4; break;
    case 32: D_RfDivSel = 5; break;
    case 64: D_RfDivSel = 6; break;
    default: return false;
  }

  // Fundamental feedback only for outdiv = 1
  D_FeedBack = (outdiv == 1) ? 1 : 0;

  // -----------------------------
  // PFD and N math
  // fPFD = REFIN * (1 + D) / (R * (1 + T))
  // -----------------------------
  double pfdHz = REF_HZ * (1.0 + RD2refdoubl) /
                 ((double)R_Counter * (1.0 + RD1_Rdiv2));

  if (pfdHz <= 0.0 || pfdHz > 125000000.0)
  {
    return false;
  }

  double N = vcoHz / pfdHz;
  int N_Int = (int)floor(N);
  double frac = N - (double)N_Int;

  int Prescal = 0; // 0 = 4/5, 1 = 8/9
  if (N_Int >= 75)
  {
    Prescal = 1;
  }
  else if (N_Int >= 23)
  {
    Prescal = 0;
  }
  else
  {
    return false;
  }

  const int MOD2 = 16383;
  const double MOD1 = 16777216.0; // 2^24

  double totalFrac = frac * MOD1;
  int FRAC1 = (int)floor(totalFrac);
  double fracRemainder = totalFrac - (double)FRAC1;
  int FRAC2 = (int)llround(fracRemainder * (double)MOD2);

  if (FRAC2 >= MOD2)
  {
    FRAC2 = 0;
    FRAC1 += 1;
  }

  if (FRAC1 >= (int)MOD1)
  {
    FRAC1 = 0;
    N_Int += 1;
  }

  // -----------------------------
  // Build registers
  // -----------------------------
  b.reg[0] = 0
           | ((uint32_t)N_Int      << 4)
           | ((uint32_t)Prescal    << 20)
           | ((uint32_t)1          << 21);   // autocal on

  b.reg[1] = 1
           | ((uint32_t)FRAC1      << 4);

  b.reg[2] = 2
           | ((uint32_t)MOD2       << 4)
           | ((uint32_t)FRAC2      << 18);

  b.reg[3] = 0x00000003UL;

  b.reg[4] = 4
           | ((uint32_t)U1_CountRes << 4)
           | ((uint32_t)U2_Cp3state << 5)
           | ((uint32_t)U3_PwrDown  << 6)
           | ((uint32_t)U4_PDpola   << 7)
           | ((uint32_t)U5_MuxLog   << 8)
           | ((uint32_t)U6_RefMode  << 9)
           | ((uint32_t)CP_ChgPump  << 10)
           | ((uint32_t)D1_DoublBuf << 14)
           | ((uint32_t)R_Counter   << 15)
           | ((uint32_t)RD1_Rdiv2   << 25)
           | ((uint32_t)RD2refdoubl << 26)
           | ((uint32_t)M_Muxout    << 27);

  b.reg[5] = 0x00800025UL;

  b.reg[6] = 0
           | ((uint32_t)0          << 31)
           | ((uint32_t)0          << 30)
           | ((uint32_t)0          << 29)
           | ((uint32_t)1          << 28)
           | ((uint32_t)0b010      << 25)
           | ((uint32_t)D_FeedBack << 24)
           | ((uint32_t)D_RfDivSel << 21)
           | ((uint32_t)CPBleed    << 13)
           | ((uint32_t)D_MTLD     << 11)
           | ((uint32_t)D_RFoutB   << 10)
           | ((uint32_t)D_RF_ena   << 6)
           | ((uint32_t)D_out_PWR  << 4)
           | 6;

  b.reg[7]  = 0x120000E7UL;
  b.reg[8]  = 0x102D0428UL;
  b.reg[9]  = 0x2A29FCC9UL;
  b.reg[10] = 0x00C0043AUL;
  b.reg[11] = 0x0061300BUL;
  b.reg[12] = 0x0001041CUL;

  Serial.println();
  Serial.print("Board ");
  Serial.print(b.name);
  Serial.print(" requested RFOUTA: ");
  Serial.print(rfoutHz / 1e6, 6);
  Serial.println(" MHz");

  Serial.print("Reference: ");
  Serial.print(REF_HZ / 1e6, 6);
  Serial.println(" MHz");

  Serial.print("PFD: ");
  Serial.print(pfdHz / 1e6, 6);
  Serial.println(" MHz");

  Serial.print("VCO: ");
  Serial.print(vcoHz / 1e6, 6);
  Serial.println(" MHz");

  Serial.print("Output Divider: /");
  Serial.println(outdiv);

  Serial.print("INT: ");
  Serial.println(N_Int);

  Serial.print("FRAC1: ");
  Serial.println(FRAC1);

  Serial.print("FRAC2: ");
  Serial.println(FRAC2);

  Serial.print("Prescaler: ");
  Serial.println(Prescal ? "8/9" : "4/5");

  Serial.print("Feedback: ");
  Serial.println(D_FeedBack ? "Fundamental" : "Divided");

  return true;
}

bool programBoard(ADFBoard& b, double freqMHz)
{
  if (freqMHz <= 0.0)
  {
    Serial.print("Board ");
    Serial.print(b.name);
    Serial.println(": invalid frequency.");
    return false;
  }

  double freqHz = freqMHz * 1e6;

  if (!buildADF5355Registers(b, freqHz))
  {
    Serial.print("Board ");
    Serial.print(b.name);
    Serial.println(": frequency out of range or invalid PLL math.");
    return false;
  }

  dumpRegisters(b);
  programADF5355(b);

  delay(50);
  Serial.print("Board ");
  Serial.print(b.name);
  Serial.print(" MUX/LD = ");
  Serial.println(digitalRead(b.pinMUX));
  Serial.print("Board ");
  Serial.print(b.name);
  Serial.println(" programmed.");
  Serial.println();

  return true;
}

// =====================================================
// Command handling
// =====================================================
// Supported commands:
//   3000            -> program BOTH A and B to 3000 MHz
//   A 3000          -> program only A
//   B 5256          -> program only B
//   AB 3000 5256    -> program A=3000 MHz, B=5256 MHz
// =====================================================
bool handleFrequencyCommand(const String& line)
{
  String s = line;
  s.trim();
  if (s.length() == 0) return false;

  char buffer[128];
  s.toCharArray(buffer, sizeof(buffer));

  char cmd[16] = {0};
  double f1 = 0.0;
  double f2 = 0.0;

  int count = sscanf(buffer, "%15s %lf %lf", cmd, &f1, &f2);

  // Case 1: user typed just a number like "3000"
  double onlyFreq = s.toFloat();
  if (onlyFreq > 0.0)
  {
    bool okA = programBoard(boardA, onlyFreq);
    bool okB = programBoard(boardB, onlyFreq);
    return okA && okB;
  }

  // Normalize command to uppercase
  for (int i = 0; cmd[i]; i++)
  {
    cmd[i] = toupper((unsigned char)cmd[i]);
  }

  if (strcmp(cmd, "A") == 0)
  {
    if (count < 2)
    {
      Serial.println("Usage: A 3000");
      return false;
    }
    return programBoard(boardA, f1);
  }

  if (strcmp(cmd, "B") == 0)
  {
    if (count < 2)
    {
      Serial.println("Usage: B 3000");
      return false;
    }
    return programBoard(boardB, f1);
  }

  if (strcmp(cmd, "AB") == 0 || strcmp(cmd, "BOTH") == 0)
  {
    if (count == 2)
    {
      bool okA = programBoard(boardA, f1);
      bool okB = programBoard(boardB, f1);
      return okA && okB;
    }
    else if (count >= 3)
    {
      bool okA = programBoard(boardA, f1);
      bool okB = programBoard(boardB, f2);
      return okA && okB;
    }
    else
    {
      Serial.println("Usage: AB 3000 5256");
      return false;
    }
  }

  Serial.println("Invalid input.");
  Serial.println("Examples:");
  Serial.println("  3000");
  Serial.println("  A 3000");
  Serial.println("  B 5256");
  Serial.println("  AB 3000 5256");
  return false;
}

void setup()
{
  Serial.begin(115200);

  pinMode(boardA.pinLE, OUTPUT);
  pinMode(boardA.pinCE, OUTPUT);
  pinMode(boardA.pinMUX, INPUT);

  pinMode(boardB.pinLE, OUTPUT);
  pinMode(boardB.pinCE, OUTPUT);
  pinMode(boardB.pinMUX, INPUT);

  digitalWrite(boardA.pinLE, LOW);
  digitalWrite(boardB.pinLE, LOW);

  digitalWrite(boardA.pinCE, HIGH);
  digitalWrite(boardB.pinCE, HIGH);

  boardA.spi->begin();
  boardB.spi->begin();

  delay(100);

  Serial.println();
  Serial.println("ADF5355 Dual-Board Serial Frequency Control");
  Serial.println("Commands:");
  Serial.println("  3000          -> program both boards to 3000 MHz");
  Serial.println("  A 3000        -> program board A only");
  Serial.println("  B 5256        -> program board B only");
  Serial.println("  AB 3000 5256  -> program A=3000 MHz, B=5256 MHz");
  Serial.println("Serial Monitor: 115200 baud, newline");
  Serial.println();

  handleFrequencyCommand("3000");
}

void loop()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (inputLine.length() > 0)
      {
        handleFrequencyCommand(inputLine);
        inputLine = "";
      }
    }
    else
    {
      inputLine += c;
    }
  }
}
