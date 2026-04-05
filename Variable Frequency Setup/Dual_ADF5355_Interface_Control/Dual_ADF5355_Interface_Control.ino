#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

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

// =====================================================
// SPI objects
// =====================================================
SPIClassRP2040 spiA(spi0, A_MISO_UNUSED, A_CS_UNUSED, A_SCLK, A_MOSI);
SPIClassRP2040 spiB(spi1, B_MISO_UNUSED, B_CS_UNUSED, B_SCLK, B_MOSI);

// =====================================================
// User settings
// =====================================================
static const double REF_HZ = 122880000.0;     // onboard TCXO
static const int RF_PWR = 3;                  // 0=-4, 1=-1, 2=+2, 3=+5 dBm

static const double MIN_RFOUTA_HZ = 54000000.0;
static const double MAX_RFOUTA_HZ = 6800000000.0;

// RFOUTB user-facing output range
static const double MIN_RFOUTB_HZ = 6800000000.0;
static const double MAX_RFOUTB_HZ = 13600000000.0;

// Internal VCO range
static const double MIN_VCO_HZ = 3400000000.0;
static const double MAX_VCO_HZ = 6800000000.0;

// =====================================================
// Output select
// =====================================================
enum RFOutputSelect
{
  OUTPUT_RFA,
  OUTPUT_RFB
};

// =====================================================
// Board structure
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
  b.spi->transfer((value >>  8) & 0xFF);
  b.spi->transfer((value >>  0) & 0xFF);
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
// Build registers from requested output frequency
// =====================================================
bool buildADF5355Registers(ADFBoard& b, double requestedHz, RFOutputSelect outputSel)
{
  if (requestedHz <= 0.0)
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
  int U5_MuxLog    = 1;
  int U6_RefMode   = 0;
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
  int D_RF_ena     = 0;   // RFOUTA enable
  int D_RFoutB     = 1;   // 1 = disabled, 0 = enabled in your register logic
  int D_MTLD       = 0;
  int CPBleed      = 126;
  int D_RfDivSel   = 0;
  int D_FeedBack   = 1;

  int outdiv = 1;
  double vcoHz = requestedHz;
  double actualOutputHz = requestedHz;

  // =====================================================
  // RFOUTA mode
  // =====================================================
  if (outputSel == OUTPUT_RFA)
  {
    if (requestedHz < MIN_RFOUTA_HZ || requestedHz > MAX_RFOUTA_HZ)
    {
      Serial.println("Requested RFA frequency is out of range.");
      return false;
    }

    vcoHz = requestedHz;
    outdiv = 1;

    while (vcoHz < MIN_VCO_HZ && outdiv < 64)
    {
      outdiv *= 2;
      vcoHz = requestedHz * outdiv;
    }

    if (vcoHz < MIN_VCO_HZ || vcoHz > MAX_VCO_HZ)
    {
      Serial.println("Could not place VCO in valid range for RFOUTA.");
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

    D_FeedBack = (outdiv == 1) ? 1 : 0;

    // Enable RFOUTA, disable RFOUTB
    D_RF_ena = 1;
    D_RFoutB = 1;

    actualOutputHz = vcoHz / outdiv;
  }
  // =====================================================
  // RFOUTB mode
  // RFOUTB is the doubled VCO output
  // =====================================================
  else
  {
    if (requestedHz < MIN_RFOUTB_HZ || requestedHz > MAX_RFOUTB_HZ)
    {
      Serial.println("Requested RFB frequency is out of range.");
      Serial.println("Use approximately 6800 to 13600 MHz in RFB mode.");
      return false;
    }

    // RFOUTB output = 2 * VCO
    vcoHz = requestedHz / 2.0;

    if (vcoHz < MIN_VCO_HZ || vcoHz > MAX_VCO_HZ)
    {
      Serial.println("Computed VCO is out of valid range for RFOUTB.");
      return false;
    }

    outdiv = 1;
    D_RfDivSel = 0;
    D_FeedBack = 1;

    // Disable RFOUTA, enable RFOUTB
    D_RF_ena = 0;
    D_RFoutB = 0;

    actualOutputHz = vcoHz * 2.0;
  }

  // -----------------------------
  // PFD frequency
  // -----------------------------
  double pfdHz = REF_HZ * (1.0 + RD2refdoubl) /
                 ((double)R_Counter * (1.0 + RD1_Rdiv2));

  if (pfdHz <= 0.0 || pfdHz > 125000000.0)
  {
    Serial.println("Invalid PFD frequency.");
    return false;
  }

  // -----------------------------
  // INT / FRAC math
  // -----------------------------
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
    Serial.println("INT value too low for prescaler.");
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

  // -----------------------------
  // Print summary
  // -----------------------------
  Serial.println();
  Serial.print("Board ");
  Serial.print(b.name);
  Serial.print(" requested output: ");
  Serial.println(outputSel == OUTPUT_RFA ? "RFOUTA" : "RFOUTB");

  Serial.print("Requested frequency: ");
  Serial.print(requestedHz / 1e6, 6);
  Serial.println(" MHz");

  Serial.print("Actual output target: ");
  Serial.print(actualOutputHz / 1e6, 6);
  Serial.println(" MHz");

  Serial.print("Reference: ");
  Serial.print(REF_HZ / 1e6, 6);
  Serial.println(" MHz");

  Serial.print("PFD: ");
  Serial.print(pfdHz / 1e6, 6);
  Serial.println(" MHz");

  Serial.print("Internal VCO: ");
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

  Serial.print("RFOUTA enabled: ");
  Serial.println(D_RF_ena ? "YES" : "NO");

  Serial.print("RFOUTB enabled: ");
  Serial.println(D_RFoutB ? "NO" : "YES");

  return true;
}

// =====================================================
// Program one board
// =====================================================
bool programBoard(ADFBoard& b, double freqMHz, RFOutputSelect outputSel)
{
  if (freqMHz <= 0.0)
  {
    Serial.print("Board ");
    Serial.print(b.name);
    Serial.println(": invalid frequency.");
    return false;
  }

  double freqHz = freqMHz * 1e6;

  if (!buildADF5355Registers(b, freqHz, outputSel))
  {
    Serial.print("Board ");
    Serial.print(b.name);
    Serial.println(": failed to build registers.");
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
// Parse serial command
// Format:
//   A 4000 RFA
//   B 8000 RFB
// =====================================================
bool handleFrequencyCommand(const String& line)
{
  String s = line;
  s.trim();
  if (s.length() == 0) return false;

  char buffer[128];
  s.toCharArray(buffer, sizeof(buffer));

  char boardStr[16] = {0};
  double freqMHz = 0.0;
  char outputStr[16] = {0};

  int count = sscanf(buffer, "%15s %lf %15s", boardStr, &freqMHz, outputStr);

  if (count != 3)
  {
    Serial.println("Invalid command format.");
    Serial.println("Use one of these:");
    Serial.println("  A 4000 RFA");
    Serial.println("  B 8000 RFB");
    return false;
  }

  for (int i = 0; boardStr[i]; i++)
  {
    boardStr[i] = toupper((unsigned char)boardStr[i]);
  }

  for (int i = 0; outputStr[i]; i++)
  {
    outputStr[i] = toupper((unsigned char)outputStr[i]);
  }

  ADFBoard* targetBoard = nullptr;
  RFOutputSelect outputSel;

  if (strcmp(boardStr, "A") == 0)
  {
    targetBoard = &boardA;
  }
  else if (strcmp(boardStr, "B") == 0)
  {
    targetBoard = &boardB;
  }
  else
  {
    Serial.println("First field must be A or B.");
    return false;
  }

  if (strcmp(outputStr, "RFA") == 0)
  {
    outputSel = OUTPUT_RFA;
  }
  else if (strcmp(outputStr, "RFB") == 0)
  {
    outputSel = OUTPUT_RFB;
  }
  else
  {
    Serial.println("Third field must be RFA or RFB.");
    return false;
  }

  return programBoard(*targetBoard, freqMHz, outputSel);
}

// =====================================================
// Setup
// =====================================================
void setup()
{
  Serial.begin(115200);
  delay(200);

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
  Serial.println("ADF5355 Dual Board Control");
  Serial.println("Command format:");
  Serial.println("  A 4000 RFA");
  Serial.println("  B 8000 RFB");
  Serial.println();
  Serial.println("Examples:");
  Serial.println("  A 4000 RFA");
  Serial.println("  A 5256 RFA");
  Serial.println("  B 5600 RFA");
  Serial.println("  B 8000 RFB");
  Serial.println("  A 10000 RFB");
  Serial.println();
  Serial.println("Notes:");
  Serial.println("  RFA mode uses output-divider math.");
  Serial.println("  RFB mode uses doubled-VCO math.");
  Serial.println("  RFA valid range: about 54 to 6800 MHz.");
  Serial.println("  RFB valid range: about 6800 to 13600 MHz.");
  Serial.println();
  Serial.println("Serial Monitor: 115200 baud, newline");
  Serial.println();

  // Optional startup test:
  // handleFrequencyCommand("A 4000 RFA");
}

// =====================================================
// Main loop
// =====================================================
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