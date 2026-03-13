#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// =====================================================
// RP2040 Pico -> ADF5355 pins
// =====================================================
static const int PIN_SCLK = 18;
static const int PIN_MOSI = 19;
static const int PIN_LE   = 20;
static const int PIN_CE   = 17;
static const int PIN_MUX  = 22;

static const int PIN_MISO_UNUSED = 16;
static const int PIN_CS_UNUSED   = 21;

SPIClassRP2040 pllSPI(spi0, PIN_MISO_UNUSED, PIN_CS_UNUSED, PIN_SCLK, PIN_MOSI);

// =====================================================
// User settings
// =====================================================
static const double REF_HZ = 122880000.0;     // EV-ADF5355SD1Z onboard TCXO
static const int RF_PWR = 2;                  // 0=-4, 1=-1, 2=+2, 3=+5 dBm

static const double MIN_RFOUT_HZ = 54000000.0;
static const double MAX_RFOUT_HZ = 6800000000.0;

uint32_t Reg[13];
String inputLine = "";

// =====================================================
// Low-level write
// =====================================================
void writeRegister32(uint32_t value)
{
  pllSPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  digitalWrite(PIN_LE, LOW);
  pllSPI.transfer((value >> 24) & 0xFF);
  pllSPI.transfer((value >> 16) & 0xFF);
  pllSPI.transfer((value >> 8)  & 0xFF);
  pllSPI.transfer((value >> 0)  & 0xFF);
  delayMicroseconds(2);
  digitalWrite(PIN_LE, HIGH);
  delayMicroseconds(2);

  pllSPI.endTransaction();

  delayMicroseconds(10);
  digitalWrite(PIN_LE, LOW);
}

void programADF5355()
{
  for (int i = 12; i >= 0; i--)
  {
    writeRegister32(Reg[i]);
    delayMicroseconds(20);
  }
}

void dumpRegisters()
{
  Serial.println("Registers:");
  for (int i = 12; i >= 0; i--)
  {
    Serial.print("R");
    Serial.print(i);
    Serial.print(" = 0x");
    if (Reg[i] < 0x10000000UL) Serial.print("0");
    Serial.println(Reg[i], HEX);
  }
}

// =====================================================
// Build registers from desired RFOUTA frequency
// =====================================================
bool buildADF5355Registers(double rfoutHz)
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
  int CP_ChgPump   = 9;   // kept from your working base; match loop filter in final tuning
  int D1_DoublBuf  = 0;
  int R_Counter    = 1;   // VALID: 1 to 1023
  int RD1_Rdiv2    = 0;   // no divide-by-2
  int RD2refdoubl  = 0;   // no doubler
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

  // D13=1 means VCO/fundamental feedback.
  // D13=0 means divided feedback.
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

  // Prescaler choice from datasheet INT minimums
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
  Reg[0] = 0
         | ((uint32_t)N_Int      << 4)
         | ((uint32_t)Prescal    << 20)
         | ((uint32_t)1          << 21);   // autocal on

  Reg[1] = 1
         | ((uint32_t)FRAC1      << 4);

  Reg[2] = 2
         | ((uint32_t)MOD2       << 4)
         | ((uint32_t)FRAC2      << 18);

  Reg[3] = 0x00000003UL;

  Reg[4] = 4
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

  Reg[5] = 0x00800025UL;

  Reg[6] = 0
         | ((uint32_t)0          << 31)    // reserved
         | ((uint32_t)0          << 30)    // gated bleed off
         | ((uint32_t)0          << 29)    // negative bleed off
         | ((uint32_t)1          << 28)    // reserved must be 1
         | ((uint32_t)0b010      << 25)    // reserved bits DB27:DB25 must be 010
         | ((uint32_t)D_FeedBack << 24)
         | ((uint32_t)D_RfDivSel << 21)
         | ((uint32_t)CPBleed    << 13)
         | ((uint32_t)D_MTLD     << 11)
         | ((uint32_t)D_RFoutB   << 10)
         | ((uint32_t)D_RF_ena   << 6)
         | ((uint32_t)D_out_PWR  << 4)
         | 6;

  Reg[7]  = 0x120000E7UL;
  Reg[8]  = 0x102D0428UL;
  Reg[9]  = 0x2A29FCC9UL;
  Reg[10] = 0x00C0043AUL;
  Reg[11] = 0x0061300BUL;
  Reg[12] = 0x0001041CUL;

  Serial.println();
  Serial.print("Requested RFOUTA: ");
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

bool handleFrequencyCommand(const String& line)
{
  String s = line;
  s.trim();
  if (s.length() == 0) return false;

  double freqMHz = s.toFloat();
  if (freqMHz <= 0.0)
  {
    Serial.println("Invalid input. Type frequency in MHz, e.g. 3000 or 3456.1");
    return false;
  }

  double freqHz = freqMHz * 1e6;

  if (!buildADF5355Registers(freqHz))
  {
    Serial.println("Frequency out of range or invalid PLL math.");
    return false;
  }

  dumpRegisters();
  programADF5355();

  delay(50);
  Serial.print("MUX/LD = ");
  Serial.println(digitalRead(PIN_MUX));
  Serial.println("Programmed.");
  Serial.println();

  return true;
}

void setup()
{
  Serial.begin(115200);

  pinMode(PIN_LE, OUTPUT);
  pinMode(PIN_CE, OUTPUT);
  pinMode(PIN_MUX, INPUT);

  digitalWrite(PIN_LE, LOW);
  digitalWrite(PIN_CE, HIGH);

  pllSPI.begin();

  delay(100);

  Serial.println();
  Serial.println("ADF5355 Serial Frequency Control");
  Serial.println("Type a frequency in MHz and press Enter.");
  Serial.println("Examples: 3000   3456.1   5256");
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
