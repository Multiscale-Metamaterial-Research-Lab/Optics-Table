#include <Arduino.h>
#include <SPI.h>

// ============================================================================
// USER SETTINGS (edit only this block day-to-day)
// ============================================================================

// Reference in Hz going into the ADF (after any external conditioning)
static constexpr double REF_IN_HZ = 10e6;      // 10 MHz ref
static constexpr bool   REF_DOUBLER = false;
static constexpr bool   REF_DIV2    = false;
static constexpr uint16_t R_DIV     = 1;       // reference divider (1..)

// Desired RF output
static constexpr double RF_OUT_HZ   = 10.525e9; // 10.525 GHz
static constexpr bool   USE_RFOUTB  = true;     // true: RFOUTB, false: RFOUTA

// Channel spacing / resolution control (affects MOD)
static constexpr double CHANNEL_STEP_HZ = 1000.0; // 1 kHz step (example)

// Output power setting (meaning depends on datasheet encoding)
enum class OutPower : uint8_t {
  PWR_MIN = 0,
  PWR_1   = 1,
  PWR_2   = 2,
  PWR_3   = 3,
  PWR_MAX = 4
};
static constexpr OutPower OUTPUT_POWER = OutPower::PWR_MAX;

// Enable output
static constexpr bool OUTPUT_ENABLE = true;

// Enable/disable CE toggling demo in loop
static constexpr bool TOGGLE_CE_IN_LOOP = false;

// ============================================================================
// PIN DEFINITIONS (ESP32 -> ADF5355 eval board test points)
// ============================================================================
static const int PIN_SCLK = 14;
static const int PIN_MOSI = 15;
static const int PIN_LE   = 27;
static const int PIN_CE   = 25;
static const int PIN_LD   = 26;   // set to -1 if not used

// ============================================================================
// SPI
// ============================================================================
SPIClass hspi(HSPI);
SPISettings pllSPI(1000000, MSBFIRST, SPI_MODE0);

static inline void pulseLE() {
  digitalWrite(PIN_LE, HIGH);
  delayMicroseconds(2);
  digitalWrite(PIN_LE, LOW);
  delayMicroseconds(2);
}

static void writeReg(uint32_t reg) {
  hspi.beginTransaction(pllSPI);
  hspi.transfer((reg >> 24) & 0xFF);
  hspi.transfer((reg >> 16) & 0xFF);
  hspi.transfer((reg >>  8) & 0xFF);
  hspi.transfer((reg >>  0) & 0xFF);
  hspi.endTransaction();
  pulseLE();
}

// ============================================================================
// ADF5355 CONFIG MODEL (you’ll map these to actual register bitfields)
// ============================================================================

// These represent the “logical knobs” we want.
// Later we pack them into actual 32-bit register values.
struct PllParams {
  double pfd_hz = 0;

  // N-divider: N = INT + FRAC/MOD
  uint32_t INT  = 0;
  uint32_t FRAC = 0;
  uint32_t MOD  = 0;

  // Output divider (if needed)
  uint8_t  out_div = 1; // 1,2,4,8,... depending on part

  bool rfouta_en = false;
  bool rfoutb_en = false;

  OutPower pwr = OutPower::PWR_MAX;
};

// Compute gcd for reducing fraction
static uint32_t gcd_u32(uint32_t a, uint32_t b) {
  while (b) { uint32_t t = a % b; a = b; b = t; }
  return a;
}

// ============================================================================
// Frequency planning: compute INT/FRAC/MOD (+ divider choice if needed)
// ============================================================================
static PllParams planFrequency(double rf_out_hz) {
  PllParams p;

  // ---- PFD calculation ----
  // PFD = REF * ( (doubler?2:1) ) / ( R_DIV * (div2?2:1) )
  double ref = REF_IN_HZ * (REF_DOUBLER ? 2.0 : 1.0) / (REF_DIV2 ? 2.0 : 1.0);
  p.pfd_hz = ref / (double)R_DIV;

  // ---- Choose output divider ----
  // Many ADF PLLs have a VCO range and you choose an output divider
  // so VCO = RF_OUT * out_div lands inside VCO limits.
  //
  // TODO: set these to the ADF5355 VCO limits from datasheet.
  const double VCO_MIN = 6.0e9;  // placeholder
  const double VCO_MAX = 13.0e9; // placeholder

  uint8_t div = 1;
  while ((rf_out_hz * div) < VCO_MIN && div < 64) div *= 2;
  double vco = rf_out_hz * div;
  if (vco > VCO_MAX) {
    Serial.println("ERROR: Cannot place VCO in range with available dividers.");
  }
  p.out_div = div;

  // ---- Compute N ----
  // N = VCO / PFD
  double N = vco / p.pfd_hz;

  // Choose MOD based on channel step:
  // step at output roughly = PFD / MOD / out_div
  // so MOD ≈ PFD / (step * out_div)
  uint32_t mod = (uint32_t)llround(p.pfd_hz / (CHANNEL_STEP_HZ * (double)p.out_div));
  if (mod < 2) mod = 2;
  if (mod > 0xFFFFFF) mod = 0xFFFFFF; // placeholder limit; datasheet gives real max

  // Compute INT/FRAC
  uint32_t INT = (uint32_t)floor(N);
  double frac_f = (N - (double)INT) * (double)mod;
  uint32_t FRAC = (uint32_t)llround(frac_f);

  if (FRAC == mod) { // handle rounding overflow
    FRAC = 0;
    INT += 1;
  }

  // Reduce FRAC/MOD
  uint32_t g = gcd_u32(FRAC, mod);
  if (g > 1) {
    FRAC /= g;
    mod  /= g;
  }

  p.INT = INT;
  p.FRAC = FRAC;
  p.MOD = mod;

  p.rfouta_en = (!USE_RFOUTB) && OUTPUT_ENABLE;
  p.rfoutb_en = ( USE_RFOUTB) && OUTPUT_ENABLE;
  p.pwr = OUTPUT_POWER;

  return p;
}

// ============================================================================
// Register packing (THIS is where datasheet mapping goes)
// ============================================================================
//
// You will replace the placeholders with the real bit positions for ADF5355.
// The idea: keep the “base” registers constant, then patch INT/FRAC/MOD,
// output divider, output enable, power, and update.
//


// Write address (register number) into low bits (typical ADF style).
static inline uint32_t withAddr(uint32_t word, uint8_t r) {
  // Many ADF parts use low 4 bits as address. Confirm for ADF5355.
  word &= ~0xFu;
  word |= (r & 0x0F);
  return word;
}

// Base register image (fill from a known-good config/export)
static uint32_t regImage[13] = {
  0x0001040C, // R12 placeholder
  0x0061300B, // R11 placeholder
  0x00C0000A, // R10 placeholder
  0x00000009, // R9  placeholder
  0x102D0428, // R8  placeholder (note your earlier concern about low bits)
  0x12000007, // R7  placeholder
  0x35000006, // R6  placeholder
  0x00800005, // R5  placeholder
  0x00000004, // R4  placeholder
  0x00000003, // R3  placeholder
  0x00001002, // R2  placeholder
  0x00000A41, // R1  placeholder
  0x00550000  // R0  placeholder
};

// Patch frequency-related fields into regImage
static void applyFrequencyToRegs(const PllParams& p) {
  // TODO: Replace these with correct mapping from ADF5355 datasheet:
  // - INT/FRAC/MOD locations and which registers they live in
  // - output divider field and its encoding
  //
  // Example pseudo-mapping (NOT REAL):
  // regImage[R0] = set INT and FRAC
  // regImage[R1] = set MOD
  // regImage[R4] = set output divider

  // Placeholder “do nothing but keep address sane” so you don’t brick writes:
  regImage[12] = withAddr(regImage[12], 12);
  regImage[11] = withAddr(regImage[11], 11);
  regImage[10] = withAddr(regImage[10], 10);
  regImage[9]  = withAddr(regImage[9],  9);
  regImage[8]  = withAddr(regImage[8],  8);
  regImage[7]  = withAddr(regImage[7],  7);
  regImage[6]  = withAddr(regImage[6],  6);
  regImage[5]  = withAddr(regImage[5],  5);
  regImage[4]  = withAddr(regImage[4],  4);
  regImage[3]  = withAddr(regImage[3],  3);
  regImage[2]  = withAddr(regImage[2],  2);
  regImage[1]  = withAddr(regImage[1],  1);
  regImage[0]  = withAddr(regImage[0],  0);

  // When you implement the real mapping, print these so you can verify:
  Serial.printf("PFD=%.3f MHz | VCO=%.3f GHz | div=%u | INT=%lu FRAC=%lu MOD=%lu\n",
                p.pfd_hz / 1e6,
                (RF_OUT_HZ * p.out_div) / 1e9,
                p.out_div,
                (unsigned long)p.INT,
                (unsigned long)p.FRAC,
                (unsigned long)p.MOD);
}

// Patch output enable + power fields into regImage
static void applyOutputToRegs(const PllParams& p) {
  // TODO: Replace with correct mapping from datasheet:
  // - RFOUTA enable bit
  // - RFOUTB enable bit
  // - power bits for each output
  //
  // For now, we just keep addresses correct.
}

// Some ADF parts require a final “update” write sequence (often R0 last).
static void writeAllRegs() {
  for (int i = 12; i >= 0; --i) {
    writeReg(regImage[i]);
    delay(2);
  }
}

// High-level: set new frequency/power based on USER SETTINGS
static void configureFromUserSettings() {
  PllParams p = planFrequency(RF_OUT_HZ);
  applyFrequencyToRegs(p);
  applyOutputToRegs(p);

  Serial.println("Writing register image (R12..R0)...");
  writeAllRegs();
  Serial.println("Done.");
}

// ============================================================================
// Arduino setup/loop
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_LE, OUTPUT);
  pinMode(PIN_CE, OUTPUT);
  digitalWrite(PIN_LE, LOW);
  digitalWrite(PIN_CE, LOW);

  if (PIN_LD >= 0) pinMode(PIN_LD, INPUT);

  hspi.begin(PIN_SCLK, -1, PIN_MOSI, -1);

  Serial.println("ADF5355 configurable synth (top-of-file settings)");
  configureFromUserSettings();

  // Enable chip
  digitalWrite(PIN_CE, HIGH);

  if (PIN_LD >= 0) {
    delay(200);
    Serial.printf("Lock detect: %d\n", digitalRead(PIN_LD));
  }
}

void loop() {
  if (!TOGGLE_CE_IN_LOOP) {
    delay(1000);
    return;
  }

  Serial.println("CE HIGH");
  digitalWrite(PIN_CE, HIGH);
  delay(2000);

  Serial.println("CE LOW");
  digitalWrite(PIN_CE, LOW);
  delay(2000);
}
