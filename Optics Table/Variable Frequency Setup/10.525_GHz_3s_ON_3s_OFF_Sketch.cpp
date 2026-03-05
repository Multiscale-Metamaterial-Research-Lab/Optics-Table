#include <Arduino.h>
#include <SPI.h>

// =======================
// Board A (SPI0 pins)
// =======================
static const int A_SCLK = 18;
static const int A_MOSI = 19;
static const int A_LE   = 20;
static const int A_CE   = 17;

// =======================
// Board B (SPI1 pins)
// =======================
static const int B_SCLK = 10;
static const int B_MOSI = 11;
static const int B_LE   = 12;
static const int B_CE   = 13;

// Unused pins for MISO/CS (not wired)
// Pick pins you are NOT using elsewhere.
static const int A_MISO_UNUSED = 16;
static const int A_CS_UNUSED   = 21;

static const int B_MISO_UNUSED = 14;
static const int B_CS_UNUSED   = 15;

// NOTE: Constructor order is (spi, rx(MISO), cs, sck, tx(MOSI))
SPIClassRP2040 spiA(spi0, A_MISO_UNUSED, A_CS_UNUSED, A_SCLK, A_MOSI);
SPIClassRP2040 spiB(spi1, B_MISO_UNUSED, B_CS_UNUSED, B_SCLK, B_MOSI);

SPISettings pllSPI(1000000, MSBFIRST, SPI_MODE0);

static inline void pulseLE(int pinLE) {
  digitalWrite(pinLE, HIGH);
  delayMicroseconds(2);
  digitalWrite(pinLE, LOW);
  delayMicroseconds(2);
}

static void writeReg(SPIClassRP2040 &spi, int pinLE, uint32_t reg) {
  spi.beginTransaction(pllSPI);
  spi.transfer((reg >> 24) & 0xFF);
  spi.transfer((reg >> 16) & 0xFF);
  spi.transfer((reg >>  8) & 0xFF);
  spi.transfer((reg >>  0) & 0xFF);
  spi.endTransaction();
  pulseLE(pinLE);
}

static void programPLL(SPIClassRP2040 &spi, int pinLE, const char *name) {
  Serial.printf("\nProgramming %s for 10.525 GHz RFOUTB...\n", name);

  uint32_t regs[13] = {
    0x0001040C, // R12
    0x0061300B, // R11
    0x00C0000A, // R10
    0x00000009, // R9
    0x102D0428, // R8
    0x12000007, // R7
    0x35000006, // R6
    0x00800005, // R5
    0x00000004, // R4
    0x00000003, // R3
    0x00001002, // R2
    0x00000A41, // R1
    0x00550000  // R0 LAST
  };

  for (int i = 0; i < 13; i++) {
    int rnum = 12 - i;
    Serial.printf("%s: Writing R%d = 0x%08lX\n", name, rnum, (unsigned long)regs[i]);
    writeReg(spi, pinLE, regs[i]);
    delay(2);
  }

  Serial.printf("%s: Done.\n", name);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Dual ADF5355 test: Board A on SPI0, Board B on SPI1");

  pinMode(A_LE, OUTPUT); pinMode(A_CE, OUTPUT);
  pinMode(B_LE, OUTPUT); pinMode(B_CE, OUTPUT);
  digitalWrite(A_LE, LOW); digitalWrite(A_CE, LOW);
  digitalWrite(B_LE, LOW); digitalWrite(B_CE, LOW);

  // Start both SPI peripherals
  spiA.begin();
  spiB.begin();

  // Program both PLLs (same register set to both)
  programPLL(spiA, A_LE, "ADF-A");
  programPLL(spiB, B_LE, "ADF-B");
}

void loop() {
  Serial.println("BOTH ON (CE HIGH)");
  digitalWrite(A_CE, HIGH);
  digitalWrite(B_CE, HIGH);
  delay(3000);

  Serial.println("BOTH OFF (CE LOW)");
  digitalWrite(A_CE, LOW);
  digitalWrite(B_CE, LOW);
  delay(3000);
}
