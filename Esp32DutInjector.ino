/*
This file allows to read and write following interfaces:
- digital
  - inputs (2)
    - Command on serial
      din nr
      - returns dinValue
  - outputs (2)
    - Command on serial
      dout nr doutvalue
- Analog
  - inputs (2)
    - Command on serial
      adc nr
      - returns adcValue
  - outputs (2)
    - Command on serial
      dac nr dacValue
- PWM
  - input (1)
    - Command on serial
      duty 0
      returns duty_cycle_as_12_Bit_value
      freq 0
      returns frequency_in_Hz
  - output (1)
    - Command on serial
      pwm 0 frequency duty

  - PARAMETER
    - nr        1 bit integer [0,   1]
    - dinValue  1 bit integer [0,   1]
    - doutValue 1 bit integer [0,   1]
    - dacValue  8 bit integer [0, 255]
    - adcValue  12 bit integer [0,4095]

PORT to arduino-esp32 Core 3.3.x (IDF 5.x)
- ledcSetup/ledcAttachPin  ->  ledcAttach / ledcChangeFrequency
- ledcWrite(channel, val)  ->  ledcWrite(pin, val)
- mcpwm_capture_enable + mcpwm_isr_register + MCPWM0.int_ena
    ->  modern capture API: mcpwm_new_capture_timer / mcpwm_new_capture_channel
        + mcpwm_capture_channel_register_event_callbacks  (no deprecated header)
 */

#include <stdio.h>
#include <Wire.h>
#include <driver/mcpwm_prelude.h>
#include <U8g2lib.h>

//------------------------------------------------------------------------------
// set to TRUE to get debug output on USB serial port
#define CFG_DBG        TRUE

// define the PIN number you want to use
#define GPIO_INP1      15
#define GPIO_INP2       2

#define PIN_ADC1       32
#define PIN_ADC2       33

#define PWMI1_PIN      27

#define GPIO_OUT1      17
#define GPIO_OUT2      18

#define PIN_DAC1       25
#define PIN_DAC2       26

#define PWMO1_PIN      14
#define PWMO1_FRQ      1000
#define PWMO1_RSL      12

#define PIN_SRL_RX     RX0
#define PIN_SRL_TX     TX0

#define SZ_RX_BUFFER   100
#define BAUD_RATE      115200

// OLED display (SH1106 128x64, I2C)
#define PIN_OLED_SDA   21
#define PIN_OLED_SCL   23
#define OLED_UPDATE_MS 200

// rotary encoder with push button (menu)
#define ENC_CLK_PIN    13
#define ENC_DT_PIN     16
#define ENC_BTN_PIN     4

constexpr long kPwmFreqStep = 10L;
constexpr long kPwmDutyStep = 20L;
constexpr long kDacStep     = 1L;
//------------------------------------------------------------------------------
HardwareSerial* pSrl       = &Serial2;
//------------------------------------------------------------------------------
// command buffer
//------------------------------------------------------------------------------
/*
    A class for handling a circular buffer
      use put to add a sign and
          get to read a sign
*/
template<int sz>
class CircularBuffer {
public:
  CircularBuffer() {
    rdIdx = wrIdx = 0;
  }
  void put(uint8_t byte) {
    buffer[wrIdx] = byte;
    wrIdx = (wrIdx + 1) % sz;
  }
  uint8_t get() {
    uint8_t byte = buffer[rdIdx];
    rdIdx = (rdIdx + 1) % sz;
    return byte;
  }
  int storedBytes() {
    int ret = wrIdx - rdIdx;
    if (wrIdx < rdIdx) {
      ret += sz;
    }
    return ret;
  }
  bool isEmpty() {
    return wrIdx == rdIdx;
  }
  bool isFull() {
    return storedBytes() >= sz - 1;
  }
  void clear() {
    rdIdx = wrIdx = 0;
  }
private:
  int rdIdx, wrIdx;
  uint8_t buffer[sz];
};
//------------------------------------------------------------------------------
// struct for pwm input handling
typedef struct {
  volatile uint32_t captureValue[2];
  volatile uint32_t dutyCounter;
  volatile uint32_t periodCounter;
  volatile uint32_t seqCounter;   // incremented on every ISR update
  volatile uint32_t lastRiseTick; // tick count of last rising edge (staleness check)
  volatile bool hasRisingCapture;
  volatile bool hasPeriod;        // period valid (2nd rising edge seen)
  bool          ready;
} typePwm;

typedef struct {
  bool     ready;
  uint32_t freq;
  uint32_t duty;
} typePwmOut;
//------------------------------------------------------------------------------
// structure that holds module wide variables
typedef struct sLocals {
  volatile typePwm pwmIn;
  volatile typePwmOut pwmOut;
  // tracked output values (not readable back from hardware)
  int      dacVals[2]  = {0, 0};
  // +1 for c-string end sign '\0'
  CircularBuffer<SZ_RX_BUFFER + 1> srlRxBfr;
} typeLocals;
//------------------------------------------------------------------------------
typeLocals lcls = {{0}, {false, PWMO1_FRQ, 0}, {0, 0}, CircularBuffer<SZ_RX_BUFFER + 1>()};
//------------------------------------------------------------------------------
// digital input handling
//------------------------------------------------------------------------------
void initDigitalIn() {
#ifdef GPIO_INP1
  pinMode(GPIO_INP1, INPUT);
#endif
#ifdef GPIO_INP2
  pinMode(GPIO_INP2, INPUT);
#endif
}
//------------------------------------------------------------------------------
bool getDigitalInput(int nr) {
  switch (nr) {
#ifdef GPIO_INP1
    case 0:
      nr = GPIO_INP1;
      break;
#endif
#ifdef GPIO_INP2
    case 1:
      nr = GPIO_INP2;
      break;
#endif
    default:
      return false;
  }
  return digitalRead(nr);
}
//------------------------------------------------------------------------------
// digital output handling
//------------------------------------------------------------------------------
void initDigitalOut() {
#ifdef GPIO_OUT1
  pinMode(GPIO_OUT1, OUTPUT);
#endif
#ifdef GPIO_OUT2
  pinMode(GPIO_OUT2, OUTPUT);
#endif
}
//------------------------------------------------------------------------------
bool getDigitalOutput(int nr) {
  switch (nr) {
#ifdef GPIO_OUT1
    case 0:
      nr = GPIO_OUT1;
      break;
#endif
#ifdef GPIO_OUT2
    case 1:
      nr = GPIO_OUT2;
      break;
#endif
    default:
      return false;
  }
  return digitalRead(nr);
}
//------------------------------------------------------------------------------
void setDigitalOutput(int nr, int val) {
  val = val != 0 ? HIGH : LOW;
  switch (nr) {
#ifdef GPIO_OUT1
    case 0:
      digitalWrite(GPIO_OUT1, val);
      break;
#endif
#ifdef GPIO_OUT2
    case 1:
      digitalWrite(GPIO_OUT2, val);
      break;
#endif
    default:
      break;
  }
}
//------------------------------------------------------------------------------
// digital to analog conversion (DAC) handling
//------------------------------------------------------------------------------
void initDac() {
#ifdef PIN_DAC1
  setDac(0, 0);
#endif
#ifdef PIN_DAC2
  setDac(1, 0);
#endif
}
//------------------------------------------------------------------------------
void setDac(int nr, int val) {
  if (val > 255) {
    val = 255;
  } else if (val < 0) {
    val = 0;
  }
  if (nr >= 0 && nr < 2) {
    lcls.dacVals[nr] = val;
  }
  switch (nr) {
#ifdef PIN_DAC1
    case 0:
      dacWrite(PIN_DAC1, (uint8_t)val);
      break;
#endif
#ifdef PIN_DAC2
    case 1:
      dacWrite(PIN_DAC2, (uint8_t)val);
      break;
#endif
    default:
      break;
  }
}
//------------------------------------------------------------------------------
// analog to digital conversion (ADC) handling
//------------------------------------------------------------------------------
void initAdc() {
}
//------------------------------------------------------------------------------
int getAdc(int nr) {
  switch (nr) {
#ifdef PIN_ADC1
    case 0:
      nr = PIN_ADC1;
      break;
#endif
#ifdef PIN_ADC2
    case 1:
      nr = PIN_ADC2;
      break;
#endif
    default:
      return -1;   // -1: invalid channel (analogRead itself is >= 0)
  }
  return analogRead(nr);
}
//------------------------------------------------------------------------------
// pulse wide modulation (PWM) output handling – Core 3.x uses pin-based LEDC API
//------------------------------------------------------------------------------
void initPwmOut() {
#ifdef PWMO1_PIN
  lcls.pwmOut.ready = ledcAttach(PWMO1_PIN, PWMO1_FRQ, PWMO1_RSL);
#endif
}

void setPwm(int nr, int val) {
#if defined(PWMO1_PIN)
  if (lcls.pwmOut.ready && nr == 0 && val >= 0 && val <= 4095) {
    ledcWrite(PWMO1_PIN, val);
  }
#endif
}
//------------------------------------------------------------------------------
// PWM input
//------------------------------------------------------------------------------
// pulse wide modulation input handling – modern capture API (Core 3.x)
static mcpwm_cap_timer_handle_t   pwmInCapTimer   = nullptr;
static mcpwm_cap_channel_handle_t pwmInCapPosEdge = nullptr;
static mcpwm_cap_channel_handle_t pwmInCapNegEdge = nullptr;
//------------------------------------------------------------------------------
// ISR callback for both capture channels, edge selected via edata->cap_edge
static bool IRAM_ATTR pwm_capture_cb(
    mcpwm_cap_channel_handle_t cap_channel,
    const mcpwm_capture_event_data_t *edata,
    void *user_ctx)
{
  (void)cap_channel;
  (void)user_ctx;
  uint32_t tmp = edata->cap_value;
  if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {   // rising edge
    if (lcls.pwmIn.hasRisingCapture) {
      lcls.pwmIn.periodCounter = tmp - lcls.pwmIn.captureValue[0];
      lcls.pwmIn.hasPeriod = true;
    }
    lcls.pwmIn.captureValue[0] = tmp;
    lcls.pwmIn.hasRisingCapture = true;
    lcls.pwmIn.lastRiseTick = xTaskGetTickCountFromISR();
    lcls.pwmIn.seqCounter++;
  } else if (lcls.pwmIn.hasRisingCapture) {      // falling edge
    lcls.pwmIn.dutyCounter = tmp - lcls.pwmIn.captureValue[0];
    lcls.pwmIn.captureValue[1] = tmp;
    lcls.pwmIn.seqCounter++;
  }
  return false;  // no task switch needed
}
//------------------------------------------------------------------------------
void initPwmIn() {
  // modern capture API:
  //   1 capture timer + 2 capture channels on the same pin:
  //     pos-edge channel → period (own capture latch)
  //     neg-edge channel → duty / on-time (own capture latch)
  //   two channels are REQUIRED so that both edge values are latched in
  //   hardware independently of ISR latency (narrow pulses!)
  mcpwm_capture_timer_config_t tcfg = {};
  tcfg.group_id     = 0;
  tcfg.clk_src      = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  tcfg.resolution_hz = 1000000;       // requested resolution; classic ESP32 forces the APB clock
  // classic ESP32 (v1) has no SOC_MCPWM_SUPPORT_SLEEP_RETENTION, so allow_pd MUST
  // be 0 – otherwise mcpwm_new_capture_timer() returns ESP_ERR_NOT_SUPPORTED (0x106).
  tcfg.flags.allow_pd = false;
  if (mcpwm_new_capture_timer(&tcfg, &pwmInCapTimer) != ESP_OK) {
    pwmInCapTimer = nullptr;
    return;
  }
  mcpwm_capture_channel_config_t posCfg = {};
  posCfg.intr_priority = 0;   // 0: driver allocates a low priority (1..3)
  posCfg.prescale      = 1;   // no prescale division
  posCfg.flags.pos_edge   = true;
  posCfg.flags.neg_edge   = false;
  posCfg.flags.pull_up    = false;
  posCfg.flags.pull_down  = true;
  posCfg.flags.invert_cap_signal = false;
  posCfg.flags.io_loop_back     = false;
  posCfg.gpio_num = PWMI1_PIN;
  mcpwm_capture_event_callbacks_t posCbs = {};
  posCbs.on_cap = pwm_capture_cb;

  if (mcpwm_new_capture_channel(pwmInCapTimer, &posCfg, &pwmInCapPosEdge) != ESP_OK ||
      mcpwm_capture_channel_register_event_callbacks(pwmInCapPosEdge, &posCbs, nullptr) != ESP_OK) {
    pwmInCapPosEdge = nullptr;
  }

  mcpwm_capture_channel_config_t negCfg = {};
  negCfg.intr_priority = 0;
  negCfg.prescale      = 1;
  negCfg.flags.pos_edge   = false;
  negCfg.flags.neg_edge   = true;
  negCfg.flags.pull_up    = false;
  negCfg.flags.pull_down  = true;
  negCfg.flags.invert_cap_signal = false;
  negCfg.flags.io_loop_back     = false;
  negCfg.gpio_num = PWMI1_PIN;

  mcpwm_capture_event_callbacks_t negCbs = {};
  negCbs.on_cap = pwm_capture_cb;

  if (pwmInCapPosEdge != nullptr &&
      mcpwm_new_capture_channel(pwmInCapTimer, &negCfg, &pwmInCapNegEdge) == ESP_OK &&
      mcpwm_capture_channel_register_event_callbacks(pwmInCapNegEdge, &negCbs, nullptr) == ESP_OK) {
    // start the shared capture counter, then enable both channel input paths
    if (mcpwm_capture_timer_enable(pwmInCapTimer) != ESP_OK ||
        mcpwm_capture_timer_start(pwmInCapTimer) != ESP_OK ||
        mcpwm_capture_channel_enable(pwmInCapPosEdge) != ESP_OK ||
        mcpwm_capture_channel_enable(pwmInCapNegEdge) != ESP_OK) {
      pwmInCapPosEdge = nullptr;
      pwmInCapNegEdge = nullptr;
    }
  } else {
    pwmInCapNegEdge = nullptr;
  }

  if (pwmInCapTimer == nullptr ||
      pwmInCapPosEdge == nullptr ||
      pwmInCapNegEdge == nullptr) {
    // tear down partial setup so getPwm() does not use a dead timer
    if (pwmInCapNegEdge != nullptr) {
      mcpwm_capture_channel_disable(pwmInCapNegEdge);
      mcpwm_del_capture_channel(pwmInCapNegEdge);
      pwmInCapNegEdge = nullptr;
    }
    if (pwmInCapPosEdge != nullptr) {
      mcpwm_capture_channel_disable(pwmInCapPosEdge);
      mcpwm_del_capture_channel(pwmInCapPosEdge);
      pwmInCapPosEdge = nullptr;
    }
    if (pwmInCapTimer != nullptr) {
      mcpwm_del_capture_timer(pwmInCapTimer);
      pwmInCapTimer = nullptr;
    }
    return;
  }
  lcls.pwmIn.ready = true;
}
//------------------------------------------------------------------------------
bool getPwmReady() {
  return lcls.pwmIn.ready;
}
//------------------------------------------------------------------------------
void getPwm(int nr, uint32_t& freq, uint32_t& duty) {
  (void)nr;  // single input channel
  freq = 0;
  duty = 0;
  if (!lcls.pwmIn.ready) {
    return;
  }
  // coherent snapshot: copy the ISR-written values with interrupts disabled
  // (bounded retries in case the capture ISR runs on the other core)
  uint32_t seq1, seq2;
  uint32_t periodTicks = 0, dutyTicks = 0;
  bool hasPeriod = false;
  for (int tries = 0; tries < 8; tries++) {
    noInterrupts();
    seq1        = lcls.pwmIn.seqCounter;
    periodTicks = lcls.pwmIn.periodCounter;
    dutyTicks   = lcls.pwmIn.dutyCounter;
    hasPeriod   = lcls.pwmIn.hasPeriod;
    seq2        = lcls.pwmIn.seqCounter;
    interrupts();
    if (seq1 == seq2) {
      break;
    }
  }

  if (!hasPeriod || periodTicks == 0) {
    duty = 0;
    freq = 0;
    return;   // no full period measured yet
  }

  // staleness: no rising edge for > 2 periods (min 100 ms) → signal is gone
  // (also covers DC levels at duty 0% / 100%, which produce no more edges)
  uint32_t timeoutTicks = pdMS_TO_TICKS(100) + (2 * periodTicks * configTICK_RATE_HZ) / 80000000ULL;
  if ((xTaskGetTickCount() - lcls.pwmIn.lastRiseTick) > timeoutTicks) {
    duty = 0;
    freq = 0;
    return;
  }

  // capture counter ticks → microseconds, using the actual capture timer resolution
  uint32_t resolution = 80000000;
  if (pwmInCapTimer != nullptr) {
    mcpwm_capture_timer_get_resolution(pwmInCapTimer, &resolution);
  }
  uint32_t scale = (resolution + 999999) / 1000000;
  if (scale == 0) {
    scale = 1;
  }
  periodTicks /= scale;
  dutyTicks   /= scale;
  if (dutyTicks > periodTicks) {   // glitch guard (e.g. missed edge after wrap)
    dutyTicks = periodTicks;
  }
  // duty in 12-bit units
  uint32_t dutyRaw = (uint32_t)(((uint64_t)dutyTicks * 4095ULL) / periodTicks);
  duty = (dutyRaw > 4095) ? 4095 : dutyRaw;
  // period in µs → frequency in Hz (rounded)
  freq = (1000000ULL + periodTicks / 2) / periodTicks;
}
//------------------------------------------------------------------------------
// menu (encoder + button)
//------------------------------------------------------------------------------
enum MenuState { ST_SELECT, ST_EDIT_DAC, ST_EDIT_FREQ, ST_EDIT_DUTY };

static MenuState menuState = ST_SELECT;
static int       menuSel   = 0;
static bool      menuDirty = false;
static int       encAccum  = 0;

constexpr int      kMenuItemCount    = 5;
constexpr int      kLastDioItem      = 1;
constexpr int      kFirstDacItem     = 2;
constexpr int      kLastDacItem      = 3;
constexpr int      kSelectThreshold  = 5;
constexpr int      kDefaultThreshold = 1;
constexpr int      kMaxStepsPerLoop  = 5;
constexpr uint32_t kBtnDebounceMs    = 30;

static bool btnPressed() {
  static uint32_t lastChangeMs = 0;
  static int      lastRaw      = LOW;
  static bool     debounced    = false;
  static bool     wasPressed   = false;

  int raw = digitalRead(ENC_BTN_PIN);
  if (raw != lastRaw) {
    lastChangeMs = millis();
    lastRaw = raw;
  }
  if (millis() - lastChangeMs < kBtnDebounceMs) {
    return false;
  }
  debounced = (raw == HIGH);
  bool pressed = debounced && !wasPressed;
  wasPressed = debounced;
  return pressed;
}

// encoder (updated in ISR)
static volatile int encDelta = 0;
static uint8_t encPrevAB = 0;

void IRAM_ATTR encIsr() {
  uint8_t ab = (uint8_t)((digitalRead(ENC_CLK_PIN) & 1) << 1) | (uint8_t)(digitalRead(ENC_DT_PIN) & 1);
  uint8_t idx = (uint8_t)((encPrevAB << 2) | ab);
  if (idx == 1 || idx == 7 || idx == 8 || idx == 14) {
    encDelta--;
  } else if (idx == 2 || idx == 4 || idx == 11 || idx == 13) {
    encDelta++;
  }
  encPrevAB = ab;
}

static int takeEncoderDelta() {
  noInterrupts();
  int delta = encDelta;
  encDelta = 0;
  interrupts();
  return delta;
}

static int encoderThreshold() {
  switch (menuState) {
    case ST_SELECT: return kSelectThreshold;
    default:        return kDefaultThreshold;
  }
}

static int consumeEncoderSteps() {
  encAccum += takeEncoderDelta();
  int threshold = encoderThreshold();
  int steps = 0;
  while (encAccum >= threshold) {
    steps++;
    encAccum -= threshold;
  }
  while (encAccum <= -threshold) {
    steps--;
    encAccum += threshold;
  }
  return steps;
}

static void resetEncoderSteps() {
  encAccum = 0;
}

static void toggleSelectedDio() {
  setDigitalOutput(menuSel, getDigitalOutput(menuSel) ? 0 : 1);
}

static void enterEditMode() {
  if (menuSel <= kLastDacItem) {
    menuState = ST_EDIT_DAC;
  } else {
    menuState = ST_EDIT_FREQ;
  }
}

static void onButtonPressed() {
  menuDirty = true;
  resetEncoderSteps();
  switch (menuState) {
    case ST_SELECT:
      if (menuSel <= kLastDioItem) {
        toggleSelectedDio();
      } else {
        enterEditMode();
      }
      break;
    case ST_EDIT_DAC:
      menuState = ST_SELECT;
      break;
    case ST_EDIT_FREQ:
      menuState = ST_EDIT_DUTY;
      break;
    case ST_EDIT_DUTY:
      menuState = ST_SELECT;
      break;
  }
}

static void selectAdjacentItem(int dir) {
  menuSel = (menuSel + dir + kMenuItemCount) % kMenuItemCount;
}

static void adjustDac(int dir) {
  int channel = menuSel - kFirstDacItem;
  setDac(channel, lcls.dacVals[channel] + dir * kDacStep);
}

static void adjustPwmFreq(int dir) {
  long freq = (long)lcls.pwmOut.freq + dir * kPwmFreqStep;
  if (freq < 1) freq = 1;
  if (freq > 1000000L) freq = 1000000L;
  if (lcls.pwmOut.ready && ledcChangeFrequency(PWMO1_PIN, (uint32_t)freq, PWMO1_RSL) != 0) {
    lcls.pwmOut.freq = (uint32_t)freq;
  }
}

static void adjustPwmDuty(int dir) {
  long duty = (long)lcls.pwmOut.duty + dir * kPwmDutyStep;
  if (duty <    0) duty =    0;
  if (duty > 4095) duty = 4095;
  if (lcls.pwmOut.ready) {
    ledcWrite(PWMO1_PIN, (uint32_t)duty);
    lcls.pwmOut.duty = (uint32_t)duty;
  }
}

static void applyStep(int dir) {
  switch (menuState) {
    case ST_SELECT:    selectAdjacentItem(dir); break;
    case ST_EDIT_DAC:  adjustDac(dir);          break;
    case ST_EDIT_FREQ: adjustPwmFreq(dir);      break;
    case ST_EDIT_DUTY: adjustPwmDuty(dir);      break;
  }
}

void handleMenu() {
  if (btnPressed()) {
    onButtonPressed();
  }

  int steps = consumeEncoderSteps();
  if (steps == 0) {
    return;
  }
  menuDirty = true;
  int dir   = (steps > 0) ? 1 : -1;
  int count = (steps > 0) ? steps : -steps;
  if (count > kMaxStepsPerLoop) {
    count = kMaxStepsPerLoop;
  }
  for (int i = 0; i < count; i++) {
    applyStep(dir);
  }
}
//------------------------------------------------------------------------------
// OLED display
//------------------------------------------------------------------------------
static bool displayReady = false;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, PIN_OLED_SCL, PIN_OLED_SDA);

void initDisplay() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(50);
  // I2C bus scan for debugging
  pSrl->printf("I2C scan (SDA=%d, SCL=%d):\n", PIN_OLED_SDA, PIN_OLED_SCL);
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      pSrl->printf("  found device at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    pSrl->println("  no I2C devices found!");
    displayReady = false;
    return;
  }
  // try common SH1106 addresses
  uint8_t oledAddr = 0;
  for (uint8_t a = 0x3C; a <= 0x3D; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      oledAddr = a;
      break;
    }
  }
  if (!oledAddr) {
    pSrl->println("  OLED not found at 0x3C/0x3D!");
    displayReady = false;
    return;
  }
  pSrl->printf("  OLED at 0x%02X\n", oledAddr);
  u8g2.setI2CAddress(oledAddr << 1);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 24, "Esp32DutInjector");
  u8g2.drawStr(0, 40, "init...");
  u8g2.sendBuffer();
  displayReady = true;
  pSrl->println("  display init OK");
}
//------------------------------------------------------------------------------
static void drawItem(int x, int y, const char* s, bool selected, bool editing) {
  if (editing) {
    int w = u8g2.getStrWidth(s);
    u8g2.setDrawColor(1);
    u8g2.drawBox(x, y - 8, w + 2, 10);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x, y, s);
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(x, y, s);
    if (selected) {
      int uy = y + 2;
      if (uy > 63) uy = 63;
      u8g2.drawHLine(x, uy, u8g2.getStrWidth(s));
    }
  }
}

void updateDisplay() {
  if (!displayReady) {
    return;
  }
  uint32_t pwmInFreq = 0, pwmInDuty = 0;
  if (lcls.pwmIn.ready) {
    getPwm(0, pwmInFreq, pwmInDuty);
  }
  int pwmInDutyPct = (int)((pwmInDuty * 100ULL + 2047) / 4095);
  int pwmOutDutyPct = (int)((lcls.pwmOut.duty * 100ULL + 2047) / 4095);
  int in1 = getDigitalInput(0) ? 1 : 0;
  int in2 = getDigitalInput(1) ? 1 : 0;
  int out1 = getDigitalOutput(0) ? 1 : 0;
  int out2 = getDigitalOutput(1) ? 1 : 0;
  int a1 = getAdc(0);
  int a2 = getAdc(1);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  char buf[32];
  snprintf(buf, sizeof(buf), "inp1:%d    inp2:%d", in1, in2);
  u8g2.drawStr(0, 8, buf);
  snprintf(buf, sizeof(buf), "out1:%d", out1);
  drawItem(0, 20, buf, menuSel == 0, false);
  snprintf(buf, sizeof(buf), "out2:%d", out2);
  drawItem(60, 20, buf, menuSel == 1, false);
  snprintf(buf, sizeof(buf), "adc1:%4d adc2:%4d", a1, a2);
  u8g2.drawStr(0, 32, buf);
  snprintf(buf, sizeof(buf), "dac1:%4d", lcls.dacVals[0]);
  drawItem(0, 44, buf, menuSel == 2, menuState == ST_EDIT_DAC && menuSel == 2);
  snprintf(buf, sizeof(buf), "dac2:%4d", lcls.dacVals[1]);
  drawItem(60, 44, buf, menuSel == 3, menuState == ST_EDIT_DAC && menuSel == 3);
  snprintf(buf, sizeof(buf), "pwmi1:%7luHz %3d%%", (unsigned long)pwmInFreq, pwmInDutyPct);
  u8g2.drawStr(0, 54, buf);
  snprintf(buf, sizeof(buf), "pwmo1:%7luHz", (unsigned long)lcls.pwmOut.freq);
  drawItem(0, 64, buf, menuSel == 4 && menuState != ST_EDIT_DUTY, menuState == ST_EDIT_FREQ);
  int x = u8g2.getStrWidth(buf);
  snprintf(buf, sizeof(buf), " %3d%%", pwmOutDutyPct);
  drawItem(x, 64, buf, menuSel == 4 && menuState != ST_EDIT_FREQ, menuState == ST_EDIT_DUTY);
  u8g2.sendBuffer();
}
//------------------------------------------------------------------------------
// command parser
//------------------------------------------------------------------------------
void processCmd() {
/*
  Command parser
    known commands, see description at top
  */
#define IS_CMD(x) (strncmp(x, cmd, sizeof(x)) == 0)
#define PRINT_ARG "%u\n", value

  uint8_t flatBuffer[SZ_RX_BUFFER];
  int idx = 0;
  // copy ringbuffer data to flat buffer
  while (!lcls.srlRxBfr.isEmpty()) {
    flatBuffer[idx++] = lcls.srlRxBfr.get();
  }
  if (idx == 0 || flatBuffer[idx - 1] != 0) {
    return;
  }
  char cmd[SZ_RX_BUFFER];
  char extra;
  bool isError = false;
  int nr, value;
  int nel;  // number of elements

#if CFG_DBG == TRUE
  pSrl->printf("\nCmd:%s\n", (const char*)&flatBuffer[0]);
#endif
  if (sscanf((const char*)&flatBuffer[0], "%99s%n", cmd, &idx) != 1) {
    return;
  }
  if (IS_CMD("pwm")) {
    // frequency, duty cycle
    int channel, frequency;
    nel = sscanf((const char*)&flatBuffer[idx], "%d %d %d %c",
           &channel, &frequency, &value, &extra);
    isError = (nel != 3) || (channel != 0) || (frequency < 1) ||
              (frequency > 1000000) || (value < 0) || (value > 4095) ||
              !lcls.pwmOut.ready;
    if (!isError) {
      isError = ledcChangeFrequency(PWMO1_PIN, frequency, PWMO1_RSL) == 0;
    }
    if (!isError) {
      ledcWrite(PWMO1_PIN, value);
      lcls.pwmOut.freq = frequency;
      lcls.pwmOut.duty = value;
    }
  } else if (IS_CMD("dac")) {
    // channel, value
    nel = sscanf((const char*)&flatBuffer[idx], "%d %d %c", &nr, &value, &extra);
    isError = (nel != 2) || (nr < 0) || (nr > 1) || (value < 0) || (value > 255);
    if (!isError) {
      setDac(nr, value);
    }
  } else if (IS_CMD("dout")) {
    // channel, value
    nel = sscanf((const char*)&flatBuffer[idx], "%d %d %c", &nr, &value, &extra);
    isError = (nel != 2) || (nr < 0) || (nr > 1) || (value < 0) || (value > 1);
    if (!isError) {
      setDigitalOutput(nr, value);
    }
  } else if (IS_CMD("duty")) {
    // channel
    nel = sscanf((const char*)&flatBuffer[idx], "%d %c", &nr, &extra);
    isError = (nel != 1) || (nr < 0) || (nr > 0) || !lcls.pwmIn.ready;
    if (!isError) {
      uint32_t freq, duty;
      getPwm(nr, freq, duty);
      // return the duty cycle
      value = duty;
      pSrl->printf(PRINT_ARG);
    }
  } else if (IS_CMD("freq")) {
    // channel
    nel = sscanf((const char*)&flatBuffer[idx], "%d %c", &nr, &extra);
    isError = (nel != 1) || (nr < 0) || (nr > 0) || !lcls.pwmIn.ready;
    if (!isError) {
      uint32_t freq, duty;
      getPwm(nr, freq, duty);
      // return the frequency
      value = freq;
      pSrl->printf(PRINT_ARG);
    }
  } else if (IS_CMD("adc")) {
    // channel
    nel = sscanf((const char*)&flatBuffer[idx], "%d %c", &nr, &extra);
    isError = (nel != 1) || (nr < 0) || (nr > 1);
    if (!isError) {
      value = getAdc(nr);
      // return the adc value
      pSrl->printf(PRINT_ARG);
    }
  } else if (IS_CMD("din")) {
    // pinNr
    nel = sscanf((const char*)&flatBuffer[idx], "%d %c", &nr, &extra);
    isError = (nel != 1) || (nr < 0) || (nr > 1);
    if (!isError) {
      value = getDigitalInput(nr) == true ? 1 : 0;
      // return the digital input value
      pSrl->printf(PRINT_ARG);
    }
  }
  else if (IS_CMD("t")) {
#define DUTY 4
    static uint32_t d = DUTY;
    if(d == DUTY) {
      d = 3*DUTY;
    }
    else {
      d = DUTY;
    }
    isError = !lcls.pwmOut.ready ||
              ledcChangeFrequency(PWMO1_PIN, 1000, PWMO1_RSL) == 0;
    if (!isError) {
      ledcWrite(PWMO1_PIN, d);
      lcls.pwmOut.freq = 1000;
      lcls.pwmOut.duty = d;
      uint32_t freq, duty;
      getPwm(0, freq, duty);
      pSrl->printf(":%u %u %u\n", freq, duty, d);
    }
  }
  else {
    isError = true;
  }
  if (isError) {
    pSrl->print("\nError: cmd parse");
  }
}
//------------------------------------------------------------------------------
// arduino setup and loop
//------------------------------------------------------------------------------
void setup() {
#if CFG_DBG == TRUE
  pSrl = &Serial;
#endif
  pSrl->begin(BAUD_RATE, SERIAL_8N1
#if CFG_DBG != TRUE
    , PIN_SRL_RX, PIN_SRL_TX
#endif
  );
  initDigitalIn();
  initDigitalOut();
  initAdc();
  initDac();
  initPwmIn();
  initPwmOut();
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN_PIN, INPUT_PULLDOWN);
  encPrevAB = (uint8_t)((digitalRead(ENC_CLK_PIN) & 1) << 1) | (uint8_t)(digitalRead(ENC_DT_PIN) & 1);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), encIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), encIsr, CHANGE);
  initDisplay();
}
//------------------------------------------------------------------------------
void loop() {
  handleMenu();
  static uint32_t lastOledUpdate = 0;
  uint32_t nowMs = millis();
  uint32_t interval = menuDirty ? 50 : OLED_UPDATE_MS;
  if (nowMs - lastOledUpdate >= interval) {
    lastOledUpdate = nowMs;
    menuDirty = false;
    updateDisplay();
  }

  if (lcls.srlRxBfr.isFull()) {
    int discarded = pSrl->read();
    if (discarded == '\n') {
      lcls.srlRxBfr.clear();
      pSrl->print("\nError: command too long\n");
    }
    return;
  }

  int inByte = pSrl->read();
  if (inByte < 0) {
    return;
  }

  /*! Get actual byte from Uart buffer */
  if (inByte == '\r') {
    return;
  }
  if (10 == inByte) {
    lcls.srlRxBfr.put(0u);
    processCmd();
  } else {
    lcls.srlRxBfr.put(inByte);
  }
}
//------------------------------------------------------------------------------
