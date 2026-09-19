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
#include <driver/mcpwm_prelude.h>

//------------------------------------------------------------------------------
// set to TRUE to get debug output on USB serial port
#define CFG_DBG     TRUE

// define the PIN number you want to use
#define GPIO_INP1   36
#define GPIO_INP2   39

#define PIN_ADC1    34
#define PIN_ADC2    35

#define PWMI1_PIN   22

#define GPIO_OUT1   14
#define GPIO_OUT2   27

#define PIN_DAC1    25
#define PIN_DAC2    26

#define PWMO1_PIN   16
#define PWMO1_FRQ   5000
#define PWMO1_RSL   12

#define PIN_SRL_RX   23
#define PIN_SRL_TX   24

#define SZ_RX_BUFFER 100
#define BAUD_RATE    115200

//------------------------------------------------------------------------------
HardwareSerial* pSrl    = &Serial2;
static bool pwmOutReady = false;
static bool pwmInReady  = false;
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

// struct for pwm input handling
typedef struct {
  volatile uint32_t captureValue[2];
  volatile uint32_t dutyCounter;
  volatile uint32_t periodCounter;
  volatile uint32_t seqCounter;   // incremented on every ISR update
  volatile uint32_t lastRiseTick; // tick count of last rising edge (staleness check)
  volatile bool hasRisingCapture;
  volatile bool hasPeriod;        // period valid (2nd rising edge seen)
} typePwm;

// structure that holds module wide variables
typedef struct sLocals {
  volatile typePwm pwmIn;
  // +1 for c-string end sign '\0'
  CircularBuffer<SZ_RX_BUFFER + 1> srlRxBfr;
} typeLocals;
//------------------------------------------------------------------------------
typeLocals lcls = {};
//------------------------------------------------------------------------------
// digital input handling
void initDigitalIn() {
#ifdef GPIO_INP1
  pinMode(GPIO_INP1, INPUT);
#endif
#ifdef GPIO_INP2
  pinMode(GPIO_INP2, INPUT);
#endif
}

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
void initDigitalOut() {
#ifdef GPIO_OUT1
  pinMode(GPIO_OUT1, OUTPUT);
#endif
#ifdef GPIO_OUT2
  pinMode(GPIO_OUT2, OUTPUT);
#endif
}

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
void initDac() {
#ifdef PIN_DAC1
  setDac(0, 0);
#endif
#ifdef PIN_DAC2
  setDac(1, 0);
#endif
}

void setDac(int nr, int val) {
  if (val > 255) {
    val = 255;
  } else if (val < 0) {
    val = 0;
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
void initAdc() {
}

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
void initPwmOut() {
#ifdef PWMO1_PIN
  pwmOutReady = ledcAttach(PWMO1_PIN, PWMO1_FRQ, PWMO1_RSL);
#endif
}

void setPwm(int nr, int val) {
#if defined(PWMO1_PIN)
  if (pwmOutReady && nr == 0 && val >= 0 && val <= 4095) {
    ledcWrite(PWMO1_PIN, val);
  }
#endif
}
//------------------------------------------------------------------------------
// pulse wide modulation input handling – modern capture API (Core 3.x)
static mcpwm_cap_timer_handle_t   pwmInCapTimer   = nullptr;
static mcpwm_cap_channel_handle_t pwmInCapPosEdge = nullptr;
static mcpwm_cap_channel_handle_t pwmInCapNegEdge = nullptr;

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
  pwmInReady = true;
}

bool getPwmReady() {
  return pwmInReady;
}

void getPwm(int nr, uint32_t& freq, uint32_t& duty) {
  (void)nr;  // single input channel
  freq = 0;
  duty = 0;
  if (!pwmInReady) {
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
              !pwmOutReady;
    if (!isError) {
      isError = ledcChangeFrequency(PWMO1_PIN, frequency, PWMO1_RSL) == 0;
    }
    if (!isError) {
      ledcWrite(PWMO1_PIN, value);                    // set duty cycle
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
    isError = (nel != 1) || (nr < 0) || (nr > 0) || !pwmInReady;
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
    isError = (nel != 1) || (nr < 0) || (nr > 0) || !pwmInReady;
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
    isError = !pwmOutReady ||
              ledcChangeFrequency(PWMO1_PIN, 1000, PWMO1_RSL) == 0;
    if (!isError) {
      ledcWrite(PWMO1_PIN, d);
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
}

void loop() {
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
