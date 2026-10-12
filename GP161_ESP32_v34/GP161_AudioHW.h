#ifndef GP161_AUDIOHW_H
#define GP161_AUDIOHW_H
// GP161 v3.1: internal 1200/300 AFSK laboratory audio backend.
// Hardware: GPIO34 ADC1, GPIO25 ESP32 DAC1, GPIO27 -> 4k7 -> BC547 base.
// 100k base-to-emitter; emitter GND, collector to VERIFIED PTT/GND-switch input.
// PTT active HIGH at GPIO27, transistor saturates and grounds radio PTT.
// 9600 sps nominal sampling with esp_timer TASK callbacks, NOT DMA.
// Real-world jitter/BER and radio interoperability are NOT yet verified.
#include <Arduino.h>
#include <esp_timer.h>
#include <string.h>
#include "GP161_AX25.h"
#include "GP161_AFSK_DSP.h"

namespace bay {  // Historical name kept for compatibility with Engine.h APIs.
static constexpr int RX=34, TX=25, PTT=27;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t rxW=0,rxR=0;
static uint8_t rxLevels[1024]={};
static volatile uint32_t dropped=0,overruns=0,rxTransitions=0,adcLevel=0;
static volatile bool ready=false, txActive=false, txPending=false, txCompleted=false;
static volatile uint16_t requestedBaud=1200;
static uint8_t txPacked[(gp::MAX_BITS+7)/8]={};
static uint16_t txBitCount=0;
static audio_dsp::Rx demod;
static audio_dsp::Tx mod;
static esp_timer_handle_t ticker=nullptr;
static uint16_t currentSpeed=1200;
static uint16_t preDelay=0;
static bool wasTx=false;
static int64_t previousTick=0;

inline void pttOff(){digitalWrite(PTT,LOW);dacWrite(TX,128);}

// esp_timer uses ESP_TIMER_TASK dispatch, not ISR. Keep callback short.
static void onSample(void*){
  int64_t now=esp_timer_get_time();
  if(previousTick && now-previousTick>280)overruns++;
  previousTick=now;
  bool started=false;uint16_t speed=0;
  portENTER_CRITICAL(&mux);
  if(!txActive&&!txPending && requestedBaud!=currentSpeed)
    speed=requestedBaud;
  if(txPending&&!txActive){txPending=false;txActive=true;started=true;}
  portEXIT_CRITICAL(&mux);
  if(speed){currentSpeed=speed;demod.setSpeed(speed);mod.setSpeed(speed);}
  if(started){
    // Coordinate PTT assertion with :DISARM on the other CPU core.
    portENTER_CRITICAL(&mux);
    if(txActive){
      mod.start(txPacked,txBitCount);
      digitalWrite(PTT,HIGH);
      preDelay=192; // 20 ms before the AX.25 flags
      wasTx=true;
    }
    portEXIT_CRITICAL(&mux);
  }
  if(txActive){
    if(preDelay){--preDelay;dacWrite(TX,128);}
    else if(mod.active())dacWrite(TX,mod.sample());
    else{
      pttOff();
      portENTER_CRITICAL(&mux);txActive=false;txCompleted=true;portEXIT_CRITICAL(&mux);
    }
  }else{
    if(wasTx){wasTx=false;demod.reset();} // Ignore local TX audio echo
    int raw=analogRead(RX);
    adcLevel=(uint32_t)raw;
    uint8_t symbol=0;
    if(demod.feed(raw,symbol)){
      uint16_t next=(rxW+1)&1023;
      if(next!=rxR){rxLevels[rxW]=symbol;rxW=next;}
      else ++dropped;
    }
    rxTransitions=demod.edges();
  }
}
inline void begin(){
  // Put the output latch LOW before changing GPIO27 to output.
  // The external 100k base-to-emitter resistor holds PTT inactive at reset.
  digitalWrite(PTT,LOW);
  pinMode(PTT,OUTPUT);
  pinMode(RX,INPUT);analogReadResolution(12);
  analogSetPinAttenuation(RX,ADC_11db);
  dacWrite(TX,128);
  currentSpeed=requestedBaud;
  demod.setSpeed(currentSpeed);mod.setSpeed(currentSpeed);
  esp_timer_create_args_t args={};
  args.callback=&onSample;
  args.arg=nullptr;
  args.dispatch_method=ESP_TIMER_TASK;
  args.name="gp161_afsk";
  esp_err_t err=esp_timer_create(&args,&ticker);
  if(err==ESP_OK && ticker)err=esp_timer_start_periodic(ticker,104);
  ready=(err==ESP_OK);
  if(!ready){pttOff();Serial.println("ERR: AFSK esp_timer failed");}
}
inline void setBaud(uint16_t speed){
  if(speed!=1200 && speed!=300)return;
  portENTER_CRITICAL(&mux);requestedBaud=speed;portEXIT_CRITICAL(&mux);
}
inline uint16_t baud(){return requestedBaud;}
inline bool readLevel(uint8_t &v){
  if(rxR==rxW)return false;
  v=rxLevels[rxR];rxR=(rxR+1)&1023;return true;
}
inline bool transmit(const gp::TxBits &bits){
  if(!ready||!bits.count||bits.count>gp::MAX_BITS)return false;
  portENTER_CRITICAL(&mux);
  if(txPending||txActive||txCompleted){portEXIT_CRITICAL(&mux);return false;}
  memcpy(txPacked,bits.bytes,(bits.count+7)/8);
  txBitCount=bits.count;txPending=true;
  portEXIT_CRITICAL(&mux);return true;
}
inline bool pollTxDone(){if(!txCompleted)return false;txCompleted=false;return true;}
inline bool busy(){return txActive||txPending||txCompleted;}
inline uint32_t overflow(){return dropped+overruns;}
inline uint32_t edges(){return rxTransitions;}
inline int auxInput(){return (int)adcLevel;}
inline void emergencyStop(){
  portENTER_CRITICAL(&mux);
  txPending=false;txActive=false;txCompleted=false;
  digitalWrite(PTT,LOW); // atomic against onSample() asserting PTT
  portEXIT_CRITICAL(&mux);
  dacWrite(TX,128);
}
inline bool pttPulse(unsigned durationMs){
  if(busy()||durationMs>150||!ready)return false;
  digitalWrite(PTT,HIGH);delay(durationMs);digitalWrite(PTT,LOW);return true;
}
} // namespace bay

#endif // GP161_AUDIOHW_H
