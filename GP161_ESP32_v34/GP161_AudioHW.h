#ifndef GP161_AUDIOHW_H
#define GP161_AUDIOHW_H
// GP161 v3.1: internal 1200/300 AFSK laboratory audio backend.
// Hardware: GPIO34 ADC1, GPIO25 ESP32 DAC1, GPIO27 -> 4k7 -> BC547 base.
// Audio samples use I2S DMA; the DSP runs in a blocking FreeRTOS task.
#include <Arduino.h>
#include <driver/adc.h>
#include <driver/i2s.h>
#include <freertos/task.h>
#include <string.h>
#include "GP161_AX25.h"
#include "GP161_AFSK_DSP.h"

namespace bay {
static constexpr int RX=34, TX=25, PTT=27;
static constexpr size_t DMA_SAMPLES=96;
static constexpr int DMA_DESCRIPTORS=4;
static constexpr i2s_port_t AUDIO_I2S=I2S_NUM_0;
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
static uint16_t currentSpeed=1200;
static uint16_t preDelay=0;
static volatile uint8_t txDrainBlocks=0;
static volatile bool txDraining=false;
static bool wasTx=false;

inline void pttOff(){digitalWrite(PTT,LOW);}

static void releaseI2s(bool adcEnabled) {
  if(adcEnabled){
    esp_err_t err=i2s_adc_disable(AUDIO_I2S);
    if(err!=ESP_OK)Serial.printf("ERR: I2S ADC shutdown failed (%s)\n",esp_err_to_name(err));
  }
  esp_err_t err=i2s_driver_uninstall(AUDIO_I2S);
  if(err!=ESP_OK)Serial.printf("ERR: I2S driver shutdown failed (%s)\n",esp_err_to_name(err));
}

static void audioTask(void*) {
  uint16_t input[DMA_SAMPLES];
  uint16_t output[DMA_SAMPLES*2];

  for(;;){
    size_t bytesRead=0;
    esp_err_t err=i2s_read(AUDIO_I2S,input,sizeof(input),&bytesRead,portMAX_DELAY);
    if(err!=ESP_OK){
      portENTER_CRITICAL(&mux);
      ready=false;txPending=false;txActive=false;txCompleted=false;txDraining=false;txDrainBlocks=0;
      digitalWrite(PTT,LOW);
      portEXIT_CRITICAL(&mux);
      Serial.printf("ERR: I2S audio read failed (%s)\n",esp_err_to_name(err));
      releaseI2s(true);
      vTaskDelete(nullptr);
      return;
    }
    if(bytesRead%sizeof(input[0])){overruns=overruns+1;bytesRead-=bytesRead%sizeof(input[0]);}
    size_t sampleCount=bytesRead/sizeof(input[0]);
    if(sampleCount>DMA_SAMPLES){overruns=overruns+1;sampleCount=DMA_SAMPLES;}
    if(!sampleCount){overruns=overruns+1;continue;}
    if(bytesRead!=sizeof(input))overruns=overruns+1;

    bool started=false;
    uint16_t speed=0;
    portENTER_CRITICAL(&mux);
    if(!txActive&&!txPending&&requestedBaud!=currentSpeed)
      speed=requestedBaud;
    if(txPending&&!txActive){txPending=false;txActive=true;started=true;}
    portEXIT_CRITICAL(&mux);

    if(speed){currentSpeed=speed;demod.setSpeed(speed);mod.setSpeed(speed);}
    if(started){
      portENTER_CRITICAL(&mux);
      if(txActive){
        txDraining=false;txDrainBlocks=0;
        mod.start(txPacked,txBitCount);
        digitalWrite(PTT,HIGH);
        preDelay=192;
        wasTx=true;
      }
      portEXIT_CRITICAL(&mux);
    }

    bool beginDrain=false;
    for(size_t i=0;i<sampleCount;i++){
      uint16_t dacSample;
      if(txActive){
        if(preDelay){--preDelay;dacSample=static_cast<uint16_t>(128)<<8;}
        else if(mod.active())dacSample=static_cast<uint16_t>(mod.sample())<<8;
        else{
          dacSample=static_cast<uint16_t>(128)<<8;
          if(!txDraining)beginDrain=true;
        }
      }else{
        dacSample=static_cast<uint16_t>(128)<<8;
        if(wasTx){wasTx=false;demod.reset();}
        int raw=input[i]&0x0fff;
        adcLevel=static_cast<uint32_t>(raw);
        uint8_t symbol=0;
        if(demod.feed(raw,symbol)){
          uint16_t next=(rxW+1)&1023;
          if(next!=rxR){rxLevels[rxW]=symbol;rxW=next;}
          else dropped=dropped+1;
        }
        rxTransitions=demod.edges();
      }
      // ESP32 built-in DAC duplex mode sends right-channel samples first.
      output[i*2]=dacSample;
      output[i*2+1]=0;
    }

    size_t bytesWritten=0;
    size_t outputBytes=sampleCount*2*sizeof(output[0]);
    err=i2s_write(AUDIO_I2S,output,outputBytes,&bytesWritten,pdMS_TO_TICKS(100));
    if(err!=ESP_OK || bytesWritten!=outputBytes){
      portENTER_CRITICAL(&mux);
      ready=false;txPending=false;txActive=false;txCompleted=false;txDraining=false;txDrainBlocks=0;
      digitalWrite(PTT,LOW);
      portEXIT_CRITICAL(&mux);
      if(err!=ESP_OK)
        Serial.printf("ERR: I2S audio write failed (%s)\n",esp_err_to_name(err));
      else
        Serial.printf("ERR: I2S audio short write (%u/%u bytes)\n",
                      (unsigned)bytesWritten,(unsigned)outputBytes);
      releaseI2s(true);
      vTaskDelete(nullptr);
      return;
    }

    if(beginDrain){
      portENTER_CRITICAL(&mux);
      if(txActive){
        txDraining=true;
        txDrainBlocks=DMA_DESCRIPTORS;
      }
      portEXIT_CRITICAL(&mux);
    }else if(txDraining){
      portENTER_CRITICAL(&mux);
      if(txActive && txDrainBlocks){
        txDrainBlocks=txDrainBlocks-1;
        if(txDrainBlocks==0){
          digitalWrite(PTT,LOW);
          txActive=false;
          txCompleted=true;
          txDraining=false;
        }
      }
      portEXIT_CRITICAL(&mux);
    }
  }
}

inline void begin(){
  digitalWrite(PTT,LOW);
  pinMode(PTT,OUTPUT);
  pinMode(RX,INPUT);
  dacWrite(TX,128);
  currentSpeed=requestedBaud;
  demod.setSpeed(currentSpeed);mod.setSpeed(currentSpeed);
  analogSetPinAttenuation(RX,ADC_11db);

  i2s_config_t config={};
  config.mode=static_cast<i2s_mode_t>(I2S_MODE_MASTER|I2S_MODE_RX|I2S_MODE_TX|
                                     I2S_MODE_ADC_BUILT_IN|I2S_MODE_DAC_BUILT_IN);
  config.sample_rate=audio_dsp::SAMPLE_RATE;
  config.bits_per_sample=I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format=I2S_CHANNEL_FMT_ONLY_RIGHT;
  config.communication_format=I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags=0;
  config.dma_desc_num=DMA_DESCRIPTORS;
  config.dma_frame_num=DMA_SAMPLES;
  config.use_apll=false;
  config.tx_desc_auto_clear=true;

  esp_err_t err=i2s_driver_install(AUDIO_I2S,&config,0,nullptr);
  bool driverInstalled=err==ESP_OK;
  if(err==ESP_OK)err=i2s_set_adc_mode(ADC_UNIT_1,ADC1_CHANNEL_6);
  if(err==ESP_OK)err=i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
  bool adcEnabled=false;
  if(err==ESP_OK){err=i2s_adc_enable(AUDIO_I2S);adcEnabled=err==ESP_OK;}
  if(err!=ESP_OK){
    pttOff();
    if(driverInstalled)releaseI2s(adcEnabled);
    ready=false;
    Serial.printf("ERR: AFSK I2S DMA init failed (%s)\n",esp_err_to_name(err));
    return;
  }

  ready=true;
  if(xTaskCreatePinnedToCore(audioTask,"gp161_audio",4096,nullptr,10,nullptr,0)!=pdPASS){
    ready=false;
    pttOff();
    releaseI2s(true);
    Serial.println("ERR: AFSK audio task creation failed");
  }
}

inline void setBaud(uint16_t speed){
  if(speed!=1200 && speed!=300)return;
  portENTER_CRITICAL(&mux);requestedBaud=speed;portEXIT_CRITICAL(&mux);
}
inline uint16_t baud(){return requestedBaud;}
inline bool available(){return ready;}
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
inline bool pollTxDone(){
  portENTER_CRITICAL(&mux);
  bool completed=txCompleted;
  txCompleted=false;
  portEXIT_CRITICAL(&mux);
  return completed;
}
inline bool busy(){return txActive||txPending||txCompleted;}
inline uint32_t overflow(){return dropped+overruns;}
inline uint32_t edges(){return rxTransitions;}
inline int auxInput(){return (int)adcLevel;}
inline void emergencyStop(){
  portENTER_CRITICAL(&mux);
  txPending=false;txActive=false;txCompleted=false;txDraining=false;txDrainBlocks=0;
  digitalWrite(PTT,LOW);
  portEXIT_CRITICAL(&mux);
}
inline bool pttPulse(unsigned durationMs){
  if(busy()||durationMs>150||!ready)return false;
  digitalWrite(PTT,HIGH);delay(durationMs);digitalWrite(PTT,LOW);return true;
}
} // namespace bay

#endif // GP161_AUDIOHW_H
