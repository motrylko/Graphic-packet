#ifndef GP161_AFSK_DSP_H
#define GP161_AFSK_DSP_H
// GP161 v3.1 - original experimental audio AFSK DSP (no Arduino dependency).
// 9600 samples per second, phase-continuous 1200/2200 and 1600/1800 Hz.
// Software modem laboratory prototype. Real RF BER must be validated.
#include <stdint.h>
#include <stddef.h>
#include <math.h>
namespace audio_dsp {
static constexpr uint32_t SAMPLE_RATE = 9600;
static constexpr float GP_PI = 3.14159265358979323846f;
struct Profile {
  uint16_t baud, markHz, spaceHz;
  uint8_t samplesPerBit;
};
inline Profile profile(uint16_t baud){
  return baud==300 ? Profile{300,1600,1800,32} : Profile{1200,1200,2200,8};
}
class Tx {
  Profile mode=profile(1200);
  const uint8_t *packed=nullptr;
  uint16_t count=0, pos=0;
  uint8_t within=0;
  float phase=0;
public:
  void setSpeed(uint16_t baud){mode=profile(baud);}
  void start(const uint8_t *levels,uint16_t bits){packed=levels;count=bits;pos=0;within=0;phase=0;}
  bool active() const {return packed && pos<count;}
  // Returns DAC code (centered on 128) and advances one sample.
  uint8_t sample(){
    if(!active())return 128;
    bool mark=((packed[pos>>3]>>(pos&7))&1u)!=0;
    float step=2.0f*GP_PI*(mark?mode.markHz:mode.spaceHz)/SAMPLE_RATE;
    uint8_t out=(uint8_t)(128+roundf(75.0f*sinf(phase)));
    phase+=step;if(phase>=2.0f*GP_PI)phase-=2.0f*GP_PI;
    if(++within>=mode.samplesPerBit){within=0;pos++;}
    return out;
  }
};
class Rx {
  Profile mode=profile(1200);
  float dc=2048, im=0,qm=0,is=0,qs=0,pm=0,ps=0;
  float cm=1,sm=0,cs=1,ss=0,cmStep=0,smStep=0,csStep=0,ssStep=0;
  float smooth=0;
  bool level=true, valid=false;
  uint8_t timer=0,stable=0;
  uint32_t toneEdges=0,symbols=0;
public:
  void setSpeed(uint16_t baud){
    mode=profile(baud); reset();
    cmStep=cosf(2*GP_PI*mode.markHz/SAMPLE_RATE);
    smStep=sinf(2*GP_PI*mode.markHz/SAMPLE_RATE);
    csStep=cosf(2*GP_PI*mode.spaceHz/SAMPLE_RATE);
    ssStep=sinf(2*GP_PI*mode.spaceHz/SAMPLE_RATE);
  }
  void reset(){ dc=2048;im=qm=is=qs=pm=ps=smooth=0;cm=cs=1;sm=ss=0;
    level=true;valid=false;timer=stable=0;toneEdges=symbols=0; }
  // Called once per sample. Returns true for a recovered NRZI symbol level.
  bool feed(int raw,uint8_t &symbol){
    dc += 0.001f*(raw-dc);
    float x=(raw-dc)*(1.0f/1024.0f);
    // Quadrature synchronous detectors for mark and space.
    float a=(mode.baud==300)?0.095f:0.24f;
    im+=(x*cm-im)*a;qm+=(x*sm-qm)*a;
    is+=(x*cs-is)*a;qs+=(x*ss-qs)*a;
    float nm=cm*cmStep-sm*smStep;sm=sm*cmStep+cm*smStep;cm=nm;
    float ns=cs*csStep-ss*ssStep;ss=ss*csStep+cs*ssStep;cs=ns;
    float delta=(im*im+qm*qm)-(is*is+qs*qs);
    smooth+=((mode.baud==300)?0.21f:0.4f)*(delta-smooth);
    bool next=smooth>=0;
    if(next!=level){
      // Hysteresis keeps noise around zero from causing spurious edges.
      if(stable>=((mode.baud==300)?2:2)){
        level=next; ++toneEdges;timer=0;stable=0;
      }
    }else if(stable<250)stable++;
    if(timer==mode.samplesPerBit/2){symbol=level?1:0;symbols++;valid=true;}
    else valid=false;
    ++timer;if(timer>=mode.samplesPerBit)timer=0;
    return valid;
  }
  uint32_t edges()const{return toneEdges;}
  uint32_t symbolCount()const{return symbols;}
};
} // namespace audio_dsp

#endif // GP161_AFSK_DSP_H
