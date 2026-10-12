#ifndef GP161_AX25_H
#define GP161_AX25_H
// GP161-ESP32 independent AX.25/HDLC core; original implementation, MIT license.
// All HDLC bits are LSB-first. CRC-16/X.25 (poly reversed 0x8408).
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
namespace gp {
static constexpr size_t MAX_AX = 330;
static constexpr size_t MAX_BITS = 4096;
struct Frame { uint8_t data[MAX_AX]; uint16_t len = 0; };
inline uint16_t crcRaw(const uint8_t *buf, size_t n) {
  uint16_t f=0xFFFF;
  for(size_t i=0;i<n;i++) { f^=buf[i]; for(int k=0;k<8;k++) f=(f>>1)^((f&1)?0x8408:0); }
  return f;
}
inline uint16_t crc(const uint8_t *b, size_t n) {return (uint16_t)~crcRaw(b,n);}
inline bool callsignValid(const char *call) {
  if(!call || !call[0])return false;
  int n=0;
  while(call[n] && call[n]!='-' && n<7) { if(!isalnum((unsigned char)call[n]))return false;n++; }
  if(n<2 || n>6)return false;
  if(call[n]=='-') {n++;if(!isdigit((unsigned char)call[n]))return false;int ss=0;while(isdigit((unsigned char)call[n])){ss=ss*10+call[n++]-'0';if(ss>15)return false;} }
  return call[n]==0;
}
inline void encodeAddress(uint8_t out[7], const char *call, bool last) {
  char upper[12]={}; snprintf(upper,sizeof(upper),"%s",call);
  for(int i=0;upper[i];i++)upper[i]=(char)toupper((unsigned char)upper[i]);
  const char *dash=strchr(upper,'-'); size_t n=dash?(size_t)(dash-upper):strlen(upper);
  for(int k=0;k<6;k++)out[k]=(uint8_t)((k<(int)n?upper[k]:' ')<<1);
  uint8_t ss=(uint8_t)(dash?atoi(dash+1):0);
  out[6]=(uint8_t)(0x60|((ss&15)<<1)|(last?1:0));
}
inline void decodeAddress(const uint8_t addr[7],char out[12]) {
  char call[7]={};int n=6;
  for(int i=0;i<6;i++)call[i]=(char)(addr[i]>>1);
  while(n>0 && call[n-1]==' ')n--;
  call[n]=0;uint8_t ss=(addr[6]>>1)&15;
  if(ss)snprintf(out,12,"%s-%u",call,ss);else snprintf(out,12,"%s",call);
}
// AX.25 V2 modulo 8, two addresses, no digipeater path in TX.
inline bool build(Frame &out,const char *destination,const char *source,uint8_t ctrl,
                  bool hasPid,const uint8_t *payload,size_t n) {
  if(!callsignValid(destination)||!callsignValid(source)||n>MAX_AX-18||(n && !payload))return false;
  encodeAddress(out.data,destination,false);encodeAddress(out.data+7,source,true);
  out.data[14]=ctrl;
  size_t off=15;
  if(hasPid)out.data[off++]=0xF0;
  if(n && payload)memcpy(out.data+off,payload,n);
  out.len=(uint16_t)(off+n);return true;
}
struct Parsed {char to[12];char from[12];uint8_t ctrl=0,pid=0;bool hasPid=false;
               const uint8_t *info=nullptr;uint16_t infoLen=0;};
inline bool parse(const Frame &f,Parsed &p) {
  if(f.len<15)return false;
  uint16_t off=0;bool last=false;
  unsigned addresses=0;
  do {if(off+7>f.len || ++addresses>10)return false;
    if(addresses==1)decodeAddress(f.data+off,p.to);
    if(addresses==2)decodeAddress(f.data+off,p.from);
    last=(f.data[off+6]&1)!=0;off+=7;
  }while(!last);
  if(addresses<2 || off>=f.len)return false;
  p.ctrl=f.data[off++];
  p.hasPid=((p.ctrl&1)==0 || (p.ctrl&0xEF)==0x03);
  if(p.hasPid){if(off>=f.len)return false;p.pid=f.data[off++];}
  p.info=f.data+off;p.infoLen=f.len-off;return true;
}
// Bit-packed NRZI levels, with leading/trailing 0x7e flag bytes and bit stuffing.
struct TxBits {uint8_t bytes[(MAX_BITS+7)/8]={};uint16_t count=0;uint8_t level=1;bool overflow=false;
  void putDataBit(uint8_t bit) { if(!bit)level^=1;
    if(count>=MAX_BITS){overflow=true;return;}if(level)bytes[count>>3]|=(uint8_t)(1u<<(count&7));
    ++count;
  }
  void putFlag(){for(int k=0;k<8;k++)putDataBit((0x7Eu>>k)&1);}
  void putFrame(const Frame &f){unsigned ones=0;uint16_t fcs=crc(f.data,f.len);
    for(size_t k=0;k<(size_t)f.len+2;k++) {uint8_t v=k<f.len?f.data[k]:(uint8_t)(k==f.len?fcs:fcs>>8);
      for(int b=0;b<8;b++){uint8_t d=(v>>b)&1;putDataBit(d);
        if(d){if(++ones==5){putDataBit(0);ones=0;}}else ones=0;
      }
    }
  }
  uint8_t at(uint16_t i) const {return (bytes[i>>3]>>(i&7))&1;}
};
inline bool encodeHdlc(const Frame &f,TxBits &out,unsigned preambleFlags,unsigned postFlags) {
  if(f.len>MAX_AX-2 || preambleFlags*8u+postFlags*8u+f.len*10u+16u>MAX_BITS)return false;
  for(unsigned i=0;i<preambleFlags;i++)out.putFlag();
  out.putFrame(f);
  for(unsigned i=0;i<postFlags;i++)out.putFlag();
  return !out.overflow;
}
// Deframe bitstream (NRZI levels) with de-stuffing, CRC and frame boundaries.
struct Deframer {
  uint8_t bits[(MAX_BITS+7)/8]={};
  uint16_t bitLen=0;uint8_t recent=0,lastLevel=1;bool inFrame=false;
  uint32_t good=0,badFcs=0,oversize=0;
  void resetBits(){bitLen=0;}
  uint8_t at(uint16_t i) const {return (bits[i>>3]>>(i&7))&1;}
  void appendBit(uint8_t b) {
    if(bitLen>=MAX_BITS){++oversize;inFrame=false;return;}
    if(b)bits[bitLen>>3]|=(uint8_t)(1u<<(bitLen&7));
    else bits[bitLen>>3]&=(uint8_t)~(1u<<(bitLen&7));
    bitLen++;
  }
  bool decode(uint16_t rawBits, Frame &ready) {
    Frame f;uint8_t cur=0,nb=0,ones=0;
    for(uint16_t i=0;i<rawBits;i++) {
      uint8_t v=at(i);
      if(v){if(++ones>5)return false;}
      else if(ones==5){ones=0;continue;} // stuffed zero
      else ones=0;
      cur|=(uint8_t)(v<<nb);
      if(++nb==8){if(f.len>=MAX_AX){++oversize;return false;}f.data[f.len++]=cur;cur=0;nb=0;}
    }
    if(nb || f.len<17)return false;
    if(crcRaw(f.data,f.len)!=0xF0B8){++badFcs;return false;}
    ready=f;ready.len-=2;++good;return true;
  }
  bool feedBit(uint8_t bit,Frame &ready){
    recent=(recent>>1)|(bit?0x80:0);
    if(inFrame)appendBit(bit);
    if(recent==0x7E){
      bool valid=inFrame && bitLen>=8 && decode(bitLen-8,ready);
      resetBits();inFrame=true;return valid;
    }
    return false;
  }
  bool feedLevel(uint8_t level,Frame &ready){uint8_t bit=(level==lastLevel);lastLevel=level;return feedBit(bit,ready);}
};
} // namespace gp

#endif // GP161_AX25_H
