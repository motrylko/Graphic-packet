#pragma once
// Provisional protocol implementation for GP161-ESP32 v3.0.
// Stop-and-wait AX.25 modulo-8 (one outstanding I frame per channel).
// Internal AFSK backend experimental; host tests are NOT physical RF tests.
#undef PI
#include "GP161_AX25.h"
#include "GP161_FileCRC.h"

static const uint32_t AX_T1=6500;
static constexpr uint8_t AX_N2=5;
static constexpr size_t AX_PACLEN=128;
// AutoBIN CRC is NOT the AX.25 frame CRC! CCITT x^16+x^12+x^5+1,
// start 0, MSB-first, non-augmented format described by OpenBCM / DC4OX.

inline bool radioAvailable(){return radioArmed();}
inline void mhUpdate(const char *call){
  for(uint8_t i=0;i<heardUsed;i++)if(strcmp(heard[i].call,call)==0){
    heard[i].lastMs=millis();heard[i].frames++;return;
  }
  if(heardUsed==24){memmove(heard,heard+1,sizeof(heard)-sizeof(heard[0]));heardUsed--;}
  Heard &h=heard[heardUsed++];snprintf(h.call,sizeof(h.call),"%s",call);h.lastMs=millis();h.frames=1;
}
inline void monitorPacket(const gp::Parsed &p){
  char buf[70],sample[23]={};
  for(int i=0;i<p.infoLen && i<22;i++)sample[i]=(p.info[i]>=32 && p.info[i]<127)?(char)p.info[i]:'.';
  snprintf(buf,sizeof(buf),"RX %-10s>%s %02X %s",p.from,p.to,p.ctrl,sample);addMonitor(buf);
}
inline bool queueFrame(const gp::Frame &f){
  if(!radioAvailable())return false;
  if(txUsed>=6)return false;
  txQueue[txTail]=f;txTail=(txTail+1)%6;txUsed++;return true;
}
inline bool sendPacket(const char *destination,uint8_t control,bool pid,const uint8_t *bytes,size_t len){
  if(strcmp(mycall,"NOCALL-0")==0 || !gp::callsignValid(mycall)){
    flashMessage("Set licensed MYCALL first");return false;
  }
  gp::Frame f;if(!gp::build(f,destination,mycall,control,pid,bytes,len))return false;
  if(!radioAvailable()){flashMessage("RF OFF: enable only after wiring check");return false;}
  if(!queueFrame(f)){flashMessage("TX queue full");return false;}
  return true;
}
inline void sendSuper(uint8_t ch,uint8_t control){
  Session &q=sessions[ch];if(!q.peer[0])return;
  if(!sendPacket(q.peer,control,false,nullptr,0))return;
}
inline void connectChannel(uint8_t ch,const char *peer){
  if(ch>=5||!gp::callsignValid(peer)){flashMessage("Invalid callsign");return;}
  if(!radioAvailable()){flashMessage("RF disabled: no connection");return;}
  Session &q=sessions[ch];if(q.state){flashMessage("Channel is busy");return;}
  snprintf(q.peer,sizeof(q.peer),"%s",peer);
  for(char *p=q.peer;*p;p++)*p=(char)toupper((unsigned char)*p);
  if(!sendPacket(q.peer,0x3F,false,nullptr,0)){q.peer[0]=0;return;}
  q.state=1;q.deadline=millis()+AX_T1;q.retries=0;q.vs=q.vr=q.va=0;
  addHistory(ch,"SYS> SABM sent");
}
inline void disconnectChannel(uint8_t ch){
  if(ch>=5)return;
  Session &q=sessions[ch];if(!q.state)return;
  if(sendPacket(q.peer,0x53,false,nullptr,0)){
    q.state=3;q.deadline=millis()+AX_T1;q.retries=0;addHistory(ch,"SYS> DISC sent");
  }
}
inline bool sendInfo(uint8_t ch,const uint8_t *msg,size_t len){
  if(ch>=5 || len>AX_PACLEN)return false;
  Session &q=sessions[ch];if(q.state!=2 || q.waiting)return false;
  uint8_t ctrl=(uint8_t)((q.vs<<1)|(q.vr<<5));
  gp::Frame f;if(!gp::build(f,q.peer,mycall,ctrl,true,msg,len))return false;
  if(!queueFrame(f))return false;
  q.pending=f;q.waiting=true;q.deadline=millis()+AX_T1;q.retries=0;q.vs=(q.vs+1)&7;
  return true;
}
inline void sendUserLine(uint8_t ch,const char *line){
  if(!sessions[ch].state || sessions[ch].state!=2){flashMessage("Channel not connected");return;}
  size_t n=strlen(line);if(n+1>AX_PACLEN){flashMessage("Line too long");return;}
  uint8_t data[AX_PACLEN];memcpy(data,line,n);data[n]='\r';
  if(sendInfo(ch,data,n+1)){
    char row[51];snprintf(row,sizeof(row),"> %.46s",line);addHistory(ch,row);paint();
  }else flashMessage("Waiting for AX.25 ACK");
}
inline void handleQsoEnter(uint8_t ch){
  QsoData &q=qso[ch];
  if(!q.len)return;
  if(q.draft[0]=='/' && (q.draft[1]=='C'||q.draft[1]=='c') && q.draft[2]==' '){connectChannel(ch,q.draft+3);}
  else if(strcmp(q.draft,"/D")==0 || strcmp(q.draft,"/d")==0){disconnectChannel(ch);}
  else if(strncmp(q.draft,"/FILE ",6)==0){startSendFile(ch,q.draft+6);}
  else sendUserLine(ch,q.draft);
  q.draft[0]=0;q.len=0;paint();
}
inline void closeReceived(bool valid);
inline void connectionTick(){
  static uint32_t nextTick=0;if((int32_t)(millis()-nextTick)<0)return;
  nextTick=millis()+50;
  for(uint8_t i=0;i<5;i++){
    Session &q=sessions[i];if(!q.state||!q.deadline||(int32_t)(millis()-q.deadline)<0)continue;
    if(q.retries++ >= AX_N2){
      addHistory(i,"SYS> Link timeout");
      if(tr.receiving && tr.channel==i)closeReceived(false);
      q.state=0;q.waiting=false;q.deadline=0;q.peer[0]=0;
      if(tr.sending && tr.channel==i){tr.sending=false;if(tr.file)tr.file.close();}
      continue;
    }
    if(q.state==1)sendPacket(q.peer,0x3F,false,nullptr,0);
    else if(q.state==3)sendPacket(q.peer,0x53,false,nullptr,0);
    else if(q.state==2 && q.waiting)queueFrame(q.pending);
    else {q.deadline=0;continue;}
    q.deadline=millis()+AX_T1;
  }
}
inline void closeReceived(bool valid){
  if(tr.file)tr.file.close();
  if(tr.expectedValid && tr.actualCrc!=tr.expectedCrc){
    valid=false;addMonitor("ERR> AutoBIN CRC mismatch (.PART kept)");
  }
  if(valid){
    char final[132];snprintf(final,sizeof(final),"/GP/RXFILES/%s",tr.filename);
    if(SD.exists(final)){ // do not overwrite pre-existing files
      for(unsigned n=1;n<1000;n++){
        snprintf(final,sizeof(final),"/GP/RXFILES/RX%03u.BIN",n);
        if(!SD.exists(final))break;
      }
    }
    if(!SD.rename(tr.path,final))flashMessage("SD rename failed; .PART retained");
    else {addMonitor("SYS> AutoBIN saved to RXFILES");flashMessage("File RX completed");}
  } else addMonitor("ERR> RX interrupted; .PART retained");
  tr.receiving=false;tr.lineLen=0;
}
inline void lineReceived(uint8_t ch){
  tr.line[tr.lineLen]=0;
  if(tr.sending && tr.waitingOK && tr.channel==ch && strstr(tr.line,"#OK#")){
    tr.waitingOK=false;addMonitor("SYS> AutoBIN remote OK, sending...");
  }
  if(strncmp(tr.line,"#BIN#",5)==0 && !tr.receiving && !tr.sending){
    if(!sdReady){tr.lineLen=0;return;}
    const char *p=tr.line+5;char *end=nullptr;
    unsigned long size=strtoul(p,&end,10);
    if(end==p || size>15000000UL){tr.lineLen=0;return;}
    tr.expectedValid=false;tr.expectedCrc=0;tr.actualCrc=0;
    if(end[0]=='#' && end[1]=='|'){
      char *afterCrc=nullptr;unsigned long receivedCrc=strtoul(end+2,&afterCrc,10);
      if(afterCrc!=end+2 && receivedCrc<=65535UL){tr.expectedValid=true;tr.expectedCrc=(uint16_t)receivedCrc;}
    }
    const char *name=strrchr(end,'#');
    name=(name && name[1])?name+1:"RXFILE.BIN";
    char clean[41]={};unsigned j=0;
    for(unsigned i=0;name[i] && j<sizeof(clean)-1;i++){
      unsigned char c=(unsigned char)name[i];
      if((isalnum(c)||c=='_'||c=='-'||c=='.') && c<127)clean[j++]=c;
    }
    if(j==0 || strcmp(clean,".")==0 || strcmp(clean,"..")==0)snprintf(clean,sizeof(clean),"RXFILE.BIN");
    snprintf(tr.filename,sizeof(tr.filename),"%s",clean);
    snprintf(tr.path,sizeof(tr.path),"/GP/RXFILES/REC%lu.PART",(unsigned long)millis());
    tr.file=SD.open(tr.path,FILE_WRITE);
    if(tr.file){tr.receiving=true;tr.total=size;tr.done=0;tr.channel=ch;
      static const uint8_t response[]={'#','O','K','#','\r'};
      sendInfo(ch,response,sizeof(response));
      addMonitor("SYS> AutoBIN inbound requested");
      if(size==0)closeReceived(true);
    }else flashMessage("SD open RX failed");
  } else if(tr.line[0] && page>=Page::Qso1 && page<=Page::Qso5) {
    char shown[51];snprintf(shown,sizeof(shown),"< %.46s",tr.line);addHistory(ch,shown);
  } else if(tr.line[0]){
    char shown[51];snprintf(shown,sizeof(shown),"< %.46s",tr.line);addHistory(ch,shown);
  }
  tr.lineLen=0;
}
inline void receiveBytes(uint8_t ch,const uint8_t *data,uint16_t len){
  // 8-bit-clean AutoBIN payload. Files saved initially with .PART extension.
  for(uint16_t i=0;i<len;i++){
    if(tr.receiving && tr.channel==ch){
      if(tr.file){tr.file.write(data[i]);tr.actualCrc=autoBinCrcStep(tr.actualCrc,data[i]);
        if(++tr.done>=tr.total)closeReceived(true);}
      continue;
    }
    uint8_t c=data[i];
    if(c=='\r'||c=='\n'){
      if(tr.lineLen)lineReceived(ch);
    } else if(tr.lineLen<sizeof(tr.line)-1){tr.line[tr.lineLen++]=(char)c;}
    else tr.lineLen=0;
  }
}
inline void onFrame(const gp::Frame &frame){
  gp::Parsed p;if(!gp::parse(frame,p))return;
  rxFrames++;mhUpdate(p.from);monitorPacket(p);
  if(strcmp(p.to,mycall)!=0)return;
  int found=-1,freeSlot=-1;
  for(int i=0;i<5;i++){
    if(sessions[i].state && strcmp(sessions[i].peer,p.from)==0){found=i;break;}
    if(!sessions[i].state && freeSlot<0)freeSlot=i;
  }
  uint8_t ctrl=p.ctrl;
  if((ctrl&0xEF)==0x2F){ // SABM with/without poll
    if(!radioAvailable())return;
    int ch=found>=0?found:freeSlot;if(ch<0){sendPacket(p.from,0x0F,false,nullptr,0);return;}
    Session &q=sessions[ch];snprintf(q.peer,sizeof(q.peer),"%s",p.from);
    q.state=2;q.vs=q.vr=q.va=0;q.waiting=false;q.deadline=0;
    sendSuper(ch,0x73);addHistory(ch,"SYS> Incoming CONNECT");return;
  }
  if(found<0)return;
  uint8_t ch=(uint8_t)found;Session &q=sessions[ch];
  if((ctrl&0xEF)==0x63){ // UA
    if(q.state==1){q.state=2;q.deadline=0;q.vs=q.vr=q.va=0;addHistory(ch,"SYS> CONNECTED");}
    else if(q.state==3){q.state=0;q.peer[0]=0;q.deadline=0;q.waiting=false;addHistory(ch,"SYS> Disconnected");}
    return;
  }
  if((ctrl&0xEF)==0x43){ // DISC
    sendSuper(ch,0x73);q.state=0;q.waiting=false;q.deadline=0;q.peer[0]=0;
    if(tr.receiving && tr.channel==ch)closeReceived(false);
    if(tr.sending && tr.channel==ch){tr.sending=false;tr.file.close();}
    addHistory(ch,"SYS> Remote disconnected");return;
  }
  if((ctrl&0xEF)==0x0F){q.state=0;q.waiting=false;q.deadline=0;q.peer[0]=0;return;}
  if(q.state!=2)return;
  if((ctrl&1)==0 || (ctrl&3)==1){
    uint8_t nr=(ctrl>>5)&7;
    if(q.waiting && nr==q.vs){q.va=nr;q.waiting=false;q.deadline=0;q.retries=0;}
    if((ctrl&1)==0){
      uint8_t ns=(ctrl>>1)&7;
      if(ns==q.vr){q.vr=(q.vr+1)&7;if(p.pid==0xF0)receiveBytes(ch,p.info,p.infoLen);}
      sendSuper(ch,(uint8_t)(0x01|(q.vr<<5)));
    }
  }
  if(page==Page::Monitor || (page>=Page::Qso1 && page<=Page::Qso5) || page==Page::MHeard)paint();
}
inline void startSendFile(uint8_t channel,const char *file){
  if(!sdReady||channel>=5||sessions[channel].state!=2){flashMessage("Need SD + QSO connected");return;}
  if(tr.sending||tr.receiving){flashMessage("Only one transfer at once");return;}
  char path[132];if(file[0]=='/')snprintf(path,sizeof(path),"%s",file);
  else snprintf(path,sizeof(path),"/GP/TXFILES/%s",file);
  File f=SD.open(path,FILE_READ);
  if(!f||f.isDirectory()){if(f)f.close();flashMessage("Cannot open TX file");return;}
  uint32_t size=f.size();if(size>15000000UL){f.close();flashMessage("File too large (15MB max)");return;}
  uint16_t checksum=0;
  while(f.available()){
    uint8_t b=(uint8_t)f.read();checksum=autoBinCrcStep(checksum,b);
    if((f.position()&1023)==0)yield();
  }
  f.seek(0);
  const char *basename=strrchr(path,'/');basename=basename?basename+1:path;
  tr.file=f;tr.sending=true;tr.waitingOK=true;tr.channel=channel;tr.total=size;tr.done=0;
  tr.crc=checksum;snprintf(tr.filename,sizeof(tr.filename),"%.40s",basename);
  char header[128];snprintf(header,sizeof(header),"\r#BIN#%lu#|%u#%s\r",(unsigned long)size,(unsigned)checksum,tr.filename);
  if(!sendInfo(channel,(uint8_t*)header,strlen(header))){tr.file.close();tr.sending=false;flashMessage("TX link busy");return;}
  addMonitor("SYS> AutoBIN header sent; awaiting #OK#");page=Page::Transfer;paint();
}
inline void sendFileFromBrowser(){
  if(!sdReady||dirSelected>=dirCount||dirItems[dirSelected].isDir){flashMessage("Select a file");return;}
  int ch=-1;
  if(sessions[chosenQso].state==2)ch=chosenQso;
  else for(int i=0;i<5;i++)if(sessions[i].state==2){ch=i;break;}
  if(ch<0){flashMessage("No connected QSO (F1..F5)");return;}
  char path[132];snprintf(path,sizeof(path),"%s/%s",dirPath,dirItems[dirSelected].name);
  startSendFile((uint8_t)ch,path);
}
inline void transferTick(){
  if(!tr.sending || tr.waitingOK)return;
  Session &s=sessions[tr.channel];
  if(s.state!=2){tr.sending=false;tr.file.close();addMonitor("ERR> AutoBIN TX link lost");return;}
  if(s.waiting || txUsed>2)return;
  uint8_t buffer[AX_PACLEN];size_t n=tr.file.read(buffer,sizeof(buffer));
  if(n){if(sendInfo(tr.channel,buffer,n)){tr.done+=n;}else tr.file.seek(tr.file.position()-n);}
  if(tr.done>=tr.total && !s.waiting){tr.file.close();tr.sending=false;addMonitor("SYS> AutoBIN TX stream finished");}
  if(page==Page::Transfer && ((millis()/500)%2)==0)paint();
}
inline void drawEditor(){
  tft.fillScreen(C_BLUE);header();titleBox("GP EDITOR / SD");
  txt(4,56,"/GP/MESSAGES/NOTE.TXT",C_YELLOW);
  // Soft-wrap the editor text to fit 320px, shows final 11 rows.
  char lines[11][51]={{0}};uint8_t cur=0,col=0;
  for(uint16_t i=0;i<editorLen;i++){
    if(editor[i]=='\n'||col>=49){if(cur==10){memmove(lines[0],lines[1],10*sizeof(lines[0]));cur=9;}
      else {cur++;}
      col=0;if(editor[i]=='\n')continue;}
    lines[cur][col++]=editor[i];lines[cur][col]=0;
  }
  for(uint8_t i=0;i<11;i++)txt(5,70+i*9,lines[i],C_WHITE);
  status("EDITOR  limited to 639 bytes");prompt("Alt+S:save   Esc:Monitor");footer();
}
inline void drawBeacon(){
  tft.fillScreen(C_BLUE);header();titleBox("BEACON / UNPROTO");
  txt(8,65,"Destination: CQ",C_WHITE);
  txt(8,83,"Beacon text:",C_YELLOW);
  txt(8,98,beaconText,C_WHITE);
  char t[38];snprintf(t,sizeof(t),"Interval: %u min (0=OFF)",beaconIntervalMin);
  txt(8,126,t,C_WHITE);status("UI frames / AX.25 PID F0");
  prompt("Enter:send now  Alt+U:settings");footer();
}
inline void drawTransfer(){
  tft.fillScreen(C_BLUE);header();titleBox("FILE TRANSFER / AUTOBIN");
  txt(8,60,tr.filename,C_YELLOW);
  txt(8,79,tr.sending?"TX FILE":tr.receiving?"RX FILE":"FILE FINISHED",C_WHITE);
  char stat[60];snprintf(stat,sizeof(stat),"%lu / %lu bytes",(unsigned long)tr.done,(unsigned long)tr.total);
  txt(8,101,stat,C_WHITE);
  uint16_t pct=tr.total?(uint16_t)(tr.done*100ULL/tr.total):100;
  tft.drawRect(8,124,304,19,C_WHITE);tft.fillRect(10,126,(int)pct*3,15,C_GREEN);
  snprintf(stat,sizeof(stat),"%u%%",(unsigned)pct);txt(8,152,stat,C_YELLOW);
  status(tr.waitingOK?"Waiting for #OK# from station":"AutoBIN experimental format");
  prompt("Esc:Monitor  transfer continues");footer();
}
inline void radioTick(){
  uint8_t level;unsigned steps=0;gp::Frame frame;
  while(++steps<700 && bay::readLevel(level))if(decoder.feedLevel(level,frame))onFrame(frame);
  bay::pollTxDone();
  if(radioAvailable() && !bay::busy() && txUsed){
    gp::Frame &f=txQueue[txHead];
    rfBitstream=gp::TxBits();
    unsigned pre=(unsigned)(txDelayMs*(uint32_t)packetBaud/8000UL);
    if(pre<4)pre=4;
    if(pre>100)pre=100;
    if(gp::encodeHdlc(f,rfBitstream,pre,7) && bay::transmit(rfBitstream)){
      txHead=(txHead+1)%6;txUsed--;txFrames++;
    }
  }
}
// KISS frames received over UART2 for controlled TNC hardware testing.
inline void serialKissTick(){
#if GP_ENABLE_KISS_UART2
  static uint8_t b[gp::MAX_AX];static uint16_t n=0;static bool inside=false,escaped=false;
  while(Serial2.available()){
    int c=Serial2.read();if(c<0)break;
    if(c==0xC0){
      if(inside && n>1 && (b[0]&0x0f)==0){gp::Frame f;f.len=n-1;
        if(f.len<gp::MAX_AX){memcpy(f.data,b+1,f.len);onFrame(f);}}
      inside=true;n=0;escaped=false;continue;
    }
    if(!inside)continue;
    if(escaped){if(c==0xDC)c=0xC0;else if(c==0xDD)c=0xDB;escaped=false;}
    else if(c==0xDB){escaped=true;continue;}
    if(n<sizeof(b))b[n++]=(uint8_t)c;else {inside=false;n=0;}
  }
  if(radioAvailable() && txUsed){
    gp::Frame &f=txQueue[txHead];Serial2.write((uint8_t)0xC0);Serial2.write((uint8_t)0);
    for(int i=0;i<f.len;i++){
      uint8_t x=f.data[i];if(x==0xC0){Serial2.write((uint8_t)0xDB);Serial2.write((uint8_t)0xDC);}
      else if(x==0xDB){Serial2.write((uint8_t)0xDB);Serial2.write((uint8_t)0xDD);}
      else Serial2.write(x);
    }
    Serial2.write((uint8_t)0xC0);txFrames++;txHead=(txHead+1)%6;txUsed--;
  }
#endif
}
inline void serialDiagnostics(){
  // Test command over USB console: :DEMO / :SELFTEST, no actual RF TX.
  static char buf[28];static uint8_t n=0;
  while(Serial.available()){
    int c=Serial.read();if(c<0)break;
    if(c=='\n'||c=='\r'){
      buf[n]=0;
      if(strcmp(buf,":DEMO")==0){
        gp::Frame demo;const char *s="TEST packet from software decoder";
        gp::build(demo,"APRS","OM0TST-1",0x03,true,(const uint8_t*)s,strlen(s));
        onFrame(demo);addMonitor("SYS> DEMO only, not RF reception");paint();
      } else if(strcmp(buf,":TESTALL")==0){
        runOfflineDiagnostics(true);paint();
      } else if(strcmp(buf,":SDTEST")==0){
        processDiagnosticSelection(1);paint();
      } else if(strcmp(buf,":FILETEST")==0){
        processDiagnosticSelection(2);paint();
      } else if(strcmp(buf,":REPORT")==0){
        processDiagnosticSelection(7);paint();
      } else if(strcmp(buf,":BAYSTATUS")==0 || strcmp(buf,":AUDIOSTATUS")==0){
        Serial.printf("AUDIO AFSK backend=%s baud=%u armed=%u rxTransitions=%lu rxOverflow=%lu good=%lu badFCS=%lu ADC=%d TX=%lu\n",
            "AFSK AUDIO",(unsigned)packetBaud,
            (unsigned)radioArmed(),(unsigned long)bay::edges(),(unsigned long)bay::overflow(),
            (unsigned long)decoder.good,(unsigned long)decoder.badFcs,bay::auxInput(),(unsigned long)txFrames);
        page=Page::Port;paint();
      } else if(strcmp(buf,":ARM")==0){
        if(!RADIO_CONFIGURED)Serial.println("DENIED: audio backend not configured");
        else{radioArmedUntil=millis()+300000UL;Serial.println("TX ARMED for 5 minutes: analog/PTT adapter MUST be verified");page=Page::Port;paint();}
      } else if(strcmp(buf,":DISARM")==0){
        radioArmedUntil=0;bay::emergencyStop();txUsed=0;txHead=txTail=0;
        Serial.println("TX DISARMED, PTT off, queue cleared");page=Page::Port;paint();
      } else if(strcmp(buf,":PTTTEST")==0){
        if(!radioArmed()){Serial.println("PTT BLOCKED - :ARM required");}
        else Serial.println(bay::pttPulse(150)?"PTT 150 ms pulse complete":"PTT busy / rejected");
        page=Page::Port;paint();
      } else if(strcmp(buf,":BEACONTEST")==0){
        if(!radioArmed())Serial.println("BEACON BLOCKED - :ARM required");
        else Serial.println(sendPacket("CQ",0x03,true,(const uint8_t*)"GP161 V31 AUDIO TEST",20)?
            "Beacon queued (NOT proof of RF success)":"Beacon rejected (set MYCALL/queue check)");
      } else if(strcmp(buf,":HELP")==0){
        Serial.println("GP161 v3.1: :TESTALL :SDTEST :FILETEST :REPORT :DEMO :QSO5 :CLEARQSO5");
        Serial.println(":AUDIOSTATUS :ARM :DISARM :PTTTEST :BEACONTEST :HELP");
        Serial.println("PS/2: Alt+M menu, then 9 DIAGNOSTICS; test 1..8; ESC monitor.");
      } else if(strcmp(buf,":QSO5")==0){
        if(RADIO_CONFIGURED) Serial.println("QSO5 DEMO denied: RF backend active");
        else {for(uint8_t i=0;i<5;i++){
          Session &q=sessions[i];q=Session();
          snprintf(q.peer,sizeof(q.peer),"OM0SIM-%u",(unsigned)i+1);
          q.state=2;addHistory(i,"SIMULATED - NO RF CONNECTION");
        }Serial.println("QSO 1..5 simulated state; no RF, no transport, no peer ACK");paint();}
      } else if(strcmp(buf,":CLEARQSO5")==0){
        if(!RADIO_CONFIGURED){for(uint8_t i=0;i<5;i++)sessions[i]=Session();
          Serial.println("QSO simulations cleared");paint();}
      } else if(strcmp(buf,":SELFTEST")==0){
        gp::Frame f,g;const uint8_t p[]={0x7E,0xFF,0xF0,0x00};
        gp::build(f,"APRS","OM0TST",0x03,true,p,sizeof(p));gp::TxBits bits;
        gp::Deframer d;bool ok=false;
        if(gp::encodeHdlc(f,bits,12,8)){
          for(unsigned i=0;i<bits.count;i++)if(d.feedLevel(bits.at(i),g))ok=true;
        }
        Serial.println(ok&&g.len==f.len&&!memcmp(g.data,f.data,f.len)?
                       "SELFTEST PASS (NRZI/HDLC/CRC)":"SELFTEST FAIL");
      }
      n=0;continue;
    }
    if(n<sizeof(buf)-1)buf[n++]=(char)c;else n=0;
  }
}
