#ifndef GP161_DIAGNOSTICS_H
#define GP161_DIAGNOSTICS_H
// GP161-ESP32 v3.1 diagnostics: internal AFSK RX, TX locked by default.
// Arduino code; doesn't modify the user's files except controlled test paths.

static constexpr uint8_t DIAG_N=8;
static const char * const diagNames[DIAG_N]={
 "AX25 / HDLC / CRC", "SD READ + WRITE", "FILE LOCAL COPY",
 "NVS SAVE / LOAD", "PS2 KEY MONITOR", "TFT COLORS / FONT",
 "AFSK AUDIO / PTT", "EXPORT REPORT"
};
// 0 NOT RUN, 1 PASS, 2 FAIL, 3 MANUAL/NOT-VERIFIED, 4 SKIPPED
static uint8_t diagStates[DIAG_N]={};
static char diagNotes[DIAG_N][54]={};
uint8_t testScreenStage=0;
static uint8_t testLastCodes[16]={};
static uint8_t testLastCount=0;
static char testLastEvent[24]="No key";
static uint32_t diagLastScan=0;

inline void diagSave(uint8_t id,uint8_t state,const char *message){
 if(id>=DIAG_N)return;
 diagStates[id]=state;
 snprintf(diagNotes[id],sizeof(diagNotes[id]),"%s",message);
 Serial.printf("TEST %u [%s] %s: %s\n",(unsigned)(id+1),diagNames[id],
        state==1?"PASS":state==2?"FAIL":state==3?"MANUAL":state==4?"SKIP":"WAIT",message);
}
inline const char *diagStateText(uint8_t state){
 switch(state){case 1:return "PASS";case 2:return "FAIL";
 case 3:return "MANUAL";case 4:return "LOCK";default:return "----";}
}
inline void noteScan(uint8_t sc,bool released,bool extended){
 if(testLastCount>=16){memmove(testLastCodes,testLastCodes+1,15);testLastCount=15;}
 testLastCodes[testLastCount++]=sc;
 snprintf(testLastEvent,sizeof(testLastEvent),"%s %s%02X",released?"UP":"DN",extended?"E0-":"",sc);
 diagLastScan=millis();
}
inline bool testAx25Core(){
 if(!gp::callsignValid("OM0ABC-1")||gp::callsignValid("INVALID-99"))return false;
 const uint8_t tests[][11]={
   {0x00,0xff,0xf8,0x7e,0x01,0x7e,0,0,0x12,0x34,0x55},
   {'G','P','1','6','1',0x7e,0xff,0xf0,0x01,0x00,0x00},
   {0,0,0,0,0,0,0,0,0,0,0}
 };
 for(uint8_t i=0;i<3;i++){
   gp::Frame a,b;gp::TxBits bits;gp::Deframer reader;
   if(!gp::build(a,"APRS","OM0TST-1",0x03,true,tests[i],sizeof(tests[i])))return false;
   if(!gp::encodeHdlc(a,bits,10,6))return false;
   bool received=false;
   for(unsigned k=0;k<bits.count;k++)if(reader.feedLevel(bits.at(k),b))received=true;
   if(!received||b.len!=a.len||memcmp(a.data,b.data,a.len))return false;
   gp::Parsed parsed;
   if(!gp::parse(b,parsed)||strcmp(parsed.from,"OM0TST-1")||parsed.infoLen!=sizeof(tests[i]))return false;
   if(memcmp(parsed.info,tests[i],sizeof(tests[i])))return false;
 }
 // CRC known vector for AX25/X25: "123456789" => 0x906E
 if(gp::crc((const uint8_t*)"123456789",9)!=0x906E)return false;
 if(autoBinCrcStep(0,'1')!=0x2672)return false;
 uint16_t autobin=0;
 for(const char *p="123456789";*p;p++)autobin=autoBinCrcStep(autobin,(uint8_t)*p);
 return autobin==0x31C3;
}
inline bool ensureDiagFolder(){
 if(!sdReady)return false;
 if(!SD.exists("/GP"))SD.mkdir("/GP");
 if(!SD.exists("/GP/TEST"))SD.mkdir("/GP/TEST");
 return SD.exists("/GP/TEST");
}
inline void makeTempPaths(const char *tag,char *source,size_t srcCap,char *dest,size_t dstCap){
 uint32_t id=(uint32_t)(millis()%1000000UL);
 snprintf(source,srcCap,"/GP/TEST/%s%06lu.A",tag,(unsigned long)id);
 snprintf(dest,dstCap,"/GP/TEST/%s%06lu.B",tag,(unsigned long)id);
}
inline bool testSDReadWrite(){
 if(!ensureDiagFolder())return false;
 char a[64],b[64];makeTempPaths("SD",a,sizeof(a),b,sizeof(b));
 if(SD.exists(a)||SD.exists(b))return false; // do not overwrite user data
 File f=SD.open(a,FILE_WRITE);if(!f)return false;
 bool ok=true;
 for(uint16_t i=0;i<1024;i++){uint8_t value=(uint8_t)((i*37+19)&255);
   if(f.write(value)!=1){ok=false;break;}}
 f.flush();f.close();
 f=SD.open(a,FILE_READ);
 if(!f){SD.remove(a);return false;}
 if(f.size()!=1024)ok=false;
 for(uint16_t i=0;ok && i<1024;i++){
   int value=f.read();if(value!=((i*37+19)&255))ok=false;
 }
 f.close();if(!SD.remove(a))ok=false;
 return ok;
}
inline bool testLocalFileIntegrity(){
 if(!ensureDiagFolder())return false;
 char a[64],b[64];makeTempPaths("FT",a,sizeof(a),b,sizeof(b));
 if(SD.exists(a)||SD.exists(b))return false;
 File src=SD.open(a,FILE_WRITE);
 if(!src)return false;
 uint16_t expected=0;
 bool ok=true;
 for(uint16_t i=0;i<777;i++){
   uint8_t x=(uint8_t)((i*29+0x7e)&255);
   if(src.write(x)!=1){ok=false;break;}
   expected=autoBinCrcStep(expected,x);
 }
 src.close();
 if(!ok){SD.remove(a);return false;}
 src=SD.open(a,FILE_READ);File dst=SD.open(b,FILE_WRITE);
 if(!src||!dst){if(src)src.close();if(dst)dst.close();SD.remove(a);SD.remove(b);return false;}
 uint8_t buff[73];uint16_t actual=0;size_t copied=0;
 while(src.available()){
  size_t n=src.read(buff,sizeof(buff));if(!n)break;
  if(dst.write(buff,n)!=n){ok=false;break;}
  copied+=n;for(size_t i=0;i<n;i++)actual=autoBinCrcStep(actual,buff[i]);
 }
 dst.flush();src.close();dst.close();
 File check=SD.open(b,FILE_READ);
 if(!check){ok=false;}else {if(check.size()!=777)ok=false;check.close();}
 if(copied!=777||expected!=actual)ok=false;
 if(!SD.remove(a))ok=false;
 if(!SD.remove(b))ok=false;
 return ok;
}
inline bool testNvsStore(){
 Preferences probe;
 if(!probe.begin("gp161v3test",false))return false;
 const uint32_t marker=0xA55A3CC3u;
 bool ok=probe.putUInt("probe",marker)==sizeof(uint32_t);
 ok=ok && probe.getUInt("probe",0)==marker;
 probe.remove("probe");probe.end();
 return ok;
}
inline void printReport(Print &out){
 out.println("GP161-ESP32 v3.1 INTERNAL AUDIO, offline/hardware diagnostics");
 out.printf("Uptime ms: %lu | SD: %s | RF: %s\n",(unsigned long)millis(),
             sdReady?"OK":"N/A",RADIO_CONFIGURED?"AFSK RX":"OFF");
 for(uint8_t i=0;i<DIAG_N;i++){
   out.printf("%u %s %-6s : %s\n",(unsigned)(i+1),diagNames[i],diagStateText(diagStates[i]),diagNotes[i]);
 }
 out.printf("PS2 frames=%lu parity_errors=%lu drops=%lu\n",
    (unsigned long)ps2Frames,(unsigned long)ps2ParityErrors,(unsigned long)ps2Drops);
 out.printf("AX25 rx=%lu tx=%lu badFcs=%lu\n",(unsigned long)rxFrames,(unsigned long)txFrames,(unsigned long)decoder.badFcs);
 out.println("NOTE: Synthetic audio and AX25 tests do NOT prove real RF, connected-mode or GP compatibility.");
}
inline bool exportReport(){
 printReport(Serial);
 if(!sdReady)return false;
 if(!SD.exists("/GP/LOGS"))SD.mkdir("/GP/LOGS");
 File f=SD.open("/GP/LOGS/TEST_V3.TXT",FILE_APPEND);
 if(!f)return false;
 f.println("------------------------------");printReport(f);f.close();return true;
}
inline void runOfflineDiagnostics(bool verbose){
 diagSave(0,testAx25Core()?1:2,"NRZI HDLC bitstuff CRC AX25 + AutoBIN CRC");
 bool sdpass=testSDReadWrite();
 diagSave(1,sdpass?1:2,sdReady?"1024B write/read/delete":"SD not mounted");
 diagSave(2,testLocalFileIntegrity()?1:2,sdReady?"777B local binary-copy/CRC":"SD not mounted");
 diagSave(3,testNvsStore()?1:2,"NVS isolated namespace / readback");
 diagSave(4,3,"Keyboard requires physical F1-F5 and Alt+M");
 diagSave(5,3,"Inspect color bars, text and layout visually");
 diagSave(6,3,"Internal AFSK 1200/300, :AUDIOSTATUS then :ARM");
 if(verbose)printReport(Serial);
}
inline void drawDiagnostics(){
 tft.fillScreen(C_BLUE);header();titleBox("DIAGNOSTICS / GP161 v3.1");
 for(uint8_t i=0;i<DIAG_N;i++){
   char row[56];snprintf(row,sizeof(row),"%u %-22s %s",(unsigned)i+1,diagNames[i],diagStateText(diagStates[i]));
   txt(4,57+i*14,row,diagStates[i]==1?C_GREEN:diagStates[i]==2?C_YELLOW:C_WHITE);
 }
 status("1:AX 2:SD 3:FILE 4:NVS 5:PS2");
 prompt("6:TFT 7:AUDIO 8:REPORT ESC:MON");footer();
}
inline void drawKeyboardTest(){
 tft.fillScreen(C_BLUE);header();titleBox("PS/2 KEYBOARD TEST");
 char row[64];
 snprintf(row,sizeof(row),"Frames:%lu Parity:%lu Lost:%lu",
   (unsigned long)ps2Frames,(unsigned long)ps2ParityErrors,(unsigned long)ps2Drops);
 txt(5,61,row,C_YELLOW);
 snprintf(row,sizeof(row),"Last: %s",testLastEvent);txt(5,79,row,C_WHITE);
 txt(5,103,"Press F1-F5, Alt+M, arrows,",C_WHITE);
 txt(5,115,"Enter, Backspace, letters.",C_WHITE);
 txt(5,133,"Scan codes (hex):",C_YELLOW);
 for(uint8_t i=0;i<testLastCount;i++){
   char n[5];snprintf(n,sizeof(n),"%02X",testLastCodes[i]);
   txt(4+i*19,148,n,C_GREEN);
 }
 status("Visual check, ESC: TESTS");prompt("No RF activity from this test");footer();
}
inline void drawTftTest(){
 tft.fillScreen(C_BLACK);
 if(testScreenStage==0){
  tft.fillRect(0,0,80,160,C_RED);tft.fillRect(80,0,80,160,C_GREEN);
  tft.fillRect(160,0,80,160,C_BLUE);tft.fillRect(240,0,80,160,C_WHITE);
 }else if(testScreenStage==1){
  for(int y=0;y<240;y+=16)for(int x=0;x<320;x+=16)
   tft.fillRect(x,y,16,16,((x+y)/16)&1?C_WHITE:C_BLACK);
 }else if(testScreenStage==2){
  tft.fillScreen(C_BLUE);for(int k=0;k<30;k++)tft.drawRect(k*3,k*2,320-k*6,240-k*4,C_YELLOW);
 }else{
  tft.fillScreen(C_DARKGRAY);for(int i=0;i<13;i++){
   char row[51];snprintf(row,sizeof(row),"Text test %02d  ABCDE 012345",i);
   txt(4,4+i*17,row,(i&1)?C_CYAN:C_WHITE);
  }
 }
 tft.fillRect(0,204,320,36,C_BLACK);
 char caption[50];snprintf(caption,sizeof(caption),"TFT TEST %u/4  key=next  ESC=back",(unsigned)testScreenStage+1);
 txt(5,220,caption,C_YELLOW);
}
inline void processDiagnosticSelection(uint8_t selection){
 switch(selection){
  case 0:diagSave(0,testAx25Core()?1:2,"Roundtrip + 2 known CRC vectors");break;
  case 1:diagSave(1,testSDReadWrite()?1:2,sdReady?"1024B write/read/delete":"SD missing");break;
  case 2:diagSave(2,testLocalFileIntegrity()?1:2,sdReady?"777B local binary test":"SD missing");break;
  case 3:diagSave(3,testNvsStore()?1:2,"NVS read/write/remove isolated key");break;
  case 4:diagSave(4,3,"Manual PS2 scan check");page=Page::KeyboardTest;break;
  case 5:diagSave(5,3,"Manual TFT color/geometry check");testScreenStage=0;page=Page::TftTest;break;
  case 6:diagSave(6,3,"Use :AUDIOSTATUS then verified :ARM");page=Page::Port;break;
  case 7:{bool ok=exportReport();diagSave(7,ok?1:2,ok?"Appended to /GP/LOGS/TEST_V3.TXT":"USB only; SD log unavailable");break;}
 }
 paint();
}

#endif // GP161_DIAGNOSTICS_H
