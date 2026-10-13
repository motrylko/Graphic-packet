/*
  GP161-ESP32 v3.4 CLASSIC MENU + AUDIO AFSK -- OM8AWA SPLASH, internal software TNC.
  ESP32-WROOM-32 DevKit 30pin, ILI9341 320x240, PS/2 keyboard, microSD SPI.
  ADC GPIO34 / DAC GPIO25 / GPIO27 to 4k7 -> BC547 transistor PTT-to-GND.

  EXPERIMENTAL v3.4: not a verified complete replica of original GP1.61.
  AX.25 codec and audio waveform DSP tested on desktop. Real ADC/DAC timing,
  connected-mode, AutoBIN interoperability and RF need physical validation.
  Software AFSK receive active; transmitting requires USB :ARM for five minutes.
  DO NOT CONNECT RADIO AUDIO OR PTT DIRECTLY WITHOUT ANALOG LEVEL ADAPTER!
  Check README.txt for electrical pinout and library versions.
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <ctype.h>
#include <string.h>
#include "GP161_AX25.h"
#include "GP161_OM8AWA_Logo.h"
// Only internal AFSK modem. 1200 Bd Bell 202 or 300 Bd HF AFSK.
#define GP_ENABLE_EXPERIMENTAL_SER12 0
#define GP_ENABLE_KISS_UART2 0
#include "GP161_AudioHW.h"
constexpr bool RADIO_CONFIGURED=true;
// Arm is required separately (USB :ARM) and expires after five minutes.
// Compile-time backend enable alone NEVER permits on-air TX.
uint32_t radioArmedUntil=0;
inline bool radioArmed(){ return RADIO_CONFIGURED && radioArmedUntil &&
  (int32_t)(radioArmedUntil-millis())>0; }

// ------------- fixed GPIO pinout (ESP32-WROOM 30-pin) -----------------
constexpr int PIN_TFT_SCK   = 18;
constexpr int PIN_TFT_MISO  = 19;
constexpr int PIN_TFT_MOSI  = 23;
constexpr int PIN_TFT_CS    = 14;
constexpr int PIN_TFT_DC    = 21;
constexpr int PIN_TFT_RST   = 22;
constexpr int PIN_SD_CS     = 13;
constexpr int PIN_PS2_DATA  = 32;
constexpr int PIN_PS2_CLOCK = 33;
// AUDIO+PTT PINS: use the conditioning/protection circuit in README.
constexpr int PIN_AUDIO_IN  = 34;  // ADC1, conditioned AC signal at 1.65 V
constexpr int PIN_AUDIO_OUT = 25;  // internal DAC, RC low-pass + attenuator
constexpr int PIN_PTT       = 27;  // 4k7 -> BC547 base; 100k base-to-emitter (GND)

Adafruit_ILI9341 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
Preferences prefs;

// Authentic DOS-style palette inspired by original GP 1.61b CONFIG.GP
uint16_t C_BLACK, C_BLUE, C_RED, C_MAGENTA, C_CYAN, C_WHITE;
uint16_t C_GRAY, C_DARKGRAY, C_LIGHTBLUE, C_LIGHTCYAN, C_GREEN, C_YELLOW;

constexpr uint8_t DISPLAY_COLS = 51;  // 6 pixels per character, text size 1
constexpr uint8_t QSO_INPUT_MAX = 46;
constexpr uint8_t MAX_DIR_ENTRIES = 48;
constexpr uint16_t MAX_VIEW_LINES = 160;
constexpr uint8_t VIEW_LINE_WIDTH = 48;
constexpr uint8_t VIEW_PAGE_LINES = 11;

enum class Page : uint8_t {
  Monitor, Qso1, Qso2, Qso3, Qso4, Qso5,
  Menu, Files, Viewer, Setup, MHeard, Port, About, Editor, Beacon, Transfer, Diagnostics, KeyboardTest, TftTest
};
Page page = Page::Monitor;
uint8_t menuIndex = 0;
uint8_t setupIndex = 0;
bool editingConfig = false;
char editingValue[24] = "";
uint8_t editingLength = 0;
char mycall[16] = "NOCALL-0";
uint16_t txDelayMs = 300;
uint16_t packetBaud = 1200; // 1200 Bell202 or 300 HF AFSK

struct QsoData { char draft[QSO_INPUT_MAX+1]; uint8_t len; };
QsoData qso[5] = {};

struct DirItem { char name[41]; bool isDir; uint32_t size; };
DirItem dirItems[MAX_DIR_ENTRIES];
uint8_t dirCount = 0;
uint8_t dirSelected = 0;
uint8_t dirFirst = 0;
char dirPath[96] = "/GP/TXFILES";
bool sdReady = false;
// SD history preferences; persisted in NVS. No RTC -> records have no wall-clock time.
bool logQsoEnabled = true;
bool logMonitorEnabled = true;
bool restoreHistoryEnabled = true;
bool historyRestoring = false;
uint32_t historyWriteErrors = 0;
void gpLogMonitor(const char *text);
void gpLogQso(uint8_t channel, const char *text);
void gpRestoreHistory();
char openedFile[132] = "";
char viewerTitle[42] = "";
char viewerLines[MAX_VIEW_LINES][VIEW_LINE_WIDTH+1] = {};
uint16_t viewerCount = 0;
uint16_t viewerFirst = 0;
bool viewerTruncated = false;

constexpr uint8_t MONITOR_CAP = 36;
char monitorLines[MONITOR_CAP][DISPLAY_COLS+1] = {};
uint8_t monitorUsed = 0;
char popMessage[64] = "";
unsigned long popExpires = 0;

// ----------------------- PS/2 bit receiver ------------------------------
// PS/2 scan code set 2; falling clock, 11-bit parity checked byte frames.
// Interrupt collects BYTES ONLY. All UI and SD operations are in loop().
volatile uint8_t ps2Raw[128];
volatile uint8_t ps2Write = 0, ps2Read = 0;
volatile uint8_t ps2Bits = 0, ps2Char = 0, ps2Ones = 0;
volatile uint32_t ps2Last = 0;
volatile uint32_t ps2Frames=0, ps2ParityErrors=0, ps2Drops=0;

void IRAM_ATTR ps2ISR() {
  uint32_t now = micros();
  if ((uint32_t)(now-ps2Last) > 2500) ps2Bits = 0;
  ps2Last = now;
  uint8_t bit = (uint8_t)digitalRead(PIN_PS2_DATA);
  if (ps2Bits == 0) {
    if (bit) return;
    ps2Char = 0; ps2Ones = 0; ps2Bits = 1;
  } else if (ps2Bits <= 8) {
    ps2Char |= (uint8_t)(bit << (ps2Bits-1));
    ps2Ones += bit;
    ++ps2Bits;
  } else if (ps2Bits == 9) {
    ps2Ones += bit; ps2Bits = 10;
  } else {
    if (bit && (ps2Ones & 1U)) {
      uint8_t next = (uint8_t)((ps2Write + 1) & 127);
      if (next != ps2Read) { ps2Raw[ps2Write] = ps2Char; ps2Write = next; ps2Frames++; }
      else ps2Drops++;
    } else ps2ParityErrors++;
    ps2Bits = 0;
  }
}

bool pullScan(uint8_t &b) {
  if (ps2Read == ps2Write) return false;
  b = ps2Raw[ps2Read];
  ps2Read = (uint8_t)((ps2Read + 1) & 127);
  return true;
}

// ---------------------- common graphics helpers -------------------------

void drawBootLogo() {
  // The 320x240 RGB565 bitmap resides in FLASH (PROGMEM).
  // Stream one 320-pixel row at a time: just 640 bytes of stack RAM.
  // The Adafruit_SPITFT writePixels API must follow startWrite()
  // and a setAddrWindow() call. writePixel() needs x/y/color,
  // so it cannot be used to stream a pixel with only one argument.
  uint16_t line[OM8AWA_LOGO_W];
  tft.startWrite();
  tft.setAddrWindow(0, 0, OM8AWA_LOGO_W, OM8AWA_LOGO_H);
  for (uint16_t y = 0; y < OM8AWA_LOGO_H; ++y) {
    for (uint16_t x = 0; x < OM8AWA_LOGO_W; ++x) {
      line[x] = pgm_read_word(&OM8AWA_LOGO[(uint32_t)y * OM8AWA_LOGO_W + x]);
    }
    tft.writePixels(line, OM8AWA_LOGO_W, true, false);
  }
  tft.endWrite();
}

void showBootLogo() {
  drawBootLogo();
  delay(3000);
}

void txt(int x, int y, const char *s, uint16_t col, uint8_t scale=1) {
  tft.setTextWrap(false);
  tft.setTextSize(scale);
  tft.setTextColor(col);
  tft.setCursor(x,y);
  tft.print(s);
}

void boxText(int x, int y, int w, int h, uint16_t fill, uint16_t border) {
  tft.fillRect(x,y,w,h,fill);
  tft.drawRect(x,y,w,h,border);
}

void ellipsize(const char *str, char *out, size_t cap, size_t maxChars) {
  if (cap == 0) return;
  size_t len = strlen(str);
  if (len <= maxChars || maxChars < 4) {
    snprintf(out,cap,"%.*s",(int)maxChars,str);
  } else {
    snprintf(out,cap,"%.*s...",(int)(maxChars-3),str);
  }
}

void addMonitor(const char *message) {
  if (monitorUsed >= MONITOR_CAP) {
    memmove(monitorLines[0], monitorLines[1],
            (MONITOR_CAP-1)*sizeof(monitorLines[0]));
    monitorUsed = MONITOR_CAP-1;
  }
  snprintf(monitorLines[monitorUsed],sizeof(monitorLines[0]),"%s",message);
  ++monitorUsed;
  gpLogMonitor(message);
}

void flashMessage(const char *message) {
  snprintf(popMessage,sizeof(popMessage),"%s",message);
  popExpires = millis()+1700;
  tft.fillRect(10,111,300,28,C_GRAY);
  tft.drawRect(10,111,300,28,C_WHITE);
  char shown[52]; ellipsize(popMessage,shown,sizeof(shown),48);
  txt(17,121,shown,C_BLUE);
  Serial.print("GP161: "); Serial.println(popMessage);
}

// v3.4: classic GP161 320x240 chrome inspired by the APPROVED v3.3 previews.
// The status clock shows UPTIME (no RTC is fitted), not fabricated wall time.
void header(uint8_t activeCh=0) {
  tft.fillRect(0,0,320,19,C_BLACK);
  for (uint8_t ch=1; ch<=5; ++ch) {
    int x=(ch-1)*64;
    boxText(x+1,1,62,17,ch==activeCh?C_CYAN:C_RED,C_GRAY);
    char tab[16];snprintf(tab,sizeof(tab),"F%u %u:----",ch,ch);
    txt(x+3,6,tab,ch==activeCh?C_BLACK:C_YELLOW);
  }
  tft.fillRect(0,19,320,13,C_MAGENTA);
  txt(3,22,"GP 1.61 / ESP32",C_WHITE);
  char radio[19];snprintf(radio,sizeof(radio),"AX25 %u",(unsigned)packetBaud);
  txt(165,22,radio,C_YELLOW);
  char uptime[20];uint32_t sec=millis()/1000UL;
  snprintf(uptime,sizeof(uptime),"UP%02lu:%02lu",(unsigned long)((sec/60)%100),(unsigned long)(sec%60));
  txt(263,22,uptime,C_WHITE);
  tft.drawFastHLine(0,31,320,C_WHITE);
}

void status(const char *s) {
  tft.fillRect(0,174,320,14,C_MAGENTA);
  char cut[55];ellipsize(s,cut,sizeof(cut),52);
  txt(3,177,cut,C_WHITE);
}

void prompt(const char *s) {
  tft.fillRect(0,188,320,19,C_LIGHTBLUE);
  char cut[52]; ellipsize(s,cut,sizeof(cut),50);
  txt(4,194,cut,C_WHITE);
}

// Same six classic button cells across ALL windows as in approved v3.3 art.
void footer() {
  static const char* names[]={"MH","EDIT","SETUP","MON","SD","SAVE"};
  tft.fillRect(0,207,320,33,C_BLACK);
  for(uint8_t i=0;i<6;++i){
    int x=i*53;
    tft.fillRect(x+1,211,52,28,C_CYAN);
    tft.drawRect(x+1,211,52,28,C_WHITE);
    tft.fillRect(x+4,213,15,23,C_DARKGRAY);
    tft.drawRect(x+4,213,15,23,C_BLACK);
    // Tiny pixel-art icons (not interactive buttons: PS/2 keyboard controls UI).
    if(i==0){tft.fillRect(x+7,217,9,12,C_YELLOW);txt(x+9,221,"H",C_BLACK);}
    else if(i==1){tft.drawRect(x+7,218,9,13,C_WHITE);tft.drawFastHLine(x+9,221,5,C_WHITE);}
    else if(i==2){tft.drawRect(x+6,218,10,12,C_YELLOW);tft.fillRect(x+9,221,4,6,C_YELLOW);}
    else if(i==3){tft.drawRect(x+7,218,9,13,C_GREEN);tft.fillRect(x+8,225,3,3,C_GREEN);tft.fillRect(x+11,222,3,4,C_GREEN);}
    else {tft.drawRect(x+7,218,9,13,C_WHITE);tft.fillRect(x+9,219,5,5,C_LIGHTBLUE);}
    txt(x+21,221,names[i],C_BLACK);
  }
}

void titleBox(const char *heading) {
  tft.fillRect(0,33,320,19,C_RED);
  txt(5,39,heading,C_WHITE);
}


// ---------------- Radio, sessions, MHEARD and file transfer state ---------
constexpr uint8_t QS_HISTORY=10;
struct Session {
  char peer[12]="";uint8_t state=0; // 0 free, 1 SABM sent, 2 connected, 3 DISC sent
  uint8_t vs=0,vr=0,va=0;uint8_t retries=0;
  uint32_t deadline=0;bool waiting=false;
  gp::Frame pending;
  char history[QS_HISTORY][51]={};uint8_t historyUsed=0;
};
Session sessions[5];
struct Heard {char call[12];uint32_t lastMs;uint16_t frames;};
Heard heard[24]={};uint8_t heardUsed=0;
gp::Deframer decoder;
gp::Frame txQueue[6];uint8_t txHead=0,txTail=0,txUsed=0;
gp::TxBits rfBitstream;
uint32_t rxFrames=0,txFrames=0,failedFcs=0;
char callEntry[16]="";bool enterConnect=false;
char beaconText[64]="GP161 ESP32 TEST";
uint16_t beaconIntervalMin=0;uint32_t beaconNext=0;
char editor[640]="";uint16_t editorLen=0;char editorFile[100]="/GP/MESSAGES/NOTE.TXT";
uint8_t editorFirst=0;
struct Transfer {
  bool sending=false,receiving=false,waitingOK=false;
  uint8_t channel=0;uint32_t total=0,done=0;
  uint16_t crc=0,expectedCrc=0,actualCrc=0;bool expectedValid=false;
  char filename[41]="";char path[132]="";
  char line[140]="";uint8_t lineLen=0;
  File file;
} tr;
uint8_t chosenQso=0;
void drawDiagnostics(); void drawKeyboardTest(); void drawTftTest();
void processDiagnosticSelection(uint8_t selection);
void runOfflineDiagnostics(bool verbose);
void noteScan(uint8_t sc, bool released, bool extended);
void onFrame(const gp::Frame &f);
void radioTick();
void connectionTick();
void transferTick();
void connectChannel(uint8_t channel,const char *peer);
void disconnectChannel(uint8_t channel);
void sendUserLine(uint8_t channel,const char *line);
void startSendFile(uint8_t channel,const char *file);
void receiveBytes(uint8_t channel,const uint8_t *data,uint16_t len);
void drawEditor();void drawBeacon();void drawTransfer();
void handleQsoEnter(uint8_t channel);
void sendFileFromBrowser();
void serialDiagnostics();
void serialKissTick();
bool sendPacket(const char *destination,uint8_t control,bool pid,const uint8_t *bytes,size_t len);
void addHistory(uint8_t ch,const char *text){
  Session &q=sessions[ch];
  if(q.historyUsed>=QS_HISTORY){memmove(q.history[0],q.history[1],(QS_HISTORY-1)*sizeof(q.history[0]));q.historyUsed=QS_HISTORY-1;}
  snprintf(q.history[q.historyUsed++],sizeof(q.history[0]),"%s",text);
  gpLogQso(ch,text);
}

#include "GP161_History.h"
// --------------------------- screen renderers ----------------------------
void drawMonitor() {
  tft.fillScreen(C_BLUE);
  header();
  boxText(0,33,105,18,C_LIGHTBLUE,C_WHITE);
  txt(5,35,"Monitor",C_WHITE,2);
  txt(110,35,"MODE:ALL",C_GRAY);
  char ident[26]; snprintf(ident,sizeof(ident),"MY:%s",mycall);
  txt(110,44,ident,C_YELLOW);
  tft.drawFastHLine(0,51,320,C_WHITE);
  if (!monitorUsed) txt(6,70,"*** Bez zaznamov ***",C_YELLOW);
  uint8_t visible = monitorUsed>12?12:monitorUsed;
  uint8_t first = monitorUsed-visible;
  for (uint8_t n=0;n<visible;n++)
    txt(4,54+n*10,monitorLines[first+n],(n&1)?C_GRAY:C_YELLOW);
  status("RX:-- DCD:-- PTT:OFF   MONITOR");
  prompt("Alt+M:MENU   F1-F5:QSO");
  footer();
}

void drawQSO(uint8_t ch) {
  tft.fillScreen(C_DARKGRAY);
  header(ch);
  char heading[54]; snprintf(heading,sizeof(heading),"CH%u %s",ch,mycall);
  txt(4,38,heading,C_WHITE);
  tft.drawFastHLine(0,51,320,C_WHITE);
  Session &q=sessions[ch-1];
  snprintf(heading,sizeof(heading),"CH%u %-11s %s",ch,q.peer,q.state==2?"CONNECTED":q.state==1?"CONNECTING":q.state==3?"DISCONNECTING":"FREE");
  tft.fillRect(0,34,320,18,C_DARKGRAY);txt(4,38,heading,C_WHITE);
  for(uint8_t i=0;i<q.historyUsed;i++)txt(5,56+i*11,q.history[i],C_GREEN);
  char stateLine[52];snprintf(stateLine,sizeof(stateLine),"V(S):%u V(R):%u Retry:%u RF:%s",q.vs,q.vr,q.retries,RADIO_CONFIGURED?"TEST":"OFF");
  status(stateLine);
  char p[64];
  if(enterConnect)snprintf(p,sizeof(p),"CONNECT CALL: %s_",callEntry);
  else snprintf(p,sizeof(p),"> %s_",qso[ch-1].draft);
  prompt(p);
  footer();
}

static const char *MAIN_ITEMS[] = {
  "MONITOR", "MHEARD", "FILES / SD", "SETUP / KONFIG.",
  "PORT / MODEM", "ABOUT / SYSTEM", "EDITOR", "BEACON", "DIAGNOSTICS"
};
constexpr uint8_t MAIN_N = sizeof(MAIN_ITEMS)/sizeof(MAIN_ITEMS[0]);
void drawMenu() {
  // Pop-up over live MONITOR, identical window geometry to the approved v3.3 preview.
  drawMonitor();
  boxText(43,35,230,163,C_GRAY,C_WHITE);
  tft.fillRect(44,36,228,18,C_RED);
  txt(50,41,"GRAPHIC PACKET 1.61",C_WHITE);
  for(uint8_t i=0;i<MAIN_N;++i){
    const int y=57+i*13;
    if(i==menuIndex)tft.fillRect(48,y-2,216,13,C_BLUE);
    char item[40];snprintf(item,sizeof(item),"%u  %s",(unsigned)(i+1),MAIN_ITEMS[i]);
    txt(55,y,item,i==menuIndex?C_YELLOW:C_BLACK);
  }
  txt(50,181,"Enter: vyber  Esc: navrat",C_BLUE);
}

void drawFiles() {
  tft.fillScreen(C_BLUE);
  header();
  titleBox("FILES / microSD (Enter: open)");
  char path[53]; ellipsize(dirPath,path,sizeof(path),49);
  txt(5,56,path,C_YELLOW);
  if (!sdReady) {
    txt(6,84,"SD karta nie je dostupna.",C_YELLOW);
    txt(6,99,"Skontroluj GPIO13 / 18/19/23",C_WHITE);
    txt(6,116,"Klaves R = opakovat inicial.",C_WHITE);
  } else if (dirCount == 0) {
    txt(6,86,"Adresar je prazdny.",C_YELLOW);
  } else {
    for (uint8_t n=0;n<9;n++) {
      uint8_t ix=dirFirst+n;
      if (ix>=dirCount) break;
      int y=68+n*11;
      if (ix==dirSelected) tft.fillRect(2,y-2,316,11,C_CYAN);
      char shortName[35]; ellipsize(dirItems[ix].name,shortName,sizeof(shortName),33);
      txt(6,y,shortName,ix==dirSelected?C_BLACK:C_WHITE);
      char size[18];
      if(dirItems[ix].isDir) snprintf(size,sizeof(size),"<DIR>");
      else snprintf(size,sizeof(size),"%lu B",(unsigned long)dirItems[ix].size);
      txt(255,y,size,ix==dirSelected?C_BLACK:C_YELLOW);
    }
  }
  status("SD VIEW  Enter:OPEN    S:SEND");
  prompt("BS:parent  T:TX/RX  R:reload");
  footer();
}

void drawViewer() {
  tft.fillScreen(C_BLUE);
  header();
  titleBox("FILE VIEW (ASCII / HEX)");
  char shown[55]; ellipsize(viewerTitle,shown,sizeof(shown),49);
  txt(5,54,shown,C_YELLOW);
  for (uint8_t n=0;n<VIEW_PAGE_LINES;n++) {
    uint16_t ix=viewerFirst+n;
    if(ix>=viewerCount) break;
    txt(4,68+n*9,viewerLines[ix],C_WHITE);
  }
  char stat[54]; snprintf(stat,sizeof(stat),"Line %u/%u%s",viewerCount?(unsigned)viewerFirst+1:0,
             (unsigned)viewerCount,viewerTruncated?" (preview)":"");
  status(stat);
  prompt("Up/Down PgUp/PgDn  Esc:MON");
  footer();
}

static const char *SET_NAMES[]={"MYCALL","TX DELAY","BEACON","PACKET BAUD",
                                 "QSO CHANNELS","MODEM","DISPLAY","QSO LOG",
                                 "MONITOR LOG","RESTORE HISTORY","SAVE SETTINGS"};
constexpr uint8_t SET_N=sizeof(SET_NAMES)/sizeof(SET_NAMES[0]);
void drawSetup() {
  tft.fillScreen(C_BLUE);
  header();
  titleBox("SETUP - GP 1.61 / ESP32");
  boxText(3,55,314,118,C_GRAY,C_WHITE);
  char td[16];snprintf(td,sizeof(td),"%u ms",(unsigned)txDelayMs);
  char bi[16];snprintf(bi,sizeof(bi),"%u min",(unsigned)beaconIntervalMin);
  char baudVal[20];snprintf(baudVal,sizeof(baudVal),"%u / %u",(unsigned)packetBaud,packetBaud==1200?300:1200);
  const char *values[]={mycall,td,bi,baudVal,"5","INTERNAL AFSK",
                         "320x240",logQsoEnabled?"ON":"OFF",
                         logMonitorEnabled?"ON":"OFF",
                         restoreHistoryEnabled?"ON":"OFF","ENTER"};
  constexpr uint8_t visible=8;
  uint8_t first=setupIndex>=visible?setupIndex-visible+1:0;
  if(first>SET_N-visible)first=SET_N-visible;
  for(uint8_t row=0;row<visible;++row){
    uint8_t i=first+row;
    int y=59+row*14;
    if(i==setupIndex)tft.fillRect(6,y-2,307,14,C_BLUE);
    txt(10,y,SET_NAMES[i],i==setupIndex?C_YELLOW:C_BLACK);
    if(editingConfig && i==setupIndex){
      char draft[25];snprintf(draft,sizeof(draft),"%s_",editingValue);
      txt(172,y,draft,C_WHITE);
    }else{
      txt(172,y,values[i],i==setupIndex?C_WHITE:C_BLUE);
    }
  }
  if(editingConfig)status("EDIT  Enter:Save  Esc:Cancel");
  else status(first==0?"UP/DN: Select  Enter: Edit  1/2":"UP/DN: Select  Enter: Edit  2/2");
  prompt(sdReady?"ESC Monitor   Alt+M Menu   SD:OK":"ESC Monitor   Alt+M Menu   SD:--");
  footer();
}

void drawPort() {
  tft.fillScreen(C_BLUE); header(); titleBox("PORT / MODEM STATUS");
  txt(6,61,"MODEM: INTERNAL AFSK 1200/300",C_YELLOW);
  txt(6,77,radioArmed()?"TX ARMED (5min max)":RADIO_CONFIGURED?"RX ONLY (TX LOCKED)":"RADIO: DISABLED (SAFE)",C_WHITE);
  txt(6,94,"AUDIO IN : GPIO34 (ADC1)",C_WHITE);
  txt(6,106,"AUDIO OUT: GPIO25 (DAC)",C_WHITE);
  txt(6,118,"PTT       GPIO27 -> BC547",C_WHITE);
  char counters[48];snprintf(counters,sizeof(counters),"RX:%lu TX:%lu CRC BAD:%lu",(unsigned long)rxFrames,(unsigned long)txFrames,(unsigned long)decoder.badFcs);
  txt(6,140,counters,C_YELLOW);
  char rfline[64];snprintf(rfline,sizeof(rfline),"Edges:%lu Loss:%lu ADC:%d",(unsigned long)bay::edges(),
     (unsigned long)bay::overflow(),bay::auxInput());
  txt(6,155,rfline,C_YELLOW);
  status(radioArmed()?"TX ARMED - USB :DISARM":"TX LOCKED - :ARM after audio/PTT check");
  prompt("USB :AUDIOSTATUS :ARM :PTTTEST"); footer();
}

void drawMHeard() {
  tft.fillScreen(C_GRAY); header();
  titleBox("MHEARD");
  if(!heardUsed)txt(7,65,"No stations received yet",C_BLACK);
  for(uint8_t i=0;i<heardUsed && i<9;i++) {
    char h[52];snprintf(h,sizeof(h),"%-12s %4u   %lus",heard[i].call,heard[i].frames,(unsigned long)((millis()-heard[i].lastMs)/1000));
    txt(7,60+i*12,h,C_BLACK);
  }
  char hs[36];snprintf(hs,sizeof(hs),"MHEARD: %u calls",heardUsed);
  status(hs);prompt("Esc:MON");footer();
}

void drawAbout() {
  tft.fillScreen(C_BLUE);header();titleBox("ABOUT - GP161 ESP32 v3.2");
  txt(7,62,"Reference: GP 1.61b (MS-DOS)",C_YELLOW);
  txt(7,78,"ESP32-WROOM-32 / 30 pin",C_WHITE);
  txt(7,90,"ILI9341 SPI 320x240",C_WHITE);
  txt(7,102,"Keyboard: PS/2 set 2",C_WHITE);
  txt(7,114,"SD: SPI shared",C_WHITE);
  txt(7,133,"AX.25 codec / AutoBIN experimental",C_YELLOW);
  txt(7,145,"Not a proven GP 1.61 replica",C_YELLOW);
  status("AFSK RX active, TX locked by default");prompt("Esc:MON");footer();
}

void paint() {
  switch(page) {
    case Page::Monitor:drawMonitor();break;
    case Page::Qso1: case Page::Qso2: case Page::Qso3:
    case Page::Qso4: case Page::Qso5:
      drawQSO((uint8_t)page-(uint8_t)Page::Qso1+1);break;
    case Page::Menu:drawMenu();break;
    case Page::Files:drawFiles();break;
    case Page::Viewer:drawViewer();break;
    case Page::Setup:drawSetup();break;
    case Page::MHeard:drawMHeard();break;
    case Page::Port:drawPort();break;
    case Page::About:drawAbout();break;
    case Page::Editor:drawEditor();break;
    case Page::Beacon:drawBeacon();break;
    case Page::Transfer:drawTransfer();break;
    case Page::Diagnostics:drawDiagnostics();break;
    case Page::KeyboardTest:drawKeyboardTest();break;
    case Page::TftTest:drawTftTest();break;
  }
}

// --------------------------- SD file functions ---------------------------
void refreshDirectory() {
  dirCount=0;dirSelected=0;dirFirst=0;
  if(!sdReady)return;
  File directory=SD.open(dirPath);
  if(!directory || !directory.isDirectory()) {
    if (directory) directory.close();
    return;
  }
  while(dirCount<MAX_DIR_ENTRIES) {
    File entry=directory.openNextFile();
    if(!entry)break;
    const char *full=entry.name();
    const char *base=strrchr(full,'/');
    base=base?base+1:full;
    if(base[0] && strcmp(base,".")!=0 && strcmp(base,"..")!=0) {
      snprintf(dirItems[dirCount].name,sizeof(dirItems[dirCount].name),"%s",base);
      dirItems[dirCount].isDir=entry.isDirectory();
      dirItems[dirCount].size=(uint32_t)entry.size();
      ++dirCount;
    }
    entry.close();
  }
  directory.close();
}

void initSD() {
  // TFT and SD use the same VSPI bus. CS must be independent.
  digitalWrite(PIN_TFT_CS,HIGH);
  digitalWrite(PIN_SD_CS,HIGH);
  if(sdReady) SD.end();
  sdReady=SD.begin(PIN_SD_CS,SPI,4000000);
  if(sdReady) {
    if(!SD.exists("/GP"))SD.mkdir("/GP");
    if(!SD.exists("/GP/TXFILES"))SD.mkdir("/GP/TXFILES");
    if(!SD.exists("/GP/RXFILES"))SD.mkdir("/GP/RXFILES");
    if(!SD.exists("/GP/MESSAGES"))SD.mkdir("/GP/MESSAGES");
    if(!SD.exists("/GP/LOGS"))SD.mkdir("/GP/LOGS");
    addMonitor("SYS> SD initialized (SPI shared)");
  } else {
    addMonitor("SYS> SD not available");
  }
  refreshDirectory();
}

void filesEnter() {
  if(!sdReady || dirSelected>=dirCount)return;
  DirItem &item=dirItems[dirSelected];
  char full[132]; snprintf(full,sizeof(full),"%s/%s",dirPath,item.name);
  if (item.isDir) {
    if(strlen(full) >= sizeof(dirPath)) {flashMessage("Path too long");return;}
    snprintf(dirPath,sizeof(dirPath),"%s",full);
    refreshDirectory();paint();
  } else {
    snprintf(openedFile,sizeof(openedFile),"%s",full);
    snprintf(viewerTitle,sizeof(viewerTitle),"%s",item.name);
    File file=SD.open(openedFile,FILE_READ);
    if(!file){flashMessage("Cannot open file");return;}
    viewerCount=0;viewerFirst=0;viewerTruncated=false;
    const char *dot=strrchr(item.name,'.');
    bool isText=false;
    if(dot) {
      String extension=String(dot);extension.toLowerCase();
      isText=extension==".txt"||extension==".log"||extension==".ini"||
             extension==".asc"||extension==".nfo"||extension==".gpi";
    }
    if(isText) {
      char row[VIEW_LINE_WIDTH+1];uint8_t n=0;
      while(file.available() && viewerCount<MAX_VIEW_LINES) {
        int x=file.read();if(x<0)break;
        if(x=='\r')continue;
        if(x=='\n' || n==VIEW_LINE_WIDTH) {
          row[n]='\0';snprintf(viewerLines[viewerCount++],VIEW_LINE_WIDTH+1,"%s",row);
          n=0;
          if(x=='\n')continue;
        }
        row[n++]=(x>=32 && x<=126)?(char)x:'.';
      }
      if(n && viewerCount<MAX_VIEW_LINES) {
        row[n]='\0';snprintf(viewerLines[viewerCount++],VIEW_LINE_WIDTH+1,"%s",row);
      }
    } else { // HEX preview, 8 bytes per row, readable on 320px TFT
      uint32_t offset=0;
      while(file.available() && viewerCount<MAX_VIEW_LINES) {
        uint8_t bytes[8];int count=0;
        while(count<8 && file.available()) {
          int x=file.read(); if(x<0) break;
          bytes[count++]=(uint8_t)x;
        }
        if(!count)break;
        char line[VIEW_LINE_WIDTH+1];
        int n=snprintf(line,sizeof(line),"%06lX ",(unsigned long)offset);
        for(int i=0;i<8;i++)n+=snprintf(line+n,sizeof(line)-n,i<count?"%02X ":"   ",i<count?bytes[i]:0);
        if(n<(int)sizeof(line)-2)line[n++]='|';
        for(int i=0;i<count && n<(int)sizeof(line)-2;i++)
          line[n++]=(bytes[i]>=32 && bytes[i]<127)?(char)bytes[i]:'.';
        line[n]='\0';
        snprintf(viewerLines[viewerCount++],VIEW_LINE_WIDTH+1,"%s",line);
        offset+=(uint32_t)count;
      }
    }
    viewerTruncated=file.available()>0;
    file.close();
    page=Page::Viewer;paint();
  }
}

void filesParent() {
  if(strcmp(dirPath,"/")==0) return;
  char *slash=strrchr(dirPath,'/');
  if(!slash)return;
  if(slash==dirPath)dirPath[1]='\0';
  else *slash='\0';
  refreshDirectory();paint();
}

// ----------------------------- configuration -----------------------------
void loadPrefs() {
  prefs.begin("gp161",false);
  String saved=prefs.getString("mycall","NOCALL-0");
  saved.trim(); saved.toUpperCase();
  if(saved.length()>0 && saved.length()<sizeof(mycall))
    saved.toCharArray(mycall,sizeof(mycall));
  txDelayMs=prefs.getUShort("txdelay",300);
  beaconIntervalMin=prefs.getUShort("beacon",0);
  packetBaud=prefs.getUShort("baud",1200);
  if(packetBaud!=300 && packetBaud!=1200)packetBaud=1200;
  if(beaconIntervalMin>1440)beaconIntervalMin=0;
  if(txDelayMs<50 || txDelayMs>3000)txDelayMs=300;
  logQsoEnabled=prefs.getUChar("logqso",1)!=0;
  logMonitorEnabled=prefs.getUChar("logmon",1)!=0;
  restoreHistoryEnabled=prefs.getUChar("restore",1)!=0;
  beaconNext=millis()+beaconIntervalMin*60000UL;
}

void savePrefs() {
  prefs.putString("mycall",mycall);
  prefs.putUShort("txdelay",txDelayMs);
  prefs.putUShort("beacon",beaconIntervalMin);
  prefs.putUShort("baud",packetBaud);
  prefs.putUChar("logqso",logQsoEnabled?1:0);
  prefs.putUChar("logmon",logMonitorEnabled?1:0);
  prefs.putUChar("restore",restoreHistoryEnabled?1:0);
  flashMessage("Settings saved in ESP32 NVS");
}

void startEdit() {
  if(setupIndex>2)return;
  editingConfig=true;
  snprintf(editingValue,sizeof(editingValue),"%s",setupIndex==0?mycall:"");
  if(setupIndex==1)snprintf(editingValue,sizeof(editingValue),"%u",(unsigned)txDelayMs);
  if(setupIndex==2)snprintf(editingValue,sizeof(editingValue),"%u",(unsigned)beaconIntervalMin);
  editingLength=(uint8_t)strlen(editingValue);
  paint();
}

void finishEdit() {
  if(setupIndex==0) {
    if(editingLength==0){flashMessage("MyCall cannot be empty");return;}
    for(uint8_t i=0;i<editingLength;i++) {
      char c=editingValue[i];
      if(!(isalnum((unsigned char)c)||c=='-')) {
        flashMessage("MyCall: use A-Z 0-9 -");return;
      }
    }
    snprintf(mycall,sizeof(mycall),"%s",editingValue);
  } else if(setupIndex==1) {
    long value=atol(editingValue);
    if(value<50 || value>3000){flashMessage("TX Delay range: 50-3000");return;}
    txDelayMs=(uint16_t)value;
  } else if(setupIndex==2) {
    long value=atol(editingValue);
    if(value<0 || value>1440){flashMessage("Beacon interval: 0-1440 min");return;}
    beaconIntervalMin=(uint16_t)value;beaconNext=millis()+beaconIntervalMin*60000UL;
  }
  editingConfig=false;
  prefs.putString("mycall",mycall);
  prefs.putUShort("txdelay",txDelayMs);
  prefs.putUShort("beacon",beaconIntervalMin);
  prefs.putUShort("baud",packetBaud);
  prefs.putUChar("logqso",logQsoEnabled?1:0);
  prefs.putUChar("logmon",logMonitorEnabled?1:0);
  prefs.putUChar("restore",restoreHistoryEnabled?1:0);
  paint();
}

void chooseMenu() {
  switch(menuIndex) {
    case 0:page=Page::Monitor;break;
    case 1:page=Page::MHeard;break;
    case 2:page=Page::Files;refreshDirectory();break;
    case 3:page=Page::Setup;break;
    case 4:page=Page::Port;break;
    case 5:page=Page::About;break;
    case 6:page=Page::Editor;break;
    case 7:page=Page::Beacon;break;
    case 8:page=Page::Diagnostics;break;
  }
  paint();

}

// ------------------------ keyboard scancode set 2 ------------------------
// Supports US ASCII layout; Slovak keyboard national layouts need key map.
char ps2ToAscii(uint8_t s, bool shift, bool caps) {
  char letter=0;
  switch(s) {
    case 0x1C:letter='a';break; case 0x32:letter='b';break;
    case 0x21:letter='c';break; case 0x23:letter='d';break;
    case 0x24:letter='e';break; case 0x2B:letter='f';break;
    case 0x34:letter='g';break; case 0x33:letter='h';break;
    case 0x43:letter='i';break; case 0x3B:letter='j';break;
    case 0x42:letter='k';break; case 0x4B:letter='l';break;
    case 0x3A:letter='m';break; case 0x31:letter='n';break;
    case 0x44:letter='o';break; case 0x4D:letter='p';break;
    case 0x15:letter='q';break; case 0x2D:letter='r';break;
    case 0x1B:letter='s';break; case 0x2C:letter='t';break;
    case 0x3C:letter='u';break; case 0x2A:letter='v';break;
    case 0x1D:letter='w';break; case 0x22:letter='x';break;
    case 0x35:letter='y';break; case 0x1A:letter='z';break;
  }
  if(letter)return (shift^caps)?(char)toupper(letter):letter;
  switch(s) {
    case 0x16:return shift?'!':'1'; case 0x1E:return shift?'@':'2';
    case 0x26:return shift?'#':'3'; case 0x25:return shift?'$':'4';
    case 0x2E:return shift?'%':'5'; case 0x36:return shift?'^':'6';
    case 0x3D:return shift?'&':'7'; case 0x3E:return shift?'*':'8';
    case 0x46:return shift?'(':'9'; case 0x45:return shift?')':'0';
    case 0x29:return ' '; case 0x4E:return shift?'_':'-';
    case 0x55:return shift?'+':'='; case 0x54:return shift?'{':'[';
    case 0x5B:return shift?'}':']'; case 0x5D:return shift?'|':'\\';
    case 0x4C:return shift?':':';'; case 0x52:return shift?'\"':'\'';
    case 0x41:return shift?'<':','; case 0x49:return shift?'>':'.';
    case 0x4A:return shift?'?':'/'; case 0x0E:return shift?'~':'`';
    default:return 0;
  }
}

bool keyBreak=false,keyExtended=false,keyAlt=false,keyShift=false,keyCaps=false;
void keyDown(uint8_t code, bool ext) {
  if(page==Page::TftTest) {
    if(code==0x76){page=Page::Diagnostics;paint();return;}
    extern uint8_t testScreenStage;
    testScreenStage=(testScreenStage+1)%4;drawTftTest();return;
  }
  if(page==Page::KeyboardTest) {
    if(code==0x76){page=Page::Diagnostics;paint();return;}
    drawKeyboardTest();return;
  }
  if(page==Page::Diagnostics) {
    if(code==0x76){page=Page::Monitor;paint();return;}
    const uint8_t digitCodes[]={0x16,0x1E,0x26,0x25,0x2E,0x36,0x3D,0x3E};
    for(uint8_t j=0;j<8;j++)if(code==digitCodes[j]){
      processDiagnosticSelection(j);return;
    }
    return;
  }
  if(code==0x76){ // ESC returns to monitor; active editing cancelled
    if(enterConnect){enterConnect=false;paint();}
    else if(editingConfig){editingConfig=false;paint();}
    else {page=Page::Monitor;paint();}
    return;
  }
  uint8_t f=0;
  switch(code) {
    case 0x05:f=1;break;case 0x06:f=2;break;case 0x04:f=3;break;
    case 0x0C:f=4;break;case 0x03:f=5;break;
  }
  if(f) { chosenQso=f-1;editingConfig=false;page=(Page)((uint8_t)Page::Qso1+f-1);paint();return; }
  if(keyAlt) {
    if(code==0x3A) {menuIndex=0;editingConfig=false;page=Page::Menu;paint();return;} // Alt+M
    if(code==0x2B) {editingConfig=false;page=Page::Files;refreshDirectory();paint();return;} // Alt+F
    if(code==0x24) {editingConfig=false;page=Page::Editor;paint();return;} // Alt+E
    if(code==0x3C) {editingConfig=false;page=Page::Setup;paint();return;} // Alt+U
    if(code==0x33) {editingConfig=false;page=Page::MHeard;paint();return;} // Alt+H
  }
  if(page==Page::Menu) {
    if(ext && code==0x75)menuIndex=(menuIndex+MAIN_N-1)%MAIN_N;
    else if(ext && code==0x72)menuIndex=(menuIndex+1)%MAIN_N;
    else if(code==0x5A){chooseMenu();return;}
    else if(!ext) {
      const uint8_t dig[]={0x16,0x1E,0x26,0x25,0x2E,0x36,0x3D,0x3E,0x46};
      for(uint8_t i=0;i<MAIN_N;i++)if(code==dig[i]) {menuIndex=i;chooseMenu();return;}
    }
    paint();return;
  }
  if(page==Page::Files) {
    if(!ext && code==0x1B) {sendFileFromBrowser();return;} // S for SEND
    if(!ext && (code==0x2D || code==0x2C)) { // 'R' refresh, 'T' toggle common dirs
      if(code==0x2D)initSD();
      else {
        snprintf(dirPath,sizeof(dirPath),"%s",strcmp(dirPath,"/GP/TXFILES")==0?"/GP/RXFILES":"/GP/TXFILES");
        refreshDirectory();
      }
      paint();return;
    }
    if(ext && code==0x75 && dirSelected>0) --dirSelected;
    else if(ext && code==0x72 && dirSelected+1<dirCount) ++dirSelected;
    else if(ext && code==0x7D)dirSelected=(dirSelected>9)?dirSelected-9:0;
    else if(ext && code==0x7A && dirCount>0)dirSelected=(uint8_t)min((int)dirCount-1,(int)dirSelected+9);
    else if(code==0x5A) {filesEnter();return;}
    else if(code==0x66) {filesParent();return;}
    if(dirSelected<dirFirst)dirFirst=dirSelected;
    if(dirSelected>=dirFirst+9)dirFirst=dirSelected-8;
    paint();return;
  }
  if(page==Page::Viewer) {
    if(ext && code==0x75 && viewerFirst>0)--viewerFirst;
    if(ext && code==0x72 && viewerFirst+VIEW_PAGE_LINES<viewerCount)++viewerFirst;
    if(ext && code==0x7D)viewerFirst=viewerFirst>VIEW_PAGE_LINES?viewerFirst-VIEW_PAGE_LINES:0;
    if(ext && code==0x7A)viewerFirst=(uint16_t)min((int)max(0,(int)viewerCount-(int)VIEW_PAGE_LINES),(int)viewerFirst+(int)VIEW_PAGE_LINES);
    if(code==0x66){page=Page::Files;paint();return;}
    paint();return;
  }
  if(page==Page::Setup) {
    if(editingConfig) {
      if(code==0x5A){finishEdit();return;}
      if(code==0x66){if(editingLength)editingValue[--editingLength]='\0';paint();return;}
      char c=ps2ToAscii(code,keyShift,keyCaps);
      if(c && editingLength<sizeof(editingValue)-1 && editingLength<(setupIndex==0?15:5)) {
        editingValue[editingLength++]=(char)toupper((unsigned char)c);
        editingValue[editingLength]='\0';paint();
      }
    } else {
      if(ext && code==0x75)setupIndex=(setupIndex+SET_N-1)%SET_N;
      else if(ext && code==0x72)setupIndex=(setupIndex+1)%SET_N;
      else if(code==0x5A) {
        if(setupIndex<=2)startEdit();
        else if(setupIndex==3){
          if(!bay::busy()){packetBaud=packetBaud==1200?300:1200;
            bay::setBaud(packetBaud);decoder=gp::Deframer();
            savePrefs();flashMessage("AFSK speed changed");}
          else flashMessage("Wait until TX ends");
        }
        else if(setupIndex==7){logQsoEnabled=!logQsoEnabled;savePrefs();}
        else if(setupIndex==8){logMonitorEnabled=!logMonitorEnabled;savePrefs();}
        else if(setupIndex==9){restoreHistoryEnabled=!restoreHistoryEnabled;savePrefs();}
        else if(setupIndex==10)savePrefs();
        else flashMessage("Fixed / planned parameter");
        return;
      }
      paint();
    }
    return;
  }
  if(page>=Page::Qso1 && page<=Page::Qso5) {
    uint8_t ch=(uint8_t)page-(uint8_t)Page::Qso1;
    QsoData &q=qso[ch];
    if(enterConnect){
      if(code==0x5A){enterConnect=false;connectChannel(ch,callEntry);paint();return;}
      if(code==0x66){size_t n=strlen(callEntry);if(n)callEntry[n-1]=0;paint();return;}
      char typed=ps2ToAscii(code,keyShift,keyCaps);size_t n=strlen(callEntry);
      if(typed && n<sizeof(callEntry)-1){callEntry[n]=(char)toupper((unsigned char)typed);callEntry[n+1]=0;paint();}return;
    }
    if(keyAlt && code==0x21){enterConnect=true;callEntry[0]=0;paint();return;}
    if(keyAlt && code==0x23){disconnectChannel(ch);paint();return;}
    if(code==0x5A) {handleQsoEnter(ch);return;}
    if(code==0x66) {if(q.len)q.draft[--q.len]='\0';paint();return;}
    char c=ps2ToAscii(code,keyShift,keyCaps);
    if(c && q.len<QSO_INPUT_MAX){q.draft[q.len++]=c;q.draft[q.len]='\0';paint();}
    return;
  }
  if(page==Page::Editor){
    if(keyAlt && code==0x1B){
      if(sdReady){if(SD.exists(editorFile))SD.remove(editorFile);File f=SD.open(editorFile,FILE_WRITE);if(f){f.print(editor);f.close();flashMessage("Editor saved on SD");}}
      else {flashMessage("No SD for editor");}
      return;
    }
    if(code==0x66){if(editorLen)editor[--editorLen]=0;paint();return;}
    if(code==0x5A){if(editorLen<sizeof(editor)-2){editor[editorLen++]='\n';editor[editorLen]=0;}paint();return;}
    char ch=ps2ToAscii(code,keyShift,keyCaps);
    if(ch && editorLen<sizeof(editor)-2){editor[editorLen++]=ch;editor[editorLen]=0;paint();}return;
  }
  if(page==Page::Beacon){
    if(code==0x5A){sendPacket("CQ",0x03,true,(const uint8_t*)beaconText,strlen(beaconText));paint();return;}
    if(code==0x66){size_t n=strlen(beaconText);if(n)beaconText[n-1]=0;paint();return;}
    char ch=ps2ToAscii(code,keyShift,keyCaps);size_t n=strlen(beaconText);
    if(ch && n<sizeof(beaconText)-1){beaconText[n]=ch;beaconText[n+1]=0;paint();}return;
  }
}

void handleScan(uint8_t b) {
  if(b==0xE0){keyExtended=true;return;}
  if(b==0xF0){keyBreak=true;return;}
  bool wasBreak=keyBreak;bool wasExtended=keyExtended;
  keyBreak=false;keyExtended=false;
  if(b==0x11){keyAlt=!wasBreak;return;} // left/right Alt
  if(b==0x12 || b==0x59){keyShift=!wasBreak;return;}
  if(b==0x58 && !wasBreak){keyCaps=!keyCaps;return;}
  noteScan(b,wasBreak,wasExtended);
  if(wasBreak)return;
  keyDown(b,wasExtended);
}

#include "GP161_Engine.h"
#include "GP161_Diagnostics.h"

// ------------------------------- setup/loop ------------------------------
void setup() {
  Serial.begin(115200);
  // RX is started AFTER splash and display/SD initialization.
  // PTT is initialized LOW before the audio worker starts.

  pinMode(PIN_TFT_CS,OUTPUT);digitalWrite(PIN_TFT_CS,HIGH);
  pinMode(PIN_SD_CS,OUTPUT);digitalWrite(PIN_SD_CS,HIGH);
  SPI.begin(PIN_TFT_SCK,PIN_TFT_MISO,PIN_TFT_MOSI);
  tft.begin();tft.setRotation(1);tft.setTextWrap(false);

  C_BLACK=tft.color565(0,0,0);C_BLUE=tft.color565(0,0,170);
  C_RED=tft.color565(170,0,0);C_MAGENTA=tft.color565(170,0,170);
  C_CYAN=tft.color565(0,170,170);C_WHITE=tft.color565(255,255,255);
  C_GRAY=tft.color565(170,170,170);C_DARKGRAY=tft.color565(85,85,85);
  C_LIGHTBLUE=tft.color565(85,85,255);C_LIGHTCYAN=tft.color565(85,255,255);
  C_GREEN=tft.color565(85,255,85);C_YELLOW=tft.color565(255,255,85);

  showBootLogo();

  loadPrefs();
  initSD();
  if(sdReady && restoreHistoryEnabled)gpRestoreHistory();
  addMonitor("SYS> GP 1.61 ESP32 v3.4 CLASSIC UI TEST");
  addMonitor("SYS> Internal AFSK RX 1200/300; TX locked");
  addMonitor("SYS> AX25/HDLC CRC core ready");
  bay::setBaud(packetBaud);
  bay::begin();
  // Keep boot/status entries in the SD log, but start the live display empty.
  memset(monitorLines,0,sizeof(monitorLines));
  monitorUsed=0;
  page=Page::Monitor;paint();

  // PS/2: use a TRUE bidirectional open-drain compatible level shifter.
  pinMode(PIN_PS2_DATA,INPUT_PULLUP);
  pinMode(PIN_PS2_CLOCK,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_PS2_CLOCK),ps2ISR,FALLING);
  Serial.println("GP161 v3.4: F1-F5 QSO; Alt+M Menu; ESC Monitor");
  Serial.println("USB: :TESTALL :SDTEST :FILETEST :AUDIOSTATUS :ARM :DISARM :PTTTEST :BEACONTEST :HELP");
}

void loop() {
  uint8_t b;
  uint8_t maxProcess=24;
  while(maxProcess-- && pullScan(b))handleScan(b);
  if(radioArmedUntil && (int32_t)(millis()-radioArmedUntil)>=0){
    radioArmedUntil=0;bay::emergencyStop();txUsed=0;txHead=txTail=0;
    addMonitor("SYS> Safety timeout: TX disarmed");Serial.println("RF DISARMED (timeout)");
    if(page==Page::Port)paint();
  }
  radioTick();serialKissTick();serialDiagnostics();connectionTick();transferTick();
  if(beaconIntervalMin && (int32_t)(millis()-beaconNext)>=0){
    beaconNext=millis()+beaconIntervalMin*60000UL;
    sendPacket("CQ",0x03,true,(const uint8_t*)beaconText,strlen(beaconText));
  }
  if(popExpires && (int32_t)(millis()-popExpires)>=0){popExpires=0;paint();}
  if(page==Page::KeyboardTest){static uint32_t lastRedraw=0;if(millis()-lastRedraw>400){lastRedraw=millis();drawKeyboardTest();}}
  delay(1);
}
