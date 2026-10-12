#ifndef GP161_HISTORY_H
#define GP161_HISTORY_H
// GP161-ESP32 v3.2 - persistence on microSD (FAT filesystem).
// The live windows show 10 QSO / 36 monitor lines; disk logs are longer.
// Logging is synchronous on the main loop (never from a PS/2 ISR).
// A line is appended and closed after each event so data survives power cycles.
// Each active log is capped at 512 KiB; previous generation in *.OLD.
// No real-time clock: entries have no trustworthy absolute date/time.

static constexpr uint32_t GP_LOG_MAX_BYTES = 512UL*1024UL;
static constexpr uint32_t GP_RESTORE_BYTES = 8192;

static void gpLogPath(int ch, char *out, size_t cap){
  if(ch<0)snprintf(out,cap,"/GP/LOGS/MONITOR.LOG");
  else snprintf(out,cap,"/GP/LOGS/CH%d.LOG",ch+1);
}
static void gpWriteLog(int ch, const char *text){
  if(!sdReady || historyRestoring || !text || !text[0])return;
  char path[40];gpLogPath(ch,path,sizeof(path));
  // Rotation: preserve previous log, never let the active log grow without bound.
  File probe=SD.open(path,FILE_READ);
  if(probe){
    bool rotate=probe.size()>=GP_LOG_MAX_BYTES;
    probe.close();
    if(rotate){
      char oldPath[44];snprintf(oldPath,sizeof(oldPath),"%s.OLD",path);
      if(SD.exists(oldPath))SD.remove(oldPath);
      if(!SD.rename(path,oldPath)){historyWriteErrors++;return;}
    }
  }
  File f=SD.open(path,FILE_APPEND);
  if(!f){historyWriteErrors++;return;}
  // Text is sanitized to avoid fake log rows and data-file corruption.
  char safe[55];size_t i=0;
  while(text[i] && i<sizeof(safe)-1){
    char c=text[i];safe[i]=(c=='\n'||c=='\r')?' ':c;i++;
  }
  safe[i]=0;
  if(f.print(safe)==0 || f.print("\n")==0)historyWriteErrors++;
  f.flush();f.close();
}
inline void gpLogMonitor(const char *text){if(logMonitorEnabled)gpWriteLog(-1,text);}
inline void gpLogQso(uint8_t ch,const char *text){if(ch<5 && logQsoEnabled)gpWriteLog(ch,text);}

static void gpRestoreOne(int ch){
  char path[40];gpLogPath(ch,path,sizeof(path));
  File f=SD.open(path,FILE_READ);
  if(!f)return;
  uint32_t length=(uint32_t)f.size();
  // Seeking near the end avoids scanning large archived histories at startup.
  uint32_t start=length>GP_RESTORE_BYTES?length-GP_RESTORE_BYTES:0;
  if(!f.seek(start)){f.close();return;}
  bool skipPartial=start!=0;
  char line[55];size_t n=0;
  while(f.available()){
    int r=f.read();if(r<0)break;
    char c=(char)r;
    if(skipPartial){if(c=='\n')skipPartial=false;continue;}
    if(c=='\r')continue;
    if(c=='\n'){
      if(n){
        line[n]=0;
        if(ch<0)addMonitor(line);
        else addHistory((uint8_t)ch,line);
        n=0;
      }
    }else if(n<sizeof(line)-1)line[n++]=c;
  }
  // Unterminated tail may be the result of an interrupted SD write.
  // Intentionally ignore it to avoid restoring incomplete messages.
  f.close();
}
inline void gpRestoreHistory(){
  if(!sdReady || !restoreHistoryEnabled)return;
  historyRestoring=true;
  // SD bootstrap status may be shown in RAM but is replaced by stored history.
  monitorUsed=0;
  gpRestoreOne(-1);
  for(int ch=0;ch<5;ch++){
    sessions[ch].historyUsed=0;
    // Intentionally DO NOT restore peers or connected state.
    gpRestoreOne(ch);
  }
  historyRestoring=false;
}

#endif // GP161_HISTORY_H
