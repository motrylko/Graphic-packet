GP161-ESP32 v3.4 CLASSIC MENUS / INTERNAL AFSK / HISTORY - README.txt
================================================
Datum: 10.10.2026 | Slovensky | UTF-8

REFERENCIA
----------
Graphic Packet 1.61b (povodny MS-DOS program), NIE GP 1.63.
Ciel: samostatny terminal ESP32-WROOM-32 DevKit 30-pin + ILI9341 + PS/2 + microSD.
Modem: PRVI EXPERIMENTALNY INTERNÝ SOFTWARE AFSK bez BayCom TCM3105.

UPOZORNENIE K TESTOVANIU
-----------------------
- RX/TX AFSK algoritmus bol testovany iba na generovanych digitalnych audio vzorkach.
- Testy nepreukazuju skutocnu kvalitu RX/TX ani frekvencnu stabilitu ADC/DAC.
- Audio modem pouziva I2S ADC/DAC DMA s buffermi 96 vzoriek; AFSK DSP bezi
  v blokovacej ulohe FreeRTOS, nie v esp_timer callbacku. DMA/backend a fyzicke
  casovanie este treba overit na doske. WiFi/BT nespustat, vysielanie a QSO su EXPERIMENTALNE.
- Realna funkcnost AX.25 QSO a AutoBIN s povodnym GP 1.61 nie je zatial overena.
- Pri zapnuti PTT=LOW (radio neprinuti vysielat); TX sa aktivuje AZ cez USB :ARM.
  Po 5 minutach sa TX zase zakaze. Vypnutie: :DISARM.
- Pocitaj s dalsimi iteraciami oprav po fyzickych meraniach na osciloskope.

1. INSTALACIA ARDUINO IDE
------------------------
Kompilacia bola overena s Arduino CLI, ESP32 core 3.3.12 a kniznicami nizsie;
fyzicky test na ESP32 a radiu este nie je vykonany:
  Boards Manager: esp32 by Espressif Systems 3.3.12
  Library Manager: Adafruit GFX Library 1.12.6
                   Adafruit ILI9341 1.6.4
                   Adafruit BusIO 1.17.4
  Vstavane s ESP32 core: SPI, SD, FS, Preferences, I2S legacy driver, FreeRTOS.
  Kompilator moze vypisat upozornenia o zastarani legacy I2S API; pouziva sa
  kvoli internemu ADC/DAC povodneho ESP32.
  PS/2: vlastny dekoder v .ino, EXTERNA KNIŽNICA PS/2 NIE JE POTREBNA.
Nastavenie Board: ESP32 Dev Module, Flash Frequency 80 MHz,
  Upload Speed 115200 (pocas prvych testov), Flash Mode DIO,
  Partition Scheme: Default (zvysit ak bude potrebne).
Vsetky subory GP161_ESP32_v34.ino a GP161_*.h su v koreni projektu.
V Arduino IDE ich umiestni spolu do priecinka GP161_ESP32_v34/ a otvor
GP161_ESP32_v34/GP161_ESP32_v34.ino. Nekopiruj dalsie stare .h do podpriecinka;
Arduino musi pouzit jednu zhodnu sadu zdrojov vedla .ino.

2. PINY ESP32-WROOM-32 (30 PIN)
--------------------------------
  ILI9341: VCC -> 3V3 pre 3.3V modul, GND -> GND
    CLK/SCK GPIO18, MOSI GPIO23, MISO GPIO19 (volitelne pre TFT),
    CS GPIO14, DC GPIO21, RESET GPIO22.
  microSD SPI (3,3V modul): VCC 3V3, GND GND,
    CLK GPIO18, MOSI GPIO23, MISO GPIO19, CS GPIO13.
  PS/2 keyboard: DATA GPIO32, CLOCK GPIO33 cez obojsmerny prevod 5V/3.3V,
    PS/2 VCC 5V, PS/2 GND spolocna zem.
  RADIO AUDIO IN (ADC1): GPIO34, iba VSTUP, predpatie ~1,65 V, ochrana!
  RADIO AUDIO OUT (DAC): GPIO25, cez dolnopriepustny filter a utlmovy obvod.
  RADIO PTT: GPIO27 -> R1 4k7 -> baza BC547; R2 100k baza-emitor;
    kolektor na overeny PTT vodic, emitor na GND (ZIADNE RELE).
  Volny: GPIO26, GPIO16, GPIO17; bez povodneho RS-232 BayCom konektora.
  GPIO12 nepouzity (strap pin). SPI ILI9341 a SD maju oddelene CS.

3. AUDIO OUT -> ALLAMAT 296 MIC AUDIO
-------------------------------------
  GPIO25 DAC -- R1 2k2 -- BOD A
  BOD A -- C1 10nF -- GND       (RC LPF)
  BOD A -- R2 100k -- vrchny koniec trimra P1 10k
  dolny koniec trimra P1 -- GND
  jazdec P1 -- C2 1uF NEPOLARNY -- overeny MIC AUDIO vstup Allamatu
  GND signalova zeme na overeny GND mikrofonu
  P1 zacat na minimalnej audio urovni, potom merat deviation/modulaciu.
  Nikdy privadzat 3.3V DAC priamo na MIC, mohlo by to prebudit vstup.

4. AUDIO IN <- ALLAMAT 296 RX AUDIO
-----------------------------------
  Prijimany zvuk z overeneho SPK/AF/LINE vystupu, NIE z vodica RX
  na mikrofonom konektore (tam moze byt len ovladanie prijmu):
  AF signal -- R3 100k -- C3 1uF NEPOLARNY -- BOD ADC -- GPIO34
  BOD ADC -- R4 10k -- 3V3    (DC bias)
  BOD ADC -- R5 10k -- GND    (DC bias ~1.65V)
  BOD ADC -- C4 10nF -- GND   (RC filtrovanie)
  BOD ADC -- 2x Schottky svorky na 3V3 a GND, overit polaritu
  RX AF GND -> ESP32 GND IBA ked sa meranim potvrdi, ze vystup
  reproduktora NIE JE mostikovy ani plavajuci. Inak oddelovacie trafo!
  Na ADC GPIO34 NESMIE prist napatie mimo rozsah 0V...3.3V.
  Obvody podla navrhu: NIE finalne kalibrovane pre konkretne radio.

5. PTT ALLAMAT 296 - JEDEN BC547, BEZ RELE
-------------------------------------------
Toto je cielove zapojenie: pouzivatel potvrdil, ze povodny BayCom
s jednym tranzistorom fungoval s jeho Allamat 296.

                   Q1 = BC547 / BC548 (NPN)

  ESP32 GPIO27 --- R1 4k7 ---+------- B  Q1
                              |           C ------- RADIO PTT/TX
                            R2 100k       E
                              |           |
  ESP32 GND ------------------+-----------+ ------- RADIO GND

R1 obmedzuje prud bazy na asi 0,5 mA pri HIGH 3,3 V.
R2 drzi tranzistor vypnuty pri startovacom HI-Z stave GPIO27.
Pozor: skontroluj skutocne poradie E-B-C podla datasheetu kupeneho
BC547, nie podla schematickej pozicie vyvodov.

PTT funguje ako otvoreny kolektor / spojenie na zem:
  GPIO27 LOW => tranzistor VYPNUTY => PTT rozpojene => PRIJEM
  GPIO27 HIGH => tranzistor ZAPNUTY => PTT spojene s GND => TX

Potrebne suciastky:
  1x BC547 / BC548 (NPN), 1x 4k7 (R1), 1x 100k (R2).
  Ziadne rele, cievka ani ochranna dioda pre rele nie su potrebne.

RADIO PTT a RADIO GND urci podla funkčneho PTT kabla povodneho
BayComu a over meranim. NEPREDPOKLADAJ poradie pinov konektora len
podla neovereneho obrazku; pohlad zo strany kablu moze byt zrkadlovy.
Pred pripojenim zmeraj napatie PTT naprazdno voci radio GND a prud,
ktorym sa PTT spina na zem; hodnoty musia byt v limite Q1 a R1.
Ak je PTT vystup plavajuci alebo galvanicky oddeleny, zvaz optoclen.
ESP32 GND spajaj s radio GND az po potvrdeni spravneho zapojenia.
Nepripajaj +12 V vodic mikrofonu ani ine radio napajanie na GPIO.
RX prepínaci vodic mikrofonu NIE JE sucastou noveho PTT obvodu.

Prvy test: bez pripojeneho radia, multimetrom skontroluj Q1:
  po zapnuti a :DISARM nesmie viest kolektor-emitor;
  :ARM a :PTTTEST => na ~150 ms sa uzemni kolektor Q1.
Az po potvrdeni napeti a polarity pripoj Allamat 296.
Zjednodusena schema je v Nahlady/12_ptt_bc547_zapojenie.jpg.

6. RADIO PARAMETRE
------------------
  1200 Bd = Bell 202 AFSK: MARK 1200Hz, SPACE 2200Hz; typicky FM packet.
  300 Bd  = HF AFSK: MARK 1600Hz, SPACE 1800Hz; typicky SSB.
  Allamat 296 (27 MHz CB, FM/AM verzia) nie je bezne SSB HF radio;
  300 Bd preto neskusaj automaticky s protistanicou na HF SSB.
  Frekvenciu a vysielanie pouzivaj len ak to miestne predpisy dovoluju.
  AFSK vzorkovanie nominalne 9600 Hz cez I2S ADC/DAC DMA; DAC 8-bit; ADC1 12-bit.

7. OVLADANIE A TESTY
--------------------
  Po zapnuti: logo OM8AWA 3 sekundy, potom MONITOR.
  F1-F5: QSO okna, Esc: MONITOR, Alt+M: MENU, Alt+U: SETUP.
  SETUP -> Packet baud -> Enter: prepina 1200/300; uklada do NVS.
  MHeard, subory microSD, AutoBIN TX/RX, editor, beacon, diagnostika.
  Po pripojeni USB Serial Monitor pri 115200: Newline / Both NL & CR.
  :TESTALL       softverove kontroly AX.25 a microSD
  :AUDIOSTATUS   RX symboly, CRC chyby, ADC uroven, rychlost
  :ARM           TX povoleny na max 5 minut (len s overenym PTT obvodom)
  :PTTTEST       kratke zopnutie BC547 PTT na 150ms
  :BEACONTEST    poslat testovaci AX.25 UI ramec cez AFSK
  :DISARM        okamzite zakazat dalsie TX / vypnut PTT
  :REPORT        zapisat diagnostiku na SD
  :HELP          zobrazit napovedu
  Pred TX MUSI byt radio pripojene k spravne dimenzovanej antene/zatazi!
  Poznamka: AutoBIN, viac QSO a decode pri slabom signale stale vyzaduju testy.

8. NÁHĽADY (JPG)
----------------
  Priecinok Nahlady: OM8AWA logo, monitor, QSO, SETUP, File Manager,
  MHEARD, AutoBIN a modem status (niektore ilustracie su z GP v3.0
  a mozu obsahovat starsie nazvy SER12/BayCom; nie firmware screenshoty).
  '03_alt_m_klasicke_320x240.png' je SCHVALENE hlavne menu Alt+M.
  '04_setup_klasicky_1_320x240.png' a '04_setup_klasicky_2_320x240.png'
  su schvalene strany SETUP; tieto obrazovky su priamo kreslene vo v3.4.
  '12_ptt_bc547_zapojenie.jpg' je nova jednutranzistorova PTT schema.
  '13_ptt_test_320x240.jpg' je navrh diagnostickej obrazovky PTT.

9. TESTY DOSTUPNE V ZIP
-----------------------
  Testy/test_protocol.cpp       test AX.25 NRZI/HDLC/CRC
  Testy/test_audio_dsp.cpp      cisty audio digitalny loopback
  Testy/test_audio_regression.cpp 144 syntetickych modulovanych paketov
  Testy/VYSLEDKY_TESTOV.txt    konkretne vysledky a obmedzenia
  Kompilacia realnym Arduino IDE/Core 3.3.12 zostava NEOVERENA.
  Fyzicke radio testy, BER, osciloskop, osadenie a meranie GPIO: NEOVERENE.

10. LICENCIA
-----------
  Vlastny experimentalny kod v GP161_AFSK_DSP.h a GP161_AudioHW.h;
  originalny GP 1.61 je referencny program a nie je sem zabaleny.
  Externe Adafruit kniznice sa instaluju samostatne (licencie ich autorov).

11. ZMENY - PTT BC547 BEZ RELE
---------------------------------
- Nahradena predosla dokumentacia SPDT rele schémou 1x BC547.
- PTT pin ostava GPIO27 aktivny HIGH; GPIO26 je volny.
- V kode sa latch GPIO27 nastavi LOW pred pinMode(OUTPUT).
- Pridany hostitelsky test startu a PTT pulzu (bez skutocneho radia).
- Neprebehla kompilacia skutocnym Arduino toolchainom ani RF test.


12. AUTOMATICKE UKLADANIE HISTORIE - PREVZATE Z v3.2
------------------------------------------------
- SD:/GP/LOGS/MONITOR.LOG - Monitor RX/TX/SYS spravy.
- SD:/GP/LOGS/CH1.LOG az CH5.LOG - komunikacia a stavove hlasenia
  piatich nezavislych QSO kanalov.
- Novy riadok sa ihned prida do textoveho suboru na microSD, flush, close.
- Po starte program precita posledne zaznamy (max. 8 KiB kazdeho LOG)
  a obnovi poslednych 36 riadkov monitoru a 10 na kazdy QSO kanal.
- Spojenia AX.25, stav protistanice, nedokoncene prenosy ani buffer TX
  sa po restarte NEOBNOVUJU; QSO kanal sa zobrazi ako FREE.
- Logy neobsahuju realny datum ani hodinu, pretoze ESP32 nema RTC/NTP
  v tomto rezime; cas nemozno zmysluplne po vypnuti porovnavat.
- Pri dosiahnuti 512 KiB sa aktivny LOG presunie do LOG.OLD
  (posledna archivna generacia) a pokracuje novy LOG.
- SETUP / Enter: QSO Log ON/OFF, Monitor Log ON/OFF, Restore History
  ON/OFF. Uklada sa do NVS. Predvolene ON.
- SD nie je pritomna / zapis chyba: program funguje dalej, ale zaznamy
  nie su garantovane; po vysunuti karty pocas zapisu hrozi strata dat.
- Na zaciatku kazdeho vypnutia nie je treba nic potvrdzovat; vytahovat
  kartu je vhodne az po vypnuti napajania, nie pocas prijmu.
- Neexistuje ochrana sukromia sifrovanim; kazdy s SD kartou vie citat
  zaznamy komunikacie.
- Pri zaplneni SD moze neuspesny zapis znamenat stratu novych logov;
  starsie nevymazavaj bez zalohy.
- QSO riadok odoslania sa uklada pri prijati do lokalneho TX frontu,
  nie ako dokaz skutocneho dorucenia protistanici.
- Pri zmene MYCALL sa povodne logy automaticky nezmazu.

Pozn.: PDF GP161_ESP32_v3_4_Pouzivatelsky_manual_SK.pdf v ZIP (21 stran).


13. NOVINKY VERZIE 3.4 - SCHVALENY KLASICKY VZHLAD
-------------------------------------------------
- Klasicke hlavne menu ALT+M: modro-sive okno (x43, y35, 230x163),
  cerveny titulok GRAPHIC PACKET 1.61, zvyraznovanie polozky.
- Klasicky SETUP: cerveny nadpis, siva plocha, tmavomodry vyber,
  posuvanie 8 viditelnych z 11 nastaveni, dva pohlady (1/2 a 2/2).
- Spodna ikonova lista MH/EDIT/SETUP/MON/SD/SAVE v rovnakom style.
- Polozky a skratky: hore/dole + Enter, Esc do MONITORU, Alt+M menu;
  F1-F5 QSO kanaly. MYCALL, TX DELAY a BEACON sa edituju.
  PACKET BAUD sa prepina 1200/300 Bd, QSO LOG/MONITOR LOG/RESTORE
  HISTORY sa prepnu ON/OFF, SAVE SETTINGS potvrdi nastavenia.
- QSO CHANNELS=5, MODEM=INTERNAL AFSK, DISPLAY=320x240 su
  informacne polozky, vo v3.4 ich zmena nie je implementovana.
- V hornom stavovom riadku je UP mm:ss cas od startu (NIE realne hodiny).
- Graficke JPG/PNG nahlady su DIZAJN, nie automaticky screenshot
  z fyzickeho TFT. Pribuzna geometria a farby su v .ino; drobne detaily
  ikon/fontov sa mozu lisit.
- Aktualny manual ma verziu 3.4, opisuje aj automaticke SD logy.
- Overene na PC: C++ syntax s TESTOVACIMI nahradami Arduino kniznic,
  216 protokolovych AX.25 pripadov, 184 AFSK pripadov, test SD historie
  a boot loga/PTT v simulacii. Nie je to realna kompilacia ESP32.
- Realna kompilacia pre ESP32, meranie AFSK casovania a Allamat 296
  este neprebehli; firmware zostava EXPERIMENTALNY.

14. POSTUP OTVORENIA PROJEKTU
---------------------------
1. Rozbal cely ZIP.
2. Umiestni vsetky subory GP161_*.h a GP161_ESP32_v34.ino vedla seba
   v priecinku GP161_ESP32_v34 a otvor GP161_ESP32_v34.ino.
3. Zvol ESP32 Dev Module a nainstaluj kniznice z bodu 1.
4. Spusti Verify a pri chybe posli cely vystup kompilacie.
5. Po uspesnom nahrati testuj najprv bez pripojeneho radia.
6. Pri testovani Allamat 296 pouzi analogove obvody a PTT ochranu
   uvedene v bodoch 3-5; GPIO nikdy nenapajaj z radio 12 V.

Dokumentacia/Dokumentacia_v3_4.pdf: PDF je v korenovom priecinku ZIP
pod povodnym nazvom GP161_ESP32_v3_4_Pouzivatelsky_manual_SK.pdf.
