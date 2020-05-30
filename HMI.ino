/** PROGRAM PANELU HMI
*   ROBOTA TYPU DELTA "ASTRA*"
*   DLA PLYTKI NODEMCU ESP8266
*   SRODOWISKO ARDUINO IDE 1.8.12 - USTAWIENIA PONIZEJ
*   WIKTOR NOWACKI wik2001(at)windowslive.com
*   FUNKCJE OBSLUGI WYSWIETLACZA, SCHEMAT POLACZEN ETC. - ZRODLA PONIZEJ
*   Wykorzystano fragmenty szablonow demonstracyjnych od Arduino
*
*   KOD UDOSTEPNIONO "AS IS" WYLACZNIE W CELACH EDUKACYJNYCH!
*   
*   Czesc komentarzy (numery pinow) pochodza z w/w stron, pozostaje moje - czesc po polsku, czesc anglojezyczna
**//***
 * Software for HMI delta Astra* - NodeMCU 1.0 (ESP-12E Module)
 * 
 * Płytka = "NodeMCU 1.0 (ESP-12E Module)"
 * Flash Size = 4M (3M SPIFFS) / 1M
 * Debug port = Disabled 
 * Debug level = None
 * IwIP Variant = v2 Prebuilt (MSS"536")
 * CPU Frequency = 80MHz
 * Upload Speed = 115200
 * Port = COMx
 * Erase Flash = Only Sketch
 * 
 *BEFORE CONNECTING USB POWER, DISCONNECT 12V DC BY THE SMALL SWITCH ON BOARD-LCD CONTROLLER !!!"
 *
 *Source:
 *https://abc-rc.pl/product-pol-7348-Modul-WIFI-ESP8266-NodeMcu-V3-CH340-Arduino-ESP12E.html
 *https://simple-circuit.com/esp8266-nodemcu-ili9341-tft-display/
 *http://educ8s.tv/arduino-2-8-ili9341-tutorial/
 *and ADAFRUIT GFX/ILI3941 Libraries!
 *
 *Lista kodow wyjsciowych:
 *a-> Start
 *b-> button R
 *c-> button G
 *d-> button B
 *
 *dowolne nadanie -> rozpoczecie wyswietlania
 *jezeli natrafi na literke b,z lub c, wyswieli O w odpowiednim kolorze
 *Aby po zakonczonym cyklu wrocic do info, nalezy nadac literke i
 */
 
#include <Adafruit_GFX.h>       // include Adafruit graphics library
#include <Adafruit_ILI9341.h>   // include Adafruit ILI9341 TFT library
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include<SoftwareSerial.h> //Included SoftwareSerial Library

SoftwareSerial SUART(4, 5); //SRX  = DPin-D2; STX = DPin-D1
 
#define TFT_CS    D0     // TFT CS  pin is connected to NodeMCU pin D0
#define TFT_RST   D3     // TFT RST pin is connected to NodeMCU pin D3
#define TFT_DC    D4     // TFT DC  pin is connected to NodeMCU pin D4
// initialize ILI9341 TFT library with hardware SPI module
// SCK (CLK) ---> NodeMCU pin D5 (GPIO14)
// MOSI(DIN) ---> NodeMCU pin D7 (GPIO13)

#define e_stop 12
#define buttons A0

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

boolean nacisnieto = false;
int wartosci_a1 = 0;
int wartosci_a2 = 0;
int wartosci_a3 = 0;
String dane[5] = {"","","","",""};
int licznik_linii = 0;
boolean koniec = true;

void setup() {
  tft.begin();
  pinMode(e_stop, INPUT); //jezeli 1, zatrzymano robota
  Serial.begin(115200); //terminal na komputer
  SUART.begin(19200); //terminal do 401, bylo 115200 ale pojawialy sie bledy
  
  //Serial.println("HMI delta Astra*"); //debugowanie przez UART
  intro();
}
 
 
void loop() {
  /*OBSLUGA E_STOPU*/
  boolean stop3 = true;
  for (int i=0; i<3; i++) {
    stop3 = stop3 & digitalRead(e_stop);
    delay(1);
  }
  if (stop3) e_screen();
  
  /*OBSLUGA PRZYCISKOW*/
  do {
    wartosci_a1 = 0;
    wartosci_a2 = 0;
    wartosci_a3 = 0;
    
    for (int i=0; i<10; i++) {
      wartosci_a1+=analogRead(buttons);
      delay(1);
    }
    delay(100);
    
      for (int i=0; i<10; i++) {
      wartosci_a2+=analogRead(buttons);
      delay(1);
    }
    delay(100);
    
      for (int i=0; i<10; i++) {
      wartosci_a3+=analogRead(buttons);
      delay(1);
    }
    delay(100);
    
    wartosci_a1 = wartosci_a1/10;
    wartosci_a2 = wartosci_a2/10;
    wartosci_a3 = wartosci_a3/10;
  }
  while (abs(wartosci_a2-wartosci_a1)>10 || abs(wartosci_a3-wartosci_a2)>10);
  //Serial.println(wartosci_a);
  Serial.println(wartosci_a3);
  if (wartosci_a3<950 && wartosci_a3>=870  && !nacisnieto) {
    //nacisnieto START
    SUART.write('d');
    Serial.println('d');
    nacisnieto = true;
  }
  else if (wartosci_a3<870 && wartosci_a3>=810  && !nacisnieto) {
    //nacisnieto LUZ
    SUART.write('c');
    Serial.println('c');
    nacisnieto = true;
  }
  else if (wartosci_a3<810 && wartosci_a3>=660  && !nacisnieto) {
    //nacisnieto DOM
    SUART.write('b');
    Serial.println('b');
    nacisnieto = true;
  }
  else if (wartosci_a3<660 && wartosci_a3>=450  && !nacisnieto) {
    //nacisnieto SMC
    SUART.write('a');
    Serial.println('a');
    nacisnieto = true;
  }
  else if (wartosci_a3<450) {
    blad_przyciski();
    Serial.println("ERROR");
    while(1) {
      delay(1);//tutaj program się zatrzyma - konieczna jest naprawa, poniewaz cos moze byc nie tak (zwarcie etc.)
    }
  }
  else if (nacisnieto && wartosci_a3>=950) {
    nacisnieto = false;
  }
  /*KONIEC OBSLUGI PRZYCISKOW*/

  koniec = true;

  do {
  /***E-STOP***/
  boolean stop3 = true;
  for (int i=0; i<3; i++) {
    stop3 = stop3 & digitalRead(e_stop);
    delay(1);
  }
  if (stop3) e_screen();
  /***/
    
  if(SUART.available() > 0) {
    koniec = false;
    if (licznik_linii<=5) licznik_linii++;

    if (licznik_linii<=5) {
      dane[licznik_linii-1] = SUART.readStringUntil('\n');
      //Serial.print(dane[licznik_linii-1]);
    }
    else {
      for(int i=0; i<4; i++) {
        dane[i] = dane[i+1];
      }
      dane[4] = SUART.readStringUntil('\n');
    }

    //Serial.print("wszedlem w obsluge HMI"); //ponownie debugowanie
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(ILI9341_WHITE);  
    tft.setTextSize(1);
    tft.setFont(&FreeSansBold18pt7b);
    tft.println("");

    for (int j=0; j<5; j++) {
      for (int i=0; i<dane[j].length(); i++) {
        char c = dane[j].charAt(i);
        if (c=='b') {
          tft.print('O');
          //nic do zrobienia, caly czas pisze na bialo
        }
        else if (c=='c') {
          tft.setTextColor(ILI9341_RED);
          tft.print('O');
          tft.setTextColor(ILI9341_WHITE);
        }
        else if (c=='z') {
          tft.setTextColor(ILI9341_YELLOW);
          tft.print('O');
          tft.setTextColor(ILI9341_WHITE);
        }
        else if (c=='h') {
          tft.print("ZEROWANIE...");
        }
        else if (c=='i') {
          tft.print("EOC");
          koniec = true;
          intro();
        }
        else if (c==' ' || c=='0' || c=='1' || c=='2' || c=='3' || c=='4' || c=='5' || c=='6' || c=='7' || c=='8' || c=='9' || c=='-' || c=='X' || c=='Y' || c=='Z') {
          tft.print(c);
        }
        else {
          Serial.println(c);
          Serial.println(dane[j]);
          koniec = true;
          intro();
        }
      }
     tft.print('\n');
    }
  }
  }
  while (!koniec);
  
}

void intro() {
  /***WAZNE - TUTAJ MUSI OD NOWA ZAINICJALIZOWAC"***/
  dane[0] = "";
  dane[1] = "";
  dane[2] = "";
  dane[3] = "";
  dane[4] = "";
  licznik_linii = 0;
  koniec = true;
   /*Ekran - intro*/
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  
  tft.setTextSize(1);
  tft.setFont(&FreeSansBold12pt7b);
  tft.println("");
  tft.setFont(&FreeSansBold24pt7b);
  tft.println("   delta");
  tft.println("      Astra*");
  tft.setFont(&FreeSansBold12pt7b);
  tft.println(" Wiktor Nowacki     (c) 2020");
  tft.println("          ZSL Poznan*");
  tft.println("");
  tft.setTextColor(ILI9341_GREEN);
  tft.print("START ");
  tft.setTextColor(ILI9341_RED);
  tft.print("   LUZ");
  tft.setTextColor(ILI9341_GREEN);
  tft.print("   DOM");
  tft.setTextColor(ILI9341_BLUE);
  tft.println("   SMC");
}

void e_screen() { 
  tft.fillScreen(ILI9341_BLACK);
  tft.setFont(&FreeSansBold24pt7b);
  while (digitalRead(e_stop)) {
    
    tft.setTextColor(ILI9341_RED); 
    tft.setCursor(0, 0);
    tft.println("");
    tft.println("    STOP");
    tft.println("    AWARIA");
    tft.println("    :(");
    delay(800);
    
    tft.setTextColor(ILI9341_BLACK); 
    tft.setCursor(0, 0);
    tft.println("");
    tft.println("    STOP");
    tft.println("    AWARIA");
    tft.println("    :(");
    delay(200);
  }
  intro();
}

void blad_przyciski() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setFont(&FreeSansBold24pt7b);
  tft.setTextColor(ILI9341_RED); 
  tft.setCursor(0, 0);
  tft.println("");
  tft.println("    ERROR");
  tft.println("    Ui < 1.5V");
  tft.println("    :("); 
}
