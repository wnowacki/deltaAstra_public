/** PROGRAM GLOWNEGO STEROWNIKA
*   ROBOTA TYPU DELTA "ASTRA*"
*   DLA PLYTKI NUCLEO F401RE
*   SRODOWISKO ARM mBED
*   WIKTOR NOWACKI wik2001(at)windowslive.com
*   FUNKCJE KINEMATYKI ODWROTNEJ
*   POCHODZA Z http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
*   Wykorzystano fragmenty szablonow demonstacyjnych od mBED
*
*   KOD UDOSTEPNIONO "AS IS" WYLACZNIE W CELACH EDUKACYJNYCH!
**/

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "mbed.h"
#include "AccelStepper.h"

/* deklaracje pinow */
AccelStepper stepperA(1,PB_13,PB_5);
AccelStepper stepperB(1,PB_14,PB_4);
AccelStepper stepperC(1,PB_15,PB_10);
Timer t;

DigitalOut krok(PC_2);
DigitalOut kier_anticw(PC_3); 
DigitalOut flash(PB_8);
DigitalIn czujnik(PC_9);

DigitalIn limitA(PC_5);
DigitalIn limitB(PC_6);
DigitalIn limitC(PC_8);

DigitalOut buzzer(PB_9);
DigitalOut ssawka(PA_7);
DigitalOut elektrozawor(PB_6);

Serial RPI(PA_9, PA_10); // tx, rx
Serial HMI(PA_11, PA_12); // tx, rx
Serial pc(USBTX, USBRX); // tx, rx

DigitalOut luz1(A5);
DigitalOut luz2(A4);
DigitalOut luz3(A3);

int kolory[9] = {0,0,0,0,0,0,0,0,0}; //0-biel 2-zolty 3-czerwony, 4-puste_gniazdo
int akt_poz_stolu = 1;
int zad_poz_stolu =1;

uint8_t text[16][30];
int licznik_wierszy = 0;

//stale wynikajace z geometrii konkretnej konstrukcji
 const float e = 121.2;     // efektor
 const float f = 150.0;     // podstawa
 const float re = 250.0;
 const float rf = 125.0;
 
 // stale trygonometryczne
 const float sqrt3 = sqrt(3.0);
 const float pi = M_PI;    // PI
 const float sin120 = sqrt3/2.0;   
 const float cos120 = -0.5;        
 const float tan60 = sqrt3;
 const float sin30 = 0.5;
 const float tan30 = 1/sqrt3;

 float theta1 = 0.0;
 float theta2 = 0.0;
 float theta3 = 0.0;
 
 int test_OK = -1;
 
 // komentarze anglojezyczne do /*** koniec IK ***/ pochodza z oryginalnego opracowania z forum Trossen
 // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     return 0;
 }
 
 // inverse kinematics
 // helper functions, calculates angle theta1 (for YZ-pane)
 int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
     float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * e;    // shift center to edge
     // z = a + b*y
     float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     float b = (y1-y0)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
     if (d < 0) return -1; // non-existing point
     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     float zj = a + b*yj;
     theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
     return 0;
 }
 
 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcInverse_napis(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
     int x_ekran = floor(abs(x0));
     int y_ekran = floor(abs(y0));
     int z_ekran = floor(abs(z0));
     
     if (x0<0) x_ekran=-x_ekran;
     if (y0<0) y_ekran=-y_ekran;
     if (z0<0) z_ekran=-z_ekran;
     HMI.printf("X%i Y%i Z%i\n",x_ekran,y_ekran,z_ekran);
     
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
     return status;
 }
 
//funkcja identyczna jak wyzej, ale aby nie robic zamieszania, ruchy mniej znaczace nie sa wyswietlane na panelu HMI
    int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
    theta1 = theta2 = theta3 = 0;
    int status = delta_calcAngleYZ(x0, y0, z0, theta1);
    if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
    if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
    return status;
 }
/*** koniec IK ***/

void movement(float a, float b, float c) {
  int motorA = floor(-a/360.0*5170.0);
  int motorB = floor(-b/360.0*5170.0);
  int motorC = floor(-c/360.0*5170.0);

  stepperA.moveTo(motorA);
  stepperB.moveTo(motorB);
  stepperC.moveTo(motorC);
  
  while (abs(stepperA.distanceToGo()) > 0 || abs(stepperB.distanceToGo()) > 0 || abs(stepperC.distanceToGo()) > 0)
     {
       stepperA.run();
       stepperB.run();
       stepperC.run();
     }
     
  wait_ms(100);
}

void zerowanie() {
    /***Zerowanie***/
    //te wartosci rowniez sa zalezne od konkretnie mojej konstrukcji
    stepperA.setSpeed(-500);
    stepperB.setSpeed(-500);
    stepperC.setSpeed(-500);

    while(!limitA || !limitB || !limitC) {
        if (!limitA) stepperA.runSpeed();   
        if (!limitB) stepperB.runSpeed(); 
        if (!limitC) stepperC.runSpeed(); 
    }
    
    //pc.printf("I ETAP - OK"); //celem debugowania
    stepperA.move(71);
    stepperB.move(71);
    stepperC.move(71);
    
    while (abs(stepperA.distanceToGo()) > 0 || abs(stepperB.distanceToGo()) > 0 || abs(stepperC.distanceToGo()) > 0)
     {
       stepperA.run();
       stepperB.run();
       stepperC.run();
     }
    
    //ponowne wyzerowanie osi, ale znacznie wolniej
    stepperA.setSpeed(-100);
    stepperB.setSpeed(-100);
    stepperC.setSpeed(-100);

    while(!limitA || !limitB || !limitC) {
        if (!limitA) stepperA.runSpeed();   
        if (!limitB) stepperB.runSpeed(); 
        if (!limitC) stepperC.runSpeed(); 
    }
    
    //ustawienie ramion poziomo
    stepperA.move(710);
    stepperB.move(710);
    stepperC.move(710);
    
    while (abs(stepperA.distanceToGo()) > 0 || abs(stepperB.distanceToGo()) > 0 || abs(stepperC.distanceToGo()) > 0)
     {
       stepperA.run();
       stepperB.run();
       stepperC.run();
     }
     
     stepperA.setCurrentPosition(0);
     stepperB.setCurrentPosition(0);
     stepperC.setCurrentPosition(0);
    
    buzzer=1;
    wait(0.2);
    buzzer=0;
    
    //wyzerowanie osi podajnika rewolwerowego
    kier_anticw=1;
    while(!czujnik) {
         krok=1;
         wait_us(400);
         krok=0;
         wait_us(400);
    }
    
    buzzer=1;
    wait(0.1);
    buzzer=0;
    
    kier_anticw=1;
    for (int i=0; i<960; i++) {
         krok=1;
         wait_us(400);
         krok=0;
         wait_us(400);
    }
    
    //i jeszcze raz, ponowne i dokladne wyzerowanie osi talerza
    kier_anticw=0;
         while(!czujnik) {
         krok=1;
         wait_us(600);
         krok=0;
         wait_us(600);
    }
    
    //ustawienie talerza tak, aby  bylo widac gniazda 1-2-3
    for(int i=0; i<853; i++) {
        krok=1;
        wait_us(400);
        krok=0;
        wait_us(400);
    }
    
    //POZYCJA STARTOWA - wszystkie 4 osie wyzerowano
    
    buzzer=1;
    wait(0.2);
    buzzer=0;
    
}

//rozbudowana funkcja, ktora szuka najkrotszej drogi miedzy dwoma gniazdami
//uzywana, ale na wyrost, bo talerz i tak obracam 1-2-3....8-9
void ustaw_stol(int pozycja) {
    zad_poz_stolu = pozycja;
    int delta = zad_poz_stolu-akt_poz_stolu;
    if (delta>0) {
        kier_anticw=0;
    }
    else if (delta<0) {
        kier_anticw=1;
    }
    if (abs(delta)>=5) {
        delta = abs(delta);
        delta=9-delta;
        kier_anticw=!kier_anticw;   
    }
    for(int i=0; i<853*abs(delta); i++) {
            krok=1;
            wait_us(400);
            krok=0;
            wait_us(400);
        }
    akt_poz_stolu = zad_poz_stolu;
}

//glowna czesc kodu - sortowanie krazkow
void sortowanie() {
    luz1=0;
    luz2=0;
    luz3=0; //zalczenie silnikow
    //pc.printf("ZACZYNAMY\n"); //debugowanie dla PC
    HMI.printf("h\n"); //na HMI wyswietlam informacje "Zerowanie..."
    ssawka=1;
    elektrozawor=1;
    zerowanie();
    
    /*** I trojka - "skanowanie kolorow"***/
    flash=1;
    wait(0.5);
    RPI.printf("q");
    char c1 = RPI.getc();
    char c2 = RPI.getc();
    char c3 = RPI.getc();
    HMI.printf("1%c 2%c 3%c\n", c1, c2, c3);
    wait(0.5);
    flash=0;
    
    /***pobrano k1%k3*/
    for(int i=0; i<2560; i++) {
        krok=1;
        wait_us(400);
        krok=0;
        wait_us(400);
    }
    
    buzzer=1;
    wait_ms(100);
    buzzer=0;
    /***koniec I trojki ***/
    
    /*** II trojka - jak wyzej***/
    flash=1;
    wait(0.5);
    RPI.printf("q");
    char c4 = RPI.getc();
    char c5 = RPI.getc();
    char c6 = RPI.getc();
    HMI.printf("4%c 5%c 6%c\n", c4, c5, c6);
    wait(0.5);
    flash=0;
    
    /***pobrano k4%k6*/
    for(int i=0; i<2560; i++) {
        krok=1;
        wait_us(400);
        krok=0;
        wait_us(400);
    }
    
    buzzer=1;
    wait_ms(100);
    buzzer=0;
    /***koniec II trojki ***/
    
    /*** III trojka - ponownie jak wyzej***/
    flash=1;
    wait(0.5);
    RPI.printf("q");
    char c7 = RPI.getc();
    char c8 = RPI.getc();
    char c9 = RPI.getc();
    HMI.printf("7%c 8%c 9%c\n", c7, c8, c9);
    wait(0.5);
    flash=0;
    
    for (int i=0; i<3; i++) {
        buzzer=1;
        wait_ms(50);
        buzzer=0;
        wait_ms(50);
    }
    /***koniec III trojki ***/
        
    if (c1=='z') kolory[0]=1; else if (c1=='c') kolory[0]=2;
    if (c2=='z') kolory[1]=1; else if (c2=='c') kolory[1]=2;
    if (c3=='z') kolory[2]=1; else if (c3=='c') kolory[2]=2;
    if (c4=='z') kolory[3]=1; else if (c4=='c') kolory[3]=2;
    if (c5=='z') kolory[4]=1; else if (c5=='c') kolory[4]=2;
    if (c6=='z') kolory[5]=1; else if (c6=='c') kolory[5]=2;
    if (c7=='z') kolory[6]=1; else if (c7=='c') kolory[6]=2;
    if (c8=='z') kolory[7]=1; else if (c8=='c') kolory[7]=2;
    if (c9=='z') kolory[8]=1; else if (c9=='c') kolory[8]=2;
    
    //for (int i=0; i<9; i++) {pc.printf("%i",kolory[i]);} debugowanie
    
    ///4 - stan w kolory[] oznacza, ze pusto
    int szufladka[9] = {0,0,0,0,0,0,0,0,0};
    int numer_szf = 0;
    /* poszczegolne indeksy tablicy szufladka to, patrzac z gory, z przodu
    0  3  6
    1  4  7
    2  5  8 */ //do tych indeksow przypisuje numery gniazd z talerza
    
    for (int x=0; x<9; x++) {
        if (kolory[x]==2) {
            szufladka[numer_szf]=x+1;
            numer_szf++;
            kolory[x]=4;
        }
    }
    
    for (int x=0; x<9; x++) {
        if (kolory[x]==1) {
            szufladka[numer_szf]=x+1;
            numer_szf++;
            kolory[x]=4;
        }
    }
    
    for (int x=0; x<9; x++) {
        if (kolory[x]==0) {
            szufladka[numer_szf]=x+1;
            numer_szf++;
            kolory[x]=4;
        }
    }
    
    //debugowanie
    //pc.printf("%i  %i  %i\n", szufladka[0], szufladka[3], szufladka[6]);
    //pc.printf("%i  %i  %i\n", szufladka[1], szufladka[4], szufladka[7]);
    //pc.printf("%i  %i  %i\n", szufladka[2], szufladka[5], szufladka[8]);
            
    //glowna petla, wykona sie 9 razy - tyle ile krazkow
    for (int f=1; f<=9; f++) {
        ustaw_stol(f);
        ssawka=0;
        test_OK = delta_calcInverse_napis(80.0, 95.0, 220.0, theta1, theta2, theta3);if (test_OK==0) {movement(theta1,theta2,theta3);}
        test_OK = delta_calcInverse(80.0, 95.0, 240.0, theta1, theta2, theta3);if (test_OK==0) {movement(theta1,theta2,theta3);}
        test_OK = delta_calcInverse(80.0, 95.0, 210.0, theta1, theta2, theta3);if (test_OK==0) {movement(theta1,theta2,theta3);}
        /*** efektor wlasnie odjechal znad gniazda na podajniku ***/
        
       int i=0;
       int j=0;
   
       // f; to jest numer obiegu petli i gniazda, skad pobieram
       // chce wiedziec,jaki indeks ma wartosc w [tablica szufladki], gdzie jest ten krazek przechowywany
       // i do jakiego numer_gniazda go odlozyc
       
       int numer_gniazda_konc=0;
       
       for (int g=0; g<9; g++) {
            if (szufladka[g]==f) numer_gniazda_konc=g;
        } 
       
        if (numer_gniazda_konc==0) {i=1; j=-1;}
        else if (numer_gniazda_konc==1) {i=0; j=-1;}
        else if (numer_gniazda_konc==2) {i=-1; j=-1;}
        else if (numer_gniazda_konc==3) {i=1; j=0;}
        else if (numer_gniazda_konc==4) {i=0; j=0;}
        else if (numer_gniazda_konc==5) {i=-1; j=0;}
        else if (numer_gniazda_konc==6) {i=1; j=1;}
        else if (numer_gniazda_konc==7) {i=0; j=1;}
        else if (numer_gniazda_konc==8) {i=-1; j=1;}
        
        test_OK = delta_calcInverse(-22.0+48.0*j, 35.0+48.0*i, 210.0, theta1, theta2, theta3);if (test_OK==0) {movement(theta1,theta2,theta3);}
        test_OK = delta_calcInverse_napis(-22.0+48.0*j, 35.0+48.0*i, 230.0, theta1, theta2, theta3);if (test_OK==0) {movement(theta1,theta2,theta3);}
        ssawka=1;
        wait_ms(500);
        //ostatni ruch wykonam za chwile, calkiem ku gorze, aby latwiej bylo wyciagnac krazki z podajnika
        if (f<9) { test_OK = delta_calcInverse(-22.0+48.0*j, 35.0+48.0*i, 210.0, theta1, theta2, theta3);if (test_OK==0) {movement(theta1,theta2,theta3);} }
                
    }
    
    test_OK = delta_calcInverse_napis(0.0, 0.0, 145.0, theta1, theta2, theta3);if (test_OK==0) {movement(theta1,theta2,theta3);} 
    
    elektrozawor=0;//wysuniecie szufladki
    wait(1);
    HMI.printf("i"); //koniec cyklu, na wszelki wypadek "wyczyszcze" zmienne
    for (int i=0; i<9; i++) {
        kolory[i]=0;   
    }
    akt_poz_stolu = 1;
    zad_poz_stolu =1;
    licznik_wierszy = 0;
    for (int i=0; i<3; i++) {
        buzzer=1;
        wait_ms(50);
        buzzer=0;
        wait_ms(50);
    }
}

int main() {
    t.start();
    ssawka=1;
    elektrozawor=1;
    HMI.baud(19200);
    
    //printf("Hello World !\n");//linijka do debugowania komunikacji Nucleo<->PC
    
    buzzer=1;
    wait(0.1);
    buzzer=0;
    
    flash=1;
    wait(0.1);
    flash=0;
    
    luz1=1;
    luz2=1;
    luz3=1;//na poczatek silniki wylaczone, aby nie grzaly sie
    
    stepperA.setMaxSpeed(4000);
    stepperA.setAcceleration(4000);
    stepperA.setMinPulseWidth(3);
    
    stepperB.setMaxSpeed(4000);
    stepperB.setAcceleration(4000);
    stepperB.setMinPulseWidth(3);
    
    stepperC.setMaxSpeed(4000);
    stepperC.setAcceleration(4000);
    stepperC.setMinPulseWidth(3);
    
    char c_HMI = '0'; //zmienna robocza, czekam na literke z panelu HMI
    while (1) {
        c_HMI = HMI.getc();
        
        if (c_HMI=='a') sortowanie();
        else if (c_HMI=='b') {
            luz1=!luz1;
            luz2=!luz2;
            luz3=!luz3;
        }
        else if (c_HMI=='c') {
            luz1=0; luz2=0; luz3=0;
            HMI.printf("h\n");
            zerowanie();
            elektrozawor=1;
            HMI.printf("i");
        }
        else if (c_HMI=='d') {
            buzzer=1;
            wait(0.1);
            buzzer=0;
            wait(0.1);
            buzzer=1;
            wait(0.1);
            buzzer=0;
            elektrozawor=!elektrozawor;
        }
    }
}
