//Harjoitus toteutettiin kiihtyvyys sensori G-61:llä. Tulostus toteutettiin Arduinon serial monitorin lisäksi vPythonin kautta.

#include <math.h>  
			// Kirjasto eri laskutoimitusten suorittamiseen(peruslaskuihin ei tarvitse), tässä koodissa asin-laskutoimitusta varten.

// Kiihtyvyysanturin kytkentänapojen määrittelyt:
signed int analogInPinX = A0;  // X-kanavan kytkentänapa, signed --> määrittyy negatiivise ja positiiviset arvot.
signed int analogInPinY = A1;  // Y-kanavan kytkentänapa
signed int analogInPinZ = A2;  // Z-kanavan kytkentänapa

int deBugging = 0;	//debugging muuttujan alustus.

int sensorValueX = 0;        // X-kiihtyvyyttä vastaava sensoriarvo 10 bittisessä järjestelmässä (0 - 1023).
int sensorValueY = 0;        // Y-kiihtyvyyttä vastaava sensoriarvo 10 bittisessä järjestelmässä (0 - 1023).
int sensorValueZ = 0;        // Z-kiihtyvyyttä vastaava sensoriarvo 10 bittisessä järjestelmässä (0 - 1023).
unsigned long aika = 0;      // aikaleima, joka luetaan millisekunteina. Int-tyyppinen määrittely tuottaisi nopean bitti-ylivuodon.
							 // arvo-alue välillä--> 0 to 4,294,967,295 (2^32 - 1).
							 // Lisää tietoa: https://www.arduino.cc/reference/en/language/variables/data-types/unsignedlong/

float AccX = 0.0; //Asetetaan muuttuja acceleraton-X m/s^2.
float AccY = 0.0; //Asetetaan muuttuja acceleraton-Y m/s^2.
float AccZ = 0.0; //Asetetaan muuttuja acceleraton-Z m/s^2.
float DegX = 0.0; //Asetetaan muuttuja kulma-arvo-X astetta.
float DegY = 0.0; //Asetetaan muuttuja kulma-arvo-Y astetta.
float DegZ = 0.0; //Asetetaan muuttuja kulma-arvo-Z astetta.


void setup() {
  
  Serial.begin(9600); //Avataan sarjaportti nopeudella 9600 baud rate.
  Serial.print("Aika(ms) " );   // Tulostetaan monitorin alkuun tietoja mitä tulostetaan.
  Serial.print(" ; Acc_x ");	//Tämä tulostesarja näkyy monitorissa rivinä.
  Serial.print(" ; Acc_y ");	//Tämä rivi tulostuu vain yhden kerran, kirjoitettu void set up()-lohkoon.
  Serial.print(" ; Acc_z ");
  Serial.print(" ; DegX ");
  Serial.print(" ; DegY ");
  Serial.print(" ; DegZ ");*/
  
}

void loop() {
  
 
  if(deBugging != 0){	//Asetetaan debugging, jos arvo on 0, then, just go for it:).
  delay(1500);
  }
  
  sensorValueX = analogRead(analogInPinX); // Luetaan analogInPinX tuottama data muuttujalle sensorValueX.     
  sensorValueY = analogRead(analogInPinY); // Luetaan analogInPinY tuottama data muuttujalle sensorValueX.
  sensorValueZ = analogRead(analogInPinZ); // Luetaan analogInPinZ tuottama data muuttujalle sensorValueX.
                 
  AccX = 0.135 * sensorValueX - 44.927;  //  Lasketaan sensoriarvo sensorValueX kiihtyvyydeksi (m/s^2) muuttujalle AccX. 
  AccY = 0.135 * sensorValueY - 44.415; // Lasketaan sensoriarvo sensorValueY kiihtyvyydeksi (m/s^2) muuttujalle AccX.
  AccZ = 0.1358 * sensorValueZ - 46.036;  // Lasketaan sensoriarvo sensorValueZ kiihtyvyydeksi (m/s^2) muuttujalle AccX.
  
  AccX = min(9.81 , AccX); // Asetetaan maksimi.
  AccY = min(9.81 , AccY);
  AccZ = min(9.81 , AccZ);
  AccX = max(-9.81 , AccX); // Asetetaan minimi.
  AccY = max(-9.81 , AccY);
  AccZ = max(-9.81 , AccZ); 
  
  DegX = asin(AccX / 9.81) * 180 / 3.141593; //Lasketaan akseli-X kulma sijoittamalla laskettu X:n kiihtyvyysarvo yhtälöön.
											 //(radiaanit muuttuu asteiksi kaavan osalla-->180 / 3.141593).
  DegY = asin(AccY / 9.81) * 180 / 3.141593;  //Lasketaan akseli-Y kulma sijoittamalla laskettu X:n kiihtyvyysarvo yhtälöön.
  DegZ = asin(AccZ / 9.81) * 180 / 3.141593;  //Lasketaan akseli-Z kulma sijoittamalla laskettu X:n kiihtyvyysarvo yhtälöön.

                      
  Serial.print(aika); //Tulostetaan aika jokaiselle näytteelle.       
  Serial.print(" ; "); //Tulostetaan ";"selkeyttää näkymää.       
  Serial.print(AccX); //Tulostetaan kiihtyvyysarvo (Acceleration X (ms^2)).    
  Serial.print(" ; ");  
  Serial.print(AccY);      
  Serial.print(" ; ");  
  Serial.print(AccZ);   
  Serial.print(" ; ");
  Serial.print(" Kulma-x "); //Tulostetaan teksti " Kulma-x ".         
  Serial.print(DegX); //Tulostetaan Kulma-arvo edellä olevan tekstin jälkeen.     
  Serial.print(",");
  Serial.print(" Kulma-y ");   
  Serial.print(DegY);      
  Serial.print(",");
  Serial.print(" Kulma-z ");   
  Serial.println(DegZ);  // Tulostetaan z-kulman arvo ja rivinvaihto lopuksi.
  

  delay(200); //Asetetaan monitorille näyttönopeus (ms).                
}

---------------------------------------------------------------------------------------------------------------------------------------
//I2C-laite skanneri. "Intern Integrated Circuit"
//Ohjelmalla voidaan tarkistaa montako I2c-laitetta on kytkennässä, ja mihin osoitteeseen laitteet viittaa.

#include <Wire.h>					//include Wire.h kirjasto

void setup()
{
  Wire.begin();						//Wire aloittaa
  Serial.begin(9600); 
  while (!Serial);					//Odotetaan Serial monitoria.
  Serial.println("\nI2C Skanneri");	// Aloitusteksti
}

void loop()
{
	byte error;		//muuttuja error
	byte address;	//Muuttuja I2C address
	int numDevices;	//Muuttuja montako laitetta

  Serial.println("Skannataan...");

  numDevices = 0;
  for (address = 1; address < 127; address++ )	//I2c_skanneri käyttää Write.endTransmission-palautusarvoa nähdäkseen,
												//onko laite kuullut osoitteen.
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C laitetta ei löytynyt osoitteesta 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      numDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Tuntematon error osoitteessa 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (numDevices == 0)
    Serial.println("I2C-laitteita löytyi:\n");
  else
    Serial.println("Skannaus valmis :)\n");

  delay(5000); // Odotetaan 5 sekunttia ennen uuttaa skannausta.
}


--------------------------------------------------------------------------------------------------------------------------------------

_______________________________________________________________________________________________________________________________________
 
//Projekti_R2_MTB_Ajotietokone
 
#include <Wire.h>        		//Wire.h kirjastossa I2C pinnit A4 -> (SDA), A5 -> (SCL).
#include <LiquidCrystal_I2C.h>  //Kirjasto ei ole esiasennettuna Arduino IDE:ssä, pitää ladata.
#include <TinyGPS++.h>      	//Kirjasto GPS toimintoihin. //Lataus: https://github.com/mikalhart/TinyGPSPlus
#include <SoftwareSerial.h>		//Kirjasto datan siirtämiseen, löytyy IDE:sta.
                                //Lisää kirjastosta: https://www.arduino.cc/en/Reference/softwareSerial
#define deBug1;
 
static const int RXPin = 4, TXPin = 3;  //Receive(RX) ja transmitted(TX), eli vastaanotto ja lähetys pinnit.GPS
static const uint32_t GPSBaud = 9600;	//32 bittinen integer tyyppinen vakio muuttuja, jossa määritellään nopeudeksi 9600 baudia.GPS
const int MPU_addres=0x68;				//MPU6050 Muistipaikan aloitusosoite, MPU-6050 rekisteritiedot löytyy omasta datasheet.
 
TinyGPSPlus gps;          			//Alustetaan TinyGpsPlus (olio).GPS
SoftwareSerial ss(RXPin, TXPin);  	//Sarjayhteys määrittely laitteelle.GPS yhteys
LiquidCrystal_I2C lcd(0x27, 20, 4); //LCD Asetetaan osoite 20 kolumnin ja 4 rivin näytölle.
                                    //Rekisterin aloitus osoite 0x27.
 
void setup(){
  Serial.begin(9600); //Avataan sarjaportti.
  ss.begin(GPSBaud);  //GPS Avataan GPS.
  lcd.begin();        //LCD Avataan portti lcd:lle.
  lcd.backlight();    //LCD Avataan näytön taustavalo.
 
  ss.begin(9600);     //Avataan GPS.
  lcd.begin();        //LCD Avataan portti lcd:lle.
  lcd.backlight();    //LCD Avataan näytön taustavalo.
 
  ss.begin(9600);     //Avataan GPS.
  lcd.setCursor(0,3); //Määritellään kursorin paikka.
  lcd.print("Alt ");  //LCD_GPS Korkeus merenpinnasta.
  lcd.setCursor(0,0); //LCD
  lcd.print("Km/h "); //LCD_GPS Tulostetaan teksti.
 
  lcd.setCursor(11,2);  //LCD_MPU6050 Määritellään kursorin paikka.
  lcd.print("Pitch ");  //Kallistuskulma.LCD_MPU6050
  lcd.setCursor(10,0);  //LCD_MPU6050
  lcd.print("Time ");   //LCD_GPS Tulostetaan aika
 
  lcd.setCursor(0,1);   //LCD
  lcd.print("Lat ");    //LCD_GPS Tulostetaan teksti "Lat:".
  lcd.setCursor(0,2);   //LCD
  lcd.print("Long ");   //LCD_GPS Tulostetaan teksti"Long:".
 
  lcd.setCursor(9,3);	//LCD
  lcd.print("Drctn ");	//LCD_GPS Kompassisuunta.
 
  lcd.setCursor(10,1);	//LCD
  lcd.print("Temp ");	//LCD_MPU6050 Lämpötila celcius asteina.
 	
	//deBug määrittely
  #ifdef deBug;							
  Serial.println("Debug käynnistetty");	//Tulostetaan teksti kerran kun deBug luetaan
  #endif
	//Päätetään deBug lohko								
 
  Wire.begin();                       //MPU6050 Käynnistetään Wire-kirjasto ja liitytään I2C-väylään isäntänä(master).Ei parametreja.
  Wire.beginTransmission(MPU_addres); //MPU6050 Aloittaa lähetyksen I2C-laitteelle (slave) MPU6050.
  Wire.write(0x6B);                   //MPU6050 Kirjoittaa tietoja muistipaikkaan 0x6B (PWR_MGMT_1 rekisteri)"Power Management 1".
  Wire.write(0);                      //MPU6050 Asetetaan 0, aktivoidaan.
  Wire.endTransmission(true);         //MPU6050 Lopetetaan lähetys, jos true.
}
 
void loop(){
  double t;	//void loop lohkossa funktion muuttujan määrittely
  double x;

    while (ss.available() > 0){       //GPS Jos laitteelta tulee dataa, go for it and do something:).
    gps.encode(ss.read());            //GPS Luetaan dataa.
    if (gps.location.isUpdated()){    //GPS Jos gps on päivittynyt, tulostetaan dataa.
 
      #ifdef deBug;								
      Serial.print("Korkeus merenpinnasta: ");	//Tulostetaan teksti
      Serial.println(gps.altitude.meters());	//deBug tulostus
      #endif									
     
       if(gps.altitude.meters() > 0){		//if-lohko, tulostetaan jos data on > 0
        lcd.setCursor(4,3);
        lcd.print("    ");					//Tulostetaan lcd screenille tyhjää välissä
        lcd.setCursor(4,3);               	//LCD Aseta kursori      
        lcd.print(gps.altitude.meters(), 1);//GPS Korkeus merenpinnasta metreinä data.    
       }else{
        lcd.setCursor(4,3);
        lcd.print("----");					//Tulostetaan "----", jos ei ole dataa
         }
 
      if (gps.speed.kmph() >= 2) {			//if-lohko, tulostetaan vasta kun nopeus on >= 2km/h
       lcd.setCursor(5,0);
       lcd.print("  ");
       lcd.setCursor(5,0);
       lcd.print(gps.speed.kmph(), 0);  	//GPS Tulostetaan nopeusarvo jos nopeus on yli 2km/h
      }else{
        lcd.setCursor(5,0);
        lcd.print("--");
        }
      #ifdef deBug;
      Serial.print("Km/h: ");
      Serial.println(gps.speed.kmph());		//deBug tulostus
      #endif
     
      lcd.setCursor(4,1);               
      lcd.print(gps.location.lat(), 2); 	//GPS Tulostetaan data-arvoja(leveysaste), tarkkuus 2 desimaalia.
      #ifdef deBug;
      Serial.print("Leveysaste: ");
      Serial.println(gps.location.lat());	//deBug tulostus
      #endif

      
      lcd.setCursor(15,0);
      lcd.print(gps.time.hour()+3);			//GPS LCD Tulostetaan tunnit
      lcd.print(":");
      lcd.print(gps.time.minute());			//GPS LCD Tulostetaan minuutit
      #ifdef deBug;
      Serial.print("Aika: ");
      Serial.print(gps.time.hour());		//deBug tulostus
      Serial.print(":");
      Serial.println(gps.time.minute());	//deBug tulostus
      #endif 
     
      lcd.setCursor(5,2);//LCD
      lcd.print(gps.location.lng(), 2); 		//GPS Tulostetaan data-arvoja(pituussaste), tarkkuus 2 desimaalia.
												//Funktion jälkeen voi määritellä tarkkuuden, max = 6 dec.     
      #ifdef deBug;
      Serial.print("Pituusaste: ");
      Serial.println(gps.location.lat());		//deBug tulostus
      #endif
 
      #ifdef deBug;
      Serial.print("Satelliittien lkm: ");
      Serial.println(gps.satellites.value());	//deBug tulostus
      #endif

      if(gps.course.deg() == false)				//jos ei dataa, tulostetaan "-----"
      {
        lcd.setCursor(15,3);
        lcd.print("---- ");
      }
      if(gps.course.deg()>= 350 && gps.course.deg()<= 360 && gps.course.deg()>= 0 && gps.course.deg() <= 10)
        {
          lcd.setCursor(15,3);  //LCD
          lcd.print("North");   //GPS Kompassisunta tekstinä(North).
        }
      if(gps.course.deg()> 10 && gps.course.deg()< 80)			//Määritellään raja-arvot
        {
          lcd.setCursor(15,3);
          lcd.print("NE   ");	//GPS Kompassisunta tekstinä(Northeast).
        }
        if(gps.course.deg()>= 80 && gps.course.deg()<= 100)		//Määritellään raja-arvot
        {
          lcd.setCursor(15,3);
          lcd.print("East ");	//GPS Kompassisunta tekstinä.
        }
         if(gps.course.deg()> 100 && gps.course.deg()< 170)		//Määritellään raja-arvot
        {
          lcd.setCursor(15,3);
          lcd.print("SE   ");	//GPS Kompassisunta tekstinä(Southeast).
        }
        if(gps.course.deg()>= 170 && gps.course.deg()<= 190)	//Määritellään raja-arvot
        {
          lcd.setCursor(15,3);
          lcd.print("South");	//GPS Kompassisunta tekstinä
        }
        if(gps.course.deg()> 190 && gps.course.deg()< 260)		//Määritellään raja-arvot
        {
          lcd.setCursor(15,3);
          lcd.print("SW   ");	//GPS Kompassisunta tekstinä
        }
        if(gps.course.deg()>= 260 && gps.course.deg()<= 280)	//Määritellään raja-arvot
        {
          lcd.setCursor(15,3);
          lcd.print("West ");	//GPS Kompassisunta tekstinä
        }
        if(gps.course.deg()> 280 && gps.course.deg()< 350)		//Määritellään raja-arvot
        {
          lcd.setCursor(15,3);
          lcd.print("NW   ");	//GPS Kompassisunta tekstinä
        }
      #ifdef deBug;
      Serial.print("Kompassi suunta: ");
      Serial.println(gps.course.deg());	//deBug tulostus
      #endif                    
   
      lampotila(t);						//Lämpötilan funktio
      lcd.setCursor(15,1);
      lcd.print("    ");
      lcd.setCursor(15,1); 
      lcd.print(t,1);      				//MPU6050 Lämpötila Celsiuksina
      delay(200);     
 
      nousukulma(x);					//Nousukulman funktio 
      if(x >=0 && x <=60){				//jos nousukulma on yli 0astetta, mutta alle 60astetta, tulostetaan arvo LCD-näytölle
      lcd.setCursor(17,2);
      lcd.print("   ");
      lcd.setCursor(17,2);
      lcd.print(x,0);
      delay(200);
      }else{
     lcd.setCursor(17,2);
     lcd.print("-- ");
      }
    }
  }
}
 
void nousukulma(double &Ydeg){ 			//Nousukulmafunktio
 
int16_t AcX,AcY,AcZ;					//16-bitin muuttujat kulma-arvojen analogisille data-arvoille
double y; 								//MPU6050 y-akseli muuttujan alustus.
double z; 								//MPU6050 z-akseli muuttujan alustus.
double Xdeg;							//MPU6050 x-akseli muuttujan alustus.
int xAng, yAng, zAng;					//Map-data muuttujat
   
int minVal=265; 						//MPU6050 Alustetaan analoginen minimarvo kulma-arvoille x, y ja z.
int maxVal=402; 						//MPU6050 Alustetaan analoginen maksimiarvo kulma-arvoille x, y ja z.
 
Wire.beginTransmission(MPU_addres);		//Aloitetaan lähetys rekisteristä 0x68
Wire.write(0x3B);						//Talennetaan rekisteriin 0x3B
Wire.endTransmission(false);			//Jatketaan lähetystä              
Wire.requestFrom(MPU_addres, 14, true);	//Luetaan 14 rekisteriä
 
AcX=Wire.read()<<8|Wire.read();			//Luetaan kaksi 8-bit rekisteriä, toisessa siirretään 8 bit vasemmalle ja tallennetaan AcX
#ifdef deBug;
Serial.print("X akselin rekisteridata: ");
Serial.println(AcX);					//deBug tulostus
#endif
 
AcY=Wire.read()<<8|Wire.read();			//Luetaan kaksi 8-bit rekisteriä, toisessa siirretään 8 bit vasemmalle ja tallennetaan AcY
#ifdef deBug;
Serial.print("Y akselin rekisteridata: ");
Serial.println(AcY);					//deBug tulostus
#endif
 
AcZ=Wire.read()<<8|Wire.read();			//Luetaan kaksi 8-bit rekisteriä, toisessa siirretään 8 bit vasemmalle ja tallennetaan AcZ
#ifdef deBug;
Serial.print("Z akselin rekisteridata: ");
Serial.println(AcZ);					//deBug tulostus
#endif
 
xAng = map(AcX,minVal,maxVal,-90,90);	//MPU6050 map(AcX=arvo, minimistä, maximiin, pieneen kohdearvoon, suureen kohdearvoon.)
#ifdef deBug;
Serial.print("XMap-data: ");
Serial.println(xAng);					//deBug tulostus
#endif
 
yAng = map(AcY,minVal,maxVal,-90,90);	//MPU6050 map(AcY=arvo, minimistä, maximiin, pieneen kohdearvoon, suureen kohdearvoon.)
#ifdef deBug;
Serial.print("YMap-data: ");
Serial.println(yAng);					//deBug tulostus
#endif
 
zAng = map(AcZ,minVal,maxVal,-90,90);	//MPU6050 map(AcZ=arvo, minimistä, maximiin, pieneen kohdearvoon, suureen kohdearvoon.)
#ifdef deBug;
Serial.print("ZMap-data: ");
Serial.println(zAng);					//deBug tulostus
#endif
												//MPU6050 Atan2 () -funktio laskee kaari-tangentin pääarvon y / x 
Xdeg= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);	//MPU6050 RAD_TO_DEG = radiaanit asteiksi, sisältyy Arduino.h kirjastoon
#ifdef deBug;									
Serial.print("X kulma: ");
Serial.println(Xdeg);							//deBug tulostus
#endif
 
Ydeg= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);	//MPU6050 RAD_TO_DEG = radiaanit asteiksi
#ifdef deBug;
Serial.print("Y kulma: ");
Serial.println(y);								//deBug tulostus
#endif
 
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);		//MPU6050 RAD_TO_DEG = radiaanit asteiksi
#ifdef deBug;
Serial.print("Z kulma: ");
Serial.println(z);								//deBug tulostus
#endif
}

void lampotila(double &temperature) {
  double tx;    				//MPU6050 Lämpötilamuuttuja (temperature) ja lämpötila data-arvo muuttuja (tx).
  int16_t AcX, AcY, AcZ, Tmp;	//16 bitin muuttujat funktiolle
  int tcal;     				//Kalibrointi muuttuja
 
  Wire.beginTransmission(MPU_addres);       //MPU6050 Aloitetaan lähetys I2C laitteelta.
  Wire.write(0x3B);                         //MPU6050 Aloittaa muistipaikasta 0x3B (ACCEL_XOUT_H). Rekisteri datasheet s.42
  Wire.endTransmission(false);              //MPU6050 Lähetyksen parametrina "false" tarkoittaa, että lähetystä pidetään aktiivisena.
  Wire.requestFrom(MPU_addres, 14, true);   //MPU6050 Master pyytää I2C laitteelta dataa,
  											//(0x68 osoite, yhteensä 7*2 = 14 rekisteriä, tila = true).

  //Kaikki rekisterit tarvitaan lämpötilarekisterin oikein lukemiseen. Tmp on viimeinen.                            
  AcX=Wire.read()<<8|Wire.read();	//MPU6050 Luetaan kaksi rekisteriä ja tallennetaan samaan muuttujaan.
                  						//Luettava rekisteri: 0x3B (ACCEL_XOUT_H) ja 0x3C (ACCEL_XOUT_L)  
  AcY=Wire.read()<<8|Wire.read();	//MPU6050 Luetaan dataa ja muutetaan 16 bittiseksi.
                  						//Luettava rekisteri: 0x3D (ACCEL_YOUT_H) ja 0x3E (ACCEL_YOUT_L)  
  AcZ=Wire.read()<<8|Wire.read();	//MPU6050 Luetaan kaksi rekisteriä ja tallennetaan samaan muuttujaan.
                  						//Luettava rekisteri: 0x3F (ACCEL_ZOUT_H) ja 0x40 (ACCEL_ZOUT_L) 
  Tmp=Wire.read()<<8|Wire.read(); 	//MPU6050 Luetaan rekisteri 0x41 (TEMP_OUT_H) ja rekisteri 0x42 (TEMP_OUT_L)
                                  		//Lämpötilan luku datasta.
  
  #ifdef deBug;
  Serial.print("Lämpötila: ");
  Serial.println(Tmp);				//deBug tulostus			
  #endif
  
  tcal = -1500;						//MPU6050 Lämpötilan analoginen korjausarvo. Tässä huomioidaan MPU6050-sirun lämpövaikutus. 
  tx = Tmp + tcal;                  //MPU6050 Lasketaan tx-arvo-->raaka data + korjausarvo.
  #ifdef deBug;
  Serial.print("Korjattu lämpötilan data-arvo: ");
  Serial.println(tx);				//deBug tulostus
  #endif
 
  temperature = tx / 340 + 36.53; 	//MPU6050 Yhtälö lämpötilan laskemiselle Celsius asteiksi saadusta datasta.
                                  	//Rekisteri datasheet s. 30 --> Tmp in degrees C = ((temperature sensor data)/340 + 36.53) °/c.
  #ifdef deBug;
  Serial.print("Laskettu lämpötila: ");
  Serial.println(temperature);		//deBug tulostus
  delay(1500);
  #endif
  
  }