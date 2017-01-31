
// ================================================================
// ===                         Librairie                        ===
// ================================================================

#include <Arduino.h>

// ************   FILE   ************

#include <FS.h>

// ************   WIFI   ************

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

// ************   OTA   ************

#include <ArduinoOTA.h>

// ************   UDP   ************

#include <WiFiUdp.h>

// ************   IMU   ************

//#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ************   ENCODER   ************

#include <Encoder.h>

// ================================================================
// ===                      PIN Configuration                   ===
// ================================================================

//************   IMU   ************
// I2C
#define IMU_SDA 10
#define IMU_SCL 9
// interuption
#define IMU_INT 14

//************   MOTEURS   ************

// Motor Left
#define MOTOR_L_F  16
#define MOTOR_L_B  0
#define MOTOR_L_S  2
// MOTOR Right
#define MOTOR_R_F  12
#define MOTOR_R_B  13
#define MOTOR_R_S  3

//************   ENCODER   ************

// ENCODER Left
#define ENCODER_L_F  4
#define ENCODER_L_B  5
// ENCODER Right
#define ENCODER_R_F  1
#define ENCODER_R_B  15

// ================================================================
// ===                       Variable Global                    ===
// ================================================================

String MODE = "" ;

bool PAUSE = true;

// ************   WIFI   ************

const char* ssid = "CathyMath";
const char* password = "4362626262";

// ************   COMMUNICATION   ************

bool CONNECTED = 0 ;
String SEND = "" ;

//************   ENCODER   ************

Encoder EncL( ENCODER_L_F, ENCODER_L_B );
Encoder EncR( ENCODER_R_F, ENCODER_R_B );

// ================================================================
// ===                            WIFI                          ===
// ================================================================

void InitWifi() 
{
    // essaye de ce connecter a un reseau
    
    // en premier a celui donne
    Serial.print( "Connection at : " );
    Serial.println( ssid );
    WiFi.mode( WIFI_STA );
    WiFi.begin( ssid, password );
    
    // si la connection est un echec
    if( WiFi.waitForConnectResult() != WL_CONNECTED ) 
    {
        // cree un reseau
        //Serial.println( "Connection Failed! Configuring access point..." );
        //Serial.print( "ESSID : testapesp   IP : " );
        WiFi.softAP( "testapesp" );
        IPAddress myIP = WiFi.softAPIP();
        //Serial.println( myIP );
    }
    else 
    {
        Serial.print( "Connected ! IP : " );
        Serial.println( WiFi.localIP() );
    }  
}

// ================================================================
// ===                            UDP                           ===
// ================================================================

//************   Variables   ************

// local port to listen for UDP packets
unsigned int localPort = 2390;    
IPAddress ipMulti ( 192,168,0,255 );
// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;  
IPAddress Ip ;
unsigned int Port ;
//buffer to hold incoming and outgoing packets
char packetBuffer[ UDP_TX_PACKET_MAX_SIZE ]; 

//************   Initialisation   ************

void InitUdp()
{
    //////Serial.print( "Udp server started at port " );
    //////Serial.println( localPort );
    Udp.begin( localPort );
    //multicas_at_start_esp8266();
}

//************   Envoie   ************

void sendUdp(String msg)
{
    // envoie le message au client
    Udp.beginPacket( Ip, Port ); //Send message to Packet-Sender   
    Udp.write( msg.c_str() );
    Udp.endPacket();
}

//************   Reception   ************

String readUdp() 
{
    int packetSize = Udp.parsePacket();
    if ( packetSize ) 
    {
        CONNECTED = 1;
        PAUSE = false ;
        Ip = Udp.remoteIP() ;
        Port = Udp.remotePort();
        
        ////Serial.print( millis() / 1000 );
        ////Serial.print( ":Packet of " );
        ////Serial.print( packetSize );
        ////Serial.print( " received from " );
        ////Serial.print( Ip );
        ////Serial.print( ":" );
        ////Serial.println( Port );
        
        // We've received a packet, read the data from it
        Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE); // read the packet into the buffer
    
        // display the packet contents
        ////Serial.print( "Contents : " );
        ////Serial.println( packetBuffer );

        return packetBuffer ;
    } 
    return "";
}

// ================================================================
// ===                            OTA                           ===
// ================================================================

//************   Initialisation   ************

void InitOta() 
{
    // Configure la mise a jour par wifi
  
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);
  
    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("myesp8266");
  
    // No authentication by default
    // ArduinoOTA.setPassword((const char *)"123");

    // Action a faire avant la mise a jour
    ArduinoOTA.onStart([]() 
    {
        ////Serial.println( "Start" );
    });

    // Action a faire apres la mise a jour ( avant reboot )
    ArduinoOTA.onEnd([]() 
    {
        ////Serial.println( "\nEnd" );
    });

    // Action a faire pendans la mise a jour
    ArduinoOTA.onProgress([]( unsigned int progress, unsigned int total ) 
    {
        ////Serial.printf( "Progress: %u%%\r", ( progress / ( total / 100 ) ) );
    });

    // Action a faire en cas d erreur de la mise a jour
    ArduinoOTA.onError([]( ota_error_t error ) 
    {
        ////Serial.printf( "Error[%u]: ", error );
        //if ( error == OTA_AUTH_ERROR ) ////Serial.println( "Auth Failed" );
        //else if ( error == OTA_BEGIN_ERROR ) ////Serial.println( "Begin Failed" );
        //else if ( error == OTA_CONNECT_ERROR ) ////Serial.println( "Connect Failed" );
        //else if ( error == OTA_RECEIVE_ERROR ) ////Serial.println( "Receive Failed" );
        //else if ( error == OTA_END_ERROR ) ////Serial.println( "End Failed" );
    });
    
    ArduinoOTA.begin();
    ////Serial.println( "OTA Mode Ready" );
}

//************   Routine   ************

void checkOta() 
{
    // a mettre dans la boucle pour attendre une mise a jour
    ArduinoOTA.handle();
}

// ================================================================
// ===                            IMU                           ===
// ================================================================

//************   Variables   ************

MPU6050 mpu;

// status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

// MPU control
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//************   Initialisation   ************

void InitImu() 
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin( IMU_SDA, IMU_SCL );

    // initialize device
    mpu.initialize();
    
    // verify connection
    mpu.testConnection();

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    
    // Setup Accel offsets
    mpu.setXAccelOffset(-7029);
    mpu.setYAccelOffset(-1294);
    mpu.setZAccelOffset(1200);
    
    // Setup gyro offsets
    mpu.setXGyroOffset(147);
    mpu.setYGyroOffset(72);
    mpu.setZGyroOffset(25);
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
}

//************   Mise a jour des angles   ************

float UpdateAngles()
{
  if ( dmpReady )
  {
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
    
        // verifie si un packet est present
        // A CHANGER !!! pour utiliser les interuptions ( if => while )
        if ( fifoCount < packetSize) return 0;
        else 
        {
            // read all packet from FIFO until the last
            while (fifoCount >= packetSize)
            {
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                fifoCount = mpu.getFIFOCount();
            }
            // nettoie les miettes
            if (fifoCount != 0) mpu.resetFIFO();
    
            // Transformation geometrique
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
            // et renvoie l angle par rapport la vertical en degree 
            return (ypr[1] * 180/M_PI);
        }
    }
    else return 0;
}

// ================================================================
// ===                          ENCODER                         ===
// ================================================================

//************   Initialisation   ************

void InitEnc() 
{
    Encoder EncL( ENCODER_L_F, ENCODER_L_B );
    Encoder EncR( ENCODER_R_F, ENCODER_R_B );
}

void EncLF()
{
    sendUdp( "encoder L F " + String(digitalRead( ENCODER_L_F )));
}

void EncLB()
{
    sendUdp( "encoder L B " + String(digitalRead( ENCODER_L_B )));
}

void EncRF()
{
    sendUdp( "encoder R F " + String(digitalRead( ENCODER_R_F )));
}

void EncRB()
{
    sendUdp( "encoder R B " + String(digitalRead( ENCODER_R_B )));
}

void InitManEnc()
{
    pinMode(ENCODER_L_F, INPUT_PULLUP);
    pinMode(ENCODER_L_B, INPUT_PULLUP);
    pinMode(ENCODER_R_F, INPUT_PULLUP);
    pinMode(ENCODER_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt( ENCODER_L_F ), EncLF, CHANGE );
    attachInterrupt(digitalPinToInterrupt( ENCODER_L_B ), EncLB, CHANGE );
    attachInterrupt(digitalPinToInterrupt( ENCODER_R_F ), EncRF, CHANGE );
    attachInterrupt(digitalPinToInterrupt( ENCODER_R_B ), EncRB, CHANGE );
}

void StopEnc()
{
    detachInterrupt(digitalPinToInterrupt( ENCODER_L_F ) );
    detachInterrupt(digitalPinToInterrupt( ENCODER_L_B ) );
    detachInterrupt(digitalPinToInterrupt( ENCODER_R_F ) );
    detachInterrupt(digitalPinToInterrupt( ENCODER_R_B ) );
}
// ================================================================
// ===                            PID                           ===
// ================================================================

//************   Coefficients   ************

// kPID Stabilite
float Kps = 39;
float Kis = 6.6;
float Kds = 59.5;
float Vertical = 1.9;

// kPID Deplacement
float Kpd = 0.3;
float Kid = 0;
float Kdd = 2;
int Origine = 0;

// kPID Rotation
float Kpr = 0;
float Kir = 0;
float Kdr = 0;
int Direction = 0;

//************   Variables   ************

// variable pid Stabilisation
float somme_erreurs_s = 0;
float erreur_precedente_s = 0;

// variable pid deplacement
float somme_erreur_d = 0;
int erreur_precedente_d = 0;

// variable pid rotation
float somme_erreur_r = 0;
int erreur_precedente_r = 0;

//************   Modification des Coefficients   ************

void set_pid( String data )
{
    String commande = data.substring( 0, data.indexOf(' ') );
    float val = data.substring( data.indexOf(' ') ).toFloat();
    if ( commande == "kPs") Kps = val;
    else if ( commande == "kIs" ) Kis = val ;
    else if ( commande == "kDs" ) Kds = val ;
    else if ( commande == "kPd" ) Kpd = val ;
    else if ( commande == "kId" ) Kid = val ;
    else if ( commande == "kDd" ) Kdd = val ;
    else if ( commande == "kPr" ) Kpr = val ;
    else if ( commande == "kIr" ) Kir = val ;
    else if ( commande == "kDr" ) Kdr = val ;
    else if ( commande == "Vertical" ) Vertical = val ;
    else if ( commande == "Origine" ) Origine = int( val );
    else if ( commande == "Direction" ) Direction = int( val );
}

//************   Envoie des Coefficients   ************

String get_pid()
{
    String msg = "PID";
    msg += " kPs " + String( Kps );
    msg += " kIs " + String( Kis );
    msg += " kDs " + String( Kds );
    msg += " kPd " + String( Kpd );
    msg += " kId " + String( Kid );
    msg += " kDd " + String( Kdd );
    msg += " kPr " + String( Kpr );
    msg += " kIr " + String( Kir );
    msg += " kDr " + String( Kdr );
    msg += " Vertical " + String( Vertical );
    msg += " Origine " + String( Origine );
    msg += " Direction " + String( Direction );
    msg += " " ;
    return msg ;
}

//************   Calcul du PID de Stabilisation   ************

float stab(float Angle, float correction = 0 )
{
    // joue sur la vitesse / direction des moteurs pour maintenir l equilibre
    // entree : angle => vitesse moteurs

    //variables
    float erreur ;
    float P ;
    float I ;
    float D ;
    float PID;

    // P
    erreur = Vertical - Angle - correction ;
    P = Kps * erreur ;

    // I
    somme_erreurs_s = constrain( somme_erreurs_s + erreur / 10 , -100, 100 );
    I = Kis * somme_erreurs_s ;
    
    // D
    D = Kds * ( erreur - erreur_precedente_s );
    erreur_precedente_s = erreur ;

    // PID
    // limitation du PID
    PID = constrain( (P + I + D) , -255, 255 );
    
    // envoie les valeurs
    if ( SEND == "Stability" )
    {
        // envoie les valeurs
        sendUdp( "Stability " + String( millis() ) + " " + String( erreur ) + " " + String( PID ) + " " + String( P ) + " " + String( I ) + " " + String( D ) + " " );
    }
    return PID ;
}

//************   Calcul du PID de deplacement   ************

float dep( int consigne = 0 ) 
{
    // joue sur l angle pour maintenir la position / atteindre une position
    // entree : valeurs des encodeurs + consigne de deplacement => sortie : correction d angle
    
    //variables
    int erreur ;
    float P ;
    float I ;
    float D ;
    float PID;

    // P
    erreur = - Origine - (EncL.read()+ EncR.read())/2 - consigne ; 
    P = Kpd * erreur ;
    
    // I
    somme_erreur_d = constrain( somme_erreur_d + erreur / 10, -100, 100 );   // -100 100
    I = Kid * somme_erreur_d ;

    // D
    D = Kdd * ( erreur - erreur_precedente_d );
    erreur_precedente_d = erreur ;

    // PID    
    // limitation du PID
    PID = constrain( (P + I + D) / 10 , -5, 5 );

    // envoie les valeurs
    //Serial.println( SEND );
    if ( SEND == "Deplacement" )
    {
        // envoie les valeurs
        sendUdp( "Deplacement " + String( millis() ) + " " + String( erreur ) + " " + String( PID ) + " " + String( P ) + " " + String( I ) + " " + String( D ) + " " );
    }
    return PID ;
}

//************   Calcul du PID de rotation   ************

float rot( int consigne = 0) 
{
    // joue sur la vitesse des moteurs gauche / droit pour maintenir la direction
    // entree : valeurs des encodeurs + consigne de rotation => sortie : correction de rotation ( +G -D)
    
    //variables
    int erreur ;
    float P ;
    float I ;
    float D ;
    float PID;

    // P
    erreur = Direction - consigne - EncL.read() + EncR.read() ;
    P = Kpr * erreur ;

    // I
    somme_erreur_r = constrain( somme_erreur_r + erreur, -100, 100 );   // -100 100
    I = Kir * somme_erreur_r ;

    // D
    D = Kdr * ( erreur = - erreur_precedente_r) ;
    erreur_precedente_r = erreur ;

    // PID
    // limitation du PID
    PID = constrain(  P + I + D , -20, 20);  

    // envoie les valeurs
    if ( SEND == "Rotation" )
    {
        // envoie les valeurs
        sendUdp( "Rotation " + String( millis() ) + " " + String( erreur ) + " " + String( PID ) + " " + String( P ) + " " + String( I ) + " " + String( D ) + " " );
    }
    return PID ;
}

//************   Mise a jour des vitesses   ************

int updateSpeed(float angle, float correction)
{
    // si chute
    if ( angle > 30 || angle < -30 )
    {
        // arret des moteurs
        ////Serial.print("chute\t");
        if ( SEND == "Stability" )
        {
             sendUdp( "Stability " + String( millis() ) + " " + String( -angle ) + " 0 0 0 0 ");
        }
        return 0;
    }
    // sinon calcul le PID
    else 
    {
        //calcul du PID
        return stab( angle, correction );
    }
}

// ================================================================
// ===                          MOTORS                          ===
// ================================================================

//************   Initialisation   ************

void InitMotors() 
{
    //Motor Left SETUP
    pinMode(MOTOR_L_S, OUTPUT);
    analogWrite(MOTOR_L_S, 0);
    pinMode(MOTOR_L_F, OUTPUT);
    digitalWrite(MOTOR_L_F, LOW);
    pinMode(MOTOR_L_B, OUTPUT);
    digitalWrite(MOTOR_L_B, LOW);
    
    //Motor Right SETUP
    pinMode(MOTOR_R_S, OUTPUT);
    analogWrite(MOTOR_R_S, 0);
    pinMode(MOTOR_R_F, OUTPUT);
    digitalWrite(MOTOR_R_F, LOW);
    pinMode(MOTOR_R_B, OUTPUT);
    digitalWrite(MOTOR_R_B, LOW);

    analogWriteRange(255);
}

//************   change la vitesse des moteurs   ************

void motorsWrite(int speedL, int speedR)
{
    // ajuste la vitesse et direction des MOTORS

    ////Serial.println( "MOTOR_L : " + String( speedL ) + "MOTOR_R : " + String( speedR ) );
    if( speedR >= 0 )
    {
        digitalWrite( MOTOR_L_B, LOW);
        analogWrite( MOTOR_L_S, speedR );
        digitalWrite( MOTOR_L_F, HIGH);
        //dirD = 1;
    }
    else
    {
        digitalWrite( MOTOR_L_F, LOW);
        analogWrite( MOTOR_L_S, -speedR );
        digitalWrite( MOTOR_L_B, HIGH);
        //dirD = -1;
    }
    if( speedL >= 0 )
    {
        digitalWrite( MOTOR_R_B, LOW);
        analogWrite( MOTOR_R_S, speedL );
        digitalWrite( MOTOR_R_F, HIGH);
        //dirG = 1;
    }
    else
    {
        digitalWrite( MOTOR_R_F, LOW);
        analogWrite( MOTOR_R_S, -speedL );
        digitalWrite( MOTOR_R_B, HIGH);
        //dirG = -1;
    }
}

//************   arret des moteurs   ************

void motorsStop()
{
    // stop les MOTORs => roues libre
    analogWrite( MOTOR_L_S, 0 );
    analogWrite( MOTOR_R_S, 0 );
}

//************   change la vitesse des moteurs   ************

void motorsSpeed( int speedM, int diff = 0 )
{
    if ( !PAUSE ) motorsWrite( speedM + diff, speedM - diff );
    else motorsStop();
}

//************   freine les moteurs   ************

void motorsBrake()
{
    // Freine les MOTORs => roues Bloquees
    digitalWrite( MOTOR_L_F, HIGH );
    digitalWrite( MOTOR_L_B, HIGH );
    digitalWrite( MOTOR_R_F, HIGH );
    digitalWrite( MOTOR_R_B, HIGH );
    analogWrite( MOTOR_L_S, 255 );
    analogWrite( MOTOR_R_S, 255 );
    
    delay(100); // relache les roues apres 100ms pour pas tout cramer
    digitalWrite( MOTOR_L_F, LOW );
    digitalWrite( MOTOR_L_B, LOW );
    digitalWrite( MOTOR_R_F, LOW );
    digitalWrite( MOTOR_R_B, LOW );
    analogWrite( MOTOR_L_S, 0 );
    analogWrite( MOTOR_R_S, 0 );
}

// ================================================================
// ===                          BATTERY                         ===
// ================================================================

// variables pour la battery
const int AvgC = 10;
float prevC[AvgC];
int prevCI = 0;
long batTemp;
int batNotRready = AvgC ;

void InitBattery() 
{
    pinMode( A0, INPUT ) ;
    prevC[prevCI] = analogRead( A0 ) / 1024.0 * 10 ;
    batTemp = millis() + 100 ;
}

void Battery() 
{
    if ( batNotRready ) 
    {
        prevCI = ( prevCI + 1 ) % AvgC ;
        prevC[ prevCI ] = analogRead( A0 ) / 1024.0 * 10 ;  // lecture de la batterie en volt
        float sum = 0 ;
        for ( int i = 0; i < prevCI; i++ ) sum += prevC[i] ;
        sendUdp( "bat " + String( sum / prevCI - 0.08 )) ;
        batNotRready -= 1 ;
    }
    else if ( batTemp < millis() ) 
    {
        prevCI = (prevCI + 1) % AvgC ;
        prevC[prevCI] = analogRead( A0 ) / 1024.0 * 10 ;  // lecture de la batterie en volt
        float sum = 0 ;
        for ( int i = 0; i < AvgC; i++ ) sum += prevC[i] ;
        sendUdp( "bat " + String( sum / AvgC - 0.08 )) ;
        batTemp = millis() + 500 ;
    }
}

// ================================================================
// ===                            FILE                          ===
// ================================================================

//************   LOAD   ************

bool loadConfig()
{
    SPIFFS.begin() ;
    // open file for reading.
    File configFile = SPIFFS.open( "/balancebot/conf.txt", "r" );
    if (!configFile) return false;
  
    // Read content from config file.
    String content = configFile.readString();
    configFile.close();

    String commande = content.substring( 0, content.indexOf( " " ) );

    if ( commande == "PID" )
    {
        content = content.substring( content.indexOf( " " ) + 1 );
;       while ( !content.startsWith( "\r\n" ) )
        {
            int pos = content.indexOf( " ", content.indexOf( " " ) + 1 );
            set_pid( content.substring( 0, pos ));
            content = content.substring( pos + 1 );
        }
    }
    return true;
}

//************   SAVE   ************

bool saveConfig()
{
    // Open config file for writing.
    File configFile = SPIFFS.open( "/balancebot/conf.txt", "w+" );
    if ( !configFile ) return false;
    // Save PID
    configFile.println( get_pid() );
    configFile.close();
    return true;
}

// ================================================================
// ===                          PARSER                          ===
// ================================================================

void parser( String packet )
{
    // 
    if ( packet != "" ) 
    {
        packet.trim() ;
        //Serial.println( packet ) ;
        sendUdp( "ar " + packet ) ;
        String commande = packet.substring( 0, packet.indexOf(' ') ) ;
        String data = packet.substring( packet.indexOf(' ') + 1 ) ;
            
        if ( commande == "set" ) set_pid( data ) ;
        else if ( commande == "MODE" ) MODE = data.substring( 0, data.indexOf(' ') ) ;
        else if ( commande == "get" ) SEND = data.substring( 0, data.indexOf(' ') ) ;
        else if ( commande == "PID" ) sendUdp( get_pid() ) ;
        else if ( commande == "save" ) sendUdp( "save " + String( saveConfig()) ) ;
        else if ( commande == "PAUSE" )
        {
            if ( data.startsWith( "1" ) ) PAUSE = true;
            else PAUSE = false;
            sendUdp( "PAUSE " + String( PAUSE )) ;
        }
    }
}

void receivemsg()
{
    String msg = readUdp() ;
    if ( msg ) parser( msg ) ;
}

// ================================================================
// ===                           MODE                           ===
// ================================================================

//************   Run   ************

void ModeRun()
{
    InitMotors();
    //InitEnc();
    InitImu();
    loadConfig();
    mpu.resetFIFO();
    
    while ( MODE == "RUN" ) 
    {
        receivemsg() ;
        Battery();
        //sendUdp( "RUN" ) ;
        float angle;
        angle = UpdateAngles();
        if ( angle ) motorsSpeed( updateSpeed( angle , dep() )); //, rot() );
        rot();
        //sendUdp( String( angle ) );
    }
}

//************   OTA   ************

void ModeOTA()
{
    InitOta();
    while ( MODE == "OTA" )
    {
        receivemsg() ;
        Battery();
        checkOta();
    }
}

//************   Cal   ************

void  Modecal()
{
    InitMotors();
    //InitEnc();
    InitImu();
    InitManEnc();
    while ( MODE == "CAL" ) 
    {
        receivemsg() ;
        Battery();
    }
    StopEnc();
}

// ================================================================
// ===                           MAIN                           ===
// ================================================================

void setup() 
{
    InitMotors();
    InitBattery();
    InitWifi();
    InitUdp();
    Battery();
}

void loop() 
{
    receivemsg() ;
    Battery() ;
    if ( MODE == "OTA" ) ModeOTA() ;
    else if ( MODE == "RUN" ) ModeRun() ;
    else if ( MODE == "CAL" ) Modecal() ;
}



