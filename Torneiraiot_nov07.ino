// Projeto De TCC
// Academico Rafael Zavalik Castro
// Sistema embarcado de Higenização de Mãos
// Professor Orientador: Dr. Alexandre Roque
// Versão 12
// Data de 11-11-2023

// Bibliotecas utilizadas 
#include <Wire.h> 
#include <LiquidCrystal_I2C.h> //BiBlioteca do Dislpay LCD
#include <Adafruit_NeoPixel.h> //Bliblioteca dos Leds Endereçavéis
#include <WiFi.h> //Biblioteca do WIFI
#include <WiFiClientSecure.h> //Biblioteca protocolos de Segurança
#include <SPI.h> // Biblioteca de comunicação SPI com a placa RFID
#include <MFRC522.h> // Biblioteca da placa de leitura RFID  MHZ
#include <MQTT.h> //Biblioteca MQTT
#include <NTPClient.h> // Data e Hora
#include <Losant.h> //Biblioteca de comunicação com o losant plataforma
//Definições In/Out e parametros

#define SENSOR_A_PIN 32 //Sensor de Entrada E1
#define SENSOR_B_PIN 33 //Sensor de Entrada E2
#define ATUADOR_A_PIN 16 // Atuador torneira
#define RGB_PIN 4 //Pino de dados dos leds enderençaveis
#define NUMLED 11 //Número de led endereçaveis
#define SOUND_SPEED 0.034 //Velocidade do som para calculo de distancia
#define CM_TO_INCH 0.393701
#define SS_PIN 15 //D3
#define RST_PIN 13 //D2


//CONSTANTES:
const int trigPin = 12;
const int echoPin = 14;

//Certificado MQTT LOSANT
const char* rootCABuff = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
"QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
"CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
"nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
"43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
"T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
"gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
"BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
"TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
"DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
"hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
"06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
"PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
"YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
"CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n" \
"-----END CERTIFICATE-----\n";

// VARIAVEIS:
int RGB [7][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255},{255, 255, 255},{0, 255, 255},{255, 0, 255}, {255,255,0}}; 
int n_access;   // numeros de acessos ao local
int n_exits; // numeros de sáidas do local
int n_occupation; // Numero de ocupantes
float n_opportunities; // Numero de oportunidades
int n_statusled; //Numero do status do painel led informativo
float tx_HM; //Taxa de Higenização
float n_hig;// numero de Higenizações
long timenow;
long timepast = 0;
int d= 0;
int stepHM;
long duration;
float distanceCm;
float l_openwater = 15;
float l_abort = 80;
float distanceInch;
char topic[100];
char payload[100];
String TAG;
String conteudo = "";
LiquidCrystal_I2C lcd(0x27,20,4);  // Seta o endereço  (0x27) para comunicação do display de 20 colunas e 4 linhas
Adafruit_NeoPixel leds(NUMLED, RGB_PIN, NEO_GRB + NEO_KHZ800); //seta configurações leds RGB endereçaveis
MFRC522 mfrc522(SS_PIN, RST_PIN);   // define os pinos de controle do modulo de leitura de cartoes RFID

//WiFi credenciais.
const char ssid[] = "wifitorneira"; // nome da rede Wifi
const char pass[] = "12345678"; // senha da rede
WiFiClientSecure wifiClient;

// Losant MQTT Credenciais.
const char* LOSANT_DEVICE_ID = "652d1fecdc0ce93585f6f4a9"; //TorneiraIoT
const char* LOSANT_ACCESS_KEY = "439cc3d6-c83e-41fd-96e5-0b930aca5745";
const char* LOSANT_ACCESS_SECRET = "26154fa26f0aeb04dabf78e7056d3eda7f1731cc934fcc24182e779a5a4c5400";

//Dispositivo 
LosantDevice device(LOSANT_DEVICE_ID);


//Servidor NTP Data e Hora
WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP);

MQTTClient mqttClient(256);

// Conexão LOSANT MQTT
void connectTorneira(){
// Connect to Losant.
  Serial.println();
  Serial.print("Conectando no Losant...");

  device.connectSecure(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);

  while(!device.connected()) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Conectado!");

}

void handleCommand(LosantCommand *command) {
  Serial.print("Command received: ");
  Serial.println(command->name);
  Serial.println(command->time);

  // { "foo" : 10 }
  JsonObject payload = *command->payload;
  long bar = payload["foo"];
}
void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);//inicia comunicação Serial
// Definição das I/O
  pinMode(trigPin, OUTPUT); //Definição de Trig como saida para Sensor Ultrasonico
  pinMode(echoPin, INPUT);  //Definição de Echo como entrada para Sensor Ultrasonico
  pinMode(SENSOR_A_PIN, INPUT); //Definição de sensor de entrada 1
  pinMode(SENSOR_B_PIN, INPUT); //Definição de sensor de entrada 2
  pinMode(ATUADOR_A_PIN, OUTPUT); 
  Wire.setClock(10000);//resolve problema de clock com o display
  lcd.init();// Incializa LCD
  lcd.backlight(); //Liga Backlight
  lcd.setCursor(3,0);
  lcd.print("UNIMED MISSOES");
  lcd.setCursor(2,1);
  lcd.print("Sistema embarcado de");
  lcd.setCursor(0,2);
  lcd.print("Higenizacao de maos");
  lcd.setCursor(0,3);
  lcd.print("   Projeto de TCC   ");
  delay(3000);
  
  lcd.clear();
  Serial.print("Conectando-se a "); // Serial print
  lcd.setCursor(0,0);
  lcd.print("Conectando-se a ");
  Serial.println(ssid);
  lcd.setCursor(0,1);
  lcd.print(ssid);
  WiFi.begin(ssid, pass); //Inicia a conexão WIFI
  wifiClient.setCACert(rootCABuff);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectada.");
  lcd.setCursor(0,0);
  lcd.print("WiFi conectada.");
  Serial.println("Endereço de IP: ");
  lcd.setCursor(0,1);
  lcd.print("IP =");
  lcd.println(WiFi.localIP());
  lcd.setCursor(0, 2);
  lcd.print("MAC :");
  lcd.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());
  delay(2000);

  ntp.begin();
  //GMT em segundos
  // +1 = 3600
  // +8 = 28800
  // -1 = -3600
  // -3 = -10800 (BRASIL)
 ntp.setTimeOffset(-10800); //Seta o Fusorario do Brasil
 leds.begin();// inicializa leds endereçaveis
  delay(500);
  
  SPI.begin(); // inicia a comunicacao SPI que sera usada para comunicacao com o mudulo RFID
  mfrc522.PCD_Init();  //inicia o modulo RFID
  Serial.println("RFID + ESP32 + DISPLAY I2C + WIFI");
  lcd.clear();
  
  
   delay(1000);
  
 for(int l=0; l< 8; l++) {
   for(int c=0; c< NUMLED; c++){
   leds.setPixelColor(c, leds.Color(RGB[l][0],RGB[l][1],RGB[l][2]));
   leds.show();
   delay(50);
   lcd.clear();
   }
  }
   for (int i=11; i >=0; i--){
    leds.setPixelColor(i,leds.Color(0,0,0));
    leds.show();
    delay(100);
    }
  
 leds.clear();
 n_occupation=0;
 n_opportunities=0;
 stepHM=0; 
 tx_HM=0;
 n_access=0;   
 n_exits=0;
 n_statusled = 0;
 //connectTorneira();
 lcd.setCursor(2, 0);
 lcd.print("INICIADO SISTEMA");
}

void entrar(){
n_occupation++;
n_access++;
n_opportunities = (n_opportunities + 2);
calculate();
publicardados(1);
publicardados(8);
publicardados(3);
} 

void sair(){
if (n_occupation == 0){
  Serial.print("Saída invalida");
} 
else 
{
  n_occupation--;
  n_exits++;
publicardados(2);
publicardados(8);
publicardados(3);
}
}

void lerrfid(){
 conteudo.clear();
 // conteudo == "";
 if (!mfrc522.PICC_IsNewCardPresent()){
  return;
 }
  if (!mfrc522.PICC_ReadCardSerial()){
  return;
  }
 
  Serial.print(conteudo);
    for (byte i = 0; i < mfrc522.uid.size; i++){       // faz uma verificacao dos bits da memoria do cartao
     //ambos comandos abaixo vão concatenar as informacoes do cartao...
     //porem os 2 primeiros irao mostrar na serial e os 2 ultimos guardarao os valores na string de conteudo para fazer as verificacoes
     Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
     Serial.print(mfrc522.uid.uidByte[i], HEX);
     conteudo.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
     conteudo.concat(String(mfrc522.uid.uidByte[i], HEX));
     }
TAG = conteudo;
 }
void monitorar(){
if (d == 0) { 
  if (digitalRead(SENSOR_A_PIN)== HIGH){
    d = 1; 
   // Serial.print("Sensor A");
    }
  if (digitalRead(SENSOR_B_PIN)== HIGH){
    d = 2; 
   // Serial.print("Sensor B");
    } 
}
if (d == 1){
timenow = millis();
if ( (digitalRead(SENSOR_A_PIN)== HIGH) && (digitalRead(SENSOR_B_PIN)== LOW)){
  Serial.print("<<< Saída >>>");
  d = 0;
  sair();
  delay(1000);  
}
   if (timenow - timepast >= 500) {
    timepast = timenow;
   // Leitura do Sensor B por 1000 ms, sair do loop
   d=0;
   //Serial.print("Zerado A");
   }
  }

if (d == 2){
  timenow = millis();
if ( (digitalRead(SENSOR_B_PIN)== HIGH) && (digitalRead(SENSOR_A_PIN)== LOW)) {
  Serial.print(">>> Entrada <<<");
  d = 0;
  entrar();
  delay(1000);
  }
  if (timenow - timepast >= 500) {
  timepast = timenow;
  // Leitura do Sensor B por 1000 ms, sair do loop
  d=0;
  //Serial.print("Zerado B");
  }  
}
}
// medidor de distancia
void distance(){
 digitalWrite(trigPin, LOW);
 delayMicroseconds(2);
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10);
 digitalWrite(trigPin, LOW);
 duration = pulseIn(echoPin, HIGH);
 distanceCm = duration * SOUND_SPEED/2;
}

void information(){
  Serial.print("Entradas :");
      Serial.println(n_access);
      Serial.print("Saídas :");
      Serial.println(n_exits);
      Serial.print("Ocuapação :");
      Serial.println(n_occupation);
      Serial.print("Oportunidades :");
      Serial.println(n_opportunities);
      Serial.print("Taxa Atual :");
      Serial.println(tx_HM);
}
void vsinal(){
   // Sensor torneira
    if ((stepHM==3)||(stepHM==2)){
   distance();
    if (distanceCm < l_openwater ){
      lcd.setBacklight(HIGH);
      for (int i=0;i< 12;i++){
        distance();// verifica distancia ;
      if (distanceCm > l_abort ){
        lcd.clear();
        lcd.setCursor(0,1);
        lcd.print("      Lavagem       ");
        lcd.setCursor(0,2);
        lcd.print("      Abortada      ");
        stepHM=0;
        TAG.clear();
        for (int h=0;h<11;h++){
          leds.setPixelColor(h, leds.Color(255, 0, 0));
          leds.show();
          delay(50);
          }
        delay(500);
        for (int x=0;x<11;x++){
          leds.setPixelColor(x, leds.Color(0, 0, 0));
          leds.show();
          }
          i=12;
          delay(1000);
          lcd.clear();
          stepHM=0;   
          conteudo.clear();
          TAG.clear();
          loop();       
        }  
        procedimento(i,2500);
      }
    }//
    }
}
void calculate(){
  tx_HM = (n_hig / n_opportunities)* 100;
  
}
void tela(int n){
switch (n)  {
  case 1:// Abertura
  stepHM++; //stepHM = 1
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("   UNIMED MISSOES   ");
  lcd.setCursor(0,1);
  lcd.print("   Favor Higenizar  ");
  lcd.setCursor(0,2);
  lcd.print("       as maos      ");
  distance();
  delay(2000);
  while (distanceCm > 100){
  distance();
  }
  tela(2);
  break;
  case 2: //Verifica profissional
  stepHM++; //stepHM= 2
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("   UNIMED MISSOES   ");
  lcd.setCursor(0,1);
  lcd.print("   Caso queira se   ");
  lcd.setCursor(0,2);
  lcd.print("identificar aproxime");
  lcd.setCursor(0,3);
  lcd.print("      cracha        ");
  Serial.print(conteudo);
   
  break;
  case 3:
  stepHM++; //stepHM= 3
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("    Profissional    ");
  lcd.setCursor(0,1);
  lcd.print("    identificado    ");
  lcd.setCursor(0,2);
  lcd.print(" aproxime do sensor ");
  lcd.setCursor(0,3);
  lcd.print("    para iniciar    ");
  break;

  case 4:// distancia
  distance();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("      S. E. H. M     ");
  lcd.setCursor(0,1);
  lcd.print ("    Mantenha-se     ");
  lcd.setCursor(0,2);
  lcd.print (" proximo a torneira ");
  lcd.setCursor(0,3);
  lcd.print (" durante o processo ");
  break;
  
  case 5: //apresenta tx
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("   Unimed Missoes   ");
  lcd.setCursor(0,1);
  lcd.print("  Taxa Atual de HM  ");
  lcd.setCursor(0,2);
  lcd.print(tx_HM);
  lcd.println("%");
  for (int i=0; i <11; i++){
  leds.setPixelColor((10-i),leds.Color(0,0,0));
  leds.show();
  delay(100);
  }
  delay(2000);
  lcd.clear();
  break;
  
  case 6: // informar gravação de dados
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("   Unimed Missoes   ");
  lcd.setCursor(0,1);
  lcd.print("    INFORMANDO HM   ");
  lcd.setCursor(0,2);
  lcd.print("   PARA O SERVIDOR  ");
  if (TAG = ""){
  lcd.setCursor(0,3);
  lcd.print("SEM ID PROFISSIONAL ");
  }
  else
  lcd.setCursor(0,3);
  lcd.print("ID :");
  lcd.println(TAG);
  delay(1000);
  
  break;
  } 
}
void publicardados(int p){
  if(!device.connected()) {
  connectTorneira();
  }
  
  StaticJsonDocument<200> jsonBuffer;
  JsonObject root = jsonBuffer.to<JsonObject>();
  switch (p) {

  case 1: // N_acessos
  root["N_access"] = n_access;
  // Send the state to Losant.
  device.sendState(root);
  break;
  
  case 2: // N_saidas
   root["n_exits"] = n_exits;
  // Send the state to Losant.
  device.sendState(root);
  break;

  case 3: // N_oportunidades
  root["n_occupation"] = n_occupation;
  // Send the state to Losant.
  device.sendState(root);
 
  break;
  
  case 4: // N_Hig
  root["N_opportunities"] = n_opportunities;
  // Send the state to Losant.
  device.sendState(root);
  break;

  case 5: // Tx_hig
   root["TXhig"] = tx_HM;
  // Send the state to Losant.
  device.sendState(root);
  break;

  case 6: // n_higenizações
  root["N_Sanitization"] = n_hig;
  // Send the state to Losant.
  device.sendState(root);
 
  break;
  case 7: // TAG
  Serial.print(TAG);
  root["TAG"] = TAG;
  // Send the state to Losant.
  device.sendState(root);
  break;

  case 8: // ocupação do leito
  root["N_opportunities"] = n_opportunities;
  // Send the state to Losant.
  device.sendState(root);
  break;
  
  case 9:
  root["N_access"] = n_access;
  root["n_exits"] = n_exits;
  root["TXhig"] = tx_HM;
  root["N_Sanitization"] = n_hig;
  root["n_occupation"] = n_occupation;
  root["TAG"] = TAG;
  root["N_opportunities"] = n_opportunities;
  // Send the state to Losant.
  device.sendState(root);
  break;
}
}
void procedimento(int n, int t){

switch (n)  {

  case 1: // Torneira Aberta
  stepHM++;
  leds.setPixelColor((n-1), leds.Color(RGB[2][0],RGB[2][1],RGB[2][2])); // Coloca em Azul
  leds.show();// Comando para ligar led endereçavel
  digitalWrite (ATUADOR_A_PIN, HIGH); // liga agua
  Serial.print("Torneira acionada"); 
  Serial.print("Etapa :");
  Serial.println(n);

  // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  break;
  
  case 2: // 
  stepHM++;
  leds.setPixelColor((n-1), leds.Color(RGB[3][0],RGB[3][1],RGB[3][2])); // Coloca em Amarelo
  leds.show();// Comando para ligar led endereçavel
  Serial.print("Etapa :");
  Serial.println(n);
  // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  break;

  case 3: // 
  stepHM++;
  digitalWrite (ATUADOR_A_PIN, LOW); // Desliga agua 
  leds.setPixelColor((n-1), leds.Color(RGB[3][0],RGB[3][1],RGB[3][2])); // Coloca em Amarelo
  leds.show();// Comando para ligar led endereçavel
  Serial.print("Etapa :");
  Serial.println(n);
 // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  break;

  case 4: // 
  stepHM++;
  leds.setPixelColor((n-1), leds.Color(RGB[3][0],RGB[3][1],RGB[3][2])); // Coloca em Amarelo
  leds.show();// Comando para ligar led endereçavel
  Serial.print("Etapa :");
  Serial.println(n);
  // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  break;

  case 5: // 
  stepHM++;
  leds.setPixelColor((n-1), leds.Color(RGB[3][0],RGB[3][1],RGB[3][2])); // Coloca em Amarelo
  leds.show();// Comando para ligar led endereçavel
  Serial.print("Etapa :");
  Serial.println(n);
  // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  break;

  case 6: // 
  stepHM++;
  leds.setPixelColor((n-1), leds.Color(RGB[3][0],RGB[3][1],RGB[3][2])); // Coloca em Amarelo
  leds.show();// Comando para ligar led endereçavel
  Serial.print("Etapa :");
  Serial.println(n);
  // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  break;

  case 7: // 
  stepHM++;
  leds.setPixelColor((n-1), leds.Color(RGB[3][0],RGB[3][1],RGB[3][2])); // Coloca em Amarelo
  leds.show();// Comando para ligar led endereçavel
  Serial.print("Etapa :");
  Serial.println(n);
  // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  break;

  case 8: // 
  stepHM++;
  leds.setPixelColor((n-1), leds.Color(RGB[3][0],RGB[3][1],RGB[3][2])); // Coloca em Amarelo
  leds.show();// Comando para ligar led endereçavel
  Serial.print("Etapa :");
  Serial.println(n);
  // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  break;

  case 9: // 
  stepHM++;
  leds.setPixelColor((n-1), leds.Color(RGB[3][0],RGB[3][1],RGB[3][2])); // Coloca em Amarelo
  leds.show();// Comando para ligar led endereçavel
  Serial.print("Etapa :");
  Serial.println(n);
  // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  break;

  case 10: // 
  stepHM++;
  leds.setPixelColor((n-1), leds.Color(RGB[2][0],RGB[2][1],RGB[2][2])); // Coloca em Amarelo
  leds.show();// Comando para ligar led endereçavel
  digitalWrite (ATUADOR_A_PIN, HIGH); // liga agua
  Serial.print("Torneira acionada"); 
  Serial.print("Etapa :");
  Serial.println(n);

  // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  break;

  case 11: // 
  stepHM++;
  n_hig++;
  calculate();
   publicardados(9);
  leds.setPixelColor((n-1), leds.Color(RGB[3][0],RGB[3][1],RGB[3][2])); // Coloca em Azul
  leds.show();// Comando para ligar led endereçavel
  Serial.print("Etapa :");
   Serial.println(n);
  // Informação na tela de manter proximo 
  tela(4);
  delay(t);
  leds.setPixelColor((n-1), leds.Color(RGB[1][0],RGB[1][1],RGB[1][2])); // Coloca em Verde
  leds.show();// Comando para ligar led endereçavel
  lcd.clear();
  delay(t);
  digitalWrite (ATUADOR_A_PIN, LOW); // Desliga agua 
  for (int x=0;x<11;x++){
          leds.setPixelColor(x, leds.Color(0, 0, 0));
          leds.show();
          }
    
 
   if (conteudo=""){
    TAG=="Sem ID";
    conteudo == "";
   }
   Serial.print(TAG);
    TAG.clear();
  information();
  stepHM=0;
  lcd.clear();
  loop();
  break;
}
}
void loop() {

  if(!device.connected()) {
  connectTorneira();
  }

 vsinal();
 if (stepHM == 2 ){
  while (conteudo == ""){
  lerrfid();
  vsinal();
  monitorar();
 }

 if (conteudo != ""){
    tela(3);
   }  
 } 

  if (n_occupation > 0){
    if (n_opportunities > 0){
      if(stepHM==0){
      tela(1);
      }
    }
  }
monitorar();
 }