// MPU6050 example
// https://wokwi.com/arduino/projects/305937248748044864

// Projetos utilizados como exemplo:
// https://wokwi.com/projects/322577683855704658
// https://wokwi.com/projects/320964045035274834
// https://www.filipeflop.com/blog/adafruit-io-plataforma-iot/
// https://www.filipeflop.com/blog/faca-seu-rastreador-veicular-com-esp32-gps-e-freertos/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

SemaphoreHandle_t xSerialSemaphore;

xQueueHandle xFila;

int tamFila = 5;

#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASS ""

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "mateussiil" // Seu usuario cadastrado na plataforma da Adafruit
#define AIO_KEY         "aio_iSKl28vowh93S0M5grv4FHFdnXvX"       // Sua key da dashboard
#define TOPIC           "/feeds/queda"       // Sua key da dashboard

void TaskBuzzer( void *pvParameters );
void TaskLed( void *pvParameters );
void TaskSensorRead( void *pvParameters );

Adafruit_MPU6050 mpu;

const int pinoLED = 26; 
const int pinoBuzzer = 27;

typedef struct{
  int dx;
  int dy;
  int dz;
} Data;


WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish _mcp9808 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME TOPIC, MQTT_QOS_1);


/*************************** Declaração dos Prototypes ************************************/
 
void initSerial();
void initPins();
void initQueue();
void initSemaphore();
void initWiFi();
void initMQTT();
void initMCP9808();
void conectar_broker();
 
/*************************** Sketch ************************************/
 

void setup(void) {
  initSerial();
  initPins();
  initQueue();
  initSemaphore();
  initWiFi();
  initMQTT();
  conectar_broker();
  while (!mpu.begin()) {
    delay(1000);
  }

  Serial.println("MPU6050 ready!");
  
  xTaskCreate( TaskSensorRead, "Sensor", 10000, NULL, 1, NULL );
  xTaskCreate( TaskBuzzer, "Buzzer", 10000, NULL, 2, NULL );
  xTaskCreate( TaskLed, "Led", 10000, NULL, 3, NULL );
 
  delay(100);
}

sensors_event_t accelerometer;

int oldAcx,oldAcy,oldAcz;
boolean led=false;


void initSerial()
{
  Serial.begin(115200);

  while (!Serial) {
    ; 
  }
}

void initPins()
{
  pinMode(pinoLED, OUTPUT);
  pinMode(pinoBuzzer, OUTPUT);
}

void initQueue()
{
  xFila = xQueueCreate(5, sizeof(Data));

  if(xFila == NULL){
    Serial.println("Erro criando fila");
  }
}

void initSemaphore()
{
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
}


/* Configuração da conexão WiFi */
void initWiFi() {
  Serial.print("Conectando-se na rede "); Serial.println(WIFI_SSID);
 
  WiFi.begin(WIFI_SSID, WIFI_PASS);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
 
  Serial.println("Conectado à rede com sucesso"); Serial.println("Endereço IP: "); Serial.println(WiFi.localIP());
}

/* Configuração da conexão MQTT */
void initMQTT() {
}


/* Conexão com o broker e também servirá para reestabelecer a conexão caso caia */
void conectar_broker() {
  int8_t ret;
 
  if (mqtt.connected()) {
    return;
  }
 
  Serial.println("Conectando-se ao broker mqtt...");
 
  uint8_t num_tentativas = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Falha ao se conectar. Tentando se reconectar em 5 segundos.");
    mqtt.disconnect();
    delay(5000);
    num_tentativas--;
    if (num_tentativas == 0) {
      Serial.println("Seu ESP será resetado.");
      while (1);
    }
  }
 
  Serial.println("Conectado ao broker com sucesso.");
}

void TaskLed( void *pvParameters )
{
  (void) pvParameters;
  for (;;)
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      if(!led){
        digitalWrite(pinoLED,HIGH);
        led = true;
      }else{
        digitalWrite(pinoLED,LOW);
        led = false;
      }
      
      xSemaphoreGive( xSerialSemaphore );
    } 
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  vTaskDelete( NULL );
}

void TaskBuzzer( void *pvParameters )
{
  (void) pvParameters;
  Data data;
  BaseType_t xStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

  // xSemaphoreTake(xSerialSemaphore, portMAX_DELAY );
  // initWiFi();
  // xSemaphoreGive(xSerialSemaphore);
  
  // /* Inciializa mqtt e faz conexao ao broker */ 
  // xSemaphoreTake(xSerialSemaphore, portMAX_DELAY );
  // initMQTT();
  // xSemaphoreGive(xSerialSemaphore);

  for (;;)
  {
    vTaskDelay( 1000 / portTICK_PERIOD_MS );

    xStatus = xQueueReceive( xFila, &data, xTicksToWait );

    if(xStatus == pdPASS){
      Serial.print("[");
      Serial.print(millis());
      Serial.print("] X: ");
      Serial.print(data.dx);
      Serial.print(", Y: ");
      Serial.print(data.dy);
      Serial.print(", Z: ");
      Serial.print(data.dz);
      Serial.println(" m/s^2");


      if(data.dx>8.5 || data.dy>8.5 || data.dz>8.5){
        tone(pinoBuzzer, 260);
        if (! _mcp9808.publish("caiu")) {
          Serial.println("Falha ao enviar pro servidor");
        }
        Serial.println("CAIUUUUUU");
      }else{
        if (! _mcp9808.publish("Tudo bem")) {
          Serial.println("Falha ao enviar pro servidor");
        }
      }
    }
  }

  vTaskDelete( NULL );
}

void TaskSensorRead(void *pvParameters)
{
  (void) pvParameters;
  Data data;
  
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  
  BaseType_t xStatus;

  for (;;)
  {
    mpu.getAccelerometerSensor()->getEvent(&accelerometer);
    
    xSemaphoreTake(xSerialSemaphore, portMAX_DELAY );
    
    data.dx = abs(accelerometer.acceleration.x-oldAcx);
    data.dy = abs(accelerometer.acceleration.y-oldAcy);
    data.dz = abs(accelerometer.acceleration.y-oldAcz);

    oldAcx=accelerometer.acceleration.x;
    oldAcy=accelerometer.acceleration.y;
    oldAcz=accelerometer.acceleration.z;

    xStatus = xQueueSendToFront( xFila, &data, xTicksToWait );
    if( xStatus == pdPASS ) {
      Serial.println("Enviando valores pela fila");
    }

    xSemaphoreGive(xSerialSemaphore);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  vTaskDelete( NULL );
}

void loop() {
}