#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient_Generic.h>
#include <DHTesp.h>

WiFiMulti         WiFiMulti;
WebSocketsClient  webSocket;
DHTesp            dht;

// SALIDAS
#define PORT_OUT_REGAR    5
#define PORT_OUT_ABONO    4
#define PORT_OUT_VENTILAR 18
#define PORT_OUT_PUERTAS  15

#define PORT_OUT_SERVER   2

// ENTRADAS
#define PORT_INT_DTH      13
#define PORT_INT_LUZ      35
#define PORT_INT_HUMEDAD  34

// CREDENCIALES WIFI
#define WIFI_SSID "OnePlus 9 5G"
#define WIFI_PASSWORD "a7wsqnmr"

// HOST WEBSOCKET SERVER
#define WS_SERVER "ws.up.railway.app"
#define WS_PORT 443

bool alreadyConnected = false;
bool serverLEDStatus = false;
bool newMessage = false;

bool accionRegar = false;
bool accionAbono = false;
bool accionVentilar = false;
bool accionPuertas = false;

unsigned long accionRegarTimestamp = 0;
unsigned long messageTimestamp = 0;
unsigned long newMessageTimestamp = 0;

void webSocketEvent(const WStype_t& type, uint8_t * payload, const size_t& length)
{
  switch (type)
  {
    case WStype_DISCONNECTED:
      if (alreadyConnected)
      {
        Serial.println("[WSc] Se ha perdido la conexión con el servidor!");
        alreadyConnected = false;
        digitalWrite(PORT_OUT_SERVER, LOW);
      }
      break;
    case WStype_CONNECTED:
    {
      alreadyConnected = true;
      Serial.print("[WSc] Conectado en la dirección: ");
      Serial.println((char *) payload);
    }
    break;
    case WStype_TEXT:
      {
        Serial.printf("[SERVER REQUEST]: %s\n", payload);

        // Convertir uint8_t a string
        String payloadString = (char*) payload;

        // Extraer los valores del payload
        int delimiterOne = payloadString.indexOf("~");
        int delimiterTwo = payloadString.indexOf("~", delimiterOne + 1);
        
        String origin = payloadString.substring(0, delimiterOne);
        String key = payloadString.substring(delimiterOne, delimiterTwo);
        String value = payloadString.substring(delimiterTwo);

        // Filtrar los mensajes
        if (origin == "microcontroller" && key == "~set-action") {
            if(value == "~regar") {
              accionRegar = true;
              accionRegarTimestamp = millis();
            }
            if(value == "~abono") accionAbono = !accionAbono;
            if(value == "~ventilar") accionVentilar = !accionVentilar;
            if(value == "~puertas") accionPuertas = !accionPuertas;
        }

        // Indicador de que estan llegando mensajes del servidor
        newMessageTimestamp = millis();
        newMessage = true;
        break;
      }
    case WStype_PING:
    case WStype_PONG:      
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  // Inicializar puertos de entrada
  initPorts();
  // Conectar a red WiFi
  connectWiFiNetwork();
  // Conectar a servidor
  connectServerSocket();
}

void initPorts() {
  // Inicializar puertos
  pinMode(PORT_OUT_SERVER, OUTPUT);
  
  pinMode(PORT_OUT_PUERTAS, OUTPUT);
  pinMode(PORT_OUT_VENTILAR, OUTPUT);
  pinMode(PORT_OUT_ABONO, OUTPUT);
  pinMode(PORT_OUT_REGAR, OUTPUT);

  dht.setup(PORT_INT_DTH, DHTesp::DHT11);
}

void connectWiFiNetwork() {
  WiFiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  
  //WiFi.disconnect();
  while (WiFiMulti.run() != WL_CONNECTED)
  {
    Serial.print(".");
    serverLEDStatus = !serverLEDStatus;
    digitalWrite(PORT_OUT_SERVER, !serverLEDStatus);
    delay(400);
  }
  
  delay(500);
}

void connectServerSocket() {
  // server address, port and URL
  Serial.print("Conectandose al servidor @ ");
  Serial.println(WS_SERVER);

  // server address, port and URL
  webSocket.beginSSL(WS_SERVER, WS_PORT);

  // event handler
  webSocket.onEvent(webSocketEvent);

  // try ever 5000 again if connection has failed
  webSocket.setReconnectInterval(5000);

  // start heartbeat (optional)
  // ping server every 15000 ms
  // expect pong from server within 3000 ms
  // consider connection disconnected if pong is not received 2 times
  webSocket.enableHeartbeat(15000, 3000, 2);

  // server address, port and URL
  Serial.print("Conectado al servidor @ Direccion IP: ");
  Serial.println(WS_SERVER);
}

void loop()
{
  // Loop para leer los mensajes entrates de sockets
  webSocket.loop();

  // Establecer valores para los puertos de salida
  digitalWrite(PORT_OUT_REGAR, accionRegar);
  digitalWrite(PORT_OUT_ABONO, accionAbono);
  digitalWrite(PORT_OUT_VENTILAR, accionVentilar);
  digitalWrite(PORT_OUT_PUERTAS, accionPuertas);

  uint64_t now = millis();

  // Control del LED de estado de la conexion del servidor
  if (alreadyConnected == true) {
    if ((now - newMessageTimestamp < 75) && newMessage == true) {
      digitalWrite(PORT_OUT_SERVER, LOW);
    } else {
      newMessageTimestamp = 0;
      newMessage = false;
      digitalWrite(PORT_OUT_SERVER, HIGH);
    }  
  }

  // Apagar la salida (del pin de del motor) siempre y cuando la accion este activa y haya pasado 4 segundos
  if ((now - accionRegarTimestamp > 4000) && accionRegar == true) {
    accionRegarTimestamp = 0;
    accionRegar = false;
  }

  // Realizar muestreo de los sensores cada 2 segundos y enviar los datos por sockets
  if(now - messageTimestamp > 2000) {
    messageTimestamp = now;

    // Obtener el valor de la humedad en la sonda de tierra
    int HUMEDAD_ADC_VALUE = analogRead(PORT_INT_HUMEDAD);
    // Obtener el valor del divisor de voltaje entre el LDR y resistencia
    int LUZ_ADC_VALUE = analogRead(PORT_INT_LUZ);

    // Tranformar los valores a porcentajes
    double HUMEDAD_PORCENTAJE = 100 - ((HUMEDAD_ADC_VALUE * 100) / 4095);
    double LUZ_PORCENTAJE = 100 - ((LUZ_ADC_VALUE * 100) / 4095);

    TempAndHumidity data = dht.getTempAndHumidity();

    // Crear el payload que despues sera enviado por sockets
    String response = "microcontroller~set-data~";
    
    response = response + LUZ_PORCENTAJE;
    response = response + "|";
    response = response + HUMEDAD_PORCENTAJE;
    response = response + "|";
    response = response + String(data.temperature, 2);
    response = response + "|";
    response = response + String(data.humidity, 1);

    // Enviar valores al servidor de sockets
    webSocket.sendTXT(response);        
  }
}
