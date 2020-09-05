// IdentificaciÓn del dispositivo. Para que sea único, pongo en el nombre la fecha de fabricación.
#define DEVICE_ID "NODEMCU-20190202"

//#define _TOTE_DEBUG_ON_   // Descomentar para mostrar info de debug por serial

// Macros para facilitar la salida de información por el Serial.
#ifdef _TOTE_DEBUG_ON_
#define _TOTE_DEBUG_(type, text) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.println(text);
#define _TOTE_DEBUG_VALUE_(type, text, value) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.print(text); Serial.println(value);
#else
#define _TOTE_DEBUG_(type, text) void();
#define _TOTE_DEBUG_VALUE_(type, text, value) void();
#endif


/*

	PINOUT DEL NodeMCU Mini D1

	Los includes de ESP8266Wifi.h y similares redefinen las constantes de los pines de Arduino para que coincidan con el pinout
	de las placas NodeMCU, por lo que se pueden usar en el código sin mayor problema.

	Ciertos pines de la placa están conectados a funciones activas del ESP8266, por lo que no se deben usar como pines de E/S, por ejemplo
	TX, RX, etc.

	Esta es la equivalencia entre los pines de la placa NodeMCU (Mimi D1) y los pines del ESP8266.

	Pin		Function						ESP - 8266 Pin
	---		------------------------------  --------------

	TX		TXD								TXD
	RX		RXD								RXD
	A0		Analog input, max 3.3V input	A0
	D0		IO								GPIO16
	D1		IO, SCL							GPIO5
	D2		IO, SDA							GPIO4
	D3		IO, 10k Pull - up				GPIO0
	D4		IO, 10k pull - up, BUILTIN_LED	GPIO2
	D5		IO, SCK							GPIO14
	D6		IO, MISO						GPIO12
	D7		IO, MOSI						GPIO13
	D8		IO, 10k pull - down, SS			GPIO15
	G		Ground							GND
	5V		5V								�
	3V3		3.3V							3.3V
	RST		Reset							RST



	Placa NodeMCU .

	IMPORTANTE: Evito usar el D3 (GPIO0) porque tiene significado especial para el arranque del dispositivo.
*/


// Configuración de los pines de NodeMCU
#define LED_POWER_ON_PIN  D1	// Pin para encender led que indica estado de encendido de la placa base.
#define AP_CONF_PIN       D2    // Para entrar en modo de selección del AP.
#define LED_WIFI_PIN      D5    // Pin para el led que indica conexión con el AP.		
#define RESET_PIN	      D6	// Pin para mandar pulso de RESET a la placa base.
#define POWER_ON_PIN	  D7	// Pin para mandar pulso de endendido a la placa base.
#define POWER_SENSOR_PIN  A0	// Pin para leer si la fuente de alimentación está encendida.

/*
	Cuando se inicia el dispositivo, entra en un bucle de espera para dar posibilidad de pulsar D2 (GPIO4). Este tiempo viene dado por el
	define CONFIG_WINDOW. El led parpadeará rápido durante este tiempo. Si se pulsa, emite un AP con el SSID dado por su correspondiente
	definición en el código, así como el PASS.

	ESP8266 emite un AP a cuya red debemos conectarnos y monta un servidor http para facilitar la configuración al AP
	verdadero. la IP de este servidor es 192.168.4.1 y nos podemos conectar con un móvil, por ejemplo. Una vez configurado, el
	dispositivo se reinicia. Debemos esperar sin hacer nada a que pase de nuevo el periodo de configuración.

	Ahora se conecta al AP correcto y después al servidor MQTT en 192.168.1.200

	Cuando el dispositivo pierde la conexión con el AP, por ejemplo, debido a que éste se ha caido o un corte eléctrico, el ESP8266 se reiniciará
	comenzando de nuevo todo el procedimiento.

 */

 // Librerías de Arduino que usará.
#include <WiFiClientSecure.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>            // Servidor DNS que se usa para redirigir todas las request al portal de configuración.
#include <ESP8266WebServer.h>     // Servidor web local que muestra el portal de configuración.
#include <time.h>

// Librerías clonadas desde Git. 
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Librerias_desde_Git\WiFiManager\WiFiManager.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Librerias_desde_Git\pubsubclient\src\PubSubClient.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Librerias_desde_Git\Time\Time.cpp"


// Librerías propias. Se encuentran disponibles en https://github.com/MisLibrerias
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteDebouncedBtn\ToteDebouncedBtn.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteBlinkOutputLed\ToteBlinkOutputLed.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteAsyncDelay\ToteAsyncDelay.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteAnalogSensor\ToteAnalogSensor.cpp"


// IMPORTANTE. VER INFORMACIÓN SOBRE EL WDT en la clase ToteESPMillisDelay
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteESPMillisDelay\ToteESPMillisDelay.cpp"


/************************************
 * PROTOTIPOS DE FUNCIONES          *
 ************************************/

void checkLeds(void);
void doWaitForConfig(void);
void doConnectSSID(void);
void doNormalRun(void);
boolean checkWiFi(void);
void checkPowerON(void);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttCheckConnectionStatusCallback(void);
void mqttReconnect();
void sendPowerOnStatusCallback(void);
void sendPulse(uint8_t thePin);
void sendHardPulse(uint8_t thePin);
void sendTestResponse(void);
void wifiCheckSignalStrengthCallback(void);
void refheshNoderedDashboardsCallback(void);


// Damos un margen de 2 segundos para que los leds parpadeen.
#define CHECKLED_WINDOW 2000UL

// Damos un margen de 10 segundos para pulsar el botón que lanza el portal de configuración.
#define CONFIG_WINDOW 10000UL  



/********************************
 * OBJETOS Y VARIABLES GLOBALES *
 ********************************/

unsigned int initialMillis;	// Almacena los milis desde el inicio del programa.
boolean isPanicked = false; // Sirve para indicar si el ESP ha perdido la conectividad con la WiFi.
long wifiSignal = 0; 		// Almacena la intensidad de la señal Wifi recibida (RSSI) en dBm (Actualizada en loop)

const char* SSID = "TOTEESPCFG20190202";      // El SSID debe ser único, pongo la fecha en la que inicio la programación (02/02/2019) como GUID.
const char* PASS = "DarthVader*1";


const char* MQTT_SERVER = "192.168.1.200";  			// IP del servidor MQTT
#define MQTT_CHECK_CONNECTION_STATUS_INTERVAL  10000UL  // Si se pierde la conexión con el servidor MQTT se reintentará pasado este tiempo.

#define MAX_MQTT_CHARS 100   // longitud de los arrays para procesar mensajes MQTT.

char mqttLastStateOpTxt[MAX_MQTT_CHARS];  // Almacena mensaje sobre la última operación con el servidor MQTT.


const char* mqttClientID = DEVICE_ID;   // El id debe ser único, pongo la fecha en la que inicio la programación (20190202) como GUID.

#define WIFI_CHECK_SIGNAL_STRENGTH 5000UL	// Tiempo (en ms) para medir la senal de la Wifi.
#define DASHBOARD_REFRESH_INTERVAL 2500UL	// Tiempo (en ms) para refrescar el Dashboard de Node-Red.


// Objeto para realizar 'delays' alimentando al WDT de software del ESP8266 para evitar los resets.
ToteESPMillisDelay myESPDelay = ToteESPMillisDelay(2000);

// Instancio objeto WiFiManager que permitirá mostrar el portal de configuración de acceso al AP.
WiFiManager myWiFiManager;

// Instancio objeto  auxiliar para PubSubClient
WiFiClient myWiFiClient;

// Instancio objeto para protocolo MQTT.
PubSubClient myMqttClient(myWiFiClient);

// Instancio un objeto ToteDebounceBtn para quitar el rebote al switch AP_CONF_PIN.
ToteDebouncedBtn  myAPConfigBtn = ToteDebouncedBtn(AP_CONF_PIN, NULL);

// Sirve para indicar el estado en el que se encuentra el dispositivo en 'loop()'
enum LOOP_RUN_MODE { CHECK_LEDS, WAIT_FOR_CONFIG, CONNECT_SSID, NORMAL_RUN };

// Modo actual de funcionamiento del programa en la función 'loop()'
int myLoopRunMode;

// Instancio un objeto BlinkOutputLed que indica el estado de la conexión con el AP.
ToteBlinkOutputLed myWiFiLed(100UL, LED_WIFI_PIN, ToteBlinkOutputLed::FINITE_BLINK, NULL); // Será obligatorio llamar a su método 'init' en 'setup'.

// Instancio un objeto BlinkOutputLed que indica el estado de encendido de la placa base.
ToteBlinkOutputLed myPowerONLed(100UL, LED_POWER_ON_PIN, ToteBlinkOutputLed::FINITE_BLINK, NULL); // Será obligatorio llamar a su método 'init' en 'setup'.

// Temporizador de indicará cuando hay que testear la conexión con el servidor MQTT.
ToteAsyncDelay myMQTTCheckConnectionStatusTimer(MQTT_CHECK_CONNECTION_STATUS_INTERVAL, mqttCheckConnectionStatusCallback);

#define VOLTAGE_LEVEL 200  // El pin analágico lee de 0-1023. Como busco un valor de 5V (convertido a 3.3 por el hardware) un limite de 200 equivale a 0.64V, suficiente para
						   // determinar si el led de encendido de la torre tiene tensión, ya que las medidas del polímetro para el led de la torre
						   // me dieron un valor de 1.9V.

// Instancio un objeto ToteAnalogSensor para leer el valor de tensión del led de encendido de la placa base.
ToteAnalogSensor myMBSensor(POWER_SENSOR_PIN, ToteAnalogSensor::GREATERTHAN, VOLTAGE_LEVEL, VOLTAGE_LEVEL, 50, 1000UL, NULL); // Será obligatorio llamar a su método 'init' en 'setup'.

// Instancio un objeto temporizador que indicará cuando enviar el estado de encendido del PC al Broker MQTT.
#define SEND_POWER_ON_STATUS_INTERVAL 5000UL
ToteAsyncDelay myPowerOnStatusTimer(SEND_POWER_ON_STATUS_INTERVAL, sendPowerOnStatusCallback);

// Instancio un objeto para realizar esperas asíncronas.
ToteESPMillisDelay my500msDelay(500UL);

// Otro
ToteESPMillisDelay my8000msDelay(8000UL);

#define MAX_PASSWORD 20	// longitud máxima del password para los comandos de inicio y reseteo.
char lastPassword[MAX_PASSWORD];

#define COMMAND_PASSWORD "2581" // El password a introducir para poder ejecutar comandos de encendido y apagado.

// Indica si se ha recibido hora desde el topic de NodeRed.
bool isValidTime = false;
time_t theTime;

// Este temporizador mide la intensidad de la señal WiFi.
ToteAsyncDelay myWiFiCheckSignalStrength(WIFI_CHECK_SIGNAL_STRENGTH, wifiCheckSignalStrengthCallback);

// Temporizador para actualizar los objetos de los Dashboards.
ToteAsyncDelay myNodeRedDashboardRefreshTimer(DASHBOARD_REFRESH_INTERVAL, refheshNoderedDashboardsCallback);




/*********************************************************
 * FIN DE LA DECLARACIÓN DE OBJETOS Y VARIABLES GLOBALES *
 *********************************************************/


void setup() {

#ifdef _TOTE_DEBUG_ON_
	// Configuramos la conexi�n serie
	Serial.begin(115200);
#endif
	// Defino los pines 'RESET_PIN' y 'POWER_ON_PIN' como salidas. Los demás pines lo inicializan los respectivos objetos.
	pinMode(RESET_PIN, OUTPUT);
	pinMode(POWER_ON_PIN, OUTPUT);

	// Inicialización del objeto MQTT
	myMqttClient.setServer(MQTT_SERVER, 1883);
	myMqttClient.setCallback(mqttCallback);

	// Inicializamos el objeto que gestiona el LED WiFi.
	myWiFiLed.init();

	// Inicializamos el objeto que gestiona el LED de equipo encendido.
	myPowerONLed.init();

	// Inicializamos el objeto que gestiona el botón para entrar en modo de configuración de AP.
	myAPConfigBtn.init();

	// Vamos a dar un segundo para que se estabilice todo.
	myESPDelay.stop(1000UL);

	// Limpiamos un poco la ventana del monitor serie.
	_TOTE_DEBUG_("setup()", "--------------------------------------------------------------------------");
	_TOTE_DEBUG_("setup()", "              INICIANDO LA EJECUCION DEL CODIGO");
	_TOTE_DEBUG_("setup()", "--------------------------------------------------------------------------");

	// Activo el parpadeo de los led, para comprobar que no estén fundidos.
	myWiFiLed.setBlinkType(ToteBlinkOutputLed::INFINITE_BLINK);
	myWiFiLed.ledON();

	myPowerONLed.setBlinkType(ToteBlinkOutputLed::INFINITE_BLINK);
	myPowerONLed.ledON();

	// Entramos en loop() en modo de chequeo de leds.
	myLoopRunMode = LOOP_RUN_MODE::CHECK_LEDS;

	// Iniciamos el temporizador
	initialMillis = millis();

	_TOTE_DEBUG_("setup", "Entrando en loop()");
}

void loop() {
	switch (myLoopRunMode) {
	case CHECK_LEDS:
		checkLeds();
		break;

	case WAIT_FOR_CONFIG:
		doWaitForConfig();
		break;

	case CONNECT_SSID:
		doConnectSSID();
		break;

	case NORMAL_RUN:
		doNormalRun();
		break;
	}

	// Actualizo objetos que lo necesitan en cada iteración del loop().
	myWiFiLed.check();

	// Leo voltaje de la placa base.
	myMBSensor.check();

	// Compruebo estado de encendido.
	checkPowerON();
}

void checkLeds(void) {
	// Esta función realiza una simple espera para dar tiempo a que parpadeen varias veces los leds.
	if (initialMillis + CHECKLED_WINDOW > millis())
		return; // Me quedo en checkLeds sin avanzar durante CHECKLED_WINDOW segundos

	// Cuando llegue aquí es porque ya han pasado el tiempo de parpadeo.
	// Apago los leds.
	myWiFiLed.ledOFF();
	myPowerONLed.ledOFF();

	_TOTE_DEBUG_("checkLeds", "Comprobacion finalizada. Entrando en ventana de configuracion de AP.");

	// Iniciamos el temporizador para el siguiente estado.
	initialMillis = millis();

	// Pongo el led Wifi a parpadear.
	myWiFiLed.setBlinkType(ToteBlinkOutputLed::INFINITE_BLINK);
	myWiFiLed.ledON();

	// Pongo el led Wifi de encendido a parpadear
	myPowerONLed.setBlinkType(ToteBlinkOutputLed::INFINITE_BLINK);
	myPowerONLed.ledON();

	// Pasamos al siguiente estado.
	myLoopRunMode = LOOP_RUN_MODE::WAIT_FOR_CONFIG;
}

void doWaitForConfig(void) {
	// Ventana de lanzamiento del portal de configuración.
	// Compruebo si se ha pulsado el botón o si la WiFi está desconectada.
	if (myAPConfigBtn.check()) {
		_TOTE_DEBUG_("doWaitForConfig", "Se ha pulsado el boton para lanzar el portal de configuracion.");

		// Se ha pulsado. Invocamos al portal de configuración
		// Creamos el AP con la siguiente configuración.
		myWiFiManager.startConfigPortal(SSID, PASS);

		// Mientras el ESP está en modo AP, nos conectamos a él con un navegador en la IP 192.168.4.1, 
		// configuramos la WIFI, la guardamos y reiniciamos el dispositivo para que se conecte al AP deseado,
		// ya sin pulsar el botón de configuración

		// Cambiamos el modo de ejecución.
		myLoopRunMode = LOOP_RUN_MODE::NORMAL_RUN;

		// Congelo la animación del led y lo apago ya que se ha terminado el proceso de configuración con el AP.
		myWiFiLed.setBlinkType(ToteBlinkOutputLed::NO_BLINK);
		myWiFiLed.ledOFF();

		// Salimos.
		return;
	}

	// Compruebo si ha terminado el periodo de configuración en cuyo caso cambiamos el modo de ejecución.
	if (initialMillis + CONFIG_WINDOW < millis()) {
		_TOTE_DEBUG_("doWaitForConfig", "Se ha acabado el tiempo de espera para lanzar el portal de configuracion");


		// Congelo la animación del led y lo apago ya que se ha terminado el proceso de configuración del AP.
		myWiFiLed.setBlinkType(ToteBlinkOutputLed::NO_BLINK);
		myWiFiLed.ledOFF();

		// Cambiamos el modo de ejecución.
		myLoopRunMode = LOOP_RUN_MODE::CONNECT_SSID;
	}
}

void doConnectSSID(void) {
	// Compruebo si la WiFi sigue conectada
	if (checkWiFi()) {
		// Cambiamos el modo de ejecución.
		myLoopRunMode = LOOP_RUN_MODE::NORMAL_RUN;

		_TOTE_DEBUG_VALUE_("doConnectSSID", "Conectado a la WiFi con la IP: ", WiFi.localIP());

		// Si hay WIFI entonces inicializo el objeto cliente NTP
		_TOTE_DEBUG_("doConnectSSID", "Inicializando objeto cliente NTP.");
	}
}

void doNormalRun(void) {
	// Esta función contiene los procedimientos que deben llamarse en cada iteración del loop() cuando la Wifi funciona y está conectada.
	// El el resto del loop() pongo las acciones que debe ejecutarse siempre.

	// Comprobación de que el AP está OK.
	if (checkWiFi()) {
		// AQUÍ HAY QUE PONER LAS LLAMADAS A UPDATE PARA LOS OBJETOS QUE NECESITAN DE CONECTIVIDAD A LA RED
		// EL RESTO PUEDE PONERSE EN EL PROPIO LOOP.

		// Compruebo que el cliente de publicación/subscripción de MQTT está conectado.
		myMQTTCheckConnectionStatusTimer.check();

		// Compruebo si hay mensajes que enviar al servidor MQTT.
		myPowerOnStatusTimer.check();

		// Obligatorio para que se procesen los mensajes
		myMqttClient.loop();

		// Compruebo si se ha cumplido el intervalo para medir la intensidad de la señal WiFi.
		myWiFiCheckSignalStrength.check();

		// Compruebo si se ha cumplido el intervalo para refrescar el Dashboard de Node-Red.
		myNodeRedDashboardRefreshTimer.check();
	}
}

boolean checkWiFi(void) {
	// Compruebo si la WiFi sigue conectada
	if (!WiFi.isConnected()) {
		// Tenemos un problema. 
		_TOTE_DEBUG_("doNormalRun", "WiFi desconectada. Entrando en ventana de autoconfiguracion de AP.");

		// Pongo el led a parpadear para indicar que se ha caido el AP.
		myWiFiLed.setBlinkType(ToteBlinkOutputLed::INFINITE_BLINK);
		myWiFiLed.ledON();

		// Cambiamos al modo de ejecución para dar la posibilidad de configurar un AP diferente.
		initialMillis = millis(); // Para que empiece la cuenta de nuevo.
		myLoopRunMode = LOOP_RUN_MODE::WAIT_FOR_CONFIG;

		// Indico que la WIFI se ha caido o no conecta.
		return false;
	}
	else {
		// Está conectada. Dejo el led Wifi encendido.
		myWiFiLed.setBlinkType(ToteBlinkOutputLed::NO_BLINK);
		myWiFiLed.ledON();

		// Indico que la WIFI está OK.
		return true;
	}
}

void checkPowerON(void) {
	// Compruebo el valor de voltaje de POWER_SENSOR_PIN_PIN
	if (myMBSensor.isFired()) {
		// La placa base está encendida.
		myPowerONLed.ledON();
	}
	else {
		// La placa base NO está encendida.
		myPowerONLed.ledOFF();
	}
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
	// Array para procesar el payload.
	static char mqttPayload[MAX_MQTT_CHARS];

	unsigned int i;
	for (i = 0; i < length && i < MAX_MQTT_CHARS - 1; i++) {
		mqttPayload[i] = (char)payload[i];
	}
	mqttPayload[i] = '\0';


	_TOTE_DEBUG_VALUE_("mqttCallback", "Ha llegado un mensaje para el topic: ", topic);
	_TOTE_DEBUG_VALUE_("mqttCallback", "con el valor: ", mqttPayload);
	_TOTE_DEBUG_VALUE_("mqttCallback", "y longitud : ", length);

	/**********************************
	*  PROCESADO DE LOS MENSAJES MQTT *
	***********************************/

	// casa/time_source
	if (strcmp(topic, "casa/time_source") == 0) {
		_TOTE_DEBUG_("mqttCallback", "Detectado topic casa/time_source");

		// Convierto topic a UnixEpoch
		theTime = atol(mqttPayload);

		_TOTE_DEBUG_VALUE_("mqttCallback", "theTime: ", theTime);

		_TOTE_DEBUG_VALUE_("mqttCallback", "day: ", day(theTime));
		_TOTE_DEBUG_VALUE_("mqttCallback", "month: ", month(theTime));
		_TOTE_DEBUG_VALUE_("mqttCallback", "year: ", year(theTime));
		_TOTE_DEBUG_VALUE_("mqttCallback", "hour: ", hour(theTime));
		_TOTE_DEBUG_VALUE_("mqttCallback", "minute: ", minute(theTime));
		_TOTE_DEBUG_VALUE_("mqttCallback", "Second: ", second(theTime));

		// Indico que la hora ya es válida.
		isValidTime = true;

		// Salimos.
		return;
	}

	// casa/DEVICE_ID/password
	if (strcmp(topic, "casa/" DEVICE_ID "/password") == 0) {
		_TOTE_DEBUG_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/password");

		// Actualizo el password escrito.
		strcpy(lastPassword, mqttPayload);

		_TOTE_DEBUG_VALUE_("mqttCallback", "El password tiene el valor: ", lastPassword);

		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/power_switch
	if (strcmp(topic, "casa/" DEVICE_ID "/power_switch") == 0) {
		_TOTE_DEBUG_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/power_switch");

		// Compruebo si el password es el correcto.
		if (strcmp(lastPassword, COMMAND_PASSWORD) == 0) {
			// Borro el password.
			myMqttClient.publish("casa/" DEVICE_ID "/password_refresh", "");

			// Envío pulso por el pin 'POWER_ON_PIN'
			sendPulse(POWER_ON_PIN);

			_TOTE_DEBUG_VALUE_("mqttCallback", "Enviando pulso de encendido por el pin: ", POWER_ON_PIN);
		}
		else {
			// Envío mensaje de password incorrecto.
			myMqttClient.publish("casa/" DEVICE_ID  "/power_on_status_refresh", "Password Incorrecto");
		}
	
		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();


	// casa/DEVICE_ID/hard_power_switch
	if (strcmp(topic, "casa/" DEVICE_ID "/hard_power_switch") == 0) {
		_TOTE_DEBUG_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/hard_power_switch");

		// Compruebo si el password es el correcto.
		if (strcmp(lastPassword, COMMAND_PASSWORD) == 0) {
			// Borro el password.
			myMqttClient.publish("casa/" DEVICE_ID  "/password_refresh", "");

			// Envío pulso por el pin 'POWER_ON_PIN'
			sendHardPulse(POWER_ON_PIN);

			_TOTE_DEBUG_VALUE_("mqttCallback", "Enviando pulso LARGO (HARD) de encendido por el pin: ", POWER_ON_PIN);
		}
		else {
			// Envío mensaje de password incorrecto.
			myMqttClient.publish("casa/" DEVICE_ID  "/power_on_status_refresh", "Password Incorrecto");
		}

		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/reset_switch
	if (strcmp(topic, "casa/" DEVICE_ID "/reset_switch") == 0) {
		_TOTE_DEBUG_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/reset_switch");

		// Compruebo si el password es el correcto.
		if (strcmp(lastPassword, COMMAND_PASSWORD) == 0) {
			// Borro el password.
			myMqttClient.publish("casa/" DEVICE_ID  "/password_refresh", "");

			// Envío pulso por el pin 'RESET_PIN'
			sendPulse(RESET_PIN);

			_TOTE_DEBUG_VALUE_("mqttCallback", "Enviando pulso de reseteo por el pin: ", RESET_PIN);
		}
		else {
			// Envío mensaje de password incorrecto.
			myMqttClient.publish("casa/" DEVICE_ID  "/power_on_status_refresh", "Password Incorrecto");
		}

		// Salimos.
		return;
	}
}

void mqttCheckConnectionStatusCallback(void) {
	// Si el estado es diferente a 0 (conexión correcta) fuerzo la reconexión.
	// Ver 'mqttGetState' para valores de retorno.
	if (!myMqttClient.connected()) {
		_TOTE_DEBUG_("mqttCheckConnectionStatusCallback", "Intentando reconectar con servidor MQTT.");

		// Fuerzo la reconexión.
		mqttReconnect();

		// Hasta la próxima.
		return;
	}

	// Consulto el estado de la última operación con el servidor MQTT.
	int status = myMqttClient.state();

	switch (status) {
	case -4:
		_TOTE_DEBUG_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECTION_TIMEOUT");
		break;

	case -3:
		_TOTE_DEBUG_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECTION_LOST - the network connection was broken.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECTION_LOST");
		break;

	case -2:
		_TOTE_DEBUG_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_FAILED - the network connection failed.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_FAILED");
		break;

	case -1:
		_TOTE_DEBUG_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_DISCONNECTED - the client is disconnected cleanly.");
		strcpy(mqttLastStateOpTxt, "MQTT_DISCONNECTED");
		break;

	case 0:
		_TOTE_DEBUG_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECTED - the client is connected.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECTED");
		break;

	case 1:
		_TOTE_DEBUG_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_BAD_PROTOCOL");
		break;

	case 2:
		_TOTE_DEBUG_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_BAD_CLIENT_ID");
		break;

	case 3:
		_TOTE_DEBUG_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_UNAVAILABLE");
		break;

	case 4:
		_TOTE_DEBUG_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_BAD_CREDENTIALS - the username / password were rejected.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_BAD_CREDENTIALS");
		break;

	case 5:
		_TOTE_DEBUG_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_UNAUTHORIZED");
		break;
	}
}

void mqttReconnect() {
	_TOTE_DEBUG_("mqttReconnect", "Intentando reconectar con servidor MQTT...");

	// Intento la conexión.		// Attempt to connect
	if (myMqttClient.connect(mqttClientID)) {
		_TOTE_DEBUG_("mqttReconnect", "Conectado al servidor MQTT!!!");

		// Una vez conectado nos volvemos a subscribir.
		myMqttClient.subscribe("casa/time_source");
		myMqttClient.subscribe("casa/" DEVICE_ID "/password");
		myMqttClient.subscribe("casa/" DEVICE_ID "/power_switch");
		myMqttClient.subscribe("casa/" DEVICE_ID "/hard_power_switch");
		myMqttClient.subscribe("casa/" DEVICE_ID "/reset_switch");
	}
	else {
		_TOTE_DEBUG_("mqttReconnect", "Error al conectar al servidor MQTT");
	}
}

void sendPowerOnStatusCallback(void) {
	// Enviamos al broker el estado de encendido del PC.
	if (myMBSensor.isFired()) {
		// Envío topic indicando que el equipo está encendido.
		_TOTE_DEBUG_("mqttCheckConnectionStatusCallback", "Enviando topic 'casa/" DEVICE_ID "/power_on_status_refresh' con el valor 'ENCENDIDO'");

		myMqttClient.publish("casa/" DEVICE_ID  "/power_on_status_refresh", "ENCENDIDO");
	}
	else {
		// Envío topic indicando que el equipo está apagado.
		_TOTE_DEBUG_("mqttCheckConnectionStatusCallback", "Enviando topic 'casa/" DEVICE_ID "/power_on_status_refresh' con el valor 'APAGADO'");

		myMqttClient.publish("casa/" DEVICE_ID  "/power_on_status_refresh", "APAGADO");
	}
}

void sendPulse(uint8_t thePin) {
	// Envía un pulso de 500ms al pin indicado.

	// Flanco de subida.
	digitalWrite(thePin, HIGH);

	// Espera asíncrona de 500 ms.
	my500msDelay.stop();

	// Flanco de bajada.
	digitalWrite(thePin, LOW);
}

void sendHardPulse(uint8_t thePin) {
	// Envía un pulso largo de 8000 ms al pin indicado.

	// Flanco de subida.
	digitalWrite(thePin, HIGH);

	// Espera asíncrona de 8000 ms.
	my8000msDelay.stop();

	// Flanco de bajada.
	digitalWrite(thePin, LOW);
}

void sendTestResponse(void) {
	// Array para construir el mensaje a enviar.
	static char mqttMsg[MAX_MQTT_CHARS];

	_TOTE_DEBUG_("sendTestResponse", "Entrando en funcion de testing. Envio mensaje con la hora actualizada.");

	if (isValidTime) {
		// Formateo la fecha.
		sprintf(mqttMsg, "Respuesta del test: %s: La fecha es %02d/%02d/%04d  %02d:%02d:%02d", mqttClientID, day(theTime), month(theTime), year(theTime), hour(theTime), minute(theTime), second(theTime));

		// Envío mensaje.
		myMqttClient.publish("casa/" DEVICE_ID "/test_refresh", mqttMsg);
	}
	else {
		// Envío mensaje.
		myMqttClient.publish("casa/" DEVICE_ID "/test_refresh", "La fecha y hora aun no son correctas");
	}
}

void wifiCheckSignalStrengthCallback(void) {
	// Leo el valor de la intensidad de la señal WiFi
	wifiSignal = WiFi.RSSI();

	_TOTE_DEBUG_VALUE_("wifiCheckSignalStrengthCallback", "La intensidad de la WiFI en dBm es: ", wifiSignal);
}

void refheshNoderedDashboardsCallback(void) {
	// Array para construir el mensaje a enviar.
	static char mqttMsg[MAX_MQTT_CHARS];

	_TOTE_DEBUG_("refheshNoderedDashboardsCallback", "Refrescando Dashboard de Node-Red.");

	// Actualizo campo "status Date" del Dashboard.
	if (isValidTime) {
		// Formateo la fecha.
		sprintf(mqttMsg, "Fecha: %02d/%02d/%04d  %02d:%02d:%02d", day(theTime), month(theTime), year(theTime), hour(theTime), minute(theTime), second(theTime));

		// Envío mensaje.
		myMqttClient.publish("casa/" DEVICE_ID  "/status_date_refresh", mqttMsg);
	}
	else {
		// Envío mensaje.
		myMqttClient.publish("casa/" DEVICE_ID "/status_date_refresh", "La fecha y hora aun no son correctas");
	}

	// Actualizo campo "mqtt ID" del Dashboard.
	sprintf(mqttMsg, "Dispositivo: %s", mqttClientID);
	myMqttClient.publish("casa/" DEVICE_ID  "/mqtt_ID_refresh", mqttMsg);

	// Actualizo campo "mqtt Last Op" del Dashboard
	sprintf(mqttMsg, "Last MQTT op: %s", mqttLastStateOpTxt);
	myMqttClient.publish("casa/" DEVICE_ID  "/mqtt_Last_Op_refresh", mqttMsg);

	// Actualizo campo "WiFi Signal" del Dashboard
	sprintf(mqttMsg, "WiFi Signal: %ld dBm", wifiSignal);
	myMqttClient.publish("casa/" DEVICE_ID "/wifi_signal_refresh", mqttMsg);
}
