#include <SPI.h>
#include <LoRa.h>

// ============================================================================
// PINES TRANSMISOR (ESP32-WROOM-32U) A LORA RA-02
// ============================================================================
#define SCK_PIN         18
#define MISO_PIN        19
#define MOSI_PIN        23
#define SS_PIN          5
#define RST_PIN         14
#define DIO0_PIN        2

// NUEVOS PINES 100% DIGITALES PUROS (SIN DAC NI ADC)
#define PIN_BOYA_NC     21        // Antes 25 -> Ahora Pin 21
#define PIN_BOYA_NO     22        // Antes 26 -> Ahora Pin 22
#define PIN_BOTON       27        // Se mantiene en Pin 27 (Funciona perfecto)

#define PIN_LED_VERDE   32
#define PIN_LED_ROJO    33

// ============================================================================
// PARÁMETROS DE RED Y TIEMPOS
// ============================================================================
#define LORA_FREQ             433E6
#define INTERVALO_TX_MS       2500      // Latido constante cada 2.5 segundos
#define ENFRIAMIENTO_TX_MS    600       // Tiempo mínimo entre envíos para no chocar con el ACK
#define TIEMPO_AIRE_MS        85        // 85 ms para salida completa del paquete en SF7
#define TIMEOUT_ACK_MS        1400      // 1.4 s esperando confirmación ACK
#define MAX_FALLOS_ACK        3
#define ALERTA_MANUAL_MS      2700000UL // 45 minutos

enum CodigoError {
  ERR_NINGUNO         = 0,
  ERR_MODULO_LORA     = 1,
  ERR_CORTO_BOYA      = 2,
  ERR_CABLE_ROTO      = 3,
  ERR_SIN_ENLACE      = 4,
  WARN_MANUAL_EXTENSO = 5
};

CodigoError errorActivo = ERR_NINGUNO;
bool loraHardwareOk = false;

// Variables estables de sensores
bool ncEstable = HIGH;
bool noEstable = HIGH;
bool botonEstable = HIGH;
unsigned long inicioModoManualMs = 0;
bool alertaTiempoManual = false;

// Variables de radio
unsigned long ultimoEnvioMs = 0;
unsigned long inicioTxMs = 0;
bool transmitiendoBackground = false;
bool esperandoAck = false;
bool envioInmediatoPendiente = false;
uint8_t contadorFallosAck = 0;
uint16_t idPaquete = 0;
String comandoActual = "BOMBA_OFF";
String comandoAnterior = "";

// Variables del LED Rojo (Check Engine)
unsigned long timerBlinkMs = 0;
uint8_t destellosEmitidos = 0;
bool estadoFisicoLedRojo = LOW;
bool enPausaDeCiclo = false;
const unsigned long DURACION_DESTELLO_MS = 250;
const unsigned long PAUSA_ENTRE_CODIGOS_MS = 2000;

bool iniciarRadioLoRa() {
  SPI.end();
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  LoRa.setSPIFrequency(1E6);

  if (!LoRa.begin(LORA_FREQ)) return false;

  LoRa.setTxPower(10);          // 10 dBm: alcance sólido sin saturar la fuente
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.disableCrc();
  LoRa.idle();
  return true;
}

// ============================================================================
// INICIALIZACIÓN
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);

  // Configurar pines digitales puros 21, 22 y 27
  pinMode(PIN_BOYA_NC, INPUT_PULLUP);
  pinMode(PIN_BOYA_NO, INPUT_PULLUP);
  pinMode(PIN_BOTON, INPUT_PULLUP);

  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_ROJO, LOW);

  delay(100); // Estabilización inicial de niveles lógicos
  ncEstable = digitalRead(PIN_BOYA_NC);
  noEstable = digitalRead(PIN_BOYA_NO);
  botonEstable = digitalRead(PIN_BOTON);

  Serial.println("\n=== TRANSMISOR INDUSTRIAL INICIANDO ===");
  for (int i = 1; i <= 5; i++) {
    if (iniciarRadioLoRa()) {
      loraHardwareOk = true;
      break;
    }
    delay(500);
  }

  if (loraHardwareOk) {
    Serial.println("[OK] Radio LoRa TX lista en 433 MHz.");
  } else {
    Serial.println("[CRITICO] No se detecta el modulo LoRa.");
  }
}

// ============================================================================
// LECTURA DE SENSORES (MISMA LÓGICA SÓLIDA DEL PIN 27)
// ============================================================================
void actualizarSensores() {
  unsigned long ahora = millis();

  // No muestrear durante los milisegundos en que la antena irradia
  if (transmitiendoBackground || (ahora - inicioTxMs < 120)) {
    return;
  }

  static unsigned long ultimoMuestreoMs = 0;
  if (ahora - ultimoMuestreoMs < 40) return;
  ultimoMuestreoMs = ahora;

  bool ncRaw = digitalRead(PIN_BOYA_NC);
  bool noRaw = digitalRead(PIN_BOYA_NO);
  bool btnRaw = digitalRead(PIN_BOTON);

  // Filtro de 3 muestras (120 ms) para el interruptor manual (Pin 27)
  static uint8_t cuentaBtn = 0;
  if (btnRaw != botonEstable) {
    cuentaBtn++;
    if (cuentaBtn >= 3) {
      botonEstable = btnRaw;
      cuentaBtn = 0;
    }
  } else {
    cuentaBtn = 0;
  }

  // Filtro para la boya en Pines 21 y 22:
  // - Estado normal de trabajo (0,1 o 1,0): valida en 4 muestras (160 ms).
  // - Estado de falla (0,0 corto o 1,1 cable abierto / paso por el centro del switch):
  //   espera 50 muestras (2 segundos) antes de declarar error para no cortar al mover el switch.
  static uint8_t cuentaBoya = 0;
  static bool candidatoNc = HIGH;
  static bool candidatoNo = HIGH;

  if (ncRaw == candidatoNc && noRaw == candidatoNo) {
    uint8_t limiteMuestras = (candidatoNc != candidatoNo) ? 4 : 50;
    if (cuentaBoya < limiteMuestras) {
      cuentaBoya++;
    } else {
      ncEstable = candidatoNc;
      noEstable = candidatoNo;
    }
  } else {
    candidatoNc = ncRaw;
    candidatoNo = noRaw;
    cuentaBoya = 0;
  }
}

void procesarLogica() {
  bool modoManualActivo = (botonEstable == LOW);

  if (modoManualActivo) {
    if (inicioModoManualMs == 0) {
      inicioModoManualMs = millis();
      alertaTiempoManual = false;
    } else if (millis() - inicioModoManualMs >= ALERTA_MANUAL_MS) {
      alertaTiempoManual = true; // Solo avisa con 5 destellos, NO apaga la bomba
    }
    comandoActual = "BOMBA_ON_MANUAL";
    digitalWrite(PIN_LED_VERDE, HIGH);
  } else {
    inicioModoManualMs = 0;
    alertaTiempoManual = false;

    if (ncEstable == LOW && noEstable == HIGH) {
      comandoActual = "BOMBA_ON";
      digitalWrite(PIN_LED_VERDE, HIGH);
    } else if (ncEstable == HIGH && noEstable == LOW) {
      comandoActual = "BOMBA_OFF";
      digitalWrite(PIN_LED_VERDE, LOW);
    } else if (ncEstable == LOW && noEstable == LOW) {
      comandoActual = "ERROR_CORTO";
      digitalWrite(PIN_LED_VERDE, LOW);
    } else {
      comandoActual = "ERROR_CABLE";
      digitalWrite(PIN_LED_VERDE, LOW);
    }
  }

  if (comandoActual != comandoAnterior) {
    if (comandoAnterior != "") {
      Serial.println(">>> CAMBIO DE ORDEN: " + comandoActual + " <<<");
      envioInmediatoPendiente = true;
    }
    comandoAnterior = comandoActual;
  }
}

void determinarErrores() {
  CodigoError nuevoError = ERR_NINGUNO;

  if (!loraHardwareOk) nuevoError = ERR_MODULO_LORA;
  else if (ncEstable == LOW && noEstable == LOW) nuevoError = ERR_CORTO_BOYA;
  else if (ncEstable == HIGH && noEstable == HIGH) nuevoError = ERR_CABLE_ROTO;
  else if (contadorFallosAck >= MAX_FALLOS_ACK) nuevoError = ERR_SIN_ENLACE;
  else if (alertaTiempoManual) nuevoError = WARN_MANUAL_EXTENSO;

  if (nuevoError != errorActivo) {
    errorActivo = nuevoError;
    destellosEmitidos = 0;
    enPausaDeCiclo = false;
    estadoFisicoLedRojo = LOW;
    digitalWrite(PIN_LED_ROJO, LOW);
    timerBlinkMs = millis();
  }
}

// ============================================================================
// GESTIÓN DE RADIO AUTÓNOMA (COMO CABLE FIJO)
// ============================================================================
void gestionarRadio() {
  if (!loraHardwareOk) return;
  unsigned long ahora = millis();

  // 1. Esperar 85 ms a que termine de salir el paquete y poner el chip en IDLE limpio
  if (transmitiendoBackground) {
    if (ahora - inicioTxMs < TIEMPO_AIRE_MS) return;
    LoRa.idle(); // Limpia el estado de transmisión antes de pasar a escuchar
    transmitiendoBackground = false;
    esperandoAck = true;
    ultimoEnvioMs = millis();
    return;
  }

  // 2. Escuchar confirmación <ACK:id> sin pausas para no perder ningún preámbulo
  if (esperandoAck) {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0 && packetSize < 64) {
      String respuesta = "";
      while (LoRa.available()) {
        char c = (char)LoRa.read();
        if (c >= 32 && c <= 126) respuesta += c;
      }

      if (respuesta.indexOf("ACK:") != -1) {
        esperandoAck = false;
        contadorFallosAck = 0;
        ultimoEnvioMs = millis();
        Serial.print("[ENLACE OK] Confirmado: ");
        Serial.print(respuesta);
        Serial.print(" | RSSI: ");
        Serial.print(LoRa.packetRssi());
        Serial.println(" dBm");
        return;
      }
    }

    if (ahora >= ultimoEnvioMs && (ahora - ultimoEnvioMs >= TIMEOUT_ACK_MS)) {
      esperandoAck = false;
      ultimoEnvioMs = millis();
      if (contadorFallosAck < 255) contadorFallosAck++;
      Serial.print("[WARN] ACK perdido. Fallos seguidos: ");
      Serial.println(contadorFallosAck);

      // Si pierde 4 ACK seguidos, reinicia completamente el bus SPI y el módulo LoRa
      if (contadorFallosAck % 4 == 0) {
        Serial.println("[AUTO-RECUPERACION TX] Reiniciando radio LoRa...");
        iniciarRadioLoRa();
      }
    }
    return;
  }

  // 3. Control de envío: inmediato si cambió un switch (respetando 600 ms de enfriamiento),
  //    en 1.0 s si busca reconectar, o cada 2.5 s si el enlace está firme.
  unsigned long intervalo = (contadorFallosAck > 0) ? 1000UL : INTERVALO_TX_MS;
  bool enfriamientoCumplido = (ahora >= ultimoEnvioMs) && (ahora - ultimoEnvioMs >= ENFRIAMIENTO_TX_MS);

  if ((envioInmediatoPendiente && enfriamientoCumplido) || (ahora >= ultimoEnvioMs && (ahora - ultimoEnvioMs >= intervalo))) {
    envioInmediatoPendiente = false;
    idPaquete++;
    String trama = "<TX:" + String(idPaquete) + ":" + comandoActual + ":" + String((int)errorActivo) + ">";

    LoRa.idle();
    if (LoRa.beginPacket() != 0) {
      LoRa.print(trama);
      LoRa.endPacket(true);
      transmitiendoBackground = true;
      inicioTxMs = millis();

      Serial.print("[PINES NC(21)=");
      Serial.print(ncEstable);
      Serial.print(" NO(22)=");
      Serial.print(noEstable);
      Serial.print(" BTN(27)=");
      Serial.print(botonEstable);
      Serial.print("] -> Enviando: ");
      Serial.println(trama);
    } else {
      iniciarRadioLoRa();
    }
  }
}

void gestionarParpadeo() {
  if (errorActivo == ERR_NINGUNO) {
    digitalWrite(PIN_LED_ROJO, LOW);
    return;
  }

  unsigned long ahora = millis();

  if (enPausaDeCiclo) {
    digitalWrite(PIN_LED_ROJO, LOW);
    if (ahora - timerBlinkMs >= PAUSA_ENTRE_CODIGOS_MS) {
      timerBlinkMs = ahora;
      destellosEmitidos = 0;
      enPausaDeCiclo = false;
    }
  } else {
    if (ahora - timerBlinkMs >= DURACION_DESTELLO_MS) {
      timerBlinkMs = ahora;
      estadoFisicoLedRojo = !estadoFisicoLedRojo;
      digitalWrite(PIN_LED_ROJO, estadoFisicoLedRojo);

      if (estadoFisicoLedRojo == LOW) {
        destellosEmitidos++;
        if (destellosEmitidos >= (uint8_t)errorActivo) {
          enPausaDeCiclo = true;
        }
      }
    }
  }
}

void loop() {
  actualizarSensores();
  procesarLogica();
  determinarErrores();
  gestionarRadio();
  gestionarParpadeo();
}