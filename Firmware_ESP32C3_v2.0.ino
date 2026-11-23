#include <SPI.h>
#include <LoRa.h>

// ============================================================================
// PINES RECEPTOR (ESP32-C3 MINI)
// ============================================================================
#define SCK_PIN         4
#define MISO_PIN        5
#define MOSI_PIN        6
#define SS_PIN          7
#define RST_PIN         3
#define DIO0_PIN        2

#define PIN_RELE_BOMBA  10
#define RELE_ON         LOW    // Lógica inversa: LOW enciende la bomba, HIGH la apaga
#define RELE_OFF        HIGH

// ============================================================================
// PARÁMETROS DE RED Y SEGURIDAD
// ============================================================================
#define LORA_FREQ             433E6
#define TIMEOUT_REFRESCO_RX   9000UL    // 9 s sin paquetes: refresca la radio sin apagar la bomba
#define TIMEOUT_WATCHDOG_MS   20000UL   // 20 s sin paquetes: apaga la bomba por seguridad (Fail-Safe)

bool loraHardwareOk = false;
bool bombaEncendida = false;
bool enlaceActivo = false;
unsigned long ultimaRecepcionValidaMs = 0;
unsigned long ultimoRefrescoRadioMs = 0;
unsigned long ultimoPollSpiMs = 0;

void establecerEstadoBomba(bool encender) {
  digitalWrite(PIN_RELE_BOMBA, encender ? RELE_ON : RELE_OFF);
  if (bombaEncendida != encender) {
    bombaEncendida = encender;
    if (encender) {
      Serial.println(">>> CAMBIO DE ESTADO: BOMBA ENCENDIDA (RELE EN LOW) <<<");
    } else {
      Serial.println(">>> CAMBIO DE ESTADO: BOMBA APAGADA (RELE EN HIGH) <<<");
    }
  }
}

bool iniciarRadioLoRa() {
  LoRa.setSPIFrequency(1E6);
  if (!LoRa.begin(LORA_FREQ)) return false;

  LoRa.setTxPower(10);          // 10 dBm
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.disableCrc();
  return true;
}

// ============================================================================
// INICIALIZACIÓN
// ============================================================================
void setup() {
  pinMode(PIN_RELE_BOMBA, OUTPUT);
  digitalWrite(PIN_RELE_BOMBA, RELE_OFF); // Inicia siempre con bomba apagada (HIGH)

  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);

  Serial.begin(115200);
  delay(3500); // Estabilización de fuente step-down
  Serial.println("\n=== RECEPTOR INDUSTRIAL (ESP32-C3) ===");

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  for (int i = 1; i <= 5; i++) {
    Serial.print("Iniciando LoRa, intento ");
    Serial.print(i);
    Serial.print("... ");
    if (iniciarRadioLoRa()) {
      loraHardwareOk = true;
      Serial.println("¡ÉXITO!");
      break;
    }
    Serial.println("Falló.");
    delay(500);
  }

  if (loraHardwareOk) {
    Serial.println("[OK] Receptor listo y escuchando en 433 MHz.");
  } else {
    Serial.println("[CRITICO] Modulo LoRa no responde.");
  }

  ultimaRecepcionValidaMs = millis();
  ultimoRefrescoRadioMs = millis();
}

// ============================================================================
// RECEPCIÓN Y DOBLE RESPUESTA ACK
// ============================================================================
void gestionarRecepcion() {
  if (!loraHardwareOk) return;
  unsigned long ahora = millis();

  // Consultar SPI cada 3 ms para no meter ruido al receptor RF
  if (ahora - ultimoPollSpiMs < 3) return;
  ultimoPollSpiMs = ahora;

  int packetSize = LoRa.parsePacket();
  if (packetSize > 0 && packetSize < 64) {
    String raw = "";
    while (LoRa.available()) {
      char c = (char)LoRa.read();
      if (c >= 32 && c <= 126) raw += c;
    }

    int idxIni = raw.indexOf("<TX:");
    int idxFin = raw.indexOf(">", idxIni);

    if (idxIni != -1 && idxFin != -1) {
      String tramaLimpia = raw.substring(idxIni + 1, idxFin);

      int p1 = tramaLimpia.indexOf(':');
      int p2 = tramaLimpia.indexOf(':', p1 + 1);
      int p3 = tramaLimpia.indexOf(':', p2 + 1);

      if (p1 != -1 && p2 != -1 && p3 != -1) {
        String idPaquete = tramaLimpia.substring(p1 + 1, p2);
        String comando = tramaLimpia.substring(p2 + 1, p3);
        String dtcRemoto = tramaLimpia.substring(p3 + 1);
        int rssi = LoRa.packetRssi();

        // 1. Mantener o cambiar el estado del relé según la orden recibida
        bool debeEncender = (comando == "BOMBA_ON_MANUAL" || comando == "BOMBA_ON");
        establecerEstadoBomba(debeEncender);

        String tramaAck = "<ACK:" + idPaquete + ">";

        // 2. Primer disparo ACK (a los 120 ms, cuando el TX ya lleva 30 ms escuchando)
        delay(120);
        if (LoRa.beginPacket() != 0) {
          LoRa.print(tramaAck);
          LoRa.endPacket(true);
          delay(75);
        }

        // 3. Segundo disparo ACK de respaldo (garantiza recepción aunque haya ruido)
        delay(60);
        if (LoRa.beginPacket() != 0) {
          LoRa.print(tramaAck);
          LoRa.endPacket(true);
          delay(75);
        }

        // 4. Actualizar relojes de enlace con tiempo fresco
        ultimaRecepcionValidaMs = millis();
        ultimoRefrescoRadioMs = millis();
        enlaceActivo = true;

        Serial.print("[RX OK] ID:");
        Serial.print(idPaquete);
        Serial.print(" | Orden:");
        Serial.print(comando);
        Serial.print(" | DTC TX:");
        Serial.print(dtcRemoto);
        Serial.print(" | RSSI:");
        Serial.print(rssi);
        Serial.print(" dBm | Bomba:");
        Serial.print(bombaEncendida ? "ON(LOW)" : "OFF(HIGH)");
        Serial.println(" | ACK x2: " + tramaAck);
      }
    }
  } else if (packetSize >= 64) {
    while (LoRa.available()) LoRa.read();
  }
}

// ============================================================================
// AUTO-REFRESCO DE RADIO (9s) Y FAIL-SAFE DE BOMBA (20s)
// ============================================================================
void verificarWatchdogEnlace() {
  if (!loraHardwareOk) return;
  unsigned long ahora = millis();

  // Si pasan 9 segundos sin escuchar nada, refresca el chip LoRa preventivamente
  // (sin apagar la bomba todavía) por si la radio quedó bloqueada tras transmitir un ACK
  if (ahora >= ultimoRefrescoRadioMs && (ahora - ultimoRefrescoRadioMs >= TIMEOUT_REFRESCO_RX)) {
    ultimoRefrescoRadioMs = ahora;
    if (ahora >= ultimaRecepcionValidaMs && (ahora - ultimaRecepcionValidaMs >= TIMEOUT_REFRESCO_RX)) {
      Serial.println("[AUTO-RESET LORA RX] 9s sin paquetes -> Refrescando escucha LoRa...");
      iniciarRadioLoRa();
    }
  }

  // Si pasan 20 segundos reales sin señal, apaga la bomba por seguridad (Fail-Safe)
  if (ahora >= ultimaRecepcionValidaMs && (ahora - ultimaRecepcionValidaMs >= TIMEOUT_WATCHDOG_MS)) {
    if (enlaceActivo || bombaEncendida) {
      enlaceActivo = false;
      establecerEstadoBomba(false);
      Serial.println("[FAIL-SAFE] 20s sin señal del transmisor -> Bomba APAGADA (HIGH).");
    }
  }
}

void loop() {
  gestionarRecepcion();
  verificarWatchdogEnlace();
}