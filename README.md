# Telemetría y Control Inalámbrico Autónomo (ESP32 + LoRa 433 MHz)

Sistema embebido transmisor-receptor para monitoreo de nivel y control remoto de relé (motobomba) mediante radiofrecuencia LoRa a 433 MHz, diseñado para operar de forma autónoma y determinista.

## 🏗️ Arquitectura de Hardware
* **Nodo Transmisor:** ESP32-WROOM-32U + módulo LoRa RA-02 (433 MHz) + lectura de estados de flotador.
* **Nodo Receptor:** ESP32-C3 Mini + módulo LoRa RA-02 (433 MHz) + etapa de control de relé e indicadores de estado.

## 🔧 Retos Técnicos y Depuración Resuelta
* **Resolución de conflictos de hardware (pines DAC):** Diagnóstico y corrección de falsos estados de fallo/corto en pines 25 y 26 mediante reasignación de pines y evaluación de resistencias pull-up físicas.
* **Estabilidad en comunicaciones (Serial / ACK):** Optimización de la máquina de estados y temporizaciones para eliminar bloqueos del transmisor/receptor y pérdidas de confirmación (ACK) tras ciclos continuos de operación.
* **Diagnóstico de RF y antenas:** Verificación física de antenas y pruebas de enlace orientadas a maximizar el alcance efectivo en campo.

## 📂 Estructura del Repositorio
* `/transmisor_esp32` — Firmware del nodo transmisor (ESP32-WROOM-32U).
* `/receptor_esp32_c3` — Firmware del nodo receptor (ESP32-C3 Mini).

## 👨‍💻 Autor
**Anderson Cardona Gómez** — Ingeniero Electrónico  
[LinkedIn](https://www.linkedin.com/in/anderson-cardona-97b5141ab/)
