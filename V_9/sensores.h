// =============================================================================
//  sensores.h  -  Sensores lentos: VL53L0X (ToF), HC-SR04, bateria
//
//  Corren en el nucleo 0 a 20 Hz. Ninguno bloquea el lazo de control:
//    - el ToF se usa en modo continuo (se consulta si ya hay dato, no se espera)
//    - el ultrasonido mide el eco por INTERRUPCION, no con pulseIn(), que
//      bloquearia hasta 30 ms y se comeria 7 ciclos del lazo de 250 Hz.
//
//  Los dos miden distancia a lo largo del eje z del CUERPO, no altura. Si el
//  dron esta inclinado, la altura real es
//        h = d * cos(phi) * cos(theta)
//  que es exactamente el termino r33 de la matriz de rotacion de las notas
//  (ec. 29). Esta correccion ya esta aplicada en `altura_m`, que es lo que
//  usara el control de altura de la siguiente version.
// =============================================================================
#ifndef SENSORES_H
#define SENSORES_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"

// ---- ultrasonido por interrupcion ----
static volatile uint32_t us_subida = 0;
static volatile uint32_t us_ancho  = 0;
static volatile bool     us_nuevo  = false;

static void IRAM_ATTR isrEco() {
  if (digitalRead(PIN_US_ECHO)) {
    us_subida = micros();
  } else if (us_subida) {
    us_ancho = micros() - us_subida;
    us_nuevo = true;
  }
}

class Sensores {
 public:
  Adafruit_VL53L0X lox;

  bool  tof_presente = false;
  float tof_m = 0;   bool tof_ok = false;
  float us_m  = 0;   bool us_ok  = false;
  float vbat  = 0;   bool vbat_ok = false;
  float altura_m = 0;
  uint8_t fuente = 0;      // 0 = ninguna, 1 = ToF, 2 = ultrasonido
  bool    hay_nueva = false;

  uint32_t tof_fallos = 0, us_fallos = 0;

  // [V_7] Diagnostico por sensor. Sin esto, "sin altura" puede ser cualquiera
  // de cuatro cosas muy distintas: el sensor no esta conectado, esta conectado
  // pero no responde, responde pero fuera de rango, o responde bien y es la
  // fusion la que lo descarta. Estos contadores las separan.
  uint16_t tof_mm_crudo = 0;    // ultima lectura, antes de filtrar y validar
  uint8_t  tof_estado   = 0;    // codigo de estado del VL53L0X
  uint32_t tof_lecturas = 0;    // cuantas veces ha contestado
  uint32_t us_ecos      = 0;    // cuantos ecos completos ha medido la ISR
  uint32_t us_ancho_us  = 0;    // ancho del ultimo eco, en microsegundos
  float    us_m_crudo   = 0;    // ultima distancia, antes de validar

  void iniciar() {
    // --- ToF en el segundo bus I2C ---
    Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, 400000);
    delay(10);
    tof_presente = lox.begin(0x29, false, &Wire1);
    if (tof_presente) lox.startRangeContinuous(50);   // una medida cada 50 ms

    // --- ultrasonido ---
    pinMode(PIN_US_TRIG, OUTPUT);
    digitalWrite(PIN_US_TRIG, LOW);
    pinMode(PIN_US_ECHO, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_US_ECHO), isrEco, CHANGE);

    // --- bateria ---
    if (PIN_VBAT >= 0) {
      analogReadResolution(12);
      analogSetPinAttenuation((uint8_t)PIN_VBAT, ADC_11db);
    }
  }

  // Llamar a 20 Hz. roll/pitch en deg, para corregir la inclinacion.
  void actualizar(float roll_deg, float pitch_deg) {
    leerToF();
    leerUltrasonido();
    leerBateria();

    float cr = cosf(roll_deg * DEG2RAD);
    float cp = cosf(pitch_deg * DEG2RAD);
    float r33 = cr * cp;
    if (r33 < 0.5f) r33 = 0.5f;    // mas de 60 deg: la correccion no tiene sentido

    // [V_5] PRIORIDAD INVERTIDA respecto de las versiones anteriores. Medido en
    // este dron: el VL53L0X solo es creible por debajo de ~20 cm, el HC-SR04
    // mide bien hasta ~1 m. Asi que manda el ULTRASONIDO y el ToF solo cubre el
    // tramo bajo, que es justo donde el ultrasonido tiene su zona muerta.
    //
    // La frontera lleva histeresis: estando abajo se sube hasta 25 cm con el
    // ToF, y estando arriba no se vuelve a el hasta bajar de 15 cm. Sin eso,
    // volando a 20 cm el sistema saltaria de un sensor al otro varias veces por
    // segundo, y cada salto es un escalon en la medida que el Kalman interpreta
    // como velocidad vertical.
    float frontera = (fuente == 1) ? (ALT_CAMBIO + ALT_HISTERESIS)
                                   : (ALT_CAMBIO - ALT_HISTERESIS);
    bool us_vale  = us_ok  && us_m > US_MIN_FIABLE && us_m < US_MAX_FIABLE;
    bool tof_vale = tof_ok && tof_m < TOF_MAX_FIABLE;
    bool abajo    = (altura_m < frontera);

    if (abajo && tof_vale)        { altura_m = tof_m * r33; fuente = 1; }
    else if (us_vale)             { altura_m = us_m  * r33; fuente = 2; }
    else if (tof_vale)            { altura_m = tof_m * r33; fuente = 1; }
    else                          { fuente = 0; }
    hay_nueva = (fuente != 0);
  }

  // Dispara una medida del ultrasonido (10 us en alto). Llamar a <= 20 Hz.
  void dispararUltrasonido() {
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);
  }

 private:
  uint32_t t_us_disparo = 0;

  void leerToF() {
    if (!tof_presente) { tof_ok = false; return; }
    if (!lox.isRangeComplete()) return;
    uint16_t mm = lox.readRangeResult();
    uint8_t  st = lox.readRangeStatus();
    tof_mm_crudo = mm;  tof_estado = st;  tof_lecturas++;   // [V_7]
    // 4 = "phase fail" (fuera de rango), 8190/8191 = sin medida
    // [V_5] Se descarta ya aqui lo que esta fuera del tramo donde este ToF es
    // creible: devolver 80 cm "medidos" cuando en realidad esta ciego es peor
    // que no devolver nada.
    if (st == 4 || mm == 0 || mm > (uint16_t)(TOF_MAX_FIABLE * 1000.0f + 100.0f)) {
      tof_ok = false; tof_fallos++; return;
    }
    float m = mm * 0.001f;
    tof_m = (tof_ok) ? (0.7f * tof_m + 0.3f * m) : m;   // filtro suave
    tof_ok = true;
  }

  void leerUltrasonido() {
    uint32_t ahora = millis();
    if (us_nuevo) {
      us_nuevo = false;
      uint32_t ancho = us_ancho;
      // 340 m/s ida y vuelta -> d = ancho_us * 1e-6 * 340 / 2
      float m = ancho * 0.00017f;
      us_ancho_us = ancho;  us_m_crudo = m;  us_ecos++;     // [V_7]
      if (m > US_MIN_FIABLE && m < US_MAX_FIABLE) {
        us_m = (us_ok) ? (0.6f * us_m + 0.4f * m) : m;
        us_ok = true;
      } else {
        us_fallos++;
        if (ahora - t_us_disparo > 500) us_ok = false;
      }
    } else if (ahora - t_us_disparo > 400) {
      us_ok = false;      // lleva 400 ms sin eco valido
    }

    if (ahora - t_us_disparo >= 60) {   // el HC-SR04 pide >= 60 ms entre disparos
      t_us_disparo = ahora;
      dispararUltrasonido();
    }
  }

  void leerBateria() {
    if (PIN_VBAT < 0) { vbat_ok = false; return; }
    float mv = analogReadMilliVolts((uint8_t)PIN_VBAT);
    float v  = mv * 0.001f * DIVISOR_VBAT;
    vbat = (vbat_ok) ? (0.9f * vbat + 0.1f * v) : v;
    vbat_ok = true;
  }
};

#endif // SENSORES_H
