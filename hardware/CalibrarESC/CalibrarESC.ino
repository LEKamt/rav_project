// CalibrarESC - calibra el rango de los 4 ESC del F450 a la vez (sin WiFi).
// Mismos pines y PWM que PrimerVueloWiFi_PIDv2 (M1=8, M2=17, M3=21, M4=18, 400 Hz).
//
// QUITA LAS HELICES ANTES DE EMPEZAR.
//
// Procedimiento (Monitor Serie a 115200, "Nueva linea"):
//  1. Bateria DESCONECTADA. Conecta el ESP32 por USB y sube este sketch.
//     Al arrancar manda 2000 us (maximo) a los 4 ESC.
//  2. Conecta la bateria. Los ESC pitan reconociendo el maximo (2 pitidos cortos
//     en la mayoria de BLHeli/SimonK).
//  3. Escribe 'c' + Enter dentro de ~2 s despues de los pitidos: manda 1000 us
//     (minimo). Los ESC pitan confirmando y quedan calibrados.
//  4. Opcional: 't' prueba cada motor por separado a 1150 us durante 2 s
//     (M1, M2, M3, M4 en orden) para verificar que todos arrancan parejo.
//  Cualquier otra tecla -> 1000 us en todos (parada).

#define M1_PIN 8
#define M2_PIN 17
#define M3_PIN 21
#define M4_PIN 18

#define PWM_FREQ 200
#define PWM_RES 14
const uint32_t DUTY_MAX = (1 << PWM_RES) - 1;
const uint32_t PERIODO_US = 1000000UL / PWM_FREQ;

#define PULSO_CAL_MAX 2000
#define PULSO_CAL_MIN 1000
#define PULSO_PRUEBA  1150

const int PINES[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};

void setPulse(int m, uint32_t us) {
  ledcWrite(PINES[m], (us * DUTY_MAX) / PERIODO_US);
}

void setTodos(uint32_t us) {
  for (int i = 0; i < 4; i++) setPulse(i, us);
}

void setup() {
  Serial0.begin(115200);
  for (int i = 0; i < 4; i++) ledcAttach(PINES[i], PWM_FREQ, PWM_RES);

  setTodos(PULSO_CAL_MAX);
  delay(500);
  Serial0.println();
  Serial0.println("=== Calibracion de ESC (SIN HELICES) ===");
  Serial0.println("Enviando 2000 us a los 4 ESC.");
  Serial0.println("1) Conecta la bateria ahora y espera los pitidos.");
  Serial0.println("2) Escribe 'c' + Enter para mandar 1000 us y terminar.");
}

void loop() {
  if (!Serial0.available()) return;
  char c = Serial0.read();
  if (c == '\n' || c == '\r') return;

  if (c == 'c' || c == 'C') {
    setTodos(PULSO_CAL_MIN);
    Serial0.println("1000 us enviado. Espera los pitidos de confirmacion: ESC calibrados.");
    Serial0.println("'t' = probar motores uno por uno a 1150 us.");
  } else if (c == 't' || c == 'T') {
    setTodos(PULSO_CAL_MIN);
    for (int i = 0; i < 4; i++) {
      Serial0.print("Probando M"); Serial0.println(i + 1);
      setPulse(i, PULSO_PRUEBA);
      delay(2000);
      setPulse(i, PULSO_CAL_MIN);
      delay(1000);
    }
    Serial0.println("Prueba terminada.");
  } else {
    setTodos(PULSO_CAL_MIN);
    Serial0.println("Parada: 1000 us en todos.");
  }
}
