// DiagnosticoESC - encuentra por que un motor (M2) no gira. Sin WiFi ni IMU.
// Mismos pines que PrimerVueloWiFi_PIDv2: M1=8, M2=17, M3=21, M4=18.
//
// QUITA LAS HELICES. Monitor Serie a 115200 con "Nueva linea".
// Escribe "ayuda" para ver los comandos.
//
// Orden sugerido para M2:
//  1. "pin3v3 17"  -> GPIO17 queda en 3.3 V fijo. Mide con multimetro entre el
//                     cable de senal (blanco/amarillo) EN EL CONECTOR DEL ESC y GND.
//                     Si no hay 3.3 V ahi: cable/soldadura/pin roto. "pin0v 17" = 0 V.
//     Despues "reset" para volver al PWM normal.
//  2. "f 50" y luego "cal": calibra los 4 a 50 Hz (cualquier ESC lo acepta).
//  3. "rampa 2": sube M2 de 1000 a 1300 us poco a poco. Mira a que pulso arranca.
//  4. "mapa 2 8": M2 sale por el GPIO8 (pasa fisicamente el cable de senal del ESC
//     de M2 al pin 8). Si asi gira, el problema es el GPIO17 o su cableado.
//  5. Si nada funciona: cambia el motor de M2 a un ESC que si funcione (o al reves)
//     para saber si falla el ESC o el motor.

#define PWM_RES 14
const uint32_t DUTY_MAX = (1 << PWM_RES) - 1;

#define PULSO_MIN    1000
#define PULSO_LIM    1400   // tope para pruebas manuales (sin helices igual)
#define PULSO_CALMAX 2000   // solo lo usa "cal"

int pines[4] = {8, 17, 21, 18};
uint32_t freqHz = 50;
uint32_t pulso[4] = {PULSO_MIN, PULSO_MIN, PULSO_MIN, PULSO_MIN};
bool enCal = false;
int pinDigital = -1;   // pin en modo multimetro, -1 = ninguno

uint32_t periodoUs() { return 1000000UL / freqHz; }

void escribir(int m, uint32_t us) {
  pulso[m] = us;
  ledcWrite(pines[m], (uint32_t)((uint64_t)us * DUTY_MAX / periodoUs()));
}

void todos(uint32_t us) { for (int i = 0; i < 4; i++) escribir(i, us); }

void adjuntar() {
  for (int i = 0; i < 4; i++) {
    ledcDetach(pines[i]);
    if (!ledcAttach(pines[i], freqHz, PWM_RES)) {
      Serial0.printf("ERROR: no se pudo asignar PWM al GPIO%d\n", pines[i]);
    }
  }
  todos(PULSO_MIN);
}

void estado() {
  Serial0.printf("Frecuencia %lu Hz (periodo %lu us)\n", freqHz, periodoUs());
  for (int i = 0; i < 4; i++)
    Serial0.printf("  M%d -> GPIO%-2d  %lu us\n", i + 1, pines[i], pulso[i]);
  if (pinDigital >= 0) Serial0.printf("  GPIO%d en modo multimetro\n", pinDigital);
}

void ayuda() {
  Serial0.println(F(
    "\nComandos:\n"
    "  s / parar        -> 1000 us en todos\n"
    "  estado           -> muestra frecuencia, pines y pulsos\n"
    "  f <hz>           -> cambia frecuencia (50..490), ej: f 400\n"
    "  cal              -> manda 2000 us; conecta bateria, espera pitidos, luego 'ok'\n"
    "  ok               -> (durante cal) manda 1000 us y termina\n"
    "  m <1-4> <us>     -> pulso a un motor (1000..1400), ej: m 2 1150\n"
    "  t <us>           -> pulso a los 4 a la vez (1000..1400)\n"
    "  rampa <1-4>      -> sube un motor 1000->1300 us en ~15 s y baja\n"
    "  prueba           -> cada motor 2 s a 1150 us, en orden\n"
    "  barrido <1-4>    -> prueba ese motor a 50,100,200,300,400,490 Hz\n"
    "  mapa <1-4> <gpio>-> cambia el pin de un motor, ej: mapa 2 8\n"
    "  pin3v3 <gpio>    -> pin fijo en 3.3 V (para multimetro)\n"
    "  pin0v <gpio>     -> pin fijo en 0 V\n"
    "  reset            -> pines y PWM originales (8,17,21,18 a 50 Hz)\n"));
}

// Espera ms milisegundos; si llega cualquier tecla, corta y devuelve true
bool esperarOCortar(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (Serial0.available()) {
      while (Serial0.available()) Serial0.read();
      todos(PULSO_MIN);
      Serial0.println("Cortado por teclado.");
      return true;
    }
    delay(5);
  }
  return false;
}

void rampa(int m) {
  Serial0.printf("Rampa M%d (GPIO%d). Cualquier tecla corta.\n", m + 1, pines[m]);
  for (uint32_t us = PULSO_MIN; us <= 1300; us += 10) {
    escribir(m, us);
    Serial0.printf("  %lu us\n", us);
    if (esperarOCortar(500)) return;
  }
  for (uint32_t us = 1300; us >= PULSO_MIN; us -= 20) {
    escribir(m, us);
    if (esperarOCortar(100)) return;
  }
  escribir(m, PULSO_MIN);
  Serial0.println("Rampa terminada. Anota a que pulso arranco (si arranco).");
}

void prueba() {
  todos(PULSO_MIN);
  for (int i = 0; i < 4; i++) {
    Serial0.printf("M%d (GPIO%d) a 1150 us\n", i + 1, pines[i]);
    escribir(i, 1150);
    if (esperarOCortar(2000)) return;
    escribir(i, PULSO_MIN);
    if (esperarOCortar(1000)) return;
  }
  Serial0.println("Prueba terminada.");
}

void barrido(int m) {
  const uint32_t fs[] = {50, 100, 200, 300, 400, 490};
  uint32_t fOrig = freqHz;
  Serial0.printf("Barrido de frecuencia en M%d. Cualquier tecla corta.\n", m + 1);
  for (uint32_t f : fs) {
    freqHz = f;
    adjuntar();
    Serial0.printf("  %lu Hz: armando 3 s...\n", f);
    if (esperarOCortar(3000)) break;
    Serial0.printf("  %lu Hz: M%d a 1150 us (gira?)\n", f, m + 1);
    escribir(m, 1150);
    if (esperarOCortar(3000)) break;
    escribir(m, PULSO_MIN);
    if (esperarOCortar(1000)) break;
  }
  freqHz = fOrig;
  adjuntar();
  Serial0.printf("Barrido terminado, de vuelta a %lu Hz.\n", freqHz);
}

void pinFijo(int gpio, bool alto) {
  for (int i = 0; i < 4; i++)
    if (pines[i] == gpio) { escribir(i, PULSO_MIN); ledcDetach(gpio); }
  pinMode(gpio, OUTPUT);
  digitalWrite(gpio, alto ? HIGH : LOW);
  pinDigital = gpio;
  Serial0.printf("GPIO%d fijo en %s. Mide en el conector del ESC. 'reset' para volver.\n",
                 gpio, alto ? "3.3 V" : "0 V");
}

void procesar(String l) {
  l.trim();
  l.toLowerCase();
  if (l.length() == 0) return;
  int a = -1, b = -1;
  int esp = l.indexOf(' ');
  String cmd = esp < 0 ? l : l.substring(0, esp);
  if (esp >= 0) {
    String resto = l.substring(esp + 1);
    resto.trim();
    a = resto.toInt();
    int esp2 = resto.indexOf(' ');
    if (esp2 >= 0) b = resto.substring(esp2 + 1).toInt();
  }

  if (cmd == "s" || cmd == "parar") { enCal = false; todos(PULSO_MIN); Serial0.println("Parado."); }
  else if (cmd == "ayuda" || cmd == "?") ayuda();
  else if (cmd == "estado") estado();
  else if (cmd == "f") {
    if (a < 50 || a > 490) { Serial0.println("Frecuencia entre 50 y 490."); return; }
    freqHz = a; adjuntar();
    Serial0.printf("Frecuencia %lu Hz. Recalibra con 'cal' si cambiaste de rango.\n", freqHz);
  }
  else if (cmd == "cal") {
    enCal = true; todos(PULSO_CALMAX);
    Serial0.println("2000 us en los 4. Conecta la bateria AHORA, espera los pitidos y escribe 'ok'.");
  }
  else if (cmd == "ok") {
    todos(PULSO_MIN); enCal = false;
    Serial0.println("1000 us. Espera los pitidos de confirmacion. Calibrado.");
  }
  else if (cmd == "m") {
    if (a < 1 || a > 4 || b < PULSO_MIN || b > PULSO_LIM) { Serial0.println("Uso: m <1-4> <1000-1400>"); return; }
    escribir(a - 1, b); Serial0.printf("M%d = %d us\n", a, b);
  }
  else if (cmd == "t") {
    if (a < PULSO_MIN || a > PULSO_LIM) { Serial0.println("Uso: t <1000-1400>"); return; }
    todos(a); Serial0.printf("Todos = %d us\n", a);
  }
  else if (cmd == "rampa")   { if (a >= 1 && a <= 4) rampa(a - 1);   else Serial0.println("Uso: rampa <1-4>"); }
  else if (cmd == "barrido") { if (a >= 1 && a <= 4) barrido(a - 1); else Serial0.println("Uso: barrido <1-4>"); }
  else if (cmd == "prueba") prueba();
  else if (cmd == "mapa") {
    if (a < 1 || a > 4 || b < 0 || b > 48) { Serial0.println("Uso: mapa <1-4> <gpio>"); return; }
    escribir(a - 1, PULSO_MIN);
    ledcDetach(pines[a - 1]);
    pines[a - 1] = b;
    adjuntar();
    Serial0.printf("M%d ahora sale por GPIO%d. Mueve fisicamente el cable de senal.\n", a, b);
  }
  else if (cmd == "pin3v3") { if (a >= 0) pinFijo(a, true);  }
  else if (cmd == "pin0v")  { if (a >= 0) pinFijo(a, false); }
  else if (cmd == "reset") {
    for (int i = 0; i < 4; i++) ledcDetach(pines[i]);
    if (pinDigital >= 0) { pinMode(pinDigital, INPUT); pinDigital = -1; }
    int orig[4] = {8, 17, 21, 18};
    for (int i = 0; i < 4; i++) pines[i] = orig[i];
    freqHz = 50; adjuntar();
    Serial0.println("Reset: pines 8,17,21,18 a 50 Hz, 1000 us.");
  }
  else Serial0.println("Comando desconocido. Escribe 'ayuda'.");
}

void setup() {
  Serial0.begin(115200);
  adjuntar();               // arranca SIEMPRE en 1000 us (no arma a fondo)
  delay(500);
  Serial0.println("\n=== Diagnostico de ESC (SIN HELICES) ===");
  estado();
  ayuda();
}

void loop() {
  if (Serial0.available()) procesar(Serial0.readStringUntil('\n'));
}
