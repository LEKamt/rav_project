void setup() {
  Serial0.begin(115200); // Usamos Serial0 para el puerto UART (CH340)
  Serial0.println("\n--- Reiniciado ---");
  Serial0.println("Echo UART listo. Escribe algo:");
}

void loop() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime > 1000) {
    Serial0.print("."); // Latido
    lastTime = millis();
  }

  if (Serial0.available()) {
    char c = Serial0.read();
    Serial0.write(c); 
  }
}