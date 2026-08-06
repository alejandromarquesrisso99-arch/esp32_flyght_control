import processing.serial.*;

Serial myPort;        
float pitch = 0;      
float roll = 0;       

void setup() {
  size(800, 600, P3D); 
  
  // ¡IMPORTANTE! Cambia "COM3" por tu puerto serie real
  String portName = "COM7"; 
  myPort = new Serial(this, portName, 115200);
  
  // Ya NO usamos bufferUntil('\n') para evitar colas acumuladas
}

void draw() {
  // --- BUCLE DE VACIADO DEL BÚFER SERIAL ---
  // Leemos TODOS los datos acumulados en la cola, pero solo nos quedamos 
  // con el último valor válido. Esto elimina por completo el retraso.
  while (myPort.available() > 0) {
    String val = myPort.readStringUntil('\n');
    if (val != null) {
      val = trim(val);
      String[] list = split(val, ',');
      if (list.length >= 2) {
        pitch = float(list[0]);
        roll = float(list[1]);
      }
    }
  }

  background(30); 
  
  lights();
  translate(width / 2, height / 2, -200); 
    // Rotación 3D en tiempo real con CERO retardo
  
  rotateX(radians(-35.264));
  rotateY(radians(45));
  
  rotateX(-radians(pitch));
  rotateY(0);
  rotateZ(-radians(roll));

  // Dibujar el rectángulo 3D (ESP32)
  pushMatrix();
  fill(0, 150, 255); 
  stroke(255);
  strokeWeight(2);
  box(250, 30, 120); 

  popMatrix();  
  
}
