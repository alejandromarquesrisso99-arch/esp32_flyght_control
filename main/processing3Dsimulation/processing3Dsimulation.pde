import processing.serial.*;

Serial myPort;

// Variables de orientación (Kalman)
float roll = 0;
float pitch = 0;

// Variables de aceleración física
float accelX = 0;
float accelY = 0;
float accelZ = 0;

//variable temperatura
float temp = 0;

// Configuración de Partículas (Polvo)
int NUM_DUST = 1000;
DustParticle[] dust = new DustParticle[NUM_DUST];

void setup() {
  size(1000, 700, P3D);

  // Cambia "COM3" por tu puerto serial real
  String portName = "COM7";
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n');

  // Inicializar partículas de polvo aleatoriamente en el espacio 3D
  for (int i = 0; i < NUM_DUST; i++) {
    dust[i] = new DustParticle();
  }
}

void draw() {
  background(15, 18, 25); // Fondo oscuro tipo espacio/laboratorio

  // Leer todos los datos del buffer serial y conservar solo el último (Sin retardo)
  readSerialData();

  // 1. DIBUJAR Y ACTUALIZAR EL POLVO
  // Se mueve en dirección contraria a la aceleración
  hint(DISABLE_DEPTH_TEST); // Para renderizado fluido del polvo
  for (int i = 0; i < NUM_DUST; i++) {
    dust[i].update(accelX, accelY);
    dust[i].display();
  }
  hint(ENABLE_DEPTH_TEST);

  // 2. DIBUJAR LA PLACA (RECTÁNGULO 3D) EN EL CENTRO
  pushMatrix();
  translate(width/2, height/2, 0);
  rotateX(radians(65));
  rotateZ(radians(-45));
  // Aplicar rotaciones según el Filtro de Kalman
  rotateX(radians(-roll));
  rotateY(radians(-pitch));

  // Estilo del Bloque / ESP32
  stroke(0, 255, 200);
  strokeWeight(2);
  fill(30, 40, 55, 230);

  // Dibujar el rectángulo 3D (Ancho, Alto, Profundidad)
  box(240, 120, 20);

  // Dibujar un indicador frontal para distinguir la orientación
  translate(-50, 0, 12);
  fill(255, 50, 50);
  noStroke();
  box(30, 10, 5);
  popMatrix();

  // HUD de información
  drawHUD();
}

// ==========================================
// CLASE PARA EL MANEJO DEL POLVO ESPACIAL
// ==========================================
class DustParticle {
  PVector pos;
  float size;
  float alpha;

  DustParticle() {
    // Posicionar dentro de un volumen 3D amplio alrededor de la cámara
    pos = new PVector(
      random(-width, width * 2),
      random(-height, height * 2),
      random(-600, 600)
      );
    size = random(1.5, 4.0);
    alpha = random(80, 200);
  }

  void update(float ax, float ay) {
    // Factor de sensibilidad para convertir la aceleración en velocidad del polvo
    float sensitivity = 0.015;

    // El movimiento es en sentido CONTRARIO a la aceleración (Fuerza Inercial)
    pos.x += ax * sensitivity;
    pos.y -= ay * sensitivity; // Ajuste según la orientación de coordenadas de Processing


    // Lógica de Teleportación (Respawn) para mantener el polvo siempre visible
    if (pos.x < -width) pos.x = width * 2;
    if (pos.x > width * 2) pos.x = -width;
    if (pos.y < -height) pos.y = height * 2;
    if (pos.y > height * 2) pos.y = -height;
  }

  void display() {
    pushMatrix();
    stroke(200, 225, 255, alpha);
    strokeWeight(size);
    point(pos.x, pos.y, pos.z);
    popMatrix();
  }
}

// ==========================================
// LECTURA SERIAL DE BAJO RETARDO
// ==========================================
void readSerialData() {
  while (myPort.available() > 0) {
    String inString = myPort.readStringUntil('\n');
    if (inString != null) {
      inString = trim(inString);
      String[] list = split(inString, ',');

      // Esperamos 6 valores: Roll, Pitch, AccelX, AccelY, AccelZ, temp
      if (list.length >= 6) {
        roll   = float(list[0]);
        pitch  = float(list[1]);
        accelX = float(list[2]);
        accelY = float(list[3]);
        accelZ = float(list[4]);
        temp = float(list[5]);
      }
    }
  }
}

void drawHUD() {
  fill(255);
  textSize(14);
  text("Roll (Kalman): " + nf(roll, 0, 2) + "°", 20, 30);
  text("Pitch (Kalman): " + nf(pitch, 0, 2) + "°", 20, 50);
  text("Accel X: " + nf(accelX, 0, 0), 20, 80);
  text("Accel Y: " + nf(accelY, 0, 0), 20, 100);
  text("Accel Z: " + nf(accelZ, 0, 0), 20, 120);
  text("Temperature: " + nf(temp, 0, 0), 20, 140);
}
