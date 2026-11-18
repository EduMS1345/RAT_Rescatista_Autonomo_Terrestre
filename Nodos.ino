#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>

// Instancia del controlador PCA9685 para PWM de servomotores en la dirección I2C 0x40
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
// Instancia para el sensor láser de distancia Time-of-Flight (ToF)
VL53L0X sensor;

// ==========================================
// CONFIGURACIÓN DE SERVOMOTORES (SG90)
// ==========================================
#define SERVO_MIN 102
#define SERVO_MAX 491

#define CAD_NEUTRO_D      70
#define CAD_ADELANTE_D   115
#define CAD_ATRAS_D       40
#define CAD_NEUTRO_I     110
#define CAD_ADELANTE_I    65
#define CAD_ATRAS_I      140
#define ROD_ARRIBA        72
#define ROD_ABAJO         88
#define ROD_ABAJO_TRAS    84
#define DELAY_PASO       180

// ==========================================
// CONFIGURACIÓN DEL SENSOR ToF (VL53L0X)
// ==========================================
#define OFFSET_SENSOR     80
#define N_BUFFER           5
#define DIST_CRITICA     150
#define DIST_PRECAUCION  300
#define DIST_LIBRE       500

int bufferDist[N_BUFFER];
int indexBuffer = 0;
int distanciaFinal = 0;
unsigned long lastLectura = 0;

// Variables para las membresías difusas de distancia
float muCritica = 0.0;
float muPrecaucion = 0.0;
float muLibre = 0.0;

// Estructura de datos compartida recibida vía ESP-NOW desde el ESP32-CAM
typedef struct {
  char cmd;
  int blobSize;
  int centroideX;
  float muIzquierda;
  float muCentro;
  float muDerecha;
  float muSinObjeto;
} Mensaje;

// Mensaje inicial por defecto (en reposo / sin objeto)
volatile Mensaje msgRecibido = {'S', 0, 80, 0.0, 0.0, 0.0, 1.0};

// ==========================================
// FILTROS DIGITALES (VL53L0X)
// ==========================================
// Función de cálculo por mediana (para filtrar ruido impulsivo)
int mediana(int *arr, int n) {
  int temp[n];
  memcpy(temp, arr, n * sizeof(int));
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (temp[j] < temp[i]) {
        int t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }
  return temp[n / 2];
}

// Función de cálculo de media móvil (para suavizar lecturas de distancia)
int promedio(int *arr, int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) suma += arr[i];
  return suma / n;
}

// Lee la distancia láser con control de tiempo de muestreo (no bloqueante)
void leerDistancia() {
  if (millis() - lastLectura < 50) return;
  lastLectura = millis();

  int d = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || d > 2000 || d == 8191) return;

  d -= OFFSET_SENSOR; // Aplica offset de montaje físico
  if (d < 0) d = 0;

  bufferDist[indexBuffer] = d;
  indexBuffer = (indexBuffer + 1) % N_BUFFER;
  distanciaFinal = promedio(bufferDist, N_BUFFER);
}

// ==========================================
// FUNCIONES DE PERTENENCIA (LÓGICA DIFUSA)
// ==========================================
float rampaBaja(float x, float a, float b) {
  if (x <= a) return 1.0;
  if (x >= b) return 0.0;
  return (b - x) / (b - a);
}

float rampaAlta(float x, float a, float b) {
  if (x <= a) return 0.0;
  if (x >= b) return 1.0;
  return (x - a) / (b - a);
}

float triangular(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  if (x <= b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

// Calcula los grados de pertenencia de la distancia frontal leída por el láser
void calcularMembresiasDistancia(float d) {
  muCritica = rampaBaja(d, 100.0, 200.0);
  muPrecaucion = triangular(d, 100.0, 300.0, 550.0);
  muLibre = rampaAlta(d, 400.0, 550.0);
}

// ==========================================
// MOTOR DE INFERENCIA DIFUSA COMBINADA
// ==========================================
// Cruza las variables visuales (cámara) y de profundidad (láser) mediante reglas heurísticas
char inferirComando() {
  float muIzq = msgRecibido.muIzquierda;
  float muCen = msgRecibido.muCentro;
  float muDer = msgRecibido.muDerecha;
  float muSin = msgRecibido.muSinObjeto;

  float aStop = 0.0;
  float aFrente = 0.0;
  float aIzq = 0.0;
  float aDer = 0.0;
  float aAtras = 0.0;
  float aBuscar = 0.0;

  // REGLAS DE OBSTÁCULO Y PRECAUCIÓN
  // R1: Si la distancia es crítica -> Detenerse (STOP) sin importar el resto
  aStop = max(aStop, muCritica);
  // R2: Precaución + Objeto a la izquierda -> Esquivar girando a la derecha
  aDer = max(aDer, min(muPrecaucion, muIzq));
  // R3: Precaución + Objeto a la derecha -> Esquivar girando a la izquierda
  aIzq = max(aIzq, min(muPrecaucion, muDer));
  // R4: Precaución + Objeto al centro o sin objeto (obstáculo físico inesperado) -> Retroceder
  aAtras = max(aAtras, min(muPrecaucion, muCen));
  aAtras = max(aAtras, min(muPrecaucion, muSin));

  // REGLAS DE SEGUIMIENTO (CAMINO LIBRE)
  // R6: Camino libre + Objetivo al centro -> Avanzar de frente
  aFrente = max(aFrente, min(muLibre, muCen));
  // R7: Camino libre + Objetivo a la izquierda -> Girar a la izquierda
  aIzq = max(aIzq, min(muLibre, muIzq));
  // R8: Camino libre + Objetivo a la derecha -> Girar a la derecha
  aDer = max(aDer, min(muLibre, muDer));
  // R9 y R10: Sin objeto a la vista -> Iniciar rutina de búsqueda
  aBuscar = max(aBuscar, min(muLibre, muSin));
  aBuscar = max(aBuscar, min(muPrecaucion * 0.3f, muSin));

  // Arreglo de activaciones y sus respectivos comandos asociados
  float acciones[] = { aStop, aFrente, aIzq, aDer, aAtras, aBuscar };
  char comandos[] = { 'S', 'F', 'L', 'R', 'T', 'B' };

  // Selecciona la acción con mayor grado de activación (Defusificación por máximo)
  int mejor = 0;
  for (int i = 1; i < 6; i++) {
    if (acciones[i] > acciones[mejor]) mejor = i;
  }

  // Si la confianza de activación es muy baja, entra en modo seguro (Reposo)
  if (acciones[mejor] < 0.05) return 'S';
  return comandos[mejor];
}

// ==========================================
// CONTROL DE ACTUADORES (SERVOMOTORES)
// ==========================================
// Traduce ángulos en grados (0-180) a pulsos PWM compatibles con el PCA9685
void escribirServo(int canal, int angulo) {
  angulo = constrain(angulo, 0, 180);
  pca.setPWM(canal, 0, map(angulo, 0, 180, SERVO_MIN, SERVO_MAX));
}

// Coloca al robot en su posición neutral y estable de descanso
void reposo() {
  escribirServo(0, CAD_NEUTRO_D);
  escribirServo(1, CAD_NEUTRO_D);
  escribirServo(2, CAD_NEUTRO_I);
  escribirServo(3, CAD_NEUTRO_I);
  escribirServo(4, ROD_ABAJO);
  escribirServo(5, ROD_ABAJO_TRAS);
  escribirServo(6, ROD_ABAJO);
  escribirServo(7, ROD_ABAJO_TRAS);
}

// Mueve de forma sincronizada la cadera y la rodilla de una pata específica
void moverPata(int idx, int angCadera, int angRodilla) {
  escribirServo(idx, angCadera);
  escribirServo(idx + 4, angRodilla);
}

// ==========================================
// RUTINAS DE CINEMÁTICA Y MARCHA
// ==========================================
// Patrón de marcha hacia adelante tipo "crawl" coordinando diagonales
void caminarAdelante() {
  if (inferirComando() != 'F') return;
  moverPata(0, CAD_ADELANTE_D, ROD_ARRIBA);
  moverPata(3, CAD_ADELANTE_I, ROD_ARRIBA);
  delay(DELAY_PASO);
  moverPata(0, CAD_ADELANTE_D, ROD_ABAJO);
  moverPata(3, CAD_ADELANTE_I, ROD_ABAJO_TRAS);
  delay(DELAY_PASO);
  moverPata(0, CAD_ATRAS_D, ROD_ABAJO);
  moverPata(3, CAD_ATRAS_I, ROD_ABAJO_TRAS);
  delay(DELAY_PASO * 2);

  moverPata(1, CAD_ADELANTE_D, ROD_ARRIBA);
  moverPata(2, CAD_ADELANTE_I, ROD_ARRIBA);
  delay(DELAY_PASO);
  moverPata(1, CAD_ADELANTE_D, ROD_ABAJO_TRAS);
  moverPata(2, CAD_ADELANTE_I, ROD_ABAJO);
  delay(DELAY_PASO);
  moverPata(1, CAD_ATRAS_D, ROD_ABAJO_TRAS);
  moverPata(2, CAD_ATRAS_I, ROD_ABAJO);
  delay(DELAY_PASO * 2);
}

// Patrón de desplazamiento en reversa
void caminarAtras() {
  moverPata(0, CAD_ATRAS_D, ROD_ARRIBA);
  moverPata(3, CAD_ATRAS_I, ROD_ARRIBA);
  delay(DELAY_PASO);
  moverPata(0, CAD_ATRAS_D, ROD_ABAJO);
  moverPata(0, CAD_ADELANTE_D, ROD_ABAJO);
  moverPata(3, CAD_ADELANTE_I, ROD_ABAJO_TRAS);
  delay(DELAY_PASO * 2);
  moverPata(1, CAD_ATRAS_D, ROD_ARRIBA);
  moverPata(2, CAD_ATRAS_I, ROD_ARRIBA);
  delay(DELAY_PASO);
  moverPata(1, CAD_ATRAS_D, ROD_ABAJO_TRAS);
  moverPata(2, CAD_ATRAS_I, ROD_ABAJO);
  delay(DELAY_PASO);
  moverPata(1, CAD_ADELANTE_D, ROD_ABAJO_TRAS);
  moverPata(2, CAD_ADELANTE_I, ROD_ABAJO);
  delay(DELAY_PASO * 2);
}

// Giro coordinado sobre su propio eje hacia la derecha
void girarDerecha() {
  moverPata(0, CAD_ATRAS_D, ROD_ARRIBA);
  moverPata(3, CAD_ADELANTE_I, ROD_ARRIBA);
  delay(DELAY_PASO);
  moverPata(0, CAD_ATRAS_D, ROD_ABAJO);
  moverPata(3, CAD_ADELANTE_I, ROD_ABAJO_TRAS);
  moverPata(1, CAD_ATRAS_D, ROD_ARRIBA);
  moverPata(2, CAD_ADELANTE_I, ROD_ARRIBA);
  delay(DELAY_PASO);
  moverPata(1, CAD_ATRAS_D, ROD_ABAJO_TRAS);
  moverPata(2, CAD_ADELANTE_I, ROD_ABAJO);
  delay(DELAY_PASO);
  reposo();
  delay(200);
}

// Giro coordinado sobre su propio eje hacia la izquierda
void girarIzquierda() {
  moverPata(0, CAD_ADELANTE_D, ROD_ARRIBA);
  moverPata(3, CAD_ATRAS_I, ROD_ARRIBA);
  delay(DELAY_PASO);
  moverPata(0, CAD_ADELANTE_D, ROD_ABAJO);
  moverPata(3, CAD_ATRAS_I, ROD_ABAJO_TRAS);
  delay(DELAY_PASO);
  moverPata(1, CAD_ADELANTE_D, ROD_ARRIBA);
  moverPata(2, CAD_ATRAS_I, ROD_ARRIBA);
  delay(DELAY_PASO);
  moverPata(1, CAD_ADELANTE_D, ROD_ABAJO_TRAS);
  moverPata(2, CAD_ATRAS_I, ROD_ABAJO);
  delay(DELAY_PASO);
  reposo();
  delay(200);
}

// Rutina de búsqueda basada en la última posición conocida del centroide objetivo
void buscarObjeto() {
  if (msgRecibido.centroideX < 80) girarIzquierda();
  else girarDerecha();
}

// ==========================================
// COMUNICACIÓN INALÁMBRICA (ESP-NOW)
// ==========================================
// Callback asíncrono que intercepta los paquetes de datos enviados por la ESP32-CAM
void onRecibido(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Mensaje)) return;
  memcpy((void*)&msgRecibido, data, sizeof(Mensaje));
}

// ==========================================
// CONFIGURACIÓN INICIAL (SETUP)
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // Inicializa bus I2C en pines GPIO 21 (SDA) y 22 (SCL)

  // Configuración del driver PCA9685 a 50Hz
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(50);
  delay(500);
  reposo();
  delay(1500);

  // Inicialización del sensor láser ToF VL53L0X
  if (!sensor.init()) {
    Serial.println("Error VL53L0X");
    while (1);
  }
  sensor.setTimeout(500);
  sensor.startContinuous();
  for (int i = 0; i < N_BUFFER; i++) bufferDist[i] = 400;

  // Configuración del protocolo ESP-NOW en modo Station
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onRecibido);
  Serial.println("Robot listo");
}

// ==========================================
// BUCLE PRINCIPAL (LOOP)
// ==========================================
void loop() {
  // Depuración manual opcional por Monitor Serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'F' || c == 'T' || c == 'R' || c == 'L' || c == 'S' || c == 'B') {
      msgRecibido.muCentro = (c == 'F') ? 1.0 : 0.0;
      msgRecibido.muIzquierda = (c == 'L') ? 1.0 : 0.0;
      msgRecibido.muDerecha = (c == 'R') ? 1.0 : 0.0;
      msgRecibido.muSinObjeto = (c == 'B') ? 1.0 : 0.0;
    }
  }

  // 1. Lectura y filtrado continuo del sensor de distancia láser
  leerDistancia();
  
  // 2. Cálculo de funciones de pertenencia de distancia
  calcularMembresiasDistancia((float)distanciaFinal);
  
  // 3. Ejecución del motor de inferencia difusa combinada
  char cmd = inferirComando();

  // 4. Ejecución motriz según el comando resultante del sistema de decisión
  switch (cmd) {
    case 'F': caminarAdelante(); break;
    case 'T': caminarAtras(); break;
    case 'R': girarDerecha(); break;
    case 'L': girarIzquierda(); break;
    case 'B': buscarObjeto(); break;
    case 'S': reposo(); delay(50); break;
  }
}
