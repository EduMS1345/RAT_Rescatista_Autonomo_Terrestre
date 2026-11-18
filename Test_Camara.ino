#include "esp_camera.h"
#include <WiFi.h>
#include <esp_now.h>

// ==========================================
// CONFIGURACIÓN DE PINES (CÁMARA AI-THINKER)
// ==========================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ==========================================
// DIRECCIÓN MAC DEL ESP32 RECEPTOR (PRINCIPAL)
// ==========================================
uint8_t macReceptor[] = {
  0x20, 0xE7, 0xC8, 0x65, 0xE0, 0x2C
};

// ==========================================
// UMBRALES DE COLOR (ESPACIO HSV)
// ==========================================
// Define el rango cromático para segmentar el estímulo objetivo
#define H_MIN 20
#define H_MAX 40
#define S_MIN 180
#define V_MIN 150

// Dimensiones de la imagen reducida y límites del blob (píxeles agrupados)
#define FRAME_W   160
#define FRAME_H   120
#define BLOB_MIN  100
#define BLOB_CERCA 3000

// ==========================================
// ESTRUCTURA DE DATOS COMPARTIDA (ESP-NOW)
// ==========================================
typedef struct {
  char cmd;               // Comando motriz resultante ('F', 'L', 'R', 'S', 'B')
  int blobSize;           // Tamaño de la masa de píxeles detectados (estimador de distancia)
  int centroideX;         // Coordenada horizontal del centroide (estimador de ángulo)
  float muIzquierda;      // Grado de membresía hacia la izquierda
  float muCentro;         // Grado de membresía hacia el centro
  float muDerecha;        // Grado de membresía hacia la derecha
  float muSinObjeto;      // Grado de membresía de ausencia de objeto
} Mensaje;

Mensaje msg;

// Almacena la última posición horizontal conocida del centroide para la rutina de búsqueda
static int lastCX = FRAME_W / 2;

// ==========================================
// CONVERSIÓN DE ESPACIO DE COLOR (RGB a HSV)
// ==========================================
// Transforma los canales RGB de la cámara a HSV para mayor robustez frente a cambios de luz
void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, float &h, float &s, float &v) {
  float rf = r / 255.0, gf = g / 255.0, bf = b / 255.0;
  float mx = max(rf, max(gf, bf));
  float mn = min(rf, min(gf, bf));
  float delta = mx - mn;
  v = mx;
  s = (mx == 0) ? 0 : delta / mx;
  if (delta == 0) { h = 0; return; }
  if (mx == rf) h = 60.0 * fmod((gf - bf) / delta, 6.0);
  else if (mx == gf) h = 60.0 * ((bf - rf) / delta + 2.0);
  else h = 60.0 * ((rf - gf) / delta + 4.0);
  if (h < 0) h += 360.0;
}

// ==========================================
// FUNCIONES DE PERTENENCIA (LÓGICA DIFUSA)
// ==========================================
// Función rampa decreciente para conjuntos difusos
float rampaBaja(float x, float a, float b) {
  if (x <= a) return 1.0;
  if (x >= b) return 0.0;
  return (b - x) / (b - a);
}

// Función rampa creciente para conjuntos difusos
float rampaAlta(float x, float a, float b) {
  if (x <= a) return 0.0;
  if (x >= b) return 1.0;
  return (x - a) / (b - a);
}

// Función triangular para modelar pertenencias intermedias
float triangular(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  if (x <= b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

// Evalúa la confianza de detección basada en el tamaño (área) del blob de píxeles
float confianzaBlob(int blob) {
  if (blob < BLOB_MIN) return 0.0;
  if (blob > BLOB_CERCA) return 0.0; // Demasiado cerca, evita seguir procesando este objeto
  float c = triangular((float)blob, (float)BLOB_MIN, 1200.0, (float)BLOB_CERCA);
  c = max(c, rampaAlta((float)blob, (float)BLOB_MIN, 300.0) * rampaBaja((float)blob, 2000.0, (float)BLOB_CERCA));
  return constrain(c, 0.0, 1.0);
}

// ==========================================
// CÁLCULO DE MEMBRESÍAS DE DIRECCIÓN (VISIÓN)
// ==========================================
// Mapea la posición horizontal del centroide (cx) en grados de pertenencia lingüísticos
void calcularMembresiasDireccion(int cx, int blob, float &muIzq, float &muCen, float &muDer, float &muSin) {
  float conf = confianzaBlob(blob);
  
  // Si no hay confianza suficiente (objeto ausente o ruido), activa el estado de búsqueda
  if (conf < 0.05) {
    muIzq = (lastCX < FRAME_W / 2) ? 0.7 : 0.1;
    muDer = (lastCX >= FRAME_W / 2) ? 0.7 : 0.1;
    muCen = 0.0;
    muSin = 1.0;
    return;
  }
  
  // Asigna grados de pertenencia según la ubicación espacial del centroide en la imagen
  muIzq = rampaBaja((float)cx, 45.0, 80.0) * conf;
  muCen = triangular((float)cx, 45.0, 80.0, 115.0) * conf;
  muDer = rampaAlta((float)cx, 80.0, 115.0) * conf;
  muSin = (1.0 - conf);
  lastCX = cx; // Actualiza la última posición conocida
}

// ==========================================
// ETAPA DE DEFUZZIFICACIÓN LOCAL (CÁMARA)
// ==========================================
// Traduce las membresías y el tamaño del blob en un carácter de comando preliminar
char defuzzificar(float muIzq, float muCen, float muDer, float muSin, int blob) {
  if (blob > BLOB_CERCA) return 'S'; // Detenerse por colisión inminente
  if (muSin > 0.6) return 'B';       // Buscar objeto si se perdió del campo visual
  if (muCen >= muIzq && muCen >= muDer) return 'F'; // Avanzar si está centrado
  if (muIzq > muDer) return 'L';     // Girar a la izquierda
  return 'R';                        // Girar a la derecha
}

// Callback de envío inalámbrico ESP-NOW (vacío ya que no requiere confirmación estricta de eco)
void onEnviado(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

// ==========================================
// CONFIGURACIÓN INICIAL (SETUP)
// ==========================================
void setup() {
  Serial.begin(115200);

  // Estructura de configuración para la cámara OV2640
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565; // Formato de píxel para procesamiento ágil
  config.frame_size = FRAMESIZE_QQVGA;  // Resolución reducida para optimizar RAM
  config.fb_count = 1;
  config.jpeg_quality = 12;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  // Inicializa la cámara y valida errores
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Error camara");
    return;
  }

  // Configura el protocolo ESP-NOW en modo estación (STA)
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error ESP-NOW");
    return;
  }

  // Registra el peer receptor (ESP32 principal)
  esp_now_register_send_cb(onEnviado);
  esp_now_peer_info_t peer;
  memcpy(peer.peer_addr, macReceptor, 6);
  peer.channel = 0;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);

  Serial.println("ESP32-CAM LISTA");
}

// ==========================================
// BUCLE PRINCIPAL (LOOP)
// ==========================================
void loop() {
  // Captura un fotograma de la cámara
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { delay(100); return; }

  long sumX = 0, count = 0;

  // Bucle anidado para recorrer cada píxel de la imagen QQVGA (160x120)
  for (int y = 0; y < FRAME_H; y++) {
    for (int x = 0; x < FRAME_W; x++) {
      int idx = (y * FRAME_W + x) * 2;
      uint16_t pixel = (fb->buf[idx] << 8) | fb->buf[idx + 1];

      // Extrae los componentes de color RGB a partir del formato RGB565
      uint8_t r = (pixel >> 11) << 3;
      uint8_t g = ((pixel >> 5) & 0x3F) << 2;
      uint8_t b = (pixel & 0x1F) << 3;

      float h, s, v;
      rgb2hsv(r, g, b, h, s, v); // Convierte a espacio HSV

      // Filtra el píxel evaluando si coincide con el rango de color del objetivo
      if (h >= H_MIN && h <= H_MAX && s * 255 >= S_MIN && v * 255 >= V_MIN) {
        sumX += x; // Acumula las coordenadas horizontales para calcular el centroide
        count++;   // Cuenta los píxeles totales que conforman el objeto (blob)
      }
    }
  }

  // Libera el búfer de la cámara para la siguiente captura
  esp_camera_fb_return(fb);

  // Calcula la coordenada horizontal del centroide (cx) si se detectó el objeto; de lo contrario, centra el valor
  int cx = (count > 0) ? sumX / count : FRAME_W / 2;

  // Calcula las membresías difusas espaciales y de tamaño del blob
  calcularMembresiasDireccion(cx, (int)count, msg.muIzquierda, msg.muCentro, msg.muDerecha, msg.muSinObjeto);
  msg.blobSize = (int)count;
  msg.cmd = defuzzificar(msg.muIzquierda, msg.muCentro, msg.muDerecha, msg.muSinObjeto, (int)count);
  msg.centroideX = cx;

  // Empaqueta y transmite la estructura de datos por ESP-NOW hacia el ESP32 principal
  esp_now_send(macReceptor, (uint8_t*)&msg, sizeof(msg));

  // Retraso controlado para estabilizar la tasa de cuadros por segundo del nodo visual
  delay(120);
}
