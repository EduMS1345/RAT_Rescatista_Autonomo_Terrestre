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
// UMBRALES DE COLOR (CONFIGURADO PARA NARANJA)
// H en grados (0-360), S y V en escala (0-255)
// ==========================================
#define H_MIN   5
#define H_MAX  22
#define S_MIN 160
#define V_MIN 120
 
// Bandera de calibración: 1 = imprime los valores HSV del centro de la imagen en el monitor serial
#define CALIBRAR 1
 
// Dimensiones reducidas de la imagen y tamaño mínimo del objeto (blob)
#define FRAME_W    160
#define FRAME_H    120
#define BLOB_MIN   100
 
// ==========================================
// ESTRUCTURA DE DATOS COMPARTIDA (ESP-NOW)
// ==========================================
typedef struct {
  char  cmd;               // Comando de movimiento resultante ('F', 'L', 'R', 'B', 'S')
  int   blobSize;          // Tamaño total de píxeles del objeto detectado
  int   centroideX;        // Posición horizontal del centroide del objeto
  float muIzquierda;       // Grado de membresía difusa hacia la izquierda
  float muCentro;          // Grado de membresía difusa hacia el centro
  float muDerecha;         // Grado de membresía difusa hacia la derecha
  float muSinObjeto;       // Grado de membresía difusa de ausencia de objetivo
} Mensaje;
 
Mensaje msg;
 
// Almacena la última coordenada X conocida para la rutina de búsqueda
static int lastCX = FRAME_W / 2;
 
// ==========================================
// CONVERSIÓN DE ESPACIO DE COLOR (RGB a HSV)
// ==========================================
void rgb2hsv(uint8_t r, uint8_t g, uint8_t b,
             float &h, float &s, float &v) {
  float rf = r / 255.0, gf = g / 255.0, bf = b / 255.0;
  float mx = max(rf, max(gf, bf));
  float mn = min(rf, min(gf, bf));
  float delta = mx - mn;
  v = mx;
  s = (mx == 0) ? 0 : delta / mx;
  if (delta == 0) { h = 0; return; }
  if      (mx == rf) h = 60.0 * fmod((gf - bf) / delta, 6.0);
  else if (mx == gf) h = 60.0 * ((bf - rf) / delta + 2.0);
  else               h = 60.0 * ((rf - gf) / delta + 4.0);
  if (h < 0) h += 360.0;
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
 
// Calcula la confianza del blob (0 a 1) sin límite superior restrictivo
float confianzaBlob(int blob) {
  if (blob < BLOB_MIN) return 0.0;
  return constrain(rampaAlta((float)blob, (float)BLOB_MIN, 300.0), 0.0, 1.0);
}
 
// ==========================================
// CÁLCULO DE MEMBRESÍAS DE DIRECCIÓN
// ==========================================
void calcularMembresiasDireccion(int cx, int blob,
                                 float &muIzq, float &muCen,
                                 float &muDer, float &muSin) {
  float conf = confianzaBlob(blob);
 
  // Si no hay confianza suficiente, activa el estado de búsqueda basado en la última posición
  if (conf < 0.05) {
    muIzq = (lastCX < FRAME_W / 2) ? 0.7 : 0.1;
    muDer = (lastCX >= FRAME_W / 2) ? 0.7 : 0.1;
    muCen = 0.0;
    muSin = 1.0;
    return;
  }
 
  // Asigna membresías espaciales escaladas por la confianza del blob detectado
  muIzq = rampaBaja((float)cx, 45.0, 80.0) * conf;
  muCen = triangular((float)cx, 45.0, 80.0, 115.0) * conf;
  muDer = rampaAlta((float)cx, 80.0, 115.0) * conf;
  muSin = (1.0 - conf);
 
  lastCX = cx;
}
 
// ==========================================
// ETAPA DE DEFUZZIFICACIÓN
// ==========================================
char defuzzificar(float muIzq, float muCen, float muDer, float muSin) {
  // Si se pierde el objeto, manda comando de búsqueda
  if (muSin > 0.6) return 'B';
 
  // Evalúa el valor máximo entre izquierda, centro y derecha
  if (muCen >= muIzq && muCen >= muDer) return 'F';
  if (muIzq > muDer)                    return 'L';
  if (muDer > muIzq)                    return 'R';
 
  return 'S';
}
 
// Callback de envío ESP-NOW
void onEnviado(const wifi_tx_info_t *info,
               esp_now_send_status_t status) {}
 
// ==========================================
// CONFIGURACIÓN INICIAL (SETUP)
// ==========================================
void setup() {
  Serial.begin(115200);
 
  camera_config_t config;
  config.ledc_channel    = LEDC_CHANNEL_0;
  config.ledc_timer      = LEDC_TIMER_0;
  config.pin_d0          = Y2_GPIO_NUM;
  config.pin_d1          = Y3_GPIO_NUM;
  config.pin_d2          = Y4_GPIO_NUM;
  config.pin_d3          = Y5_GPIO_NUM;
  config.pin_d4          = Y6_GPIO_NUM;
  config.pin_d5          = Y7_GPIO_NUM;
  config.pin_d6          = Y8_GPIO_NUM;
  config.pin_d7          = Y9_GPIO_NUM;
  config.pin_xclk        = XCLK_GPIO_NUM;
  config.pin_pclk        = PCLK_GPIO_NUM;
  config.pin_vsync       = VSYNC_GPIO_NUM;
  config.pin_href        = HREF_GPIO_NUM;
  config.pin_sscb_sda    = SIOD_GPIO_NUM;
  config.pin_sscb_scl    = SIOC_GPIO_NUM;
  config.pin_pwdn        = PWDN_GPIO_NUM;
  config.pin_reset       = RESET_GPIO_NUM;
  config.xclk_freq_hz    = 20000000;
  config.pixel_format    = PIXFORMAT_RGB565;
  config.frame_size      = FRAMESIZE_QQVGA;
  config.fb_count        = 1;
  config.jpeg_quality    = 12;
  config.grab_mode       = CAMERA_GRAB_WHEN_EMPTY;
 
  // Inicialización de la cámara
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Error camara");
    return;
  }
 
  // Configuración de Wi-Fi y protocolo ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error ESP-NOW");
    return;
  }
 
  esp_now_register_send_cb(onEnviado);
 
  // Registro del peer receptor principal
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, macReceptor, 6);
  peer.channel = 0;
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_STA;
  esp_now_add_peer(&peer);
 
  Serial.println("ESP32-CAM LISTA (NARANJA)");
}
 
// ==========================================
// BUCLE PRINCIPAL (LOOP)
// ==========================================
void loop() {
  // Obtención del fotograma actual de la cámara
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { delay(100); return; }
 
  long sumX = 0, count = 0;
 
  // Variables acumuladoras para calibración óptica en una ventana central de 11x11 píxeles
  float calH = 0, calS = 0, calV = 0;
  int   calN = 0;
 
  // Barrido matricial de cada píxel de la imagen QQVGA (160x120)
  for (int y = 0; y < FRAME_H; y++) {
    for (int x = 0; x < FRAME_W; x++) {
      int idx = (y * FRAME_W + x) * 2;
      uint16_t pixel = (fb->buf[idx] << 8) | fb->buf[idx + 1];
 
      // Extracción de canales RGB a partir de formato RGB565
      uint8_t r = (pixel >> 11) << 3;
      uint8_t g = ((pixel >> 5) & 0x3F) << 2;
      uint8_t b = (pixel & 0x1F) << 3;
 
      float h, s, v;
      rgb2hsv(r, g, b, h, s, v);
 
      // Filtrado por color configurado (Naranja)
      if (h >= H_MIN && h <= H_MAX &&
          s * 255 >= S_MIN && v * 255 >= V_MIN) {
        sumX += x;
        count++;
      }
 
      // Muestreo del centro para calibración en vivo
      if (abs(x - FRAME_W / 2) <= 5 && abs(y - FRAME_H / 2) <= 5) {
        calH += h; calS += s * 255; calV += v * 255; calN++;
      }
    }
  }
 
  // Libera la memoria del búfer del fotograma
  esp_camera_fb_return(fb);
 
  // Cálculo de la coordenada del centroide horizontal (cx)
  int cx = (count > 0) ? sumX / count : FRAME_W / 2;
 
  // Procesamiento de membresías difusas y defusificación
  calcularMembresiasDireccion(cx, (int)count,
    msg.muIzquierda, msg.muCentro,
    msg.muDerecha,   msg.muSinObjeto);
 
  msg.cmd        = defuzzificar(msg.muIzquierda, msg.muCentro,
                                msg.muDerecha, msg.muSinObjeto);
  msg.blobSize   = (int)count;
  msg.centroideX = cx;
 
  // Transmisión de datos empaquetados hacia el ESP32 principal mediante ESP-NOW
  esp_now_send(macReceptor, (uint8_t*)&msg, sizeof(msg));
 
  // Impresión de depuración en el monitor serial
  Serial.print("Blob:"); Serial.print(count);
  Serial.print(" X:");   Serial.print(cx);
  Serial.print(" uI:");  Serial.print(msg.muIzquierda, 2);
  Serial.print(" uC:");  Serial.print(msg.muCentro,    2);
  Serial.print(" uD:");  Serial.print(msg.muDerecha,   2);
  Serial.print(" uS:");  Serial.print(msg.muSinObjeto, 2);
  Serial.print(" CMD:"); Serial.println(msg.cmd);
 
#if CALIBRAR
  if (calN > 0) {
    Serial.print("   [CENTRO] H:"); Serial.print(calH / calN, 0);
    Serial.print(" S:");            Serial.print(calS / calN, 0);
    Serial.print(" V:");            Serial.println(calV / calN, 0);
  }
#endif
 
  delay(120);
}
