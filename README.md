# Rescatista Autonomo Terrestre (RAT)

Proyecto integrador de aprendizaje enfocado en el diseño, construcción y programación de un robot cuadrúpedo capaz de navegar de forma autónoma en un entorno delimitado, detectando un estímulo visual y evadiendo obstáculos mediante un sistema de control distribuido e inteligencia artificial basada en lógica difusa[cite: 3, 4, 5, 24].

## Arquitectura del Sistema
* **Cómputo Visual:** ESP32-CAM con sensor OV2640 (segmentación de color HSV y cálculo de centroide)[cite: 16, 23, 27].
* **Cómputo Principal y Decisión:** ESP32 de 38 pines ejecutando un motor de inferencia de lógica difusa[cite: 16, 49].
* **Comunicación:** Enlace inalámbrico de baja latencia mediante el protocolo **ESP-NOW**[cite: 4, 27].
* **Sensores Auxiliares:** Sensor láser de distancia **VL53L0X (Time-of-Flight)** con filtrado de media móvil/mediana[cite: 17, 29, 48].
* **Actuación:** Controlador PWM **PCA9685** de 16 canales gobernando 8 servomotores **SG90** (2 grados de libertad por pata)[cite: 6, 7, 8].
* **Alimentación:** Banco de celdas 18650 (2S2P, 7.4V nominales) con regulador Step-Down Buck (XL4016) a 5V[cite: 18].
