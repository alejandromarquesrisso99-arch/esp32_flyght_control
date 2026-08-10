#ifndef KALMAN_1D_H
#define KALMAN_1D_H

class Kalman1D {
  private:
    /* Variables de Estado Interno (Protegidas) */
    float angle;   // El ángulo filtrado final
    float bias;    // El sesgo (drift) estimado del giroscopio
    float P[2][2]; // Matriz de covarianza de error 2x2 (Incertidumbre)

    /* Parámetros de Ruido (Personalidad del filtro) */
    float Q_angle;   // Varianza del ruido del modelo (Giroscopio)
    float Q_bias;    // Varianza de la deriva del giroscopio
    float R_measure; // Varianza del ruido de medición (Acelerómetro)

  public:
    Kalman1D(); // Constructor

    // Método principal del ciclo Kalman
    float getAngle(float newAngle, float newRate, float dt);

    // Métodos para cambiar la "personalidad" del filtro en tiempo de ejecución
    void setQangle(float Q_angle);
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    // Métodos para leer parámetros internos (Debugging)
    float getRate(); 
    float getQangle();
    float getQbias();
    float getRmeasure();
};

#endif