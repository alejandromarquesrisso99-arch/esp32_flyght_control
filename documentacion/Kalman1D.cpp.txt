#include "Kalman1D.h"

Kalman1D::Kalman1D() {
    // Configuración estándar de arranque
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f;
    bias = 0.0f;

    P[0][0] = 0.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.0f;
}

float Kalman1D::getAngle(float newAngle, float newRate, float dt) {
    // --- FASE 1: PREDICCIÓN ---
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // --- FASE 2: CORRECCIÓN ---
    float y = newAngle - angle; // Innovación (Error entre medición y predicción)
    float S = P[0][0] + R_measure; // Incertidumbre total
    
    float K[2]; // Ganancia de Kalman
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Corrección del estado
    angle += K[0] * y;
    bias += K[1] * y;

    // Actualización de la covarianza
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

// Implementación de Setters y Getters
void Kalman1D::setQangle(float Q_angle) { this->Q_angle = Q_angle; }
void Kalman1D::setQbias(float Q_bias)   { this->Q_bias = Q_bias; }
void Kalman1D::setRmeasure(float R_measure) { this->R_measure = R_measure; }
float Kalman1D::getRate()     { return bias; }
float Kalman1D::getQangle()   { return Q_angle; }
float Kalman1D::getQbias()    { return Q_bias; }
float Kalman1D::getRmeasure() { return R_measure; }