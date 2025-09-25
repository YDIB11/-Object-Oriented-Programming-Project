#ifndef CONSTANTES_H
#define CONSTANTES_H

#include "shape.h" // necessary to use epsil_zero and have the symbols from graphic.h
enum Etat_neutraliseur { EN_PANNE, EN_MARCHE };


constexpr short unsigned maxF(25);
constexpr double dmax(128.0);
constexpr double delta_t(0.125); // second
constexpr double r_spatial(16.0);
constexpr double r_reparateur(2.0);
constexpr double r_neutraliseur(4.0);
constexpr double vtran_max(4.0); // per second
constexpr double vrot_max(0.125); // rd/s approx. 7°/s
constexpr double epsil_alignement(0.01); // rd approx. 0.6°
constexpr double desintegration_rate(0.002);
constexpr double risk_factor(3.0);
constexpr double d_particule_min(8 * epsil_zero);
constexpr unsigned max_update(600);
constexpr unsigned modulo_update(100);

#endif // CONSTANTES_H
