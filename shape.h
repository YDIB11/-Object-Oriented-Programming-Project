#ifndef SHAPEH
#define SHAPEH

#include <iostream>
#include "graphic.h"
using namespace std;
//Constantes Globales

constexpr double epsil_zero(0.125);

//Structures

struct S2d {double x = 0.0; double y = 0.0; };
struct cercle {S2d centre; double r ;};
struct carre {S2d centre; double d;};

//Fonctions

bool collision_cercles(cercle C1, cercle C2, bool use_epsilon_zero);

bool collision_carres(carre C1, carre C2, bool use_epsilon_zero);

bool collision_carre_cercle(carre Ca, cercle Ce, bool use_epsilon_zero);

void draw_particle_(carre P);
void draw_robot_spatial_(cercle C);
void draw_robot_repairer_(cercle C);
void draw_robot_neutralizer_panne_(cercle C, double angle);
void draw_robot_neutralizer_service_(cercle C, double angle);
void draw_robot_neutralizer_collistion_(cercle C, double angle);

#endif //SHAPEH
