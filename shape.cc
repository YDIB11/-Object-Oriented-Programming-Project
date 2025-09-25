#include <iostream>
#include <cmath>
#include "shape.h"

using namespace std;

//Collison entre cercle et cercle	
bool collision_cercles(cercle C1, cercle C2, bool use_epsilon_zero){
	double epsilon(0);
	if (use_epsilon_zero){
		epsilon = epsil_zero;
	}
	double D = sqrt((C1.centre.x-C2.centre.x)*(C1.centre.x-C2.centre.x) 
				+(C1.centre.y-C2.centre.y)*(C1.centre.y-C2.centre.y));
	return (D < C1.r + C2.r + epsilon);
}
//Collison entre carre et carre
bool collision_carres(carre C1, carre C2, bool use_epsilon_zero){
	double epsilon(0);
	if (use_epsilon_zero){
		epsilon = epsil_zero;
	}
	double D_x = abs(C2.centre.x - C1.centre.x);
	double D_y = abs(C2.centre.y - C1.centre.y);
	return ((D_x < C1.d/2 + C2.d/2 + epsilon) and
			(D_y < C1.d/2 + C2.d/2 + epsilon));
 }

//Collison entre carre et cercle
bool collision_carre_cercle(carre Ca, cercle Ce, bool use_epsilon_zero){
	double epsilon(0);
	if (use_epsilon_zero){
		epsilon = epsil_zero;
	}
	double D_x = abs(Ce.centre.x - Ca.centre.x);
	double D_y = abs(Ce.centre.y - Ca.centre.y);
	double L = sqrt( (D_x - Ca.d/2)*(D_x - Ca.d/2) + 
					 (D_y - Ca.d/2)*(D_y - Ca.d/2) );
	if ((D_x > Ca.d/2) and (D_y > Ca.d/2) and (L > Ce.r + epsilon)){
		return false;
	} else {
		return ((D_x < Ca.d/2 +Ce.r +epsilon) 
		and (D_y < Ca.d/2 + Ce.r + epsilon));
	}
}

void draw_particle_(carre P) {
	draw_particle(P.centre.x,P.centre.y,P.d);
}
void draw_robot_spatial_(cercle C){
	draw_robot_spatial(C.centre.x,C.centre.y,C.r);
}
void draw_robot_repairer_(cercle C){
	draw_robot_repairer(C.centre.x,C.centre.y,C.r);
}
void draw_robot_neutralizer_panne_(cercle C, double angle){
	draw_robot_neutralizer_panne(C.centre.x,C.centre.y,C.r, angle);
}
void draw_robot_neutralizer_service_(cercle C, double angle){
	draw_robot_neutralizer_service(C.centre.x,C.centre.y,C.r,  angle);
}
void draw_robot_neutralizer_collistion_(cercle C, double angle){
	draw_robot_neutralizer_collistion(C.centre.x,C.centre.y,C.r,  angle);
}
