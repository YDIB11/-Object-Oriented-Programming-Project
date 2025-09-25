#include "particule.h"


bool Particule::check_d_min(){
    if (getD() < d_particule_min) {
        cout << message::particle_too_small(getX(), getY(), getD());
        return true;
    }
        return false;

}

bool Particule::check_inside_domain(){
    if ((abs(getX())+getD()/2 > dmax) or (abs(getY())+getD()/2 > dmax)){
    cout << message::particle_outside(getX(), getY(), getD());
    return true;
    }
        return false;

}

bool Particule::check_superpos(Particule p2){
    if (collision_carres(getSquare(), p2.getSquare(), false)){
        cout << message::particle_superposition(getX(), getY(), p2.getX(), p2.getY());
    return true;

    }
        return false;

}

void Particule::draw_particle(){
	draw_particle_(getSquare());
}

