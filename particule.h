#ifndef PARTICULEH
#define PARTICULEH


#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include "shape.h"
#include "message.h"
#include "constantes.h"
using namespace std;

class Particule {
private:
    carre square;

public:
    Particule(){}
    Particule(double x, double y, double d) : square({{ x,y },d}) {}

    double getX() const { return square.centre.x; }
    double getY() const { return square.centre.y; }
    S2d get_position() const{return{getX(),getY()};}
    double getD() const { return square.d; }
    carre getSquare() const { return {{getX(), getY()}, getD()};}

    bool check_d_min();
    bool check_inside_domain();
    bool check_superpos(Particule p2);
    
    void draw_particle();
    
    
};

#endif
