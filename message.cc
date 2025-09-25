// message.cc : fonctions pour l'affichage des messages d'erreur
// Version 1.2: fonctions supplémentaire pour indiquer le succès de la lecture 
// Version 1.1: 9 fonctions de messages d'erreurs à utiliser pour le projet 
// 

#include "message.h"

using namespace std;

namespace
{
    void reorder_for_consistency(double &  x1, double & y1, double & x2, double & y2)
    {
        if(x1 > x2 || (x1 == x2 && y1 > y2))
        {
            swap(x1, x2);
            swap(y1, y2);
        }
    }
}

string message::particle_outside(double x, double y, double s)
{
    return string("Particle at (") + to_string(x) + string(";") + to_string(y)
        + string(") of size ") + to_string(s) + string(" is outside boundaries\n");
}

string message::particle_too_small(double x, double y, double s)
{
    return string("Particle at (") + to_string(x) + string(";") + to_string(y)
        + string(") of size ") + to_string(s) + string(" is too small\n");
}


string message::spatial_robot_ouside(double x, double y)
{
    return string("Spatial robot at (") + to_string(x) + string(";") + to_string(y)
        + string(") is outside boundaries\n");
}

std::string message::invalid_k_update(double x, double y, int k_update, int max_update)
{
    return string("A robot at (") + to_string(x) + string(";") + to_string(y)
        + string(") has a k_update of ") + to_string(k_update)
        + string(" which is greater than the maximum of ") + to_string(max_update)
        + string("\n");
}


string message::particle_superposition(double x1, double y1, double x2, double y2)
{
    reorder_for_consistency(x1, y1, x2, y2);

    return string("Particle at (") + to_string(x1) + string(";") + to_string(y1)
        + string(") and particle at (") + to_string(x2) + string(";") + to_string(y2)
        + string(") are in superposition\n");
}

string message::repairers_superposition(double x1, double y1, double x2, double y2)
{
    reorder_for_consistency(x1, y1, x2, y2);

    return string("Repairer at (") + to_string(x1) + string(";") + to_string(y1)
        + string(") and repairer at (") + to_string(x2) + string(";") + to_string(y2)
        + string(") are in superposition\n");
}

string message::neutralizers_superposition(double x1, double y1, double x2, double y2)
{
    reorder_for_consistency(x1, y1, x2, y2);

    return string("Neutralizer at (") + to_string(x1) + string(";") + to_string(y1)
        + string(") and neutralizer at (") + to_string(x2) + string(";")
        + to_string(y2) + string(") are in superposition\n");
}

string message::repairer_neutralizer_superposition(double xr, double yr,
                                                   double xn, double yn)
{
    return string("Repairer at (") + to_string(xr) + string(";") + to_string(yr)
        + string(") and neutralizer at (") + to_string(xn) + string(";")
        + to_string(yn) + string(") are in superposition\n");
}

std::string message::particle_robot_superposition(double xp, double yp, double sp,
                                                  double xr, double yr, double rr)
{
    return string("Particle at (") + to_string(xp) + string(";") + to_string(yp)
        + string(") of size ") + to_string(sp) + string(" and robot at (")
        + to_string(xr) + string(";") + to_string(yr) + string(") of radius ")
        + to_string(rr) + string(" are in superposition\n");
}

string message::success()
{
	return string("Correct file\n");
}
