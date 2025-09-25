#ifndef MESSAGE_H_INCLUDED
#define MESSAGE_H_INCLUDED

// message.h  : fonctions pour l'affichage des messages d'erreur
// Version 1.2: fonctions supplémentaire pour indiquer le succès de la lecture 
// Version 1.1: 9 fonctions de messages d'erreurs à utiliser pour le projet 
// 

#include <string>

namespace message
{
    std::string particle_outside(double x, double y, double s);
    std::string particle_too_small(double x, double y, double s);

    std::string spatial_robot_ouside(double x, double y);
    std::string invalid_k_update(double x, double y, int k_update, int max_update);

    std::string particle_superposition(double x1, double y1, double x2, double y2);
    std::string repairers_superposition(double x1, double y1, double x2, double y2);
    std::string neutralizers_superposition(double x1, double y1, double x2, double y2);
    std::string repairer_neutralizer_superposition(double xr, double yr,
                                                   double xn, double yn);
    std::string particle_robot_superposition(double xp, double yp, double sp,
                                             double xr, double yr, double rr);

    // Everything went well => file reading and all validation checks
	std::string success();
}

#endif // MESSAGE_H_INCLUDED
