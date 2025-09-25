#ifndef ROBOTH
#define ROBOTH

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include "shape.h"
#include "particule.h"
#include "constantes.h"


class Robot {
protected:
    cercle circle;

public:
    Robot(double x, double y, double r) : circle({ {x,y},r }) {}

    virtual ~Robot() {}

    double getX() const { return circle.centre.x; }
    double getY() const { return circle.centre.y; }
    double getR() const { return circle.r; }
    S2d get_position() const{return{getX(),getY()};}
    cercle getCircle() const { return {{getX(), getY()}, getR()};}
};

class RobotNeutralisateur : public Robot {
private:

    double a, c_n;
    bool panne;
    int k_update_panne;
public:
    RobotNeutralisateur(double x, double y, double a, double c_n, double panne, 
    double k_update_panne) :
            Robot(x, y, r_neutraliseur), a(a), c_n(c_n), panne(panne), 
            k_update_panne(k_update_panne){}

    // Getters
    double getA() const { return a; }
    double getC_n() const { return c_n; }
    bool getPanne() const { return panne; }
    bool is_within_risk_zone(const Particule &p);
    void setPanne(bool panneTemp) {panne = panneTemp;}
    void setK_update_panne(int k_update_panne_in){k_update_panne = k_update_panne_in;}
    int getK_update_panne() const { return k_update_panne; }
    void draw_robot_neutralizer_panne() const;
	void draw_robot_neutralizer_service()const;
	void draw_robot_neutralizer_collistion()const;
	bool isColliding(const vector<RobotNeutralisateur>& robots);
    void rotateToAngle(double desiredAngle);
    void moveForwards();
    void moveTowardsTarget(const S2d& target);
    double getDirectionToPoint(double x1, double y1, double x2, double y2);
    double getDistance(double x1, double y1, double x2, double y2);
    int getCollisionSide(const Particule& target, double x, double y);
    int checkDiagonal(double x, double y, double cornerX,
        double cornerY,int nonDiagonalSide1, int nonDiagonalSide2, int diagonalSide);
    int getSide(int corner, double x, double y, double left, 
    double right, double top, double bottom);
    int getCollisionSide2(const Particule& target);
    double desiredAngle0(const Particule& target);

    double calculateTimeToTarget(const Particule &target);
    void move_to_type0(const Particule& target); 

};


class RobotReparateur : public Robot {
private:
    double reachtime;

public:
    RobotReparateur(double x, double y) : Robot(x, y, r_reparateur) {}
    void draw_robot_repairer() const;
    void move_reparateur(const RobotNeutralisateur& target);
    double getReachTime(){return reachtime;}
    bool repCheckCollision(const vector<RobotReparateur>& reparateurs,
    const vector<RobotNeutralisateur>& neutralisateurs);
    
    
};


class RobotSpatial : public Robot {
private:
    int nbUpdate, nbNr, nbNs, nbNd, nbRr, nbRs;
    int nbRs_initial, nbNs_initial; 
    // Ajout de variables pour stocker les valeurs initiales
    std::vector<RobotReparateur> reparateurs;
    std::vector<RobotNeutralisateur> neutralisateurs;

public:
    RobotSpatial(double x, double y, int nbUpdate, int nbNr, int nbNs, int nbNd, 
    int nbRr, int nbRs) :
            Robot(x, y, r_spatial), nbUpdate(nbUpdate), nbNr(nbNr), nbNs(nbNs), 
            nbNd(nbNd), nbRr(nbRr), nbRs(nbRs),
            nbRs_initial(nbRs), nbNs_initial(nbNs) {} // Initialiser les variables
            
            
    void setNbUpdate(int newNbUpdate){ nbUpdate = newNbUpdate;}

    // Getters
    int getNbUpdate() const { return nbUpdate; }
    int getNbNr() const { return nbNr; }
    int getNbNs() const { return nbNs; }
    int getNbNd() const { return nbNd; }
    int getNbRr() const { return nbRr; }
    int getNbRs() const { return nbRs; }
    int getnbRs_initial() const { return nbRs_initial; }
    int getnbNs_initial() const { return nbNs_initial; }
    vector<RobotReparateur> getReparateurs() const { return reparateurs; }
    vector<RobotNeutralisateur> getNeutralisateurs() const { return neutralisateurs; }
    int getNbRs_initial() const { return nbRs_initial; } // Getter pour nbRs_initial
    int getNbNs_initial() const { return nbNs_initial; } // Getter pour nbNs_initial
	int getNbNp() const;
// Setter pour ajouter un réparateur
    void addReparateur(const RobotReparateur& reparateur) {
        reparateurs.push_back(reparateur);
        --nbRs;
    }

// Setter pour ajouter un neutralisateur
    void addNeutralisateur(const RobotNeutralisateur& neutralisateur) {
        neutralisateurs.push_back(neutralisateur);
        --nbNs;
    }
    // Check les erreurs

    bool check_inside_domain(RobotSpatial RS);
    bool check_superpos_N_N(RobotNeutralisateur n1, RobotNeutralisateur n2);
    bool check_superpos_R_R(RobotReparateur r1, RobotReparateur r2);
    bool check_superpos_R_N(RobotReparateur r1, RobotNeutralisateur n1);
    bool check_superpos_R_P(Robot R, Particule p);
    bool check_nbUpdate(RobotNeutralisateur n, int k_update_panne_in);
    

    void draw_robot_spatial();
    void clear_robot();
    int findCollidingParticle(const RobotNeutralisateur& robot,
                const std::vector<Particule>& particles);
    void manage_movements( vector<Particule> & vector_particules);
    int findNearestUnassignedRobot(const Particule& p,
                const vector<bool>& robotAssigned);
    void manage_reparateurs();
    void spawnNeutralisateur(vector<Particule> vector_particules);
    void spawnReparateur();
    int chooseReparateur(const RobotNeutralisateur& robot, 
                const vector<bool>& reparateurAssigned);
    void neutralisateurReturnDock(vector<Particule> particulesTemp);
    void reparateurReturnDock();
    void neutraliseurPanne (int index);
    void destroy_neutraliseur(int i);
};
#endif
