#ifndef SIMULATIONH
#define SIMULATIONH

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "robot.h"
#include "particule.h"
#include "shape.h"
#include "message.h"
#include <random>
#include <iostream>
#include <list>
#include <algorithm>

using namespace std;

class Simulation
{
private:
    RobotSpatial robotSpatial{999, 999, 0, 0, 0, 0, 0, 0};
    std::vector<Particule> particules;

    void readNbParticles(std::istringstream &ss, int &nbP, int &mode);
    void readParticle(std::istringstream &ss, int &nbP, int &mode);
    void readRobotSpatial(std::istringstream &ss, int &mode);
    void readReparateur(std::istringstream &ss, int &mode);
    void readNeutralisateur(std::istringstream &ss, int &mode);
    bool erreur = false;
    static default_random_engine e;

public:
    // Getter methods for RobotSpatial properties
    int getNbUpdates() const;
    int getNbRs() const;
    int getNbRr() const;
    int getNbNs() const;
    int getNbNp() const;
    int getNbNd() const;
    int getNbNr() const;

    // Getter method for Particules size
    size_t getParticulesSize() const;

    void check_errors();
    bool check_domain(vector<Particule> P, RobotSpatial RS,
                      vector<RobotNeutralisateur> RN,
                      vector<RobotReparateur> RR);

    bool check_superpositions(vector<Particule> P, RobotSpatial RS,
                              vector<RobotNeutralisateur> RN,
                              vector<RobotReparateur> RR);

    void readData(const std::string &filename);

    bool getErreur() const;
    void setErreur(bool erreurreset);

    const RobotSpatial &getRobotSpatial();
    const std::vector<Particule> &getParticules() const;

    void draw_planet();
    void clean_ancien_sim();
    void sim_nul();

    void setParticule(vector<Particule> particulenul);
    void setRobotSpatial(RobotSpatial robotSpatialnul);

    void sauvegarde(string nom_fichier);
    void setRSnbupdate(int nbUpdate_in);

    vector<Particule> divide_particle_into_four(unsigned int index);
    void disintegrateParticles();
    void destroy_particle(int i);
    void danger_zone(Particule particule);
    void update();
    void trier_particules();
    static bool compareParticuleSize(Particule p1, Particule p2);
    int nbPanne(vector<RobotNeutralisateur> neutralisateursTemp);
    bool verifierPanne(vector<RobotNeutralisateur> neutralisateursTemp);
    void destroyLongPanne(vector<RobotNeutralisateur> neutralisateursTemp);
    bool end_simulation();
};

#endif
