#include "simulation.h"

default_random_engine Simulation::e;

// Getter methods for RobotSpatial properties
int Simulation::getNbUpdates() const { return robotSpatial.getNbUpdate(); }
int Simulation::getNbRs() const { return robotSpatial.getnbRs_initial(); }
int Simulation::getNbRr() const { return robotSpatial.getNbRr(); }
int Simulation::getNbNs() const { return robotSpatial.getnbNs_initial(); }
int Simulation::getNbNp() const { return robotSpatial.getNbNp(); }
int Simulation::getNbNd() const { return robotSpatial.getNbNd(); }
int Simulation::getNbNr() const { return robotSpatial.getNbNr(); }

// Getter method for Particules size
size_t Simulation::getParticulesSize() const { return particules.size(); }

void Simulation::check_errors()
{
    // Parametre temporaire
    vector<Particule> p = particules;
    RobotSpatial robotspatial = getRobotSpatial();
    vector<RobotNeutralisateur> robotsneutralisateurs =
        robotspatial.getNeutralisateurs();
    vector<RobotReparateur> robotsreparateurs = robotspatial.getReparateurs();

    // Gerer tout les differents cas de superposition
    if (check_superpositions(p, robotspatial, robotsneutralisateurs,
                             robotsreparateurs) or
        check_domain(p, robotspatial, robotsneutralisateurs,
                     robotsreparateurs))
    {
        cout << "Erreur" << endl;
    }
    else
    {
        // Pas d'erreurs
        cout << message::success();
    }
}

bool Simulation::check_domain(vector<Particule> P, RobotSpatial RS,
                              vector<RobotNeutralisateur> RN, vector<RobotReparateur> RR)
{

    erreur = RS.check_inside_domain(RS);
    if (erreur)
        return true;

    // Le coté d’une particule doit être supérieur ou égal à d_particule_min
    for (unsigned int i = 0; i < P.size(); ++i)
    {
        erreur = P[i].check_d_min();
        if (erreur)
            return true;
    }
    // Le robot spatial et les particules doivent être entièrement
    // à l’intérieur du domaine [-dmax, dmax]
    for (unsigned int i = 0; i < P.size(); ++i)
    {
        erreur = P[i].check_inside_domain();
        if (erreur)
            return true;
    }
    return false;
}

bool Simulation::check_superpositions(vector<Particule> P, RobotSpatial RS,
                        vector<RobotNeutralisateur> RN, vector<RobotReparateur> RR){   
    if (P.size() > 0){
        for (unsigned int i = 0; i < P.size() - 1; ++i){
            for (unsigned int j = i + 1; j < P.size(); ++j){
                erreur = P[i].check_superpos(P[j]);
                if (erreur)
                    return true;
            }
        }
    }
    if ((RN.size() > 0) and (RR.size() > 0)){
        for (unsigned int i = 0; i < RN.size(); ++i){
            for (unsigned int j = 0; j < RR.size(); ++j){
                erreur = RS.check_superpos_R_N(RR[j], RN[i]);
                if (erreur)
                    return true;
            }
        }
    }
    if (RN.size() > 0){
        for (unsigned int i = 0; i < RN.size() - 1; ++i){
            for (unsigned int j = i + 1; j < RN.size(); ++j){
                erreur = RS.check_superpos_N_N(RN[i], RN[j]);
                if (erreur)
                    return true;
            }
        }
    }
    if (RR.size() > 0){
        for (unsigned int i = 0; i < RR.size() - 1; ++i){
            for (unsigned int j = i + 1; j < RR.size(); ++j){
                erreur = RS.check_superpos_R_R(RR[i], RR[j]);
                if (erreur)
                    return true;
            }
        }
    }
    if (RN.size() > 0){
        for (unsigned int i = 0; i < RN.size(); ++i){
            erreur = RS.check_nbUpdate(RN[i], RN[i].getK_update_panne());
            if (erreur)
                return true;
        }
    }
    if ((P.size() > 0)){
        for (unsigned int i = 0; i < P.size(); ++i){
            erreur = RS.check_superpos_R_P(RS, P[i]);
            if (erreur)
                return true;
        }
    }
    if ((RN.size() > 0) and (P.size() > 0)){
        for (unsigned int i = 0; i < P.size(); ++i){
            for (unsigned int j = 0; j < RN.size(); ++j){
                erreur = RS.check_superpos_R_P(RN[j], P[i]);
                if (erreur)
                    return true;
            }
        }
    }
    if ((RR.size() > 0) and (P.size() > 0)){
        for (unsigned int i = 0; i < P.size(); ++i){
            for (unsigned int j = 0; j < RR.size(); ++j){
                erreur = RS.check_superpos_R_P(RR[j], P[i]);
                if (erreur)
                    return true;
            }
        }
    }
    return false;
}

void Simulation::readNbParticles(std::istringstream &ss, int &nbP, int &mode)
{
    ss >> nbP;
    mode = 1;
    if (nbP == 0)
    {
        mode = 2;
    }
}

void Simulation::readParticle(std::istringstream &ss, int &nbP, int &mode)
{
    double x, y, d;
    ss >> x >> y >> d;
    particules.push_back(Particule(x, y, d));
    if (--nbP == 0)
        mode = 2;
}

void Simulation::readRobotSpatial(std::istringstream &ss, int &mode)
{
    double x, y;
    int nbUpdate, nbNr, nbNs, nbNd, nbRr, nbRs;
    ss >> x >> y >> nbUpdate >> nbNr >> nbNs >> nbNd >> nbRr >> nbRs;
    robotSpatial = RobotSpatial(x, y, nbUpdate, nbNr, nbNs, nbNd, nbRr, nbRs);

    if (nbRs == 0)
    {
        mode = 4;
    }
    else
    {
        mode = 3;
    }
}

void Simulation::readReparateur(std::istringstream &ss, int &mode)
{
    double x, y;
    ss >> x >> y;
    robotSpatial.addReparateur(RobotReparateur(x, y));
    if (robotSpatial.getNbRs() == 0)
    {
        mode = 4;
    }
}

void Simulation::readNeutralisateur(std::istringstream &ss, int &mode)
{
    double x, y, a, c_n;
    bool panne;
    double k_update_panne;
    std::string panne_str;
    ss >> x >> y >> a >> c_n >> panne_str;
    panne = panne_str == "true" ? true : false;
    ss >> k_update_panne;
    robotSpatial.addNeutralisateur(RobotNeutralisateur(x, y, a, c_n,
                                                       panne, k_update_panne));
    if (robotSpatial.getNbNs() == 0)
        mode = 5;
}

void Simulation::readData(const std::string &filename)
{
    std::ifstream file(filename);
    std::string line;
    if (!file.fail())
    {
        int mode = 0;
        int nbP = 0;
        while (getline(file >> ws, line))
        {
            if (line[0] == '#')
                continue;
            std::istringstream ss(line);
            switch (mode)
            {
            case 0:
                readNbParticles(ss, nbP, mode);
                break;
            case 1:
                readParticle(ss, nbP, mode);
                break;
            case 2:
                readRobotSpatial(ss, mode);
                break;
            case 3:
                readReparateur(ss, mode);
                break;
            case 4:
                readNeutralisateur(ss, mode);
                break;
            }

            if (mode == 5)
            {
                break;
            }
        }
        file.close();
    }
    check_errors();
    e.seed(1);
    update();
}

bool Simulation::getErreur() const { return erreur; }
void Simulation::setErreur(bool erreurreset) { erreur = erreurreset; }

const RobotSpatial &Simulation::getRobotSpatial() { return robotSpatial; }
const vector<Particule> &Simulation::getParticules() const { return particules; }

void Simulation::draw_planet()
{

    // Temporary Parameters
    vector<Particule> p = particules;
    RobotSpatial robot_spatial = getRobotSpatial();
    vector<RobotNeutralisateur> vector_robots_neutralisateurs =
        robot_spatial.getNeutralisateurs();
    vector<RobotReparateur> vector_robots_reparateurs = robot_spatial.getReparateurs();
    if (!(particules.empty()))
    {
        // Draw particles
        for (size_t i(0); i < p.size(); i++)
        {
            p[i].draw_particle();
        }
    }
    // Draw spacial Robot
    if((robot_spatial.getX() != 999) and (robot_spatial.getY() != 999)){
    robot_spatial.draw_robot_spatial();
    }
    if (!(robotSpatial.getNeutralisateurs().empty()))
    {

        // Draw the neutralizer robots
        for (const RobotNeutralisateur &robot_neutralisateur :
             vector_robots_neutralisateurs)
        {
            // Determine the state of the neutralizer robot
            Etat_neutraliseur neutralizer_state;
            bool collision(false);
            if (robot_neutralisateur.getPanne())
            {
                neutralizer_state = EN_PANNE;
            }
            else
            {
                neutralizer_state = EN_MARCHE;
                for (const Particule &particule : p)
                {
                    if (collision_carre_cercle(particule.getSquare(),
                                               robot_neutralisateur.getCircle(), true))
                    {
                        collision = true;
                        break;
                    }
                }
                for (const RobotNeutralisateur &other_neutralisateur :
                     vector_robots_neutralisateurs)
                {
                    if (&robot_neutralisateur != &other_neutralisateur &&
                        collision_cercles(robot_neutralisateur.getCircle(),
                                          other_neutralisateur.getCircle(), true))
                    {
                        collision = true;
                        break;
                    }
                }
            }
            if (collision)
            {
                robot_neutralisateur.draw_robot_neutralizer_collistion();
            }
            else
                switch (neutralizer_state)
                {

                case EN_MARCHE:
                    robot_neutralisateur.draw_robot_neutralizer_service();
                    break;

                case EN_PANNE:
                    robot_neutralisateur.draw_robot_neutralizer_panne();
                    break;
                }
        }
    }

    if (!(robotSpatial.getReparateurs().empty()))
    {

        // Draw the repairer robots
        for (const RobotReparateur &robot_reparateur : vector_robots_reparateurs)
        {
            robot_reparateur.draw_robot_repairer();
        }
    }
}

void Simulation::clean_ancien_sim()
{

    particules.clear();
    robotSpatial.clear_robot();
}

void Simulation::sim_nul()
{
    vector<Particule> p;
    particules = p;
    setParticule(particules);
}

void Simulation::setParticule(vector<Particule> particulenul)
{
    particules = particulenul;
}

void Simulation::setRobotSpatial(RobotSpatial robotSpatialnul)
{
    robotSpatial = robotSpatialnul;
}

void Simulation::sauvegarde(string nom_fichier)
{
    ofstream file(nom_fichier);
    file << "# Nom du scenario de test"
         << "\n"
         << "#"
         << "\n"
         << "# nombre de particules particules puis les donnees \
                 d'une particule ligne"
         << "\n";
    file << to_string(particules.size()) << "\n";
    for (unsigned int i = 0; i < particules.size(); i++)
    {
        file << "         " << to_string(particules[i].getX()) << " "
             << to_string(particules[i].getY()) << " "
             << to_string(particules[i].getD()) << "\n";
    }
    file << "\n"
         << "# donnees des nbRs robots reparateurs en service (un par ligne) "
         << "\n"
         << to_string(robotSpatial.getX()) << " " << to_string(robotSpatial.getY())
         << " " << to_string(robotSpatial.getNbUpdate()) << " "
         << to_string(robotSpatial.getNbNr()) << " " << 
         to_string(robotSpatial.getnbNs_initial()) << " "
         << to_string(robotSpatial.getNbNd()) << " "
         << to_string(robotSpatial.getNbRr())
         << " " << to_string(robotSpatial.getnbRs_initial()) << "\n"
         << "# donnees des nbRs robots reparateurs en service (un par ligne)"
         << "\n";
    vector<RobotReparateur> RR = robotSpatial.getReparateurs();
    for (unsigned int i = 0; i < RR.size(); i++)
    {
        file << "         " << to_string(RR[i].getX()) << " "
             << to_string(RR[i].getY()) << "\n";
    }
    vector<RobotNeutralisateur> RN = robotSpatial.getNeutralisateurs();
    file << "# donnees des nbRs robots reparateurs en service (un par ligne)"
         << "\n";
    for (unsigned int i = 0; i < RN.size(); i++)
    {
        file << "         " << to_string(RN[i].getX()) << " "
             << to_string(RN[i].getY()) << " " << to_string(RN[i].getA()) << " "
             << to_string(RN[i].getC_n()) << " ";
        if (RN[i].getPanne() == true)
        {
            file << "true";
        }
        else
        {
            file << "false";
        }
        file << " " << RN[i].getK_update_panne() << "\n";
    }
}

void Simulation::setRSnbupdate(int nbUpdate_in)
{
    robotSpatial.setNbUpdate(nbUpdate_in);
}

vector<Particule> Simulation::divide_particle_into_four(unsigned int index)
{
    vector<Particule> new_particles;
    double x = particules[index].getX();
    double y = particules[index].getY();
    double d = particules[index].getD();
    double new_d = d / 2 - 2 * epsil_zero;

    // Add four new particles
    new_particles.push_back(Particule(x - d / 4, y - d / 4, new_d));
    new_particles.push_back(Particule(x + d / 4, y - d / 4, new_d));
    new_particles.push_back(Particule(x - d / 4, y + d / 4, new_d));
    new_particles.push_back(Particule(x + d / 4, y + d / 4, new_d));

    // Remove the original particle
    particules.erase(particules.begin() + index);

    return new_particles;
}

void Simulation::disintegrateParticles()
{
    double p(desintegration_rate);
    int nb_elem(particules.size());
    bernoulli_distribution b(p / nb_elem);
    vector<Particule> new_particles;
    for (unsigned int i = 0; i < particules.size(); ++i)
    {
        if (b(e))
        {
            // Check if the future side length is larger than
            // d_particule_min + epsil_zero
            double d = particules[i].getD();
            double new_d = d / 2 - 2 * epsil_zero;
            if (new_d >= d_particule_min + epsil_zero)
            {   
                danger_zone(particules[i]);
                vector<Particule> disintegrated_particles =
                    divide_particle_into_four(i);
                for (size_t i(0); i < disintegrated_particles.size(); i++)
                {   
                    
                    new_particles.push_back(disintegrated_particles[i]);
                }
                i--;
            }
        }
    }
    for (size_t i(0); i < new_particles.size(); i++)
    {
        particules.push_back(new_particles[i]);
    }
}

void Simulation::danger_zone(Particule particule)
{

    vector<RobotNeutralisateur> neutralisateurs = robotSpatial.getNeutralisateurs();
    for (size_t i(0); i < robotSpatial.getNbNs_initial(); ++i)
    {

        if (neutralisateurs[i].is_within_risk_zone(particule))
        {
            robotSpatial.neutraliseurPanne(i);
        }
    }
}
int Simulation::nbPanne(vector<RobotNeutralisateur> neutralisateursTemp)
{

    int nb_robots_panne = 0;
    for (size_t i = 0; i < neutralisateursTemp.size(); ++i)
    {
        if (neutralisateursTemp[i].getPanne())
            ++nb_robots_panne;
    }
    return nb_robots_panne;
}

void Simulation::destroyLongPanne(vector<RobotNeutralisateur> neutralisateursTemp)
{

    for (size_t i = 0; i < neutralisateursTemp.size(); ++i)
    {
        if (neutralisateursTemp[i].getPanne())
            if (robotSpatial.getNbUpdate() -
                    neutralisateursTemp[i].getK_update_panne() >=
                max_update)
            {
                robotSpatial.destroy_neutraliseur(i);
            }
    }
}
bool Simulation::verifierPanne(vector<RobotNeutralisateur> neutralisateursTemp)
{

    bool panneExiste = false;
    for (size_t i = 0; i < neutralisateursTemp.size(); ++i)
    {
        if (neutralisateursTemp[i].getPanne())
            panneExiste = true;
    }
    return panneExiste;
}

void Simulation::destroy_particle(int i)
{
    particules.erase(particules.begin() + i);
}

void Simulation::update()
{
    vector<RobotNeutralisateur> RN = robotSpatial.getNeutralisateurs();
    if (getParticulesSize() > 0)
    {
        disintegrateParticles();
        trier_particules();
        if (verifierPanne(robotSpatial.getNeutralisateurs()) and
            (robotSpatial.getNbRs_initial() < 
            nbPanne(robotSpatial.getNeutralisateurs()))
             and (robotSpatial.getNbRr() > 0))
        {
            robotSpatial.spawnReparateur();
        }
        else{robotSpatial.spawnNeutralisateur(particules);}
        robotSpatial.manage_movements(particules);
        for (int i = particules.size() - 1; i >= 0; --i){
            for (unsigned int j = 0; j < RN.size(); ++j){
                double desiredangle = RN[j].desiredAngle0(particules[i]);
                if (collision_carre_cercle(particules[i].getSquare(),
                                           RN[j].getCircle(), true) and
                    (abs(desiredangle - RN[j].getA()) < epsil_alignement))
                {
                    destroy_particle(i);
                    trier_particules();
                    break;
                }
            }
        }
    }
    if (getParticulesSize() == 0){
        robotSpatial.neutralisateurReturnDock(particules);
    }
    if (!(verifierPanne(RN))){robotSpatial.reparateurReturnDock();}
    if (verifierPanne(RN)){robotSpatial.spawnReparateur();}
    destroyLongPanne(RN);
    
    if (!(robotSpatial.getReparateurs().empty())){robotSpatial.manage_reparateurs();}
}

/**
 * Updates the simulation.
 *
 * @return void
 */


bool Simulation::compareParticuleSize(Particule p1, Particule p2)
{
    return p1.getD() > p2.getD();
}

void Simulation::trier_particules()
{
    sort(particules.begin(), particules.end(), compareParticuleSize);
}

bool Simulation::end_simulation(){
    return((getParticulesSize()==0) and (robotSpatial.getNbNs_initial() == 0) );
}