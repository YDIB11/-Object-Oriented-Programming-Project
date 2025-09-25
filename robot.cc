#include "robot.h"

bool RobotSpatial::check_inside_domain(RobotSpatial RS)
{
    if ((abs(RS.getX()) + getR() > dmax) or (abs(RS.getY()) + getR() > dmax))
    {
        cout << message::spatial_robot_ouside(RS.getX(), RS.getY());
        return true;
    }
    return false;
}
bool RobotSpatial::check_superpos_N_N(RobotNeutralisateur n1, RobotNeutralisateur n2)
{
    if (collision_cercles(n1.getCircle(), n2.getCircle(), false))
    {
        cout << message::neutralizers_superposition(n1.getX(), n1.getY(),
             n2.getX(), n2.getY());
        return true;
    }
    return false;
}
bool RobotSpatial::check_superpos_R_R(RobotReparateur r1, RobotReparateur r2)
{
    if (collision_cercles(r1.getCircle(), r2.getCircle(), false))
    {
        cout << message::repairers_superposition(r1.getX(), r1.getY(),
             r2.getX(), r2.getY());
        return true;
    }
    return false;
}
bool RobotSpatial::check_superpos_R_N(RobotReparateur r1, RobotNeutralisateur n1)
{
    if (collision_cercles(r1.getCircle(), n1.getCircle(), false))
    {
        cout << message::repairer_neutralizer_superposition(r1.getX(), r1.getY(),
                     n1.getX(), n1.getY());
        return true;
    }
    return false;
}
bool RobotSpatial::check_superpos_R_P(Robot R, Particule p)
{
    if (collision_carre_cercle(p.getSquare(), R.getCircle(), false))
    {
        cout << message::particle_robot_superposition(p.getX(), p.getY(), p.getD(),
             R.getX(), R.getY(), R.getR());
        return true;
    }
    return false;
}
bool RobotSpatial::check_nbUpdate(RobotNeutralisateur n, int k_update_panne_in)
{
    if (k_update_panne_in > getNbUpdate())
    {
        cout << message::invalid_k_update(n.getX(), n.getY(), k_update_panne_in, 
            getNbUpdate());
        return true;
    }
    return false;
}
int RobotSpatial::getNbNp() const
{
    int NbNp(0);
    vector<RobotNeutralisateur> robotsneutralisateurs = getNeutralisateurs();
    for (int i(0); i < getnbRs_initial(); i++)
    {
        if (robotsneutralisateurs[i].getPanne())
        {
            NbNp += 1;
        }
    }
    return NbNp;
}

void RobotReparateur::draw_robot_repairer() const
{
    draw_robot_repairer_(getCircle());
}
void RobotNeutralisateur::draw_robot_neutralizer_panne() const
{
    draw_robot_neutralizer_panne_(getCircle(), getA());
}
void RobotNeutralisateur::draw_robot_neutralizer_service() const
{
    draw_robot_neutralizer_service_(getCircle(), getA());
}
void RobotNeutralisateur::draw_robot_neutralizer_collistion() const
{
    draw_robot_neutralizer_collistion_(getCircle(), getA());
}
void RobotSpatial::draw_robot_spatial()
{
    draw_robot_spatial_(getCircle());
}

void RobotSpatial::clear_robot()
{
    reparateurs.clear();
    neutralisateurs.clear();
};

bool RobotNeutralisateur::isColliding(const vector<RobotNeutralisateur> &robots)
{
    for (const auto &robot : robots)
    {
        // Skip checking collision with self
        if (this == &robot)
            continue;

        if (collision_cercles(this->getCircle(), robot.getCircle(), true))
        {
            return true;
        }
    }
    return false;
}


void RobotSpatial::destroy_neutraliseur(int i){
	neutralisateurs.erase(neutralisateurs.begin() + i);
	++nbNd;
	--nbNs;
	
}

/*
 * Rotates the robot to the desired angle using a PID controller.
 *
 * @param desiredAngle the desired angle in radians
 *
 * @return void
 *
 * @throws None
 */
void RobotNeutralisateur::rotateToAngle(double desiredAngle)
{
    double dt = delta_t;
    double da;
    // Calculate angle difference using atan2
    double angleDifference = 
                    atan2(sin(desiredAngle - getA()), cos(desiredAngle - getA()));

    // Calculate the rotation speed
    double rotationSpeed = angleDifference > 0 ? vrot_max : -vrot_max;

    da = rotationSpeed * dt;

    if (abs(angleDifference - da) <= epsil_alignement)
    {
        a = desiredAngle;
    }
    else
    {
        a += da;
    }

    // Ensure the angle stays within -pi to pi
    if (a > M_PI)
    {
        a -= 2 * M_PI;
    }
    else if (a < -M_PI)
    {
        a += 2 * M_PI;
    }
}

/*
 * Moves the RobotNeutralisateur forwards until collision is detected.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void RobotNeutralisateur::moveForwards()
{
    double dt = delta_t;
    // Move the robot until collision

    circle.centre.x += cos(a) * vtran_max * dt;
    circle.centre.y += sin(a) * vtran_max * dt;
}

/*
 * Moves the robot towards a target point.
 *
 * @param target the target point to move towards
 *
 * @return void
 *
 * @throws None
 */
void RobotNeutralisateur::moveTowardsTarget(const S2d &target)
{
    // Calculate the distance to the target
    double distance = sqrt(pow(target.x - getX(), 2) + pow(target.y - getY(), 2));

    // Define a threshold distance for slowing down
    // (for instance, 2 times the maximum possible translation)
    double slowDownDistance = 2 * vtran_max * delta_t;

    // Adjust the speed according to the distance to the target
    double speed;
    if (distance < slowDownDistance)
    {
        // If we're within the slow-down distance, linearly interpolate the speed 
        //between 0 (at the target location) and vtran_max (at the slow-down distance)
        speed = (distance / slowDownDistance) * vtran_max;
    }
    else
    {
        // If we're farther than the slow-down distance, move at maximum speed
        speed = vtran_max;
    }

    // Apply the movement
    double dx = cos(getA()) * speed * delta_t;
    double dy = sin(getA()) * speed * delta_t;
    circle.centre.x += dx;
    circle.centre.y += dy;
}



double RobotNeutralisateur::getDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

int RobotNeutralisateur::getSide(int corner, double x, double y, double left, 
double right, double top, double bottom)
{
    switch (corner)
    {
    case 0: // Top Left
        return (y <= top) ? 0 : ((x >= left) ? 3 : 4);
    case 1: // Top Right
        return (y <= top) ? 2 : ((x <= right) ? 3 : 4);
    case 2: // Bottom Left
        return (y >= bottom) ? 0 : ((x >= left) ? 1 : 4);
    case 3: // Bottom Right
        return (y >= bottom) ? 2 : ((x <= right) ? 1 : 4);
    default:
        return -1; // This should never happen
    }
}

/*
 * Calculates the collision side of a particule with respect to a point in a 2D space.
 *
 * @param target the particule whose collision side will be calculated
 * @param x the x coordinate of the point in the space
 * @param y the y coordinate of the point in the space
 *
 * @return an integer indicating the side 
 * of the particule that collided with the point
 *
 * @throws None
 */
int RobotNeutralisateur::getCollisionSide(const Particule &target, double x, double y)
{
    double left = target.getX() - target.getD() / 2;
    double right = target.getX() + target.getD() / 2;
    double top = target.getY() + target.getD() / 2;
    double bottom = target.getY() - target.getD() / 2;

    // Calculate distances to the four corners
    double distToTopLeft = getDistance(x, y, left, top);
    double distToTopRight = getDistance(x, y, right, top);
    double distToBottomLeft = getDistance(x, y, left, bottom);
    double distToBottomRight = getDistance(x, y, right, bottom);

    // Initialize corner and minDist to top left and distToTopLeft
    int corner = 0; // Top Left
    double minDist = distToTopLeft;

    // Check top right
    if (distToTopRight < minDist)
    {
        corner = 1; // Top Right
        minDist = distToTopRight;
    }

    // Check bottom left
    if (distToBottomLeft < minDist)
    {
        corner = 2; // Bottom Left
        minDist = distToBottomLeft;
    }

    // Check bottom right
    if (distToBottomRight < minDist)
    {
        corner = 3; // Bottom Right
        minDist = distToBottomRight;
    }

    // Return side based on closest corner
    return getSide(corner, x, y, left, right, top, bottom);
}

/*
 * Calculates the desired angle of the robot's neutralizer 
 arm to target the given particle.
 *
 * @param target the particle to target
 *
 * @return the desired angle in radians
 *
 * @throws None
 */
double RobotNeutralisateur::desiredAngle0(const Particule &target)
{
    // Determine which side of the particle the robot is closest to
    int side = getCollisionSide(target, circle.centre.x, circle.centre.y);

    // Calculate the desired angle based on the side of collision
    double desiredAngle;

    switch (side)
    {
    case 0: // Left
        desiredAngle = 0;
        break;
    case 1: // Bottom
        desiredAngle = M_PI / 2;
        break;
    case 2: // Right
        desiredAngle = M_PI;
        break;
    case 3: // Top
        desiredAngle = -M_PI / 2;
        break;
    case 4: // Top-Left
        desiredAngle = atan2(target.getY() - getY(), target.getX() - getX());
        break;
    default:
        // This case should not happen as side will always be between 0 and 7
        // However, in case an error occurs, we default to 0 
        //to prevent any undefined behavior
        desiredAngle = 0;
        break;
    }

    return desiredAngle;
}

/*
 * Moves the robot to a type 0 particle.
 * 
 * @param target The target type 0 particle.
 * @return void
 */
void RobotNeutralisateur::move_to_type0(const Particule& target) {
    // Calculate the angle between the robot and the particle's center
    double angleToTarget = atan2(target.getY() - getY(), target.getX() - getX());

    if ((abs(angleToTarget - getA()) >= epsil_alignement) and
                 (!collision_carre_cercle(target.getSquare(), getCircle(), true)))
    {
        // Rotate to face the target
        rotateToAngle(angleToTarget);
    }
    else if (!collision_carre_cercle(target.getSquare(), getCircle(), true))
    {
        // move_to_type0 towards the target until collision
        moveTowardsTarget(target.get_position());
    }
    else if (collision_carre_cercle(target.getSquare(), getCircle(), true))
    {
        // Align perpendicularly with the particle's side
        rotateToAngle(desiredAngle0(target));
    }
}


double RobotNeutralisateur::calculateTimeToTarget(const Particule &target)
{
    // Calculate the angle between the robot and the particle's center
    double desiredAngle = desiredAngle0(target);

    // Distance between the robot and the target
    double distanceToTarget = sqrt(pow(target.getX() - getX(), 2) 
                                            + pow(target.getY() - getY(), 2));

    // Calculate the time for each state

    double timeToMoveToTarget = distanceToTarget / vtran_max;
    double timeToAlignWithTarget = abs(atan2(sin(desiredAngle - getA()),
                                         cos(desiredAngle - getA()))) / vrot_max;

    // The total time is the sum of the times for each state
    return timeToMoveToTarget + timeToAlignWithTarget;
}

int RobotSpatial::findNearestUnassignedRobot(const Particule &p,
                                         const vector<bool> &robotAssigned)
{
    double minTime = 1e30; // Large value for initialization
    vector<RobotNeutralisateur> NeutraliseurTemp = neutralisateurs;
    int index = -1;
    for (int i = 0; i < getNbNs_initial(); ++i)
    {
        if (robotAssigned[i])
        {
            continue;
        }
        if (NeutraliseurTemp[i].getPanne()) {
            continue;
        }
        if (NeutraliseurTemp[i].isColliding(NeutraliseurTemp))
        {
            continue;
        }

        double delta_t = NeutraliseurTemp[i].calculateTimeToTarget(p);
        if (delta_t < minTime && !NeutraliseurTemp[i].isColliding(NeutraliseurTemp))
        {
            minTime = delta_t;
            index = i;
        }
    }
    return index;
}

/*
 * Finds the index of the colliding particle, if any.
 *
 * @param robot The RobotNeutralisateur instance to check for collision.
 * @param particles The vector of Particule instances to check for collision.
 * @return The index of the colliding particle, or -1 if no collision.
 */

int RobotSpatial::findCollidingParticle(const RobotNeutralisateur &robot,
                                             const std::vector<Particule> &particles)
{
    for (size_t i = 0; i < particles.size(); ++i)
    {
        if (collision_carre_cercle(particles[i].getSquare(), robot.getCircle(), true))
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}


/*
 * Manage movements of robots and particles.
 *
 * @param vector_particules vector of particules
 *
 * @return void
 *
 * @throws None
 */
void RobotSpatial::manage_movements(vector<Particule> &vector_particules)
{
    std::vector<bool> robotAssigned(getNbNs_initial(), false);
    std::vector<bool> particleAssigned(vector_particules.size(), false);
    bool allAssigned = false;
    while (!allAssigned)
    {
        allAssigned = true; // Assume all are assigned until proven otherwise

        for (size_t i = 0; i < vector_particules.size(); i++)
        {
            if (particleAssigned[i])
            {
                continue; // Skip this particle if it's already assigned to a robot
            }
            int index = findNearestUnassignedRobot(vector_particules[i], 
                                                                    robotAssigned);
            if (index != -1)
            {
                int collidingParticuleIndex = 
                    findCollidingParticle(neutralisateurs[index], vector_particules);
                if (collidingParticuleIndex != -1)
                {
                    // If colliding, update target to the colliding particle
                    neutralisateurs[index].move_to_type0(vector_particules
                                                        [collidingParticuleIndex]);
                    particleAssigned[collidingParticuleIndex] = true; 
                    // Mark this particle as assigned
                }
                else{
                    // If not colliding, move towards current target as usual
                    neutralisateurs[index].move_to_type0(vector_particules[i]);
                    particleAssigned[i] = true; // Mark this particle as assigned
                }
                robotAssigned[index] = true;
                allAssigned = false; // We assigned a robot, so not all were assigned
            }
        }
    }
}

/*
 * Chooses a repair robot that has not been assigned yet and is closest to
 * the target robot without colliding with it.
 *
 * @param robot the target robot
 * @param reparateurAssigned a boolean vector indicating which repair robots
 * have already been assigned
 *
 * @return the index of the chosen repair robot
 *
 * @throws none
 */
int RobotSpatial::chooseReparateur(const RobotNeutralisateur &robot,
                                 const vector<bool> &reparateurAssigned)
{
    double minTime = 1e30;
    vector<RobotReparateur> ReparateurTemp = reparateurs;
    int index = -1;
    for (int i = 0; i < reparateurs.size(); ++i)
    {
        if (reparateurAssigned[i])
        {
            continue;
        }

        while (!collision_cercles(robot.getCircle(), 
                                                ReparateurTemp[i].getCircle(), true))
        {
            ReparateurTemp[i].move_reparateur(robot);
        }
        double timetotarget = ReparateurTemp[i].getReachTime();
        if (timetotarget < minTime)
        {
            minTime = timetotarget;
            index = i;
        }
    }
    return index;
}

/*
 * Move the RobotReparateur towards the RobotNeutralisateur target until collision.
 *
 * @param target The RobotNeutralisateur towards which the RobotReparateur should move.
 *
 * @throws None
 */
void RobotReparateur::move_reparateur(const RobotNeutralisateur &target)
{

    double dt = delta_t;
    // Move the robot until collision
    double goal_a = atan2(target.getY() - getY(), target.getX() - getX());

    circle.centre.x += cos(goal_a) * vtran_max * dt;
    circle.centre.y += sin(goal_a) * vtran_max * dt;

    reachtime += dt;
}

/*
 * Finds the index of the colliding particle, if any.
 *
 * @param robot The RobotNeutralisateur instance to check for collision.
 * @param particles The vector of Particule instances to check for collision.
 * @return The index of the colliding particle, or -1 if no collision.
 */

bool RobotReparateur::repCheckCollision(const vector<RobotReparateur> &reparateurs,
                                const vector<RobotNeutralisateur> &neutralisateurs)
{
    for (const auto &robot : reparateurs)
    {
        // Skip checking collision with self
        if (this == &robot)
            continue;

        if (collision_cercles(this->getCircle(), robot.getCircle(), true))
        {
            return true;
        }
    }

    for (const auto &robot : neutralisateurs)
    {
        // Skip checking collision with self

        if (collision_cercles(this->getCircle(), robot.getCircle(), true))
        {
            return true;
        }
    }
    return false;
}

/*
 * Spawn a RobotReparateur if the current update count is divisible by modulo_update
 * and there are still available repair robots. The robot is spawned at the location
 * of this RobotSpatial instance, provided that it does not collide with any existing
 * RobotReparateurs or neutralizers. 
 *
 * @return void
 *
 * @throws None
 */
void RobotSpatial::spawnReparateur()
{
    if (nbUpdate % modulo_update == 0 && nbRr > 0)
    {

        RobotReparateur reparateurTemp(getX(), getY());
        if (nbUpdate != 0 && !(reparateurTemp.repCheckCollision(reparateurs,
                                                                 neutralisateurs)))
        {
            reparateurs.push_back(reparateurTemp);
            ++nbRs_initial;
            --nbRr;
        }
    }
}

/*
 * Spawns a neutralizing robot with the same position as the RobotSpatial
 * and the default radius if nbUpdate is a multiple of modulo_update and there 
 * are still robots to be spawned. The robot is added to the list of 
 * neutralizing robots if it does not collide with other robots and if there 
 * are enough particles in the environment. The number of initial neutralizing 
 * robots and the number of robots to be spawned are updated accordingly.
 * 
 * @param vector_particules a vector of Particule objects representing the 
 *                          environment
 * 
 * @throws None
 */
void RobotSpatial::spawnNeutralisateur(vector<Particule> vector_particules)
{

    if (nbUpdate % modulo_update == 0 && nbNr > 0)
    {
        // Create a new neutralizing robot with the same position 
        //as the RobotSpatial and the default radius

        RobotNeutralisateur robotTemp(getX(), getY(), 0, 0, 0, 0);
        if (nbUpdate != 0 && vector_particules.size() >= nbNs_initial && 
                                !robotTemp.isColliding(neutralisateurs))
        {

            // Add the new robot to the list of neutralizing robots
            neutralisateurs.push_back(robotTemp);

            // Update the number of initial neutralizing robots
            ++nbNs_initial;
            --nbNr;
        }
    }
}

/*
 * Updates the state of the RobotSpatial object by moving neutralizer
  robots towards the dock and removing them 
 * from the list of active neutralizers once they reach the dock. 
 *
 * @param particulesTemp a vector of Particule objects representing 
 * the current state of the system
 *
 * @throws None
 */
void RobotSpatial::neutralisateurReturnDock(vector<Particule> particulesTemp)
{
    Particule particuleTemp(getX(), getY(), 0);
    vector<bool> backToDock(neutralisateurs.size(), false);
    cercle Station = {get_position(), getR() - 2};

    for (size_t i = 0; i < neutralisateurs.size(); ++i)
    {
        if (!backToDock[i])
        {

            if (neutralisateurs[i].getPanne())
            {
                continue;
            }
            if (neutralisateurs[i].isColliding(neutralisateurs))
            {
                continue;
            }
            if (!(neutralisateurs[i].getPanne()))
            {
                neutralisateurs[i].move_to_type0(particuleTemp);
            }
            if (collision_cercles(neutralisateurs[i].getCircle(), Station, true))
            {
                neutralisateurs.erase(neutralisateurs.begin() + i);
                ++nbNr;
                --nbNs_initial;
                backToDock[i] = true;
            }
        }
    }
}


/*
 * Returns reparator robot to the dock.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void RobotSpatial::reparateurReturnDock()
{   
    RobotNeutralisateur robotTemp(getX(), getY(), 0, 0, 0, 0);
    vector<bool> backToDock(reparateurs.size(), false);
    bool collision = false;
    cercle Station = {get_position(), getR() - 2};

    for (size_t i = 0; i < reparateurs.size(); ++i)
    {   
        if (!backToDock[i]){
        for (size_t j = 0; j < neutralisateurs.size(); ++j)
        {

            if (collision_cercles(reparateurs[i].getCircle(), 
                                        neutralisateurs[j].getCircle(), true))
            {
                collision = true;
            }
            if (!collision)
            {
                reparateurs[i].move_reparateur(robotTemp);
            }
            
        }
        if (collision_cercles(reparateurs[i].getCircle(), Station, false))
            {
                reparateurs.erase(reparateurs.begin() + i);
                ++nbRr;
                --nbRs_initial;
                backToDock[i] = true;
            }
        }
    }
}

/*
 * Manages the redistribution of repairers to neutralizers that need repair.
 *
 * @return void
 *
 * @throws None
 */
void RobotSpatial::manage_reparateurs()
{
    std::vector<bool> reparateurAssigned(reparateurs.size(), false);
    bool allAssigned = false;

    while (!allAssigned)
    {
        allAssigned = true;

        for (size_t i = 0; i < neutralisateurs.size(); i++)
        {
            if (!neutralisateurs[i].getPanne())
            {
                continue;
            }
            int index = chooseReparateur(neutralisateurs[i], reparateurAssigned);
            if (index != -1)
            {

                if (collision_cercles(neutralisateurs[i].getCircle(), 
                                            reparateurs[index].getCircle(), true))
                {
                    neutralisateurs[i].setPanne(false);
                }
                else if (!reparateurs[index].repCheckCollision(reparateurs,
                                                                     neutralisateurs))
                {
                    reparateurs[index].move_reparateur(neutralisateurs[i]);
                }
                reparateurAssigned[index] = true;
                allAssigned = false;
            }
        }
    }
}

/*
 * Neutralize a specific malfunctioning robot by setting its "panne" value to true.
 *
 * @param index the index of the robot to be neutralized.
 *
 * @throws None.
 */
void RobotSpatial::neutraliseurPanne(int index)
{

    neutralisateurs[index].setPanne(true);
    neutralisateurs[index].setK_update_panne(nbUpdate);
}

/*
 * Determines if the given particle is within the risk zone.
 *
 * @param p the particle to check
 *
 * @return true if the robot is within the risk zone, false otherwise
 *
 * @throws None
 */
bool RobotNeutralisateur::is_within_risk_zone(const Particule &p)
{
    // Calculate the bounds of the risk zone around the particle
    double particle_d = p.getD();
    double risk_d = particle_d * risk_factor;
    Particule p_risk = Particule(p.getX(), p.getY(), risk_d);

    // Check if the robot is within the risk zone
    if (collision_carre_cercle(p_risk.getSquare(), getCircle(), true))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* END */







































/*
// MOUVEMENT TYPE 1
S2d RobotNeutralisateur::point_outide_riskZone(const Particule &p)
{
    S2d target;

    // Calculate the bounds of the risk zone around the particle
    double demi_d = p.getD() / 2;

    double left = p.getX() - risk_factor * demi_d - getR();
    double right = p.getX() + risk_factor * demi_d + getR();
    double top = p.getY() + risk_factor * demi_d + getR();
    double bottom = p.getY() - risk_factor * demi_d - getR();

    double p_left = p.getX() - demi_d;
    double p_right = p.getX() + demi_d;
    double p_top = p.getY() + demi_d;
    double p_bottom = p.getY() - demi_d;

    // Get the current position of the robot
    S2d pos = get_position();

    // Scenario 1 - 4: Robot is in top, right, bottom, or left region
    if (pos.y > top && pos.x >= p_left && pos.x <= p_right)
    { // Top
        target.x = pos.x;
        target.y = top;
    }
    else if (pos.x > right && pos.y <= p_top && pos.y >= p_bottom)
    { // Right
        target.x = right;
        target.y = pos.y;
    }
    else if (pos.y < bottom && pos.x >= p_left && pos.x <= p_right)
    { // Bottom
        target.x = pos.x;
        target.y = bottom;
    }
    else if (pos.x < left && pos.y <= p_top && pos.y >= p_bottom)
    { // Left
        target.x = left;
        target.y = pos.y;
    }
    // Scenario 5 - 8: Robot is in top-right, bottom-right
    , bottom-left, or top-left region
    else if (pos.y > p_top && pos.x > p_right)
    { // Top-right
        if (pos.y - p_top >= pos.x - p_right)
        { // Top-right top
            target.x = p_right;
            target.y = top;
        }
        else
        { // Top-right right
            target.x = right;
            target.y = p_top;
        }
    }
    else if (pos.y < p_bottom && pos.x > p_right)
    { // Bottom-right
        if (p_bottom - pos.y >= pos.x - p_right)
        { // Bottom-right bottom
            target.x = p_right;
            target.y = bottom;
        }
        else
        { // Bottom-right right
            target.x = right;
            target.y = p_bottom;
        }
    }
    else if (pos.y < p_bottom && pos.x < p_left)
    { // Bottom-left
        if (p_bottom - pos.y >= p_left - pos.x)
        { // Bottom-left bottom
            target.x = p_left;
            target.y = bottom;
        }
        else
        { // Bottom-left left
            target.x = left;
            target.y = p_bottom;
        }
    }
    else if (pos.y > p_top && pos.x < p_left)
    { // Top-left
        if (pos.y - p_top >= p_left - pos.x)
        { // Top-left top
            target.x = p_top;
            target.y = top;
        }
        else
        { // Top-left left
            target.x = left;
            target.y = p_left;
        }
    }
    return target;
}*/

/*
S2d RobotNeutralisateur::nearest_point_outside_risk_zone(const Particule& p,
 const S2d& pt) {
    // Initialize nearest point
    S2d nearest;

    // Calculate the bounds of the risk zone around the particle
    double demi_d = p.getD()/2;
    double left = p.getX() - risk_factor*demi_d - getR();
    double right = p.getX() + risk_factor*demi_d + getR();
    double top = p.getY() + risk_factor*demi_d + getR();
    double bottom = p.getY() - risk_factor*demi_d - getR();

    // Check which quadrant of the "cross" the point is in and calculate the 
    nearest point accordingly
    if (pt.x <= p.getX() && pt.y > p.getY()) {
        // Top-left quadrant: nearest point is on the top or left side
        nearest.x = std::max(pt.x, left);
        nearest.y = std::min(pt.y, top);
    } else if (pt.x > p.getX() && pt.y >= p.getY()) {
        // Top-right quadrant: nearest point is on the top or right side
        nearest.x = std::min(pt.x, right);
        nearest.y = std::min(pt.y, top);
    } else if (pt.x <= p.getX() && pt.y < p.getY()) {
        // Bottom-left quadrant: nearest point is on the bottom or left side
        nearest.x = std::max(pt.x, left);
            }
        nearest.y = std::max(pt.y, bottom);
    } else if(pt.x > p.getX() && pt.y < p.getY()) {
        // Bottom-right quadrant: nearest point is on the bottom or right side
        nearest.x = std::min(pt.x, right);
        nearest.y = std::max(pt.y, bottom);
    }

    cout << nearest.x << " " << nearest.y << endl;
    return nearest;

}*/

/*
 * Moves the robot to a type 1 particle.
 * @param p The particle to move towards.
 * @return The time needed to reach the particle.
 */
/*
double RobotNeutralisateur::desiredAngle1(const Particule &target)
{
    // Calculate the relative position of the robot to the square's center
    double relX = getX() - target.getX();
    double relY = getY() - target.getY();

    // Determine the closest side of the square
    double angleToFace;
    if (abs(relX) > abs(relY))
    {                                        // Closer to left or right side
        angleToFace = (relX > 0) ? M_PI : 0; // face right if robot is to the
         right of the square, else face left
    }
    else
    {                                                        
        angleToFace = (relY > 0) ? -0.5 * M_PI : 0.5 * M_PI; // face up if robot 
        is above the square, else face down
    }
    return angleToFace;
}
*/
/*void RobotNeutralisateur::move_to_type1(const Particule& p) {
    // Calculate the target point outside the risk zone


    if((is_within_risk_zone(p)) or ((abs(getX()-currentTarget.x) < epsil_zero)
     and (abs(getY()-currentTarget.y) < epsil_zero))){
        releaseTarget();
        // Determine the closest side of the square
        double angleToFace = desiredAngle1(p);
        // If the robot is not aligned with the closest side, rotate to face it
        if (abs(angleToFace - getA()) >= epsil_alignement) {
            rotateToAngle(angleToFace);
            timeNeeded += delta_t;
        }
        // If the robot is aligned, move towards the square
        else {
            moveTowardsTarget();
            timeNeeded += delta_t;
        }

    }else{// If the robot hasn't reached the target point yet

        if (!hasStoredTarget) {
            currentTarget = align_with_particle(p);
            hasStoredTarget = true;
        }
        // Calculate the angle between the robot and the target
        double angleToTarget = atan2(currentTarget.y - getY(),
         currentTarget.x - getX());

        // If the robot is not aligned with the target point,rotate to face the target
        if (abs(angleToTarget - getA()) > epsil_alignement) {
            rotateToAngle(angleToTarget);

            timeNeeded += delta_t;
        }
        // If the robot has not reached the target point, move towards it
        else {
            moveTowardsTarget();
            timeNeeded += delta_t;
        }
    }
}*/
/*
void RobotNeutralisateur::move_to_type1(const Particule &p)
{
    bool born_inside = false;
    if (!hasStoredTarget)
    {
        currentTarget = point_outide_riskZone(p);
        hasStoredTarget = true;
    }
    cout << currentTarget.x << " " << currentTarget.y << endl;
    // Calculate the target point outside the risk zone
    if ((getTimeNeeded() <= delta_t) and is_within_risk_zone(p))
    {
        born_inside = true;
    }
    if ((abs(getX() - currentTarget.x) < epsil_zero) and (abs(getY() -
     currentTarget.y) < epsil_zero))
    {
        double angleToFace = desiredAngle1(p);
        std::cout << "Check 1: Reached the target position\n";
        born_inside = false;
        if (abs(angleToFace - getA()) >= epsil_alignement)
        {
            rotateToAngle(angleToFace);
            timeNeeded += delta_t;
        }
        else
        {
            moveForwards();
            timeNeeded += delta_t;
        }
    }
    else if (is_within_risk_zone(p) and !born_inside)
    {
        std::cout << "Check 2: Within risk zone\n";
        // Start new modification here
        double angleToFace = desiredAngle1(p);
        if (abs(angleToFace - getA()) >= epsil_alignement)
        {
            rotateToAngle(angleToFace);
            timeNeeded += delta_t;
            return; // Return early if the robot is not yet facing the right direction
        }
        // End modification
        releaseTarget();
        moveForwards();
        timeNeeded += delta_t;
    }
    else
    {
        std::cout << "Check 3: Moving towards target\n";

        double angleToTarget = atan2(currentTarget.y - getY(), 
        currentTarget.x - getX());
        if (abs(angleToTarget - getA()) > epsil_alignement)
        {
            rotateToAngle(angleToTarget);
            timeNeeded += delta_t;
        }
        else
        {
            moveTowardsTarget(currentTarget);
            timeNeeded += delta_t;
        }
    }
}*/

/*
int RobotNeutralisateur::getCollisionSide2(const Particule& target,
 double x, double y) {

    double left = target.getX() - target.getD() / 2;
    double right = target.getX() + target.getD() / 2;
    double top = target.getY() + target.getD() / 2;
    double bottom = target.getY() - target.getD() / 2;

    // Calculate distances to the four corners
    double distToTopLeft = sqrt(pow(x - left, 2) + pow(y - top, 2));
    double distToTopRight = sqrt(pow(x - right, 2) + pow(y - top, 2));
    double distToBottomLeft = sqrt(pow(x - left, 2) + pow(y - bottom, 2));
    double distToBottomRight = sqrt(pow(x - right, 2) + pow(y - bottom, 2));

    // Initialize corner and minDist to top left and distToTopLeft
    int corner = 0; // Top Left
    double minDist = distToTopLeft;

    // Check top right
    if (distToTopRight < minDist) {
        corner = 1; // Top Right
        minDist = distToTopRight;
    }

    // Check bottom left
    if (distToBottomLeft < minDist) {
        corner = 2; // Bottom Left
        minDist = distToBottomLeft;
    }

    // Check bottom right
    if (distToBottomRight < minDist) {
        corner = 3; // Bottom Right
        minDist = distToBottomRight;
    }


    // Return side based on closest corner
    switch (corner) {
        case 0: // Top Left
            if ( y <= top ) {
                return 0; // Left
            } else if (x >= left) {
                return 3; // Top
            }
        case 1: // Top Right
            if (y <= top){
                return 2; // Right
            } else if (x <= right){
                return 3; // Top
            }
        case 2: // Bottom Left
            if (y >= bottom) {
                return 0; // Left
            } else if (x >= left){
                return 1; // Bottom
            }
        case 3: // Bottom Right
            if (y >= bottom) {
                return 2; // Right
            }else if (x <= right) {
                return 1; // Bottom
            }
        default:
            return -1; // This should never happen
    }
}*/
/*
double RobotNeutralisateur::distanceToLineSegment(double x, double y, double x1, 
double y1, double x2, double y2)
{
    double A = x - x1;
    double B = y - y1;
    double C = x2 - x1;
    double D = y2 - y1;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = -1;
    if (len_sq != 0) // in case of 0 length line
        param = dot / len_sq;

    double xx, yy;

    if (param < 0)
    {
        xx = x1;
        yy = y1;
    }
    else if (param > 1)
    {
        xx = x2;
        yy = y2;
    }
    else
    {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }

    double dx = x - xx;
    double dy = y - yy;
    return std::sqrt(dx * dx + dy * dy);
}

double RobotNeutralisateur::getDirectionToPoint(double x1, double y1, double x2,
 double y2)
{
    return atan2(y1 - y2, x1 - x2);
}

int RobotNeutralisateur::getCollisionSide2(const Particule &target)
{
    // Compute the corners of the target box
    double d_half = target.getSquare().d / 2; // Half of the side length
    double corners[4][2] = {
        {target.getSquare().centre.x - d_half, target.getSquare().centre.y - d_half},
        {target.getSquare().centre.x + d_half, target.getSquare().centre.y - d_half},
        {target.getSquare().centre.x + d_half, target.getSquare().centre.y + d_half}, 
        {target.getSquare().centre.x - d_half, target.getSquare().centre.y + d_half}  
    };

    // Calculate the distance and angle to each side (line segment between corners)
    double minTime = std::numeric_limits<double>::max();
    int closestSide = -1;

    for (int i = 0; i < 4; i++)
    {
        double cornerX1 = corners[i][0];
        double cornerY1 = corners[i][1];
        double cornerX2 = corners[(i + 1) % 4][0];
        double cornerY2 = corners[(i + 1) % 4][1];

        // Find the distance from the robot to the
         line segment defined by the two corners
        double distance = distanceToLineSegment(getX(), getY(), cornerX1,
         cornerY1, cornerX2, cornerY2);

        // Compute the time to reach the side with the given speeds (4 for 
        translation and 0.125 for rotation)
        double timeToReachSide = distance / 4; // Here, we ignore rotation 
        time as the robot moves perpendicularly to the side

        // If this side can be reached quicker,
         update the minimum time and closest side
        if (timeToReachSide < minTime)
        {
            minTime = timeToReachSide;
            closestSide = i;
        }
    }

    return closestSide;
}
double RobotNeutralisateur::desiredAngle2(const Particule &target)
{
    // Determine which side of the particle the robot is closest to
    int side = getCollisionSide2(target ,getX(),getY());

    // Calculate the desired angle based on the side of collision
    double desiredAngle;

    switch (side)
    {
    case 0: // Top
        desiredAngle = -M_PI / 2;
        break;
    case 1: // Right
        desiredAngle = M_PI;
        break;
    case 2: // Bottom
        desiredAngle = M_PI / 2;
        break;
    case 3: // Left
        desiredAngle = 0;
        break;
    default:
        // This case should not happen as side will always be between 0 and 3
        // However, in case an error occurs, we default to 0 to prevent any 
        undefined behavior
        desiredAngle = 0;
        break;
    }

    return desiredAngle;
}

void RobotNeutralisateur::move_to_type2(const Particule &p)
{
    if (!collision_carre_cercle(p.getSquare(), getCircle(), true))
    {
        // Calculate desired angle towards the target
        double desiredAngle = desiredAngle2(p);
        cout << desiredAngle << endl;

        // Calculate angle difference
        double delta_a = desiredAngle - getA();

        // If the angle difference is more significant than pi/3, 
        just rotate towards the target
        if (abs(delta_a) > M_PI / 3)
        {
            rotateToAngle(desiredAngle);
            timeNeeded += delta_t;
            return;
        }

        // Calculate translational speed based on angle difference,
         with a cosine-based decay
        double vtran = vtran_max * cos(delta_a);
        if (vtran < 0)
            vtran = 0; // Ensure vtran is non-negative

        // Calculate travel direction
        double x = getX(), y = getY(), orient = getA();
        S2d travel_dir = {cos(orient), sin(orient)};
        S2d init_pos_to_goal = {p.getX() - x, p.getY() - y};

        // Project the initial position onto the goal direction
        double proj_goal = s2d_prod_scal(init_pos_to_goal, travel_dir);

        // Ensure the robot moves towards the goal, not away from it
        if (proj_goal < 0)
            proj_goal = 0;
        if (proj_goal > vtran * delta_t)
            proj_goal = vtran * delta_t;

        // Move the robot
        s2d_add_scaled_vector(circle.centre, travel_dir, proj_goal);

        // Rotate the robot to the desired angle
        rotateToAngle(desiredAngle2(p));
        if (a < -M_PI || a >= M_PI)
        {
            a -= round(a / (2 * M_PI)) * 2 * M_PI;
        }

        timeNeeded += delta_t;
    }
    else
    {
        rotateToAngle(desiredAngle2(p));
        timeNeeded += delta_t;
    }
}

double RobotNeutralisateur::s2d_prod_scal(S2d v1, S2d v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

void RobotNeutralisateur::s2d_add_scaled_vector(S2d &pos, 
const S2d &pos_to_goal, double scaling)
{
    circle.centre.x += scaling * pos_to_goal.x;
    circle.centre.y += scaling * pos_to_goal.y;
}
*/