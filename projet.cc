#include <string>
#include "simulation.h"
#include "gui.h"
#include <gtkmm/application.h>
#include <gtkmm/window.h>
#include <iostream>

using namespace std;


int main(int argc, char *argv[]) {


  
  // Lancement de la simulation
  
  Simulation sim;
  if(argc == 2)
  sim.readData(argv[1]);
  auto app = Gtk::Application::create();
  return app->make_window_and_run<MyWindow>(1, argv, sim);
 
}
