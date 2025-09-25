#ifndef GTKMM_EXAMPLE_Gui_H
#define GTKMM_EXAMPLE_Gui_H

#include <gtkmm.h>
#include <gtkmm/drawingarea.h>
#include <gtkmm/application.h>
#include <gtkmm/window.h>
#include <cairomm/context.h>
#include <iostream>
#include "graphic.h"
#include "simulation.h"
#include <string>
#include <sstream>

constexpr unsigned taille_dessin(500); 
struct Frame // Model Framing and window parameters
{
    double xMin; // frame parameters
    double xMax;
    double yMin;
    double yMax;
    double asp;  // frame aspect ratio
    int width;   // window width
    int height;  // window height
};


class Gui : public Gtk::DrawingArea
{
public:
    Gui(Simulation& sim);
    virtual ~Gui();
    void setFrame(Frame x);
    void adjustFrame(int width, int height);
    void setsimul(Simulation& new_sim);
private:
    Frame frame;
protected:
	Simulation* m_sim;
    void on_draw(const Cairo::RefPtr<Cairo::Context>& cr, 
        int width, int height);
    
};


class MyWindow : public Gtk::Window
{
public:
    MyWindow(Simulation& sim);
    virtual ~MyWindow();

    void connect_signals();
    void update_values();
    void make_zeros();
    void update_simulation();


protected:

	Simulation& m_sim;

	//My Area
	Gui my_area;


    //Signal handlers:
    void on_button_clicked_exit();
    void on_button_file_clicked_open();
    void on_button_file_clicked_save();
    // button signal handlers
    void on_button_clicked_start();
    void on_button_clicked_step();

    void on_file_dialog_response_open(int response_id, 
        Gtk::FileChooserDialog* dialog);
    void on_file_dialog_response_save(int response_id, 
        Gtk::FileChooserDialog* dialog);
    
    // This is the standard prototype of the Timer callback function
    bool on_timeout();

    // Key event handler
    bool on_window_key_pressed(guint keyval, 
        guint keycode, Gdk::ModifierType state);
	
	Gtk::Box m_Main_Box, m_Secondary_Box, m_Buttons_Box,m_Info_Box, 
        m_Parametres_Box,m_Valeurs_Box;
    Gtk::AspectFrame m_AspectFrame;
	Gtk::Frame m_Frame1,m_Frame2,m_Frame3;
    Gtk::Label  m_Label_general;
    Gtk::Label  m_Label_titre;
    Gtk::Label  m_Label_nbupdates;
    Gtk::Label  m_Label_particule;
    Gtk::Label  m_Label_rs;
    Gtk::Label  m_Label_rr;
    Gtk::Label  m_Label_ns;
    Gtk::Label  m_Label_np;
    Gtk::Label  m_Label_nd;
    Gtk::Label  m_Label_nr;
    

    Gtk::Button m_Button_exit;
    Gtk::Button m_Button_File_open;
    Gtk::Button m_Button_File_save;
    Gtk::ToggleButton toggle_timer;
    Gtk::Button m_Button_step;
    Gtk::Label text_label, data_label;
    
    // to handle a single timer
    bool timer_added;
    // to store a single timer disconnect request
    bool disconnect;

    const int timeout_value;
    
    Gtk::Label  m_Valeurs_nbupdates;
    Gtk::Label  m_Valeurs_particule;
    Gtk::Label  m_Valeurs_rs;
    Gtk::Label  m_Valeurs_rr;
    Gtk::Label  m_Valeurs_ns;
    Gtk::Label  m_Valeurs_np;
    Gtk::Label  m_Valeurs_nd;
    Gtk::Label  m_Valeurs_nr;
    
    

};

#endif // GTKMM_EXAMPLE_Gui_H


