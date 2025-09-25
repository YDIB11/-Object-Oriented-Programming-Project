#include "gui.h"

using namespace std;

// fonction utilitaire confidentielle à ce module
static void orthographic_projection(const Cairo::RefPtr<Cairo::Context> &cr,
                                    const Frame &frame);

// default Model Framing and window parameters
static Frame default_frame = {-1.0 * dmax, dmax, -1.0 * dmax, dmax,
 static_cast<double>(taille_dessin) / taille_dessin,
  taille_dessin, taille_dessin};

Gui::Gui(Simulation &sim) : m_sim(&sim)
{
    setFrame(default_frame);
    set_content_width(default_frame.width);
    set_content_height(default_frame.height);

    set_draw_func(sigc::mem_fun(*this, &Gui::on_draw));
}

void Gui::setsimul(Simulation &new_sim)
{
    m_sim = &new_sim;
}

Gui::~Gui() {}

void Gui::setFrame(Frame f)
{
    if ((f.xMin <= f.xMax) and (f.yMin <= f.yMax) and (f.height > 0))
    {
        f.asp = f.width / f.height;
        frame = f;
    }
    else
        std::cout<<"incorrect Model framing or window parameters" << std::endl;
}

void Gui::adjustFrame(int width, int height)
{
    frame.width = width;
    frame.height = height;

    // Preventing distorsion by adjusting the frame (cadrage)
    // to have the same proportion as the graphical area

    // use the reference framing as a guide for preventing distortion
    double new_aspect_ratio((double)width / height);
    if (new_aspect_ratio > default_frame.asp)
    {
        // keep yMax and yMin. Adjust xMax and xMin
        frame.yMax = default_frame.yMax;
        frame.yMin = default_frame.yMin;

        double delta(default_frame.xMax - default_frame.xMin);
        double mid((default_frame.xMax + default_frame.xMin) / 2);
        // the new frame is centered on the mid-point along X
        frame.xMax = mid + 0.5 * (new_aspect_ratio / default_frame.asp) * delta;
        frame.xMin = mid - 0.5 * (new_aspect_ratio / default_frame.asp) * delta;
    }
    else
    {
        // keep xMax and xMin. Adjust yMax and yMin
        frame.xMax = default_frame.xMax;
        frame.xMin = default_frame.xMin;

        double delta(default_frame.yMax - default_frame.yMin);
        double mid((default_frame.yMax + default_frame.yMin) / 2);
        // the new frame is centered on the mid-point along Y
        frame.yMax = mid + 0.5 * (default_frame.asp / new_aspect_ratio) * delta;
        frame.yMin = mid - 0.5 * (default_frame.asp / new_aspect_ratio) * delta;
    }
}

static void orthographic_projection(const Cairo::RefPtr<Cairo::Context> &cr,
                                    const Frame &frame)
{
    // déplace l'origine au centre de la fenêtre
    cr->translate(frame.width / 2, frame.height / 2);

    // normalise la largeur et hauteur aux valeurs fournies par le cadrage
    // ET inverse la direction de l'axe Y
    cr->scale(frame.width / (frame.xMax - frame.xMin),
              -frame.height / (frame.yMax - frame.yMin));

    // décalage au centre du cadrage
    cr->translate(-(frame.xMin + frame.xMax) / 2, -(frame.yMin + frame.yMax) / 2);
}

void Gui::on_draw(const Cairo::RefPtr<Cairo::Context> &cr,
    int width, int height)
{
    // adjust the frame (cadrage) to prevent 
    //distortion when changing the window size

    graphic_set_context(cr);
    adjustFrame(width, height);
    orthographic_projection(cr, frame);

    // Set background color to white (light mode)
    cr->save();
    set_color(White);
    cr->paint();
    cr->restore();

    if (m_sim->getErreur() == false)
    {
        m_sim->draw_planet();
    }
    double border_thickness = 3.0; // Adjust the border thickness as needed
    draw_border(frame.xMin - 1, frame.yMin - 1, frame.xMax + 1, 
        frame.yMax + 1, border_thickness);
}


void MyWindow::connect_signals()
{
    m_Button_exit.signal_clicked().connect(sigc::mem_fun(*this, 
                            &MyWindow::on_button_clicked_exit));
    m_Button_File_open.signal_clicked().connect(sigc::mem_fun(*this, 
                            &MyWindow::on_button_file_clicked_open));
    m_Button_File_save.signal_clicked().connect(sigc::mem_fun(*this,
                             &MyWindow::on_button_file_clicked_save));
    toggle_timer.signal_clicked().connect(sigc::mem_fun(*this,
                             &MyWindow::on_button_clicked_start));
    m_Button_step.signal_clicked().connect(sigc::mem_fun(*this, 
                            &MyWindow::on_button_clicked_step));
}

MyWindow::MyWindow(Simulation &sim)
    : m_sim(sim), my_area(sim), m_Main_Box(Gtk::Orientation::HORIZONTAL, 5),
      m_Secondary_Box(Gtk::Orientation::VERTICAL, 10),
      m_Buttons_Box(Gtk::Orientation::VERTICAL, 0),
      m_Info_Box(Gtk::Orientation::HORIZONTAL, 0),
      m_Parametres_Box(Gtk::Orientation::VERTICAL, 0),
      m_Valeurs_Box(Gtk::Orientation::VERTICAL, 0),
      m_Label_general("General"), m_Label_titre("info : nombre de ..."),
      m_Label_nbupdates("mises à jour:"), m_Label_particule("particules:"),
      m_Label_rs("robots réparateurs en service:"),
      m_Label_rr("robots réparateurs en reserve:"),
      m_Label_ns("robots neutralisateurs en service:"),
      m_Label_np("robots neutralisateurs en panne:"),
      m_Label_nd("robots neutralisateurs détruits:"),
      m_Label_nr("robots neutralisateurs en réserve:"),
      m_Valeurs_nbupdates(to_string(sim.getNbUpdates())),
      m_Valeurs_particule(to_string(sim.getParticulesSize())),
      m_Valeurs_rs(to_string(sim.getNbRs())),
      m_Valeurs_rr(to_string(sim.getNbRr())),
      m_Valeurs_ns(to_string(sim.getNbNs())),
      m_Valeurs_np(to_string(sim.getNbNp())),
      m_Valeurs_nd(to_string(sim.getNbNd())),
      m_Valeurs_nr(to_string(sim.getNbNr())),
      m_Button_exit("exit"), m_Button_File_open("open"),
      m_Button_File_save("save"), toggle_timer("start", true),
      m_Button_step("step", true), text_label("Simulation step : "),
      data_label("0"), timer_added(false),
      disconnect(false), timeout_value(25) //500 ms = 0.5s
{
    set_title("Propre en Ordre");
    set_child(m_Main_Box);

    m_Main_Box.append(m_Secondary_Box);
    m_Main_Box.append(m_AspectFrame);
    m_AspectFrame.set_child(m_Frame3);
    m_Frame3.set_child(my_area);
    my_area.set_expand();

    m_Secondary_Box.append(m_Frame1);
    m_Secondary_Box.append(m_Frame2);

    m_Frame1.set_label("General");
    m_Frame2.set_label("info : nombre de ...");

    m_Frame1.set_child(m_Buttons_Box);
    m_Buttons_Box.append(m_Button_exit);
    m_Buttons_Box.append(m_Button_File_open);
    m_Buttons_Box.append(m_Button_File_save);
    m_Buttons_Box.append(toggle_timer);
    m_Buttons_Box.append(m_Button_step);

    m_Frame2.set_child(m_Info_Box);
    m_Info_Box.append(m_Parametres_Box);
    m_Info_Box.append(m_Valeurs_Box);
    m_Valeurs_Box.set_margin_end(1);

    Gtk::Box *boxes[] = {&m_Parametres_Box, &m_Valeurs_Box};
    Gtk::Label *labels[] = {&m_Label_nbupdates, &m_Label_particule,&m_Label_rs,
             &m_Label_rr, &m_Label_ns, &m_Label_np, &m_Label_nd, &m_Label_nr};
    Gtk::Label *values[] = {&m_Valeurs_nbupdates, &m_Valeurs_particule,
             &m_Valeurs_rs, &m_Valeurs_rr, &m_Valeurs_ns, &m_Valeurs_np, 
             &m_Valeurs_nd, &m_Valeurs_nr};

    for (int i = 0; i < 8; i++)
    {
        m_Parametres_Box.append(*labels[i]);
        m_Valeurs_Box.append(*values[i]);
        labels[i]->set_xalign(0);
    }

    connect_signals();

    auto controller = Gtk::EventControllerKey::create();
    controller->signal_key_pressed().connect(sigc::mem_fun(*this,
         &MyWindow::on_window_key_pressed), false);
    add_controller(controller);

    if (m_sim.getErreur() == true)
    {
        make_zeros();
    }
}




MyWindow::~MyWindow()
{
}

void MyWindow::update_simulation()
{
    m_sim.update(); 
    my_area.queue_draw();         
}

void MyWindow::on_button_clicked_exit()
{
    hide(); // to close the application.
}

void MyWindow::on_button_file_clicked_open()
{
    auto dialog = new Gtk::FileChooserDialog("Please choose a file",
                                             Gtk::FileChooser::Action::OPEN);
    dialog->set_transient_for(*this);
    dialog->set_modal(true);
    dialog->signal_response().connect(sigc::bind(
        sigc::mem_fun(*this, &MyWindow::on_file_dialog_response_open), dialog));

    // Add response buttons to the dialog:
    dialog->add_button("_Cancel", Gtk::ResponseType::CANCEL);
    dialog->add_button("_Open", Gtk::ResponseType::OK);

    // Add filters, so that only certain file types can be selected:
    auto filter_text = Gtk::FileFilter::create();
    filter_text->set_name("Text files");
    filter_text->add_mime_type("text/plain");
    dialog->add_filter(filter_text);

    auto filter_cpp = Gtk::FileFilter::create();
    filter_cpp->set_name("C/C++ files");
    filter_cpp->add_mime_type("text/x-c");
    filter_cpp->add_mime_type("text/x-c++");
    filter_cpp->add_mime_type("text/x-c-header");
    dialog->add_filter(filter_cpp);

    auto filter_any = Gtk::FileFilter::create();
    filter_any->set_name("Any files");
    filter_any->add_pattern("*");
    dialog->add_filter(filter_any);

    // Show the dialog and wait for a user response:
    dialog->show();
}

void MyWindow::on_file_dialog_response_open(int response_id,
                                            Gtk::FileChooserDialog *dialog)
{
    // Handle the response:
    switch (response_id)
    {
    case Gtk::ResponseType::OK:
    {
        std::cout << "Open clicked." << std::endl;

        // Notice that this is a std::string, not a Glib::ustring.
        auto filename = dialog->get_file()->get_path();

        m_sim.clean_ancien_sim();

        m_sim.readData(filename);
        my_area.setsimul(m_sim);
        my_area.queue_draw();
        update_values();
        std::cout << "File selected: " << filename << std::endl;
        break;
    }
    case Gtk::ResponseType::CANCEL:
    {
        std::cout << "Cancel clicked." << std::endl;
        break;
    }
    default:
    {
        std::cout << "Unexpected button clicked." << std::endl;
        break;
    }
    }
    dialog->hide();
}

void MyWindow::on_button_file_clicked_save()
{
    auto dialog = new Gtk::FileChooserDialog("Please choose a file",
                                             Gtk::FileChooser::Action::SAVE);
    dialog->set_transient_for(*this);
    dialog->set_modal(true);
    dialog->signal_response().connect(sigc::bind(
        sigc::mem_fun(*this, &MyWindow::on_file_dialog_response_save), dialog));

    // Add response buttons to the dialog:
    dialog->add_button("_Cancel", Gtk::ResponseType::CANCEL);
    dialog->add_button("_Save", Gtk::ResponseType::OK);

    // Add filters, so that only certain file types can be selected:

    auto filter_text = Gtk::FileFilter::create();
    filter_text->set_name("Text files");
    filter_text->add_mime_type("text/plain");
    dialog->add_filter(filter_text);

    auto filter_cpp = Gtk::FileFilter::create();
    filter_cpp->set_name("C/C++ files");
    filter_cpp->add_mime_type("text/x-c");
    filter_cpp->add_mime_type("text/x-c++");
    filter_cpp->add_mime_type("text/x-c-header");
    dialog->add_filter(filter_cpp);

    auto filter_any = Gtk::FileFilter::create();
    filter_any->set_name("Any files");
    filter_any->add_pattern("*");
    dialog->add_filter(filter_any);

    // Show the dialog and wait for a user response:
    dialog->show();
}

void MyWindow::on_file_dialog_response_save(int response_id,
                                            Gtk::FileChooserDialog *dialog)
{
    // Handle the response:
    switch (response_id)
    {
    case Gtk::ResponseType::OK:
    {
        std::cout << "Save clicked." << std::endl;

        // Notice that this is a std::string, not a Glib::ustring.
        auto filename = dialog->get_file()->get_path();
        m_sim.sauvegarde(filename);
        std::cout << "File selected: " << filename << std::endl;
        break;
    }
    case Gtk::ResponseType::CANCEL:
    {
        std::cout << "Cancel clicked." << std::endl;
        break;
    }
    default:
    {
        std::cout << "Unexpected button clicked." << std::endl;
        break;
    }
    }
    dialog->hide();
}

void MyWindow::on_button_clicked_start()
{
    if (!timer_added)
    {
        sigc::slot<bool()> my_slot = sigc::bind(sigc::mem_fun(*this,
                                                              &MyWindow::on_timeout));

        auto conn = Glib::signal_timeout().connect(my_slot, timeout_value);

        timer_added = true;

        std::cout << "Timer added" << std::endl;

        toggle_timer.set_label("stop");
       

    }
    else
    {
        std::cout << "Manually disconnecting the timer" << std::endl;
        disconnect = true;
        timer_added = false;

        toggle_timer.set_label("start");
    }
}

void MyWindow::on_button_clicked_step()
{
    unsigned int current_val = std::stoi(m_Valeurs_nbupdates.get_text());

    if (!timer_added)
    {
        current_val++;
        m_sim.setRSnbupdate(current_val);
        m_Valeurs_nbupdates.set_text(std::to_string(current_val));
        update_simulation();
        update_values();
        m_sim.trier_particules();

    }
    else if (timer_added)
    {
        std::cout << "Timer is running. Cannot perform a single step." << std::endl;
    }
}
bool MyWindow::on_timeout()
{
    unsigned int val = std::stoi(m_Valeurs_nbupdates.get_text());

    if (disconnect or m_sim.end_simulation())
    {
        disconnect = false;

        return false;
    }

    val++;
    m_Valeurs_nbupdates.set_text(std::to_string(val));
    m_sim.setRSnbupdate(val);
    update_simulation();
    update_values();
    m_sim.trier_particules();

    return true;
}

bool MyWindow::on_window_key_pressed(guint keyval, guint, Gdk::ModifierType state)
{
    switch (gdk_keyval_to_unicode(keyval))
    {
    case 's':
        on_button_clicked_start();
        return true;
    case '1':
        on_button_clicked_step();
        return true;
    }
    return false;
}

void MyWindow::make_zeros()
{
    m_Valeurs_nbupdates.set_text(to_string(0));
    m_Valeurs_particule.set_text(to_string(0));
    m_Valeurs_rs.set_text(to_string(0));
    m_Valeurs_rr.set_text(to_string(0));
    m_Valeurs_ns.set_text(to_string(0));
    m_Valeurs_np.set_text(to_string(0));
    m_Valeurs_nd.set_text(to_string(0));
    m_Valeurs_nr.set_text(to_string(0));
}

void MyWindow::update_values()
{

    if (m_sim.getErreur() == true)
    {
        make_zeros();
    }
    else
    {
        m_Valeurs_nbupdates.set_text(to_string(m_sim.getNbUpdates()));
        m_Valeurs_particule.set_text(to_string(m_sim.getParticulesSize()));
        m_Valeurs_rs.set_text(to_string(m_sim.getNbRs()));
        m_Valeurs_rr.set_text(to_string(m_sim.getNbRr()));
        m_Valeurs_ns.set_text(to_string(m_sim.getNbNs()));
        m_Valeurs_np.set_text(to_string(m_sim.getNbNp()));
        m_Valeurs_nd.set_text(to_string(m_sim.getNbNd()));
        m_Valeurs_nr.set_text(to_string(m_sim.getNbNr()));

    }
}


