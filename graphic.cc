#include "graphic.h"
#include <cmath>

static const Cairo::RefPtr<Cairo::Context>* ptcr(nullptr);
void graphic_set_context( const Cairo::RefPtr<Cairo::Context>& cr)
{
ptcr = &cr;
}


void draw_border(double xMin, double yMin, double xMax, 
                                                double yMax, double thickness) {
	(*ptcr)->save();
    set_color(Gray);
    (*ptcr)->set_line_width(thickness);

    // Draw the four lines to form the border
    (*ptcr)->move_to(xMin, yMin);
    (*ptcr)->line_to(xMax, yMin);
    (*ptcr)->stroke();

    (*ptcr)->move_to(xMax, yMin);
    (*ptcr)->line_to(xMax, yMax);
    (*ptcr)->stroke();

    (*ptcr)->move_to(xMax, yMax);
    (*ptcr)->line_to(xMin, yMax);
    (*ptcr)->stroke();

    (*ptcr)->move_to(xMin, yMax);
    (*ptcr)->line_to(xMin, yMin);
    (*ptcr)->stroke();
    (*ptcr)->restore();
}

void set_color(ColorIndex color) {
    switch (color) {
        case White:
            (*ptcr)->set_source_rgb(1, 1, 1);
            break;
        case Gray:
            (*ptcr)->set_source_rgb(0.5, 0.5, 0.5);
            break;
        case Red:
            (*ptcr)->set_source_rgb(1, 0, 0);
            break;
        case LightBlue:
            (*ptcr)->set_source_rgb(0, 255, 255);
            break;
        case Black:
            (*ptcr)->set_source_rgb(0, 0, 0);
            break;
        case Violet:
            (*ptcr)->set_source_rgb(128, 0, 128);
            break;
        case Orange:
            (*ptcr)->set_source_rgb(255, 0, 0);
            break;
        case Green:
            (*ptcr)->set_source_rgb(0, 128, 0);
            break;
    }
}


void draw_particle(double x, double y, double d) {
	
	double half_d = d/2;
	
	(*ptcr)->save();
    set_color(Red);
    (*ptcr)->set_line_width(1);

    // Draw the four lines to form the border
    (*ptcr)->move_to(x - half_d, y - half_d);
    (*ptcr)->line_to(x + half_d, y - half_d);
    (*ptcr)->line_to(x + half_d, y + half_d);
    (*ptcr)->line_to(x - half_d, y + half_d);
    (*ptcr)->line_to(x - half_d, y-half_d);
    (*ptcr)->close_path();

    (*ptcr)->stroke_preserve(); 
    set_color(Gray);
    (*ptcr)->fill();
    (*ptcr)->restore();
}

void draw_robot_spatial(double x, double y, double r) {
	(*ptcr)->save();
    set_color(LightBlue);
    (*ptcr)->arc(x, y, r, 0, 2 * M_PI);
    (*ptcr)->set_line_width(1);
    (*ptcr)->stroke();
    
    // Draw a light blue dot in the middle of the spatial robot
    set_color(LightBlue);
    (*ptcr)->arc(x, y, r / 16, 0, 2 * M_PI); 
    (*ptcr)->fill();
    (*ptcr)->restore();
}


void draw_robot_repairer(double x, double y, double r) {
    // Draw a green circle
    (*ptcr)->save();
    (*ptcr)->arc(x, y, r, 0.0, 2.0 * M_PI); // full circle
    set_color(Green); 
    (*ptcr)->fill_preserve();
    
    // Draw a black outline
    set_color(Black);
    (*ptcr)->set_line_width(r / 8); // Adjust the line width if needed
    (*ptcr)->stroke();
    (*ptcr)->restore();
}


void draw_robot_neutralizer_panne(double x, double y, double r, double angle){
	(*ptcr)->save();

    set_color(White);
    (*ptcr)->arc(x, y, r, 0, 2 * M_PI);
    (*ptcr)->fill_preserve();
	set_color(Orange);
    (*ptcr)->set_line_width(1);
    (*ptcr)->stroke();

    set_color(Green); // Change orientation line color to green
    (*ptcr)->move_to(x, y);
    (*ptcr)->line_to(x + r * cos(angle), y + r * sin(angle));
    (*ptcr)->set_line_width(r / 6); // Increase orientation line width
    (*ptcr)->stroke();
    set_color(Black);
    (*ptcr)->arc(x, y, 0.5, 0, 2 * M_PI); 
    (*ptcr)->fill();
    
    (*ptcr)->restore();
}

void draw_robot_neutralizer_service(double x, double y, double r, double angle){
	(*ptcr)->save();
	

    
    set_color(White);
    (*ptcr)->arc(x, y, r, 0, 2 * M_PI);
    (*ptcr)->fill_preserve();
	set_color(Black);
    (*ptcr)->set_line_width(1);
    (*ptcr)->stroke();

    set_color(Green); // Change orientation line color to green
    (*ptcr)->move_to(x, y);
    (*ptcr)->line_to(x + r * cos(angle), y + r * sin(angle));
    (*ptcr)->set_line_width(r / 6); // Increase orientation line width
    (*ptcr)->stroke();
    
    set_color(Black);
    (*ptcr)->arc(x, y, 0.5, 0, 2 * M_PI); 
    (*ptcr)->fill();
    
    (*ptcr)->restore();
}

void draw_robot_neutralizer_collistion(double x, double y, double r, double angle){
	(*ptcr)->save();
	

    
    set_color(White);
    (*ptcr)->arc(x, y, r, 0, 2 * M_PI);
    (*ptcr)->fill_preserve();
    
	set_color(Violet);
    (*ptcr)->set_line_width(1);
    (*ptcr)->stroke();

    set_color(Green); // Change orientation line color to green
    (*ptcr)->move_to(x, y);
    (*ptcr)->line_to(x + r * cos(angle), y + r * sin(angle));
    (*ptcr)->set_line_width(r / 6); // Increase orientation line width
    (*ptcr)->stroke();
    
    set_color(Black);
    (*ptcr)->arc(x, y, 0.5, 0, 2 * M_PI); 
    (*ptcr)->fill();
    
    (*ptcr)->restore();
}
