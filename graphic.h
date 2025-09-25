#ifndef GRAPHIC_H
#define GRAPHIC_H

#include <gtkmm.h>
#include <cairomm/context.h>

enum ColorIndex {
    White,
    Gray,
    Red,
    LightBlue,
    Black,
    Violet,
    Orange,
    Green
};

void graphic_set_context(const Cairo::RefPtr<Cairo::Context>& cr);


void draw_border(double xMin, double yMin, double xMax, double yMax, double thickness);
void set_color(ColorIndex color);
void draw_particle(double x, double y, double d);
void draw_robot_spatial(double x, double y, double size);
void draw_robot_repairer(double x, double y, double size);
void draw_robot_neutralizer_panne(double x, double y, double size, double angle);
void draw_robot_neutralizer_service(double x, double y, double size, double angle);
void draw_robot_neutralizer_collistion(double x, double y, double size, double angle);

#endif // GRAPHIC_H
