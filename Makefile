# Definitions de macros

OUT = projet
CC     = g++
CCFLAGS = -g -Wall -std=c++17 $(LINKING)
LINKING = `pkg-config --cflags gtkmm-4.0`
LDLIBS = `pkg-config --libs gtkmm-4.0`
OFILES = simulation.o robot.o particule.o message.o shape.o projet.o gui.o graphic.o


all: $(OUT)

graphic.o: graphic.cc graphic.h 
	$(CC) $(CCFLAGS) $(LINKING) -c $< -o $@ $(LINKING)	
	
shape.o: shape.cc shape.h graphic.h
	$(CC) $(CCFLAGS) $(LINKING) -c $< -o $@ $(LINKING)

robot.o: robot.cc robot.h shape.h particule.h message.h constantes.h
	$(CC) $(CCFLAGS) $(LINKING) -c $< -o $@ $(LINKING)

particule.o: particule.cc particule.h shape.h message.h constantes.h
	$(CC) $(CCFLAGS) $(LINKING) -c $< -o $@ $(LINKING)
	
simulation.o: simulation.cc simulation.h robot.h shape.h particule.h \
 message.h constantes.h
	$(CC) $(CCFLAGS) $(LINKING) -c $< -o $@ $(LINKING)

message.o: message.cc message.h
	$(CC) $(CCFLAGS) $(LINKING) -c $< -o $@ $(LINKING)

gui.o: gui.cc gui.h constantes.h
	$(CC) $(CXXFLAGS) $(LINKING) -c $< -o $@ $(LINKING)

projet.o: projet.cc simulation.h robot.h shape.h particule.h message.h \
constantes.h
	$(CC) $(CCFLAGS) $(LINKING) -c $< -o $@ $(LINKING)
	

$(OUT): $(OFILES)
	$(CC) $(CCFLAGS) $(LINKING) $(OFILES) -o $@ $(LDLIBS)

clean:
	@echo " *** EFFACE MODULES OBJET ET EXECUTABLE ***"
	@/bin/rm -f *.o $(OUT) *.x *.cc~ *.h~ prog
