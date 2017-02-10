#ifndef WORLDOBJECT_H_
#define WORLDOBJECT_H_

#include <SFML/Graphics.hpp>
#include <armadillo>

using namespace sf;
using namespace arma;

class WorldObject {
public:
	//// GENERAL PUBLIC FUNCTIONS ////

	WorldObject(vec, double);

	vec getLocation() const { return location; }
	double getRadius() const { return radius; }

	void translate(vec dx) { location = location + dx; }

	virtual const Color getColor() const = 0;

protected:
	//// WORLD OBJECT STATE ////

	vec location;
	int radius;
};

#endif
