#ifndef AGENT_H_
#define AGENT_H_

#include <vector>

#include <armadillo>
#include <SFML/Graphics.hpp>

#include "WorldObject.h"

using namespace std;
using namespace arma;
using namespace sf;

class Agent : public WorldObject {
public:
	//// GENERAL PUBLIC FUNCTIONS ////

	Agent(vec, double, vec, vector<int>);
	Agent(vec, double, vec, vector<int>, Mat<double>, Mat<double>);

	const Color getColor() const { return Color::Red; }
	const double getDirection() const { return direction; }

	void updateVisionRays();


	//// WORLD INTERACTION fUNCTIONS ////

	vec sense(vector<WorldObject*>);
	vec think(vec) const;
	void move(vec, vector<WorldObject*>);


	//// PUBLIC GENETIC ALGORITHM FUNCTIONS ////

	void addFitness();
	double getTotalFitness() const { return fitness; }

	vector< Mat<double> > getOffspringBrain(Agent*, double) const;

private:
	//// GENERAL PRIVATE FUNCTIONS ////

	double intersectionDistance(vec, vector<WorldObject*> worldObjects) const;
	void sortByDistance(vector<WorldObject*> worldObjects);


	//// AGENT CONSTANTS ////

	static constexpr double PI = 3.141592653589793238463;
	vector<int> DIMENSIONS;

	// Set of rays that extend from the player.
	// The origin for these are defined to be the agent's location.
	static const int NUM_VISION_RAYS = 5;
	static constexpr double VISION_RANGE = PI / 2;

	// Input vector is vision rays, and signed difference between direction to goal, and current direction.
	static constexpr int HYPERPARAMS[3] = {NUM_VISION_RAYS + 1, 5, 1};


	//// AGENT STATE VARIABLES ////

	double direction = 0;
	vec goalLocation;
	vector<vec> visionRays;


	//// GENETIC ALGORITHM ////

	double initDistance;
	double fitness = 0;

	Mat<double> W1, W2;

	static vec nonlinear(vec input) {
		input.transform( [](double val) {return 1 / (1 + exp(-val)); });
		return input;
	}
};

#endif
