#ifndef WORLD_H_
#define WORLD_H_

#include <stdlib.h>
#include <assert.h>
#include <vector>

#include <armadillo>
#include <SFML/Graphics.hpp>

#include "Agent.h"
#include "Tree.h"

using namespace std;
using namespace sf;
using namespace arma;

class World {
public:
	//// GENERAL PUBLIC FUNCTIONS ////

	World(RenderWindow&);

	void run();
	void getNextRound();

	RenderWindow& window;

private:
	//// GENERAL PRIVATE FUNCTIONS ////

	vector<Agent*> getAgents() const { return agents; }
	void generateTrees();


	//// GRAPHICS FUNCTIONS ////

	void draw(WorldObject*) const;
	void drawDirection(Agent*) const;
	void drawGoal() const;


	//// WORLD CONSTANTS ////

	static const int ROUND_TIME = 1500;
	static const int WORLD_SPEED = 10;

	static const int NUM_AGENTS = 30;
	static const int NUM_TREES = 15;

	static const int AGENT_RADIUS = 10;
	static const int GOAL_RADIUS = 10;
	static const int MIN_TREE_RADIUS = 10;
	static const int MAX_TREE_RADIUS = 30;

	static const int OFFSPRING_PER_COUPLE = 5;

	// This constant is from -1/2 to 1/2, and will modify the threshold for mutation.
	// Use it if you want to increase the number of mutations in the entire population.
	static constexpr double MUTATION_DAMPENER = 0;

	// These depend on the size of the map, so they're not initialized until we initialize the world.
	vector<int> DIMENSIONS;
	int AGENT_INIT_X, AGENT_INIT_Y, GOAL_LOC_X, GOAL_LOC_Y;


	//// WORLD STATE ////

	int t = 0;

	// worldObjects is a vector of pointers to all agents and trees, with agents first.
	// agents is a vector of pointers to all agents.
	vector<WorldObject*> worldObjects;
	vector<Agent*> agents;
};

#endif
