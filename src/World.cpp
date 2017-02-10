#include "World.h"

World::World(RenderWindow& worldWindow) :
	window(worldWindow),
	DIMENSIONS({window.getSize().x, window.getSize().y})
{
	AGENT_INIT_X = 100;
	AGENT_INIT_Y = 100;

	GOAL_LOC_X = DIMENSIONS[0] - 200;
	GOAL_LOC_Y = DIMENSIONS[1] - 200;

	for (int i = 0; i < NUM_AGENTS; i++) {
		Agent* agent = new Agent(vec({AGENT_INIT_X, AGENT_INIT_Y}), AGENT_RADIUS, vec({GOAL_LOC_X, GOAL_LOC_Y}), DIMENSIONS);
		agents.push_back(agent);
		worldObjects.push_back(agent);
	}

	generateTrees();

}

void World::run() {
	while (window.isOpen()) {
		while (t < ROUND_TIME) {
			Event event;

			while (window.pollEvent(event)) {
				if (event.type == Event::Closed)
					window.close();
			}

			vector<WorldObject*> treeObjects = worldObjects;

			// Remove agents from list of objects to be checked for sensor collision.
			treeObjects.erase(treeObjects.begin(), treeObjects.begin() + NUM_AGENTS);

			// Update the agents.
			for (Agent* agent : agents) {
				agent->updateVisionRays();
				vec sensedWorld = agent->sense(treeObjects);
				vec actions = agent->think(sensedWorld);

				agent->move(actions, treeObjects);
				agent->addFitness();

			}

			// Draw everything.
			if (t % WORLD_SPEED == 0) {
				window.clear(Color::White);
				for (WorldObject* object : worldObjects) {
					draw(object);
				}

				for (Agent* agent : agents) {
					drawDirection(agent);
				}
				drawGoal();

				window.display();
			}

			t++;
		}

		getNextRound();
	}
}

void World::getNextRound() {
	vector<Agent*> newAgents;

	// Sorts agents by fitness.
	sort(agents.begin(), agents.end(),
		[&](const Agent* a, const Agent* b) -> bool
	{
		return a->getTotalFitness() > b->getTotalFitness();
	});

	default_random_engine generator;
	// Sets exponential distribution with 50th percentile at NUM_AGENTS.
	exponential_distribution<double> distribution(0.693147 / sqrt(NUM_AGENTS));

	unsigned parent1, parent2;
	// For the number of couples we need...
	for (int i = 0; i < NUM_AGENTS / OFFSPRING_PER_COUPLE; i++) {

		// Find two parents and remove them from the list.
		parent1 = (int) distribution(generator) % agents.size();
		parent2 = (int) distribution(generator) % agents.size();

		// Determine the mutation rate as the shifted average fitness from 0 to 1.

		double mutationRate = -(agents[parent1]->getTotalFitness() + agents[parent2]->getTotalFitness()) / (4 * ROUND_TIME) + 0.5 - MUTATION_DAMPENER;

		// For the number of offspring per couple...
		for (int j = 0; j < OFFSPRING_PER_COUPLE; j++) {
			// Make a brain for the offspring.

			vector< Mat<double> > childBrain = agents[parent1]->getOffspringBrain(agents[parent2], mutationRate);

			// Add the child to a new list of agents.
			Agent* child = new Agent(vec({AGENT_INIT_X, AGENT_INIT_Y}), AGENT_RADIUS, vec({GOAL_LOC_X, GOAL_LOC_Y}), DIMENSIONS, childBrain[0], childBrain[1]);
			newAgents.push_back(child);
		}

		agents.erase(agents.begin() + parent1);
		agents.erase(agents.begin() + parent2);
	}

	// Clear the world and reset objects and agents.
	agents.clear();
	worldObjects.clear();

	// Truncate the number of agents if we have a rounding error.
	for (int i = 0; i < NUM_AGENTS; i++) agents.push_back(newAgents[i]);

	for (WorldObject* agent : agents) {
		worldObjects.push_back(agent);
	}

	generateTrees();
	t = 0;
}

void World::generateTrees() {
	for (int i = 0; i < NUM_TREES; i++) {

		// Find a new location and new radius.
		vec newLocation({rand() % DIMENSIONS[0], rand() % DIMENSIONS[1]});
		double newRadius(rand() % 20 + 10);

		// Check for intersection.
		bool checkingPoint;
		do {
			checkingPoint = false;

			for (WorldObject* object : worldObjects) {
				// If we're too close to the goal, or too close to another object...
				if (norm(newLocation - vec({GOAL_LOC_X, GOAL_LOC_Y})) < object->getRadius() + GOAL_RADIUS
					or norm(object->getLocation() - newLocation) < object->getRadius() + newRadius) {

					// Find a new location, new radius, and check that point too.
					newLocation = vec({rand() % DIMENSIONS[0], rand() % DIMENSIONS[1]});
					newRadius = rand() % (MAX_TREE_RADIUS - MIN_TREE_RADIUS) + MIN_TREE_RADIUS;
					checkingPoint = true;
					break;
				}
			}
		} while(checkingPoint);

		// Push our tree onto the list of objects.
		WorldObject* tree = new Tree(newLocation, newRadius);
		worldObjects.push_back(tree);
	}
}

void World::draw(WorldObject* object) const {
	CircleShape objSprite(max((float) object->getRadius() - 2, 5.f)); object->getRadius();
	vec objLocation = object->getLocation();

	objSprite.setPosition((int) objLocation[0] - object->getRadius(), (int) objLocation[1] - object->getRadius());
	objSprite.setFillColor(object->getColor());
	objSprite.setOutlineColor(Color::Black);
	objSprite.setOutlineThickness(2);

	window.draw(objSprite);
}

void World::drawDirection(Agent* agent) const {
	vec endRay = agent->getLocation() + vec({50*cos(agent->getDirection()), 50*sin(agent->getDirection())});

	Vertex line[] =
	{
		Vertex(Vector2f(agent->getLocation()[0] - 2, agent->getLocation()[1] - 2)),
		Vertex(Vector2f(endRay[0] - 2, endRay[1] - 2))
	};

	line[0].color = Color::Black;
	line[1].color = Color::Black;

	window.draw(line, 2, Lines);
}

void World::drawGoal() const {
	CircleShape goalCircle(GOAL_RADIUS);

	goalCircle.setPosition(GOAL_LOC_X, GOAL_LOC_Y);
	goalCircle.setFillColor(Color::Blue);
	goalCircle.setOutlineColor(Color::Black);
	goalCircle.setOutlineThickness(2);

	window.draw(goalCircle);
}
