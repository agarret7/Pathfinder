#include "Agent.h"

Agent::Agent(vec location, double radius, vec goalLocation, vector<int> worldDimensions) :
	WorldObject(location, radius),
	goalLocation({goalLocation[0], goalLocation[1]}),
	initDistance(norm(location - goalLocation)),
	DIMENSIONS(worldDimensions),
	W1(HYPERPARAMS[1], HYPERPARAMS[0]),
	W2(HYPERPARAMS[2], HYPERPARAMS[1])
{
	mat::iterator a = W1.begin();
	mat::iterator b = W1.end();

	mat::iterator c = W2.begin();
	mat::iterator d = W2.end();

	do {
		*a = ((rand() % (int) 1e6) - 0.5e6) / 1e6;
		a++;
	} while (a < b);
	do {
		*c = ((rand() % (int) 1e6) - 0.5e6) / 1e6;
		c++;
	} while (c < d);
}

Agent::Agent(vec location, double radius, vec goalLocation, vector<int> worldDimensions, Mat<double> W1, Mat<double> W2) :
	WorldObject(location, radius),
	goalLocation({goalLocation[0], goalLocation[1]}),
	initDistance(norm(location - goalLocation)),
	DIMENSIONS(worldDimensions),
	W1(W1),
	W2(W2)
{
	visionRays.reserve(NUM_VISION_RAYS);
}

void Agent::updateVisionRays() {
	// Get our vision rays based on last given direction.
	visionRays.clear();
	for (double theta = direction - VISION_RANGE; theta <= direction + VISION_RANGE + 1e-6; theta += (2*VISION_RANGE) / (NUM_VISION_RAYS-1)) {
		visionRays.push_back(vec({cos(theta), sin(theta)}));
	}
}

vec Agent::sense(vector<WorldObject*> worldObjects) {
	sortByDistance(worldObjects);

	// Input is the vision limits, our location, and goal location.
	vec input(HYPERPARAMS[0]);

	// For each vision ray...
	for (int i = 0; i < NUM_VISION_RAYS; i++) {
		double distanceToIntersection = intersectionDistance(visionRays[i], worldObjects);

		// If we receive a valid distance to an intersection with a tree...
		if (distanceToIntersection > 0) {
			// The input is the distance to the intersection, normalized to the largest dimension.
			input[i] = 1 - distanceToIntersection / max(DIMENSIONS[0], DIMENSIONS[1]);
		} else {
			// Otherwise, our input is just the max value, indicating it's clear.
			// (This should be OK, since far distance will be around the same as not hitting anything.)
			input[i] = 0;
		}
	}

	double directionToGoal = atan2(goalLocation[1] - location[1], goalLocation[0] - location[0]);
	double phi = fmod(abs(direction - directionToGoal), 2*PI);
	double distance = phi > PI ? 2*PI - phi : phi;

	bool isClockwise = cos(direction) * sin(directionToGoal) - sin(direction) * cos(directionToGoal) > 0;

	input[NUM_VISION_RAYS] = (isClockwise ? distance : -distance) / PI;

	//cout << input << endl;
	return input;
}

// Takes in an input of sensor data and returns an output direction.
vec Agent::think(vec input) const {
	vec Y1 = W1 * input;
	vec Z1 = nonlinear(Y1);
	vec Y2 = W2 * Z1;
	vec Z2 = nonlinear(Y2);

	return Z2;
}

void Agent::move(vec actions, vector<WorldObject*> worldObjects) {
	sortByDistance(worldObjects);

	double dtheta = (actions[0] - 0.5) * PI/3;
	direction = fmod(direction + dtheta, 2*PI);

	vec dx({cos(direction), sin(direction)});

	for (WorldObject* object : worldObjects) {
		// If our agent runs too close to a tree or passes over the border...
		if (norm(object->getLocation() - (location + dx)) < radius + object->getRadius()
			or location[0] + dx[0] > DIMENSIONS[0] - radius or location[0] + dx[0] < radius
			or location[1] + dx[1] > DIMENSIONS[1] - radius or location[1] + dx[1] < radius) {
			// Don't do anything, and penalize fitness.
			fitness -= 0.01;
			return;
		}
	}

	// Otherwise we can just translate.
	translate(dx);
}

void Agent::addFitness() {
	// Fitness is linearly defined such that at our initial location the fitness is 0,
	// and at the goal itself fitness is 1.
	double distanceToGoal = norm(location - goalLocation);
	fitness += 1 - distanceToGoal / initDistance;
}

vector< Mat<double> > Agent::getOffspringBrain(Agent* other, double mutationRate) const {
	Mat<double> offspringW1(HYPERPARAMS[1], HYPERPARAMS[0]);
	Mat<double> offspringW2(HYPERPARAMS[2], HYPERPARAMS[1]);

	default_random_engine generator(rand());
	normal_distribution<double> gaussian(0.0, mutationRate / 100);

	for (unsigned i = 0; i < offspringW1.n_rows; i++) {
		for (unsigned j = 0; j < offspringW1.n_cols; j++) {
			// Generate offspring by randomly selecting weights from parents,
			// and mutating them at mutationRate, with a standard deviation of mutationRate.
			offspringW1(i,j) = (rand() % 2 == 0 ?
								W1(i,j) + (mutationRate * 1e6 > (rand() % (int) 1e6) ? gaussian(generator) : 0) :
								other->W1(i,j)) + (mutationRate * 1e6 > (rand() % (int) 1e6) ? gaussian(generator) : 0);
		}
	}

	for (unsigned i = 0; i < offspringW2.n_rows; i++) {
		for (unsigned j = 0; j < offspringW2.n_cols; j++) {
			// Generate offspring by randomly selecting weights from parents,
			// and mutating them at mutationRate, with a standard deviation of mutationRate.

			offspringW2(i,j) = (rand() % 2 == 0 ?
								W2(i,j) + (mutationRate * 1e6 > (rand() % (int) 1e6) ? gaussian(generator) : 0) :
								other->W2(i,j)) + (mutationRate * 1e6 > (rand() % (int) 1e6) ? gaussian(generator) : 0);
		}
	}

	vector< Mat<double> > outputMatrices;
	outputMatrices.push_back(offspringW1);
	outputMatrices.push_back(offspringW2);

	return outputMatrices;
}

double Agent::intersectionDistance(vec visionRay, vector<WorldObject*> worldObjects) const {
	for (WorldObject* object : worldObjects) {
		// First, get the object's location relative to our location.
		vec relObjLocation = object->getLocation() - location;
		// Project the object's location onto our vision ray.
		double objLocProjOntoVisionRayComponent = dot(visionRay, relObjLocation);

		// If the object's location puts it BEHIND our vision RAY...
		if (objLocProjOntoVisionRayComponent > 0) {
			// Create the vector representing the relative component in the visionRay direction.
			vec objLocProjOntoVisionRay = objLocProjOntoVisionRayComponent * visionRay;
			// Finally, get the vector from the object location to the intersection with our visionRay.
			vec objToVisionRay = objLocProjOntoVisionRay - relObjLocation;

			// If the length of the vector from the object to the intersect with our visionRay
			// is less than the radius of the tree, we have "seen" the tree...

			if (norm(objToVisionRay) < object->getRadius()) {
				// Thus we return the distance to the intersection. This is not exactly the point on the tree,
				// but rather the distance to the point on our visionRay that comes closest to the center of the object.
				return norm(objLocProjOntoVisionRay);
			}
		}
	}

	// If we failed to intersect with any tree, return a no-intersect value.
	return -1;
}

void Agent::sortByDistance(vector<WorldObject*> worldObjects) {
	// Sorts the worldObjects by closest objects to our location.
	sort(worldObjects.begin(), worldObjects.end(),
		[&](const WorldObject* a, const WorldObject* b) -> bool
	{
		return norm(a->getLocation() - location) < norm(b->getLocation() - location);
	});
}
