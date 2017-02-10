# Pathfinder

## Description
Genetic algorithm program to optimize path-finding solutions in a dynamic environment. The agents are initialized with a single hidden layer neural network, and take in a set of vectorized vision parameters decreasing proportionally to an increase in distance. The agents reproduce based on an exponential distribution with the most likely partners being selected from a fitness function defined to be the average distance of the agent from the goal over the round. The weights of the neural net are then randomly selected in equal proportion from the two parents, and mutated based on a rate which monotonically decreases with an increase in fitness. Further documentation about specifics can be found in the details of the functions and state variables in the different classes.

## Dependencies
- SFML (https://www.sfml-dev.org/)
- Armadillo (http://arma.sourceforge.net/)
