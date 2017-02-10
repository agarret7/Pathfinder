#ifndef TREE_H_
#define TREE_H_

#include <SFML/Graphics.hpp>

#include "WorldObject.h"

class Tree : public WorldObject {
public:
	Tree(vec, double);
	const Color getColor() const { return Color::Green; }
};

#endif
