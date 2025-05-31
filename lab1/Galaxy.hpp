#pragma once
#include "Mover.hpp"

typedef std::shared_ptr<Mover> MoverPtr;

class Galaxy {
public:
	Galaxy() = default;
	MoverPtr addParticle(const MoverPtr& particle);
	void draw() const;
private:
	std::vector<std::shared_ptr<Mover>> particles;
};