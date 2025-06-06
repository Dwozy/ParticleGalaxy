#pragma once
#define _USE_MATH_DEFINES
#include "Mover.hpp"
#include <math.h>
#include <random>

class Galaxy {
public:
	Galaxy();
	void addParticle(Mover &particle);
	void draw() const;
	void createGalaxyDisk(int numParticlesPerGalaxy, float galaxyRadius);
	void setBaseVelocity(cyclone::Vector3 center, float scale);
	void update(float duration);
private:
	cyclone::Vector3 setDiskRotationVelocity(
		const cyclone::Vector3& center,
		const cyclone::Vector3& position,
		float velocityScale
	) const;
	cyclone::Vector3 gravityForceForParticle(Mover& p);
private:
	std::shared_ptr<std::vector<Mover>> particles;
};