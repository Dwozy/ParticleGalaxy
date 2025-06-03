#pragma once
#define _USE_MATH_DEFINES
#include "Mover.hpp"
#include <math.h>
#include <random>

typedef std::shared_ptr<Mover> MoverPtr;

class Galaxy {
public:
	Galaxy() = default;
	MoverPtr addParticle(const MoverPtr& particle);
	void draw() const;
	void createGalaxies(int numParticlesPerGalaxy, float galaxyRadius);
	void setBaseVelocity(cyclone::Vector3 center, float scale);
	void update(float duration);
private:
	cyclone::Vector3 setDiskRotationVelocity(
		const cyclone::Vector3& center,
		const cyclone::Vector3& position,
		float velocityScale
	) const;
	cyclone::Vector3 gravityForceForParticle(MoverPtr p);
private:
	std::vector<std::shared_ptr<Mover>> particles;
};