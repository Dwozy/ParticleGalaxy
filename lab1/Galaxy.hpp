#pragma once
#include "Mover.hpp"

typedef std::shared_ptr<Mover> MoverPtr;

class Galaxy {
public:
	Galaxy() = default;
	MoverPtr addParticle(const MoverPtr& particle);
	void draw() const;
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