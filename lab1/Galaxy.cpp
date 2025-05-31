#include <Galaxy.hpp>

MoverPtr Galaxy::addParticle(const MoverPtr& particle) {
	particle->id = this->particles.size();
	this->particles.push_back(particle);
	return particle;
}

void Galaxy::setBaseVelocity(cyclone::Vector3 center, float scale) {
	for (const auto& particle : this->particles) {
		particle->m_particle->setVelocity(
			setDiskRotationVelocity(center, particle->m_particle->getPosition(), scale)
		);
	}
}

void Galaxy::draw() const {
	for (const auto& particle : this->particles) {
		particle->draw();
	}
}

void Galaxy::update(float duration) {
	for (const auto& particle : this->particles) {
		particle->update(duration, this->gravityForceForParticle(particle));
	}
}


// Sets the base velocity for a particle to rotate on a disk around the given center.
// The velocity is tangent to the circle defined by the center and the particle's position.
// The farther the particle from the center, the larger the velocity (proportional to distance).
cyclone::Vector3 Galaxy::setDiskRotationVelocity(
	const cyclone::Vector3& center,
	const cyclone::Vector3& position,
	float velocityScale
) const {
	// Vector from center to position
	cyclone::Vector3 r = position - center;
	// If the particle is at the center, velocity is zero
	if (r.magnitude() == 0.0f) {
		return cyclone::Vector3(0, 0, 0);
	}

	cyclone::Vector3 up(0, 1, 0);
	// Tangent direction is cross(up, r)
	cyclone::Vector3 tangent = up.cross(r);
	tangent.normalise();
	// Velocity magnitude proportional to distance from center
	float speed = r.magnitude() * velocityScale;
	return tangent * speed;
}

cyclone::Vector3 Galaxy::gravityForceForParticle(MoverPtr p)
{
    // Gravitational constant (arbitrary units)
    const float G = 1.0f;

	// original particle data
	const auto p_pos = p->m_particle->getPosition();
	const auto p_mass = p->mass;

    cyclone::Vector3 totalForce(0, 0, 0);

    for (const auto& particle : this->particles) {
        cyclone::Vector3 pos = particle->m_particle->getPosition();
        float mass = particle->mass;

        cyclone::Vector3 distVec = pos - p_pos;
        float distanceSq = distVec.squareMagnitude();

        if (distanceSq > 1e-6f) { // Avoid division by zero or self-force
            float distance = std::sqrt(distanceSq);
            cyclone::Vector3 direction = distVec / distance;
            // F = G * m1 * m2 / distVec^2
            float forceMagnitude = G * p_mass * mass / distanceSq;
            totalForce += direction * forceMagnitude;
        }
    }

    return totalForce;
}
