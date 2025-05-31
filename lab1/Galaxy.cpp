#include <Galaxy.hpp>

MoverPtr Galaxy::addParticle(const MoverPtr& particle) {
	particle->id = this->particles.size();
	this->particles.push_back(particle);
	return particle;
}

void Galaxy::draw() const {
	for (const auto& particle : this->particles) {
		particle->draw();
	}
}