#include "Mover.hpp"
#include "MyGlWindow.h"
#include <iostream>
#include <particle.h>
#include <pfgen.h>
#include <random.h>

void Mover::draw() const {
	glLoadName(this->id);
	glColor3f(0.5f, 0.5f, 1.0f);

	cyclone::Vector3 position = this->m_particle->getPosition();

	glPushMatrix();
	glTranslated(position.x, position.y, position.z);
	glutSolidSphere(this->size, 30, 30);
	glPopMatrix();
	glLoadName(0);
}

void Mover::update(float duration, cyclone::Vector3 gravity) {
	if (this->is_selected) {
		return;
	}

	std::cout << "gravity for mover " << this->id << " is " << gravity.toString() << std::endl;

	/*auto m_gravity = new cyclone::ParticleGravity(gravity);
	m_forces->clear();
	m_forces->add(m_particle, m_gravity);
	m_forces->updateForces(duration);*/
	this->m_particle->addForce(gravity);
	this->m_particle->integrate(duration);
}