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

void Mover::update(float duration) {
	if (this->is_selected) {
		return;
	}

	m_forces->updateForces(duration);
	m_particle->integrate(duration);
}