#pragma once
#include <iostream>
#include <particle.h>
#include <pfgen.h>

class Mover {
public:
	Mover(cyclone::Vector3 position, float mass = 1.0f, float size = 1.0f) : id(0), mass(mass), size(size) {
		m_particle = cyclone::Particle();
		m_particle.setPosition(position.x, position.y, position.z);
		m_particle.setMass(mass);
		m_particle.setDamping(1.0f); // no damping in space
	};
	~Mover() {};
	cyclone::Particle m_particle;

	float size = 1.0f;
	float mass = 1.0f;
	bool is_selected = false;
	int id;

	void draw() const;
	void update(float duration, cyclone::Vector3 gravity);
};