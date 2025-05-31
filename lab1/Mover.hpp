#pragma once
#include <iostream>
#include <particle.h>
#include <pfgen.h>
#include <DrawUtils.H>

class Mover {
public:
	Mover(cyclone::Vector3 position, float mass = 1.0f) : id(0), mass(mass) {
		m_particle = new cyclone::Particle();
		m_particle->setPosition(position.x, position.y, position.z);
		m_particle->setMass(mass);
		m_particle->setDamping(1.0f); // no damping in space

		auto m_gravity = new cyclone::ParticleGravity(cyclone::Vector3(0, -9, 0));
		m_forces = new cyclone::ParticleForceRegistry();  //Container
		m_forces->add(m_particle, m_gravity);
	};
	~Mover() {};
	cyclone::ParticleForceRegistry* m_forces;
	cyclone::Particle* m_particle;

	float size = 1.0f;
	float mass = 1.0f;
	bool is_selected = false;
	int id;

	void draw() const;
	void update(float duration);
};