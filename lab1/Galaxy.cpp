#include <Galaxy.hpp>
#include <thread>
#include <mutex>
#include <tuple>
#include "MyGlWindow.h"

constexpr float DURATION = 1.0 / 480.0;

cyclone::Vector3 gravityForceForParticle(cyclone::Vector3 particle_position, std::shared_ptr<std::vector<Field>> fields)
{
	// Gravitational constant (arbitrary units)
	const float G = 20;
	const float smooth_force = 2.0f;
	cyclone::Vector3 totalForce(0, 0, 0);

	for (const auto& field : *fields) {
		cyclone::Vector3 distVec = field.position - particle_position;
		float distanceSq = distVec.squareMagnitude();

		if (distanceSq > 1e-6f) { // Avoid division by zero or self-force
			float distance = std::sqrt(distanceSq);
			cyclone::Vector3 direction = distVec / distance;
			// F = G * m_other / distVec^2
			float forceMagnitude = G * field.mass / (distanceSq + smooth_force);
			totalForce += direction * forceMagnitude;
		}
	}
	return totalForce;
}

void thread_function(
	std::shared_ptr<std::mutex> first_job_starter,
	std::shared_ptr<std::mutex> second_job_starter,
	std::shared_ptr<int> thread_counter,
	std::tuple<int, int> range,
	std::shared_ptr<std::vector<cyclone::Vector3>> forces,
	std::shared_ptr<std::mutex> output_mutex,
	std::shared_ptr<std::vector<Mover>> particles,
	std::shared_ptr<std::vector<Field>> fields
) {
	const int start = std::get<0>(range);
	const int end = std::get<1>(range);
	//std::cout << "thread start" << std::endl;

	while (true) {
		first_job_starter->lock(); // wait for the start signal
		output_mutex->lock(); // lock the output mutex
		*thread_counter += 1;
		first_job_starter->unlock();

		for (int i = start; i < end; i++) {
			//std::cout << "computing particle " << i << std::endl;
			(*forces)[i - start] = gravityForceForParticle((*particles)[i - start].m_particle.getPosition(), fields); // store the computed force
		}
		output_mutex->unlock(); // signal that this thread finished

		second_job_starter->lock(); // wait for the start signal
		output_mutex->lock(); // lock the output mutex
		*thread_counter += 1;
		second_job_starter->unlock();
		for (int i = start; i < end; i++) {
			//(*particles)[i].m_particle.addForce((*forces)[i - start]);
			//std::cout << "integrating particle " << i << std::endl;
			//(*particles)[i].m_particle.integrate(DURATION);
			(*particles)[i].m_particle.setVelocity(
				(*particles)[i].m_particle.getVelocity() + (*forces)[i - start] * DURATION
			);
			(*particles)[i].m_particle.setPosition(
				(*particles)[i].m_particle.getPosition() + (*particles)[i].m_particle.getVelocity() * DURATION
			);
		}

		output_mutex->unlock(); // signal that this thread finished
	}
}


Galaxy::Galaxy(int particles_nb, float radius, float base_velocity_scale):
	particles(std::make_shared<std::vector<Mover>>()), fields(std::make_shared<std::vector<Field>>())
{
	//// Debug part
	//this->addParticle(Mover(cyclone::Vector3(0, 0, 0), particles_nb / 3, 0.5)); // massive center particle
	//particles_nb += 1;

	//// Threads initialization
	this->thread_counter = std::make_shared<int>(0);
	this->first_job_starter = std::make_shared<std::mutex>();
	this->first_job_starter->lock();
	this->second_job_starter = std::make_shared<std::mutex>();

	const int NUM_THREADS = std::thread::hardware_concurrency();
	this->outputs_mutexes.resize(NUM_THREADS);
	this->thread_ranges.resize(NUM_THREADS);
	this->outputs_forces.resize(NUM_THREADS);

	for (int t = 0; t < NUM_THREADS; ++t) {
		this->outputs_mutexes[t] = std::make_shared<std::mutex>();
		this->outputs_forces[t] = std::make_shared<std::vector<cyclone::Vector3>>();
		this->thread_ranges[t] = std::make_pair(
			t * (particles_nb / NUM_THREADS),
			std::min(particles_nb, (t + 1) * (particles_nb / NUM_THREADS))
		);
		outputs_forces[t]->resize(particles_nb / NUM_THREADS, cyclone::Vector3(0, 0, 0));
		this->threads.emplace_back(std::thread([this, t] {
			thread_function(
				first_job_starter,
				second_job_starter,
				thread_counter,
				thread_ranges[t],
				outputs_forces[t],
				outputs_mutexes[t],
				particles,
				fields
			);
		}));
	}
	// sets a correct size for the last thread's output forces
	this->thread_ranges[NUM_THREADS - 1] = std::make_pair(
		(NUM_THREADS - 1) * (particles_nb / NUM_THREADS),
		particles_nb
	);
	outputs_forces[NUM_THREADS - 1]->resize(
		std::get<1>(this->thread_ranges[NUM_THREADS - 1]) - std::get<0>(this->thread_ranges[NUM_THREADS - 1]),
		cyclone::Vector3(0, 0, 0));

	//// Particle initialization
	this->createGalaxyField(64, radius, 0, cyclone::Vector3(0, 0, 0));
	this->createGalaxyDisk(particles_nb, radius);
	this->setBaseVelocity(cyclone::Vector3(0, 0, 0), base_velocity_scale);

	//// debug to check consistant values of threads ranges
	for (int i = 0; i < outputs_mutexes.size(); ++i) {
		std::cout << "thread " << i << std::endl;
		std::cout << "range: " << std::get<0>(thread_ranges[i]) << "-" << std::get<1>(thread_ranges[i]) << std::endl;
		std::cout << "size from range: " << std::get<1>(thread_ranges[i]) - std::get<0>(thread_ranges[i]) << std::endl;
		std::cout << "size from output forces: " << outputs_forces[i]->size() << std::endl;
		std::cout << std::endl;
	}
}

Galaxy::~Galaxy()
{
	for (auto& thread : this->threads) {
		thread.join();
	}
}

void Galaxy::addParticle(Mover& particle) {
	particle.id = this->particles->size();
	this->particles->push_back(particle);
}

void Galaxy::setBaseVelocity(cyclone::Vector3 center, float scale) {
	for (auto& particle : *this->particles) {
		particle.m_particle.setVelocity(
			setDiskRotationVelocity(center, particle.m_particle.getPosition(), scale)
		);
	}
}

void Galaxy::draw() const {
	glColor3f(0.5f, 0.5f, 1.0f);

	for (const auto& particle : *this->particles) {
		particle.draw();
	}
}

void Galaxy::update(float duration) {

	std::cout << "u" << std::endl;
	this->computeFieldMass();
	std::cout << "f" << std::endl;
	this->second_job_starter->lock();
	*this->thread_counter = 0;
	this->first_job_starter->unlock(); // signal threads to start computing forces
	while (*this->thread_counter != this->threads.size()) {
		//std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	for (auto& mutex : this->outputs_mutexes) {
		mutex->lock(); // wait for threads to finish
		mutex->unlock();
	}
	std::cout << "1" << std::endl;
	this->first_job_starter->lock();
	*this->thread_counter = 0;
	this->second_job_starter->unlock(); // signal threads to start updating particles
	while (*this->thread_counter != this->threads.size()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	for (auto& mutex : this->outputs_mutexes) {
		mutex->lock(); // wait for threads to finish
		mutex->unlock();
	}
	std::cout << "2" << std::endl;
}


float randomFloat(float min, float max) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<float> dis(min, max);

	return dis(gen);
}

void Galaxy::createGalaxyDisk(int numParticlesPerGalaxy, float galaxyRadius) {
	for (int i = 0; i < numParticlesPerGalaxy; ++i) {
		float angle = randomFloat(0.0f, 2.0f * M_PI);
		float radius = std::sqrt(randomFloat(0.0f, 1.0f)) * galaxyRadius;
		float height = randomFloat(-0.05f * galaxyRadius, 0.05f * galaxyRadius);
		cyclone::Vector3 position(
			radius * std::cos(angle),
			height,
			radius * std::sin(angle)
		);
		this->addParticle(Mover(position, 1, 0.2));
	}
}

void Galaxy::createGalaxyField(int recursions, float radius, bool even, cyclone::Vector3 center) {
	for (int r = 1; r <= recursions; ++r) {
        // Use (1 - sqrt(1 - r / recursions)) to bias density toward the center
        float f = 1.0f - std::sqrt(1.0f - r / (float)recursions);
		if (even) f = r / (float)recursions;
        std::cout << "placing points up to " << radius * f << " at recursion " << r << " with math " << f << std::endl;
		this->fields->push_back(Field{ center + cyclone::Vector3{ f * radius , 0, f * radius * (r % 2) }, 0.0 });
        this->fields->push_back(Field{center + cyclone::Vector3{ -f * radius, 0, -f * radius * (r % 2) }, 0.0});
        this->fields->push_back(Field{center + cyclone::Vector3{ -f * radius * (r % 2), 0, f * radius }, 0.0});
        this->fields->push_back(Field{center + cyclone::Vector3{ f * radius * (r % 2), 0, -f * radius }, 0.0});
	}
}

void Galaxy::computeFieldMass() {
	for (auto& field : *this->fields) {
		field.mass = 0.0f;
	}
	for (auto& particle : *this->particles) {
		float lowest_distance = -1.0f;
		int nearest_field_index = -1;
		for (int f = 0; f < this->fields->size(); f++) {
			cyclone::Vector3 distVec = particle.m_particle.getPosition() - (*this->fields)[f].position;
			float distanceSq = distVec.squareMagnitude();

			if (distanceSq < lowest_distance || nearest_field_index < 0) {
				lowest_distance = distanceSq;
				nearest_field_index = f;
			}
		}
		(*this->fields)[nearest_field_index].mass += particle.mass;
	}
	for (auto& field : *this->fields) {
		std::cout << "field at " << field.position.toString() << " has mass " << field.mass << std::endl;
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
