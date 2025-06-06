#include <Galaxy.hpp>
#include <thread>
#include <mutex>
#include <tuple>

void thread_function() {
	while (true) {
		
	}
}


Galaxy::Galaxy(int particles_nb, float radius, float base_velocity_scale) : particles(std::make_shared<std::vector<Mover>>())
{
	//// Debug part
	this->addParticle(Mover(cyclone::Vector3(0, 0, 0), 3000, 0.5)); // massive center particle
	particles_nb += 1;

	//// Threads initialization
	this->job_starter = std::make_shared<std::mutex>();
	this->job_starter->lock();
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
		this->threads.push_back(std::thread());
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
	for (const auto& particle : *this->particles) {
		particle.draw();
	}
}

void Galaxy::update(float duration) {
	// for stability despite lag :
	duration = 1.0 / 240.0; // 60 FPS

	const int numThreads = std::thread::hardware_concurrency();
	const int numParticles = static_cast<int>(particles->size());
	const int blockSize = (numParticles + numThreads - 1) / numThreads;

	std::vector<std::thread> threads;

	for (int t = 0; t < numThreads; ++t) {
		int start = t * blockSize;
		int end = std::min(start + blockSize, numParticles);

		if (start >= end) break;

		threads.emplace_back([this, start, end, duration]() {
			for (int i = start; i < end; ++i) {
				Mover& particle = (*particles)[i];
				cyclone::Vector3 force = this->gravityForceForParticle(particle);
				particle.update(duration, force);
			}
			});
	}

	for (auto& thread : threads) {
		thread.join();
	}
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

cyclone::Vector3 Galaxy::gravityForceForParticle(Mover& p)
{
	// Gravitational constant (arbitrary units)
	const float G = 30;
	const float smooth_force = 2.0f;

	// original particle data
	const auto p_pos = p.m_particle.getPosition();
	const auto p_mass = p.mass;

	cyclone::Vector3 totalForce(0, 0, 0);

	for (const auto& particle : *this->particles) {
		if (particle.id == p.id) continue;
		cyclone::Vector3 pos = particle.m_particle.getPosition();
		float mass = particle.mass;

		cyclone::Vector3 distVec = pos - p_pos;
		float distanceSq = distVec.squareMagnitude();

		if (distanceSq > 1e-6f) { // Avoid division by zero or self-force
			float distance = std::sqrt(distanceSq);
			cyclone::Vector3 direction = distVec / distance;
			// F = G * m1 * m2 / distVec^2
			float forceMagnitude = G * p_mass * mass / (distanceSq + smooth_force);
			totalForce += direction * forceMagnitude;
		}
	}

	return totalForce;
}
