#pragma once
#define _USE_MATH_DEFINES
#include "Mover.hpp"
#include <math.h>
#include <random>
#include <mutex>

class Galaxy {
public:
	Galaxy(int, float, float);
	~Galaxy();
	Galaxy(Galaxy const&) = delete;
	void operator=(Galaxy const& x) = delete;

	void addParticle(Mover &particle);
	void draw() const;
	void createGalaxyDisk(int numParticlesPerGalaxy, float galaxyRadius);
	void setBaseVelocity(cyclone::Vector3 center, float scale);
	void update(float duration);
private:
	cyclone::Vector3 setDiskRotationVelocity(
		const cyclone::Vector3& center,
		const cyclone::Vector3& position,
		float velocityScale
	) const;
private:
	std::shared_ptr<std::vector<Mover>> particles;
	std::vector<std::thread> threads;
	std::vector<std::tuple<int, int>> thread_ranges; // (start, end) for each thread
	std::vector<std::shared_ptr<std::mutex>> outputs_mutexes; // 1 per thread
	std::vector<std::shared_ptr<std::vector<cyclone::Vector3>>> outputs_forces; // 1 per thread
	std::shared_ptr<std::mutex> first_job_starter;
	std::shared_ptr<std::mutex> second_job_starter;
};
cyclone::Vector3 gravityForceForParticle(int p_index, std::shared_ptr<std::vector<Mover>> particles);