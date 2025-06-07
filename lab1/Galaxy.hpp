#pragma once
#define _USE_MATH_DEFINES
#include "Mover.hpp"
#include <math.h>
#include <random>
#include <mutex>

struct Field {
	cyclone::Vector3 position;
	float mass;
};

class Node {
public:
	Node(cyclone::Vector3, cyclone::Vector3);
	void insert(cyclone::Vector3 pos, double mass);
	void create_children();
	void draw(bool recurse) const;
	std::shared_ptr<Node> children[8];
	double mass = 0.0;
	cyclone::Vector3 bound1;
	cyclone::Vector3 bound2;
};

class Galaxy {
public:
	Galaxy(int, float, float);
	~Galaxy();
	Galaxy(Galaxy const&) = delete;
	void operator=(Galaxy const& x) = delete;

	void addParticle(Mover &particle);
	void draw();
	void createGalaxyDisk(int numParticlesPerGalaxy, float galaxyRadius);
	void createGalaxyField(int recursions, float galaxyRadius, bool even, cyclone::Vector3 center);
	void remapBarnesHutTree();
	void computeFieldMass();
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
	std::shared_ptr<std::vector<Field>> fields;
	float simulation_radius;
	std::shared_ptr<Node> root;
	std::vector<std::thread> threads;
	std::shared_ptr<int> thread_counter;
	std::vector<std::tuple<int, int>> thread_ranges; // (start, end) for each thread
	std::vector<std::shared_ptr<std::mutex>> outputs_mutexes; // 1 per thread
	std::vector<std::shared_ptr<std::vector<cyclone::Vector3>>> outputs_forces; // 1 per thread
	std::shared_ptr<std::mutex> first_job_starter;
	std::shared_ptr<std::mutex> second_job_starter;
};
cyclone::Vector3 gravityForceForParticleBarnesHut(
	const cyclone::Vector3& particle_position,
	std::shared_ptr<Node> root,
	float theta
);