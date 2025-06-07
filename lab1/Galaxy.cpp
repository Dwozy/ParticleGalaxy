#include <Galaxy.hpp>
#include <thread>
#include <mutex>
#include <tuple>
#include <functional>
#include "MyGlWindow.h"

constexpr float DURATION = 1.0 / 480.0;
constexpr bool DEBUG = false;
constexpr bool BH_DEBUG = false;

// Barnes-Hut gravity calculation using Node tree
cyclone::Vector3 gravityForceForParticleBarnesHut(
	const cyclone::Vector3& particle_position,
	std::shared_ptr<Node> root,
	float theta,
	bool debug
) {
	const float G = 50.0f;
	const float smooth_force = 2.0f;

	std::function<cyclone::Vector3(const std::shared_ptr<Node>)> computeForce;
	computeForce = [&](const std::shared_ptr<Node> node) -> cyclone::Vector3 {
		if (!node || node->mass == 0.0f) return cyclone::Vector3(0, 0, 0);

		cyclone::Vector3 force(0, 0, 0);
		// distance between the node's center and the particle
		cyclone::Vector3 distVec = node->bound1 + (node->bound2 - node->bound1) / 2.0 - particle_position;
		float distanceSq = distVec.squareMagnitude();

		// Size of the node (max side length)
		float size = std::max({
			std::abs(node->bound2.x - node->bound1.x),
			std::abs(node->bound2.y - node->bound1.y),
			std::abs(node->bound2.z - node->bound1.z)
			});

		// If node is an extremity (leaf)
		bool isLeaf = true;
		for (const auto& child : node->children) {
			if (child && child->mass > 0) {
				isLeaf = false;
				break;
			}
		}

		if (isLeaf || (size / (std::sqrt(distanceSq) + 1e-6f)) < theta) {
			if (debug) {
				node->draw(false);
				glLineWidth(0.005f);
				glBegin(GL_LINES);
				glColor3f(0, 0, 1);
				auto node_pos = node->bound1 + (node->bound2 - node->bound1) / 2.0;
				glVertex3f(particle_position.x, particle_position.y, particle_position.z);
				glVertex3f(node_pos.x, node_pos.y, node_pos.z);
				glEnd();
			}
			float distance = std::sqrt(distanceSq);
			cyclone::Vector3 direction = distVec / distance;
			float forceMagnitude = G * node->mass / (distanceSq + smooth_force);
			force += direction * forceMagnitude;
		} else {
			// Recursively sum force from children
			for (int i = 0; i < 8; ++i) {
				if (node->children[i]) {
					force += computeForce(node->children[i]);
				}
			}
		}
		return force;
		};

	auto force = computeForce(root);
	//std::cout << "force for this particle" << force.toString() << std::endl;
	return force;
}

void thread_function(
	std::shared_ptr<std::mutex> first_job_starter,
	std::shared_ptr<std::mutex> second_job_starter,
	std::shared_ptr<int> thread_counter,
	std::tuple<int, int> range,
	std::shared_ptr<std::vector<cyclone::Vector3>> forces,
	std::shared_ptr<std::mutex> output_mutex,
	std::shared_ptr<std::vector<Mover>> particles,
	std::shared_ptr<Node> root
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
			(*forces)[i - start] = gravityForceForParticleBarnesHut(
				(*particles)[i - start].m_particle.getPosition(),
				root,
				0.5,
				false
			); // store the computed force
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


Galaxy::Galaxy(int particles_nb, float radius, float base_velocity_scale) :
	particles(std::make_shared<std::vector<Mover>>()), fields(std::make_shared<std::vector<Field>>()),
	simulation_radius(radius * 2.0f),
	root(std::make_shared<Node>(
		cyclone::Vector3(-simulation_radius, -simulation_radius, -simulation_radius),
		cyclone::Vector3(simulation_radius, simulation_radius, simulation_radius))
	)
{
	//// Debug part
	this->addParticle(Mover(cyclone::Vector3(0, 0, 0), particles_nb / 3, 0.5)); // massive center particle
	particles_nb += 1;

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
				root
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
	//this->createGalaxyField(12, radius + 10, 1, cyclone::Vector3(0, 0, 0));
	this->remapBarnesHutTree(); // create the Barnes-Hut tree structure for gravity calculations
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

static void drawWireCube(cyclone::Vector3 bound1, cyclone::Vector3 bound2, cyclone::Vector3 color = { 1, 0, 0 }) {
	glLineWidth(0.005f);
	glBegin(GL_LINES);
	glColor3f(color.x, color.y, color.z);

	// Bottom face
	glVertex3f(bound1.x, bound1.y, bound1.z);
	glVertex3f(bound2.x, bound1.y, bound1.z);

	glVertex3f(bound2.x, bound1.y, bound1.z);
	glVertex3f(bound2.x, bound1.y, bound2.z);

	glVertex3f(bound2.x, bound1.y, bound2.z);
	glVertex3f(bound1.x, bound1.y, bound2.z);

	glVertex3f(bound1.x, bound1.y, bound2.z);
	glVertex3f(bound1.x, bound1.y, bound1.z);

	// Top face
	glVertex3f(bound1.x, bound2.y, bound1.z);
	glVertex3f(bound2.x, bound2.y, bound1.z);

	glVertex3f(bound2.x, bound2.y, bound1.z);
	glVertex3f(bound2.x, bound2.y, bound2.z);

	glVertex3f(bound2.x, bound2.y, bound2.z);
	glVertex3f(bound1.x, bound2.y, bound2.z);

	glVertex3f(bound1.x, bound2.y, bound2.z);
	glVertex3f(bound1.x, bound2.y, bound1.z);

	// Vertical edges
	glVertex3f(bound1.x, bound1.y, bound1.z);
	glVertex3f(bound1.x, bound2.y, bound1.z);

	glVertex3f(bound2.x, bound1.y, bound1.z);
	glVertex3f(bound2.x, bound2.y, bound1.z);

	glVertex3f(bound2.x, bound1.y, bound2.z);
	glVertex3f(bound2.x, bound2.y, bound2.z);

	glVertex3f(bound1.x, bound1.y, bound2.z);
	glVertex3f(bound1.x, bound2.y, bound2.z);

	glEnd();
}

void Node::draw(bool recurse) const {
	if (DEBUG || BH_DEBUG) {
		auto color = recurse ? cyclone::Vector3(1, 0, 0) : cyclone::Vector3(0, 1, 0);
		drawWireCube(this->bound1, this->bound2, color);
		if (!recurse) return;
		for (const auto& child : children) {
			if (child) child->draw(true);
		}
	}
}

void Galaxy::draw() {
	//if (DEBUG) computeFieldMass(); // ensure fields are up to date before drawing forces

	for (const auto& particle : *this->particles) {
		glColor3f(0.5f, 0.5f, 1.0f);
		particle.draw();
		if (DEBUG || BH_DEBUG) {
			auto pos = particle.m_particle.getPosition();
			auto vel = particle.m_particle.getVelocity() * DURATION;
			auto force = gravityForceForParticleBarnesHut(pos, this->root, 0.5, particle.id == 5) * DURATION;
			//glPushMatrix();
			//glTranslated(particle.m_particle.getPosition().x, particle.m_particle.getPosition().y, particle.m_particle.getPosition().z);
			if (DEBUG) {
				glLineWidth(0.005f);
				glBegin(GL_LINES);
				glColor3f(0, 1, 0);

				glVertex3f(pos.x, pos.y, pos.z);
				glVertex3f(pos.x + vel.x, pos.y + vel.y, pos.z + vel.z);
				glEnd();
				glLineWidth(0.005f);
				glBegin(GL_LINES);
				glColor3f(1, 1, 1);

				glVertex3f(pos.x, pos.y, pos.z);
				glVertex3f(pos.x + force.x, pos.y + force.y, pos.z + force.z);
				glEnd();
				//glPopMatrix();
			}
		}
	}

	if (DEBUG) {
		glColor3f(1.0f, 0.2f, 0.2f);

		/*for (const auto& field : *this->fields) {
			glPushMatrix();
			glTranslated(field.position.x, field.position.y, field.position.z);
			glutSolidCube(0.5);
			glPopMatrix();
		}*/
		this->root->draw(true);
	}
}

void Galaxy::update(float duration) {

	std::cout << "u" << std::endl;
	//this->computeFieldMass();
	this->remapBarnesHutTree();
	std::cout << "b" << std::endl;
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

Node::Node(cyclone::Vector3 bound1, cyclone::Vector3 bound2) :
	bound1(bound1), bound2(bound2), mass(0.0f), children() {
	if (bound1.x > bound2.x || bound1.y > bound2.y || bound1.z > bound2.z) {
		throw std::invalid_argument("Invalid bounding box bounds");
	}
}

void Node::create_children() {
	cyclone::Vector3 center = (bound1 + bound2) / 2.0f;
	cyclone::Vector3 min = bound1;
	cyclone::Vector3 max = bound2;

	children[0] = std::make_shared<Node>(
		cyclone::Vector3(min.x, min.y, min.z),
		cyclone::Vector3(center.x, center.y, center.z)
	);
	children[1] = std::make_shared<Node>(
		cyclone::Vector3(center.x, min.y, min.z),
		cyclone::Vector3(max.x, center.y, center.z)
	);
	children[2] = std::make_shared<Node>(
		cyclone::Vector3(min.x, center.y, min.z),
		cyclone::Vector3(center.x, max.y, center.z)
	);
	children[3] = std::make_shared<Node>(
		cyclone::Vector3(center.x, center.y, min.z),
		cyclone::Vector3(max.x, max.y, center.z)
	);
	children[4] = std::make_shared<Node>(
		cyclone::Vector3(min.x, min.y, center.z),
		cyclone::Vector3(center.x, center.y, max.z)
	);
	children[5] = std::make_shared<Node>(
		cyclone::Vector3(center.x, min.y, center.z),
		cyclone::Vector3(max.x, center.y, max.z)
	);
	children[6] = std::make_shared<Node>(
		cyclone::Vector3(min.x, center.y, center.z),
		cyclone::Vector3(center.x, max.y, max.z)
	);
	children[7] = std::make_shared<Node>(
		cyclone::Vector3(center.x, center.y, center.z),
		cyclone::Vector3(max.x, max.y, max.z)
	);
}

void Node::insert(cyclone::Vector3 position, double mass) {
	if (this->mass == 0.0f) {
		this->mass = mass;
		this->create_children();
		return;
	}

	for (auto& child : children) {
		if (position.x >= child->bound1.x && position.x <= child->bound2.x &&
			position.y >= child->bound1.y && position.y <= child->bound2.y &&
			position.z >= child->bound1.z && position.z <= child->bound2.z) {
			child->insert(position, mass);
			this->mass += mass;
			return;
		}
	}
	std::cout << "/!\\" << std::endl;
	//throw std::runtime_error("Position out of bounds in Node::insert");
}

void Galaxy::remapBarnesHutTree() {
	this->root = std::make_shared<Node>(
		cyclone::Vector3(-this->simulation_radius, -this->simulation_radius, -this->simulation_radius),
		cyclone::Vector3(this->simulation_radius, this->simulation_radius, this->simulation_radius)
	);
	for (const auto& particle : *this->particles) {
		this->root->insert(particle.m_particle.getPosition(), particle.mass);
	}
}

void Galaxy::createGalaxyField(int recursions, float radius, bool even, cyclone::Vector3 center) {
	this->fields->push_back(Field{ center, 0.0 });
	//for (int r = 1; r <= recursions; ++r) {
	//	// Use (1 - sqrt(1 - r / recursions)) to bias density toward the center
	//	float f = 1.0f - std::sqrt(1.0f - r / (float)recursions);
	//	if (even) f = r / (float)recursions;
	//	std::cout << "placing points up to " << radius * f << " at recursion " << r << " with math " << f << std::endl;
	//	this->fields->push_back(Field{ center + cyclone::Vector3{ f * radius , 0, f * radius * (r % 2) }, 0.0 });
	//	this->fields->push_back(Field{ center + cyclone::Vector3{ -f * radius, 0, -f * radius * (r % 2) }, 0.0 });
	//	this->fields->push_back(Field{ center + cyclone::Vector3{ -f * radius * (r % 2), 0, f * radius }, 0.0 });
	//	this->fields->push_back(Field{ center + cyclone::Vector3{ f * radius * (r % 2), 0, -f * radius }, 0.0 });
	//}
	/*auto bottom_left = center - cyclone::Vector3(radius, 0, radius) / 2.0;
	for (int i = 0; i < recursions + 1; ++i) {
		for (int j = 0; j < recursions + 1; ++j) {
			this->fields->push_back(Field{
				bottom_left + cyclone::Vector3(i * (radius / (float)recursions), 0, j * (radius / (float)recursions)),
				0
			});
		}
	}*/
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
