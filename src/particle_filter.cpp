/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set the number of particles
	num_particles = 100;

	default_random_engine gen;
	//double std_x, std_y, std_theta; // Standard deviations for x,y, and theta

	// This line creates a normal (Gaussian) distribution for x,y and theta.
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	Particle point;

	// Init all the particles
	for (int i = 0; i < num_particles; i++) {
		point.id = i;
		//point.x = x;
		//point.y = y;
		//point.theta = theta;
		point.weight = 1.0;

		//// Add random Gaussian noise to each particle
		//point.x += dist_x(gen);
		//point.y += dist_y(gen);
		//point.theta += dist_theta(gen);

		point.x = dist_x(gen);
		point.y = dist_y(gen);
		point.theta = dist_theta(gen);

		particles.push_back(point);
		weights.push_back(point.weight);
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	static unsigned fulseed = 0;
	++fulseed;
	default_random_engine gfun(fulseed);
	// This line creates a normal (Gaussian) distribution for x,y and theta.
	normal_distribution<double> pos_x(0, std_pos[0]);
	normal_distribution<double> pos_y(0, std_pos[1]);
	normal_distribution<double> pos_theta(0, std_pos[2]);

	for (int i = 0; i < num_particles; i++) {
		if (fabs(yaw_rate) < 0.001) {
			particles[i].x += velocity * delta_t*cos(particles[i].theta);
			particles[i].y += velocity * delta_t*sin(particles[i].theta);
		}
		else {
			particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		// Add random Gaussian noise to each particle
		particles[i].x += pos_x(gfun);
		particles[i].y += pos_y(gfun);
		particles[i].theta += pos_theta(gfun);

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	for (int i = 0; i < observations.size(); i++) {
		int temp_id = -1;
		double min_dist = numeric_limits<double>::max();

		for (int j= 0;j < predicted.size(); j++) {
			double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

			if (distance < min_dist)
			{
				temp_id = j;
				min_dist = distance;
			}
		}
		observations[i].id = temp_id;
	}


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	//double total = 0.0;

	//for (int i = 0; i < num_particles; i++) {

	//	// for each particle get x,y and theta
	//	double par_x = particles[i].x;
	//	double par_y = particles[i].y;
	//	double par_theta = particles[i].theta;

	//	vector<LandmarkObs> transformed_obs;
	//	vector<LandmarkObs> range_landmarks;

	//	// transform coordinates of landmarks from VEHICLES's coordinate system to MAP's coordinate 
	//	// system
	//	
	//	for (int j = 0; j < observations.size(); j++) {

	//		LandmarkObs transformed_ob;

	//		double tran_x = cos(par_theta)*observations[j].x - sin(par_theta)*observations[j].y + par_x;
	//		double tran_y = sin(par_theta)*observations[j].x + cos(par_theta)*observations[j].y + par_y;
	//		
	//		//transformed_ob.id = observations[j].id;
	//		transformed_ob.x = tran_x;
	//		transformed_ob.y = tran_y;

	//		//cout << "observations[j].id =" << observations[j].id << endl;
	//		//cout << "transformed_ob.id =" << transformed_ob.id << endl;
	//		//cout << "transformed_ob.x =" << transformed_ob.x << endl;

	//		//add transform to vector
	//		transformed_obs.push_back(transformed_ob);
	//	}

	//	// choose landmarks within sensor range
	//	for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
	//		int range_landmark_id = map_landmarks.landmark_list[k].id_i;
	//		float  range_landmark_x = map_landmarks.landmark_list[k].x_f;
	//		float  range_landmark_y = map_landmarks.landmark_list[k].y_f;
 //           
	//		// calculate the distanc using dist()
	//		float range_dist = dist(range_landmark_x, range_landmark_y, par_x, par_y);

	//		if (range_dist < sensor_range) {
	//			LandmarkObs range_landmark;
	//			range_landmark.id = range_landmark_id;
	//			range_landmark.x = range_landmark_x;
	//			range_landmark.y = range_landmark_y;

	//			//cout << "range_landmark.id =" << range_landmark.id << endl;

	//			range_landmarks.push_back(range_landmark);
	//		}
	//	}

	//	dataAssociation(range_landmarks, transformed_obs);

	//	//init weight
	//	particles[i].weight = 1.0;

	//	//cout << "transformed_obs.size()=" << transformed_obs.size() << endl;
	//	
	//	for (int l = 0; l < transformed_obs.size(); l++) {
	//		int associate_id = transformed_obs[l].id;
	//		//double trans_x = 0.0, trans_y = 0.0;
	//		double trans_x = transformed_obs[l].x;
	//		double trans_y = transformed_obs[l].y;

	//		particles[i].associations.push_back(range_landmarks[associate_id].id);
	//		particles[i].sense_x.push_back(trans_x);
	//		particles[i].sense_y.push_back(trans_y);

	//		// get the range landmarks x,y associate with current observcation
	//		for (int m = 0; m < range_landmarks.size(); m++) {
	//			if (range_landmarks[m].id == associate_id)
	//			{
	//				double assos_x = range_landmarks[m].x;
	//				double assos_y = range_landmarks[m].y;

	//				double std_x = std_landmark[0];
	//				double std_y = std_landmark[1];

	//				

	//				double Update_weight = (1 / (2 * std_x* std_y*M_PI))*exp(-(pow(assos_x - trans_x, 2) /
	//					(2 * pow(std_x, 2)) + (pow(assos_y - trans_y, 2) / (2 * pow(std_y, 2)))));


	//				//double x_diff = (trans_x - assos_x)*(trans_x - assos_x) / (2 * std_x*std_x);
	//				//double y_diff = (trans_y - assos_y)*(trans_y - assos_y) / (2 * std_y*std_y);
	//				//double Update_weight = 1 / (2 * M_PI*std_x*std_y)*exp(-(x_diff + y_diff));

	//				cout << "std_x=" << std_x<<endl;
	//				cout << "std_y=" << std_y << endl;
	//				cout << "assos_x=" << assos_x << endl;
	//				cout << "assos_y=" << assos_y << endl;
	//				cout << "trans_x=" << trans_x << endl;
	//				cout << "trans_y=" << trans_y << endl;

	//				//cout << "Update_weight=" << Update_weight << endl;

	//				//multiply the weight
	//				particles[i].weight *= Update_weight;
	//				//weights[i] *= Update_weight;
	//			}
	//		}

	//	}

	//	total += particles[i].weight;
	//	//cout << "total=" << total << endl;
	//}

	//for (int n = 0; n < particles.size(); n++) {
	//	weights[n] = particles[n].weight / total;
	//	particles[n].weight = weights[n];

	//	//cout << "weights[n]=" << n<<weights[n] << endl;
	//}

	// get map coordinates for particles
for (int i = 0; i < particles.size(); i++) {
	std::vector<LandmarkObs> observations_map;
	particles[i].weight = 1;
	for (LandmarkObs observs : observations) {
		LandmarkObs obs_t;
		obs_t.id = observs.id;
		obs_t.x = observs.x * cos(particles[i].theta) - observs.y * sin(particles[i].theta) + particles[i].x;
		obs_t.y = observs.x * sin(particles[i].theta) + observs.y * cos(particles[i].theta) + particles[i].y;
		observations_map.push_back(obs_t);

		// calculate distance to each landmark
		vector <double> distances;
		for (Map::single_landmark_s landmks : map_landmarks.landmark_list) {
			double distance = dist(landmks.x_f, landmks.y_f, obs_t.x, obs_t.y);
			distances.push_back(distance);
		}

		// find the id of the closest landmark
		vector<double>::iterator result = min_element(begin(distances), end(distances));
		Map::single_landmark_s lm = map_landmarks.landmark_list[distance(begin(distances), result)];
		obs_t.id = lm.id_i;

		// calculate probability
		double normalizer = 1. / (2 * M_PI*std_landmark[0] * std_landmark[1]);
		double power = -1 * (pow((obs_t.x - lm.x_f), 2) / (2 * pow(std_landmark[0], 2)) + pow((obs_t.y - lm.y_f), 2) / (2 * pow(std_landmark[1], 2)));

		
		particles[i].weight *= normalizer * exp(power);

		cout << "update_weight=" << particles[i].weight << endl;
	}
}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> re_particles;
	static unsigned seed = 0;
	seed++;
	default_random_engine gen(seed);
	default_random_engine real_eng;

	//// Update the current weights
	//for (int i = 0; i < num_particles; i++) {
	//	weights.push_back(particles[i].weight);
	//}

	double bite = 0.0;
	discrete_distribution<int> intdisc(0, num_particles-1);
	int index = intdisc(gen);

	double weight_max = 0.0;
	//double weight_max = *max_element(weights.begin(), weights.end());
	for (int o = 0; o < particles.size(); o++) {
		//cout << "particles.size()" << particles.size() << endl;
		if (weight_max < particles[o].weight)
		{
			weight_max = particles[o].weight;
		}
	}

	uniform_real_distribution<double> w_unifor(0.0, 2.0*weight_max);

	for (int j = 0; j < num_particles; j++) {
		bite += w_unifor(gen);
		//bite += w_unifor(real_eng)*2.0*weight_max;
		//cout << "bite=" << bite << endl;
		//bite += (intdisc(gen) + 1) * 2 * weight_max / num_particles;
		while (weights[index] < bite)
		{
			bite -= weights[index];
			index = (index + 1) % num_particles;

			//cout << "index=" << index << endl;
		}
		re_particles.push_back(particles[index]);
	}

	particles = re_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
