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
	
	// set the number of particles
	num_particles=100;
    
	default_random_engine gen;
    
	// this line creates a normal (Gaussian) distribution for x,y and theta.
	normal_distribution<double> dist_x(x,std[0]);
	normal_distribution<double> dist_y(y,std[1]);
	normal_distribution<double> dist_theta(theta,std[2]);
    
	Particle point;
    
	// Init all the particles
	for (int i=0;i<num_particles;i++)
	{
		point.id=i;
		point.weight=1.0;
        
		// add random Gaussian noise to each particle
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
    
	// this line creates a normal (Gaussian) distribution for x,y and theta.
	normal_distribution<double> pos_x(0,std_pos[0]);
	normal_distribution<double> pos_y(0,std_pos[1]);
	normal_distribution<double> pos_theta(0,std_pos[2]);
    
	for (int i=0;i<num_particles;i++){
		if (fabs(yaw_rate) < 0.001){
			particles[i].x += velocity * delta_t*cos(particles[i].theta);
			particles[i].y += velocity * delta_t*sin(particles[i].theta);
		}
		else{
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
	
	for (int i = 0; i < observations.size();i++){
		int temp_id = -1;
		double min_dist = numeric_limits<double>::max();

		for (int j=0;j<predicted.size();j++){
			double distance = dist(observations[i].x, observations[i].y,predicted[j].x,predicted[j].y);

			if (distance < min_dist)
			{
				temp_id = predicted[j].id;
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

    // update 
	for (int i =0;i<particles.size();i++){
		vector<LandmarkObs> transformed_obs;
		particles[i].weight = 1.0;

		for (LandmarkObs observs : observations) {
            LandmarkObs transformed_ob;
            transformed_ob.id = observs.id;
            transformed_ob.x = observs.x * cos(particles[i].theta) - observs.y * sin(particles[i].theta) + particles[i].x;
            transformed_ob.y = observs.x * sin(particles[i].theta) + observs.y * cos(particles[i].theta) + particles[i].y;
            transformed_obs.push_back(transformed_ob);

            cout<<"transformed_ob.x = "<<transformed_ob.x<<endl;
			cout<<"transformed_ob.y = "<<transformed_ob.y<<endl;
			cout<<"transformed_ob.id = "<<transformed_ob.id<<endl;

			// calculate distance
			vector <double> distances;
			for (int k =0; k<map_landmarks.landmark_list.size();k++){
                float range_landmark_x = map_landmarks.landmark_list[k].x_f;
                float range_landmark_y = map_landmarks.landmark_list[k].y_f;

				double distance = dist(range_landmark_x,range_landmark_y,transformed_ob.x,transformed_ob.y);
				distances.push_back(distance);
			}

			// find the closest landmark id
            vector<double>::iterator result = min_element(begin(distances), end(distances));
            Map::single_landmark_s range_map = map_landmarks.landmark_list[distance(begin(distances), result)];
            transformed_ob.id = range_map.id_i;
            
            // Update weights
            double std_x = std_landmark[0];
			double std_y = std_landmark[1];
            double uppart = 1./(std_x*std_y*M_PI*2);
            cout << "uppart="<<uppart<<endl;;
            double dwpart = -1*(pow((transformed_ob.x-range_map.x_f),2)/(2*pow(std_x,2)) + pow((transformed_ob.y-range_map.y_f),2)/(2*pow(std_y,2)));
            cout <<"dwpart="<<dwpart<<endl;
			//multiple the weight
		    particles[i].weight *= uppart *exp(dwpart);

		    cout << "particles[i].weight" << particles[i].weight<<endl;
		}
	}


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	vector<Particle> re_particles;
	default_random_engine gen;
	default_random_engine real_eng;
	uniform_int_distribution<> intdisc{0,num_particles-1};
	int index = intdisc(gen);
	double bite = 0.0;

    double weight_max =0;
	for (int i=0;i<particles.size();i++){
		if(weight_max < particles[i].weight)
			weight_max = particles[i].weight;
	}


    uniform_real_distribution<double> w_unifor(0,2*weight_max);

	for(int j=0;j<num_particles;j++){
		bite += w_unifor(real_eng);
		while (particles[index].weight<bite){
			bite -= particles[index].weight;
			index = (index + 1) % num_particles;
		}
		re_particles.push_back(particles[index]);

	}
	particles=re_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
