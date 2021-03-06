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

using namespace std;
//define a random value to inti particles and to init sensor noise
default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	if(!is_initialized)
    {  
		// Set the number of particales, update the number of particals if the run time is too long
		num_particles = 1000;
  
		//Define normal distribution for GPS x,y and theta
		normal_distribution<double> dist_x(x, std[0]);
		normal_distribution<double> dist_y(y, std[1]);
		normal_distribution<double> dist_theta(theta, std[2]);
  
		//Init all the particals
		for(unsigned int i=0; i<num_particles;i++)
    	{
			particles[i].id = i;
			particles[i].x = dist_x(gen);
			particles[i].y = dist_y(gen);
			particles[i].theta = dist_theta(gen);
			particles[i].weight = 1;     
			cout << "Particles " << i + 1 << " " << particles[i].x << " " << particles[i].y << " " << particles[i].theta << particles[i].weight << endl;
		}        
		is_initialized = 1;
	} 
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
	//Define normal distribution for GPS x,y and theta, for noise case, the mean value should be 0
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);
  
	//Add measurements to each particle
	for(unsigned int i=0; i<num_particles; i++)
    {
      particles[i].x = particles[i].x + velocity/yaw_rate*(sin(particles[i].theta + yaw_rate*delta_t)- sin(particles[i].theta));
      particles[i].y = particles[i].y + velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta = particles[i].theta + yaw_rate*delta_t;
      
      //Add noise to the measurement
      particles[i].x += dist_x(gen);
      particles[i].y += dist_x(gen);
      particles[i].theta += dist_x(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for(unsigned i=0; i<observations.size(); i++)
    {
    	double minDist = numeric_limits<double>::max();
    	for(unsigned j=0; j<predicted.size(); j++)
        {
        	double distance = dist(observations[j].x,observations[j].y,predicted[j].x,predicted[j].y);
        	if(distance<minDist)
            {
            	minDist = distance;
            	observations[j].id = predicted[j].id;
            }
        }
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
	vector<LandmarkObs> observations_map;
	vector<LandmarkObs> landmarkobj_inrange;
 	for(unsigned int i=0; i<num_particles; i++)
    {
		//Step 1:Filter out the landmarks which are in the sensor range
		for(unsigned int k; k<map_landmarks.landmark_list.size(); k++)
        {
        	LandmarkObs filter_range;
			if(dist(particles[i].x,particles[i].y,map_landmarks.landmark_list[k].x_f,map_landmarks.landmark_list[k].y_f) < sensor_range)
            {
            	filter_range.x = map_landmarks.landmark_list[k].x_f;
            	filter_range.y = map_landmarks.landmark_list[k].y_f;
				filter_range.id = map_landmarks.landmark_list[k].id_i;
            	landmarkobj_inrange.push_back(filter_range);
            }
        }
		//Step 2:Transform coordinate from vehicle to map
		for(unsigned int j=0; j<observations.size(); j++)
        {
			LandmarkObs coordi_transfer;
        	coordi_transfer.x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
			coordi_transfer.y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
			coordi_transfer.id = -1;
			observations_map.push_back(coordi_transfer);
        }
		//Step 3:Associate transformed observations with landmarks
		dataAssociation(particles, observations);
		//Step 4:Update weights
		
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
