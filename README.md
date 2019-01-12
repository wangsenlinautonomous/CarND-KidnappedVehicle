# CarND-KidnappedVehicle
## Overview
Partical filter mainly contains the following four steps:
* Initialization
* Prediction
* WeightUpdate
* Resample

<img src="https://user-images.githubusercontent.com/40875720/51068684-ae627080-165c-11e9-81ed-da9b6f2e2830.png" width="600">

## Initialization
In this step, the following tasks need to be done:
* Init the number of partical
* Create normal distribution for the x, y position and heading
* Init id and weight for all the particals
```
Init(double x, double y, double theta, double std[])
{
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
    }        
    is_initialized = 1;
  } 
}
```

### Note
1. Number of particals
* Don't have two many particals: Two many particals will increase the work load of computer or MCU

## Prediction
In this step, each partical will precidit their next position according to Yaw rate and vehicle speed. It mainly contains the following steps:
* Prediction next step according motion model (Bicycle model instead) below pic shows
* Add noise (Normal distribution)
<img src="https://user-images.githubusercontent.com/40875720/51070148-3b1a2800-1677-11e9-81ce-ed06e50370eb.PNG" width="600">

```
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
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
```

## WeightUpdate
Weight update is used to update the weight attribute of all the particals. The next steps will use this information to filter out the particals which are far from the real obeservation. This step main contains the following steps:
* Find out the landmarks in the map which are in the sensors' range
* Transform vehicle cooridinate to map coordinate
* Associate transformed observations with landmarks
* Weight update

The following pic can increas your intuition of your idea
**Predicted measurement** means the predition coordinate based on the **partical position & landmark position**. 
**Actual measurement** means the real observed measurement

<img src="https://user-images.githubusercontent.com/40875720/51070623-42453400-167f-11e9-8c09-3ed8bfa00d82.PNG" width="600">

### Step 1 : Find out the landmarks in the map which are in the sensors' range
To reduce the compute workload, each partical only cares about the landmark which are in the sensors' range. The following code shows the details of this step. Please pay atension that before coordinate transformation it is also ok to measure the disctance between partical and lanmarks duo the fact that **the partical's position has the map coordinate, only partical's observation need to change the coordinate**.

