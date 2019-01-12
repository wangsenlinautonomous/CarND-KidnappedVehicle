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
* Transform vehicle coordinate to map coordinate
* Associate transformed observations with landmarks
* Weight update

The following pic can increas your intuition of your idea
**Predicted measurement** means the predition coordinate based on the **partical position & landmark position**. 
**Actual measurement** means the real observed measurement

**_The Truth: Abservated landmarks' coordinate VS Predicted landmarks' coordinate_**

<img src="https://user-images.githubusercontent.com/40875720/51070623-42453400-167f-11e9-8c09-3ed8bfa00d82.PNG" width="600">

### Step 1: Find out the landmarks in the map which are in the sensors' range
To reduce the compute workload, each partical only cares about the landmark which are in the sensors' range. The following code shows the details of this step. Please pay atension that before coordinate transformation it is also ok to measure the disctance between partical and lanmarks duo the fact that **the partical's position has the map coordinate, only partical's observation need to change the coordinate**. After this step, we get the landmarks on board.

```
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
```

### Step 2: Transform vehicle coordinate to map coordinate
This step is used to transform landmarks' coordinate from vehicle coordinate to map coordinate. Then on the next step the algorithm can calculate the weight.

<img src="https://user-images.githubusercontent.com/40875720/51070860-1b88fc80-1683-11e9-8d71-1b18a2baa46c.PNG" width="600">

```
//Step 2:Transform coordinate from vehicle to map
for(unsigned int j=0; j<observations.size(); j++)
{
  LandmarkObs coordi_transfer;
  coordi_transfer.x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
  coordi_transfer.y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
  coordi_transfer.id = -1;
  observations_map.push_back(coordi_transfer);
}
```

### Step 3: Associate transformed observations with landmarks
This step is used to associate the transformed observations with the nearest landmarks. For each observation, partical will loop all the predicted landmark and find out the nearest one to match. Then set the predicted lanmark's id to observated landmark id.
Till now, we have predicted landmark's coordinate and observed landmarks' coordinate. Also we have their corresponding id. So they are fully aligned.

```
dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) 
{
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
```

### Step 4: Weight update
Now we have everything on board to update the weight attribute of the partical filter.

<img src="https://user-images.githubusercontent.com/40875720/51071260-15961a00-1689-11e9-8745-c4404642ac57.PNG" width="600">

```
//Step 4:Update weights
//For each particals: (1) Go through each obeservation and calculate weight then product
for(unsigned int l; l<observations_map.size(); l++)
{
  //Prepare for the observation and prediction value
  double x_obs,y_obs,x_pre,y_pre;
  x_obs = observations_map[l].x;
  y_obs = observations_map[l].y;
  for(unsigned int m; m<landmarkobj_inrange.size(); m++)
  {
    if(observations_map[l].id == landmarkobj_inrange[m].id)
    {
      x_pre = landmarkobj_inrange[m].x;
      y_pre = landmarkobj_inrange[m].y;
    }
  }
  // formula of multivariate Gaussian probability
  double obs_w = (1/(2*M_PI*std_landmark[0]*std_landmark[1])) * exp( -( pow(x_pre-x_obs,2)/(2*pow(std_landmark[0], 2)) + 
                 (pow(y_pre-y_obs,2)/(2*pow(std_landmark[1], 2)))));
  particles[i].weight *= obs_w;
}
```
