# CarND-KidnappedVehicle
## Overview
Partical filter mainly contains the following four steps:
* Initialization
* Prediction
* WeightUpdate
* Resample

![overview](https://user-images.githubusercontent.com/40875720/51068684-ae627080-165c-11e9-81ed-da9b6f2e2830.png)
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
