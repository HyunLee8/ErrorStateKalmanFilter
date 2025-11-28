# Extended Error State Kalman Filter w/ Quaternions

The purpose of this project is to transform raw IMU data into estimation of a flight computer’s orientation, velocity, and position. Raw IMU measurements that typically consist of tri-axial accelerometer and gyroscope output are susceptible to long-term integration errors due to external factors such as wind, air pressure and matter. To address these shortcomings, this work employs an Extended Error State Kalman Filter (ESKF) framework that separates the system into a nominal state and a small error state, enabling accurate linearization and improved numerical conditioning for nonlinear inertial navigation problems. Quaternion-based attitude representation is then taken in the nominal state to avoid locks make smooth readings.



The filter propagates the nominal state forward in time using continuous IMU readings while also updating the error state with a linearized model that takes in the influence of sensor noise, and bias drift. This representation allows the estimator to maintain small, tractable error dynamics even when the platform goes through complex motions. Measurement updates from external sensors like GPS, magnetometers, and accelerometers are incorporated within the same framework to constrain drift. The quaternion-based reset step ensures that corrections applied to the orientation remain physically valid while maintaining proper covariance.

## Orientation vs. Time

![image alt](https://github.com/HyunLee8/Quaternion-Filter/blob/main/docs/orientation_time.png?raw=true)

## Euler vs. Time

![image alt](https://github.com/HyunLee8/Quaternion-Filter/blob/main/docs/euler_time.png?raw=true)

resources:

basic filter: 
https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

ESKF with quaternions discretized Sections 5.1-5.4.3: 
https://arxiv.org/pdf/1711.02508

ESKF generalized formulas: 
https://ieeexplore.ieee.org/document/9206131

Quaternion intuition: 
https://www.youtube.com/watch?si=hcMDjBNQwnIs122w&v=d4EgbgTm0Bg&feature=youtu.be

Jacobian Matrix intuition: 
https://www.youtube.com/watch?v=6FesMicc844

Extended Kalman visualized: 
https://www.youtube.com/watch?v=9X3jGGnbcvU&t=55s



