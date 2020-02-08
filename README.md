# Oneloop Controls: Kalman Filter

#### Note: This installation uses the FilterPy library to implement Bayesian filter functionalities: https://filterpy.readthedocs.io/en/latest/

### Members
* Christina
* Varadraj
* Vishal
* Soumil

### Overview
We will be implementing a Kalman Filter to enable our navigation system to perform better. Based on the results we get on implementing a Kalman Filter on the navigation system we would want to extend the Kalman Filter to all redundant sensors on the pod. Hence while the version we develop over the break will primarily be for navigation we would want to make sure that it is easily extendable to multiple to measuring temperature, voltage, and pressure too.

While implementing the Kalman Filter for navigation we will work under the assumption that we have two different navigation systems, laser strip detection and accelerometer, for determination of pod position and velocity.

Our codebase will be C/C++ as in all likelihood we will publish the Kalman Filter on an arduino. We might also have to integrate the Kalman Filter with the PLC in which case translate C/C++ to Structured Text should not be too hard.

### Project Phases
* Research Kalman Filter
* Familiarize with an open source Kalman Filter codebase or library
* Implement Kalman Filter ourselves
* Design criteria for evaluation of Kalman Filter
* Extended Kalman Filter for all sensor measurements

### Inputs
* From laser sensor, distance traveled by the pod (inaccurate)
* every pulse = a certain fixed length, e.g. 10 m
* set distance markers within the loop that we can measure the pod against OR calculate distance based on reflective tape on wheel
* accelerometer gives us velocity

### Outputs
* accurate speed 
* accurate distance
* acceleration (if time)

##### meeting 2/8/2020 - akira learned github:)

#THANKS CHRISTINA
