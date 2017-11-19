# CarND-PID-Control-Project
In this project we implemented a PID controller in C++ to maneuver the vehicle around the track in a simulator.
The simulator provided the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle with the help of the PID controller.
He had to build a PID controller and tune the PID hyperparameters by testing the solution on the simulator.

# Reflection
## Effect of the P, I, D components
The P,I,D components of the implemented controller behaved as mentioned in the lession:
* Having only a P component, the car oversteers and is oscillating around the target trajectory.
* Adding a D component mitigates oversteering and oscillating
* Finally, adding an I component counteracts a systematic drift of the car and lets the car drive closer to the target trajectory.
The values of the P,I,D components must be selected carefully, because choosing a parameter value to high will lead to catastrophic behavior of the car, like loosing control and driving off the road.

## Selection of final hyperparameters
The tuning of the hyperparameters was made manually. 
I increased/decreased one parameter a little bit at a time and tested it in the simulator.
Step by step, I tried to improve the behavior of the car on the track.
I started having only a P coefficient by setting the other coefficients to zero. 
With a small P value, the car was not able to follow the road and drove of the track in the first turn. 
So I increased the P value to allow stronger turning of the car. I increased the P value until the car started to oscillate wildly. 
Then, to mitigate the oscillation, I introduced a D component. Now the car was able to drive smoothly around turn 1.
But, as the second turn of the track is stronger, the car drove of the track in turn 2.
Increasing the P value again, made the car turn stronger, but led to an oscillating behavior, which I reduced by increasing the D value.
After several interations of increasing P and then D, the car was able to drive around the whole track.

To evaluate the behavior more, I introduced an error value (the sum of all absolute values of the cross-track errors of all the measurements of one lap). 
Then I tried to minimize this error value by adjusting the P,I,D values.
At first, I added more iterations of increasing P and then D like I did before, until the error value did not get smaller anymore.
Then I added a small I value which resulted in the smallest error.

The final parameters were: P=0.14, I=0.001, D=1.5
