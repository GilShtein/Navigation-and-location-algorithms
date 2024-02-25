Extended Kalman Filter in 2-D for Navigation and location algorithms

the blue line represent the actual location of the object.
the green dots represent the data the sensors are giving us.
the red line represent the kalman evaluation of the location of the object.
the red circle represent the 'p' matrix , the cov matrix that shows the uncertainty around the solution.
the black line represent a 'blind' navigation that's not using kalman evaluation. it updates the location based on the info we had in the last step.

we can see that As time passes 'blind' navigation drifts further and further away from the real result.



![kalman](https://github.com/GilShtein/Navigation-and-location-algorithms/assets/110115156/f5fc0d5d-9ab6-4818-8112-080e90f67a49)

![kalman1](https://github.com/GilShtein/Navigation-and-location-algorithms/assets/110115156/d9a4b90b-7e2d-483e-aaac-dd66608adbc7)
