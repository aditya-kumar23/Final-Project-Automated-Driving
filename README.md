# Final-Project-Automated-Driving
This is group project repo for sem two elective Automated driving in FHDO
## Work package
Make a scenario => two cars. one after the other, match of the speed of the car ahead. can refine more as you go forwarded. 

First two people 
1. data from the sensor + noise
2. data pre processing 
3. data cleaning ( assumptions if the point is too far then remove it! .. then look at the state of the art)
4. noise removal %% which order 
5. state estimation ( Kalman filter ) 

* first tune with clean data and focus on noise reduction 
-----------------------------------------------------------------------------
same steps from 1 - 4 but with mapping
5. Mapping using grid
6. [acc] matching the speed of the car in the front with PID instead

Implement the architecture in the paper [ UML stuff ]

## First step 
How to work with the data when ego vehical is moving. How to Track the second vehicle. how to map the trigectory of both vehicals. 
