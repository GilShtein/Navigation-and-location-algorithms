import pandas as pd
import os
import numpy as np
from dataclasses import dataclass
import random
import time



np.random.seed(14)
@dataclass
class Particle:
    x: float
    y: float
    theta: float
    weight: float 
        


@dataclass
class LandMark:
    x: float
    y: float
    index: int
gt_data = pd.read_csv('data/gt_data.txt', names=['X','Y','Orientation'], sep=' ')
map_data = pd.read_csv('data/map_data.txt', names=['X','Y','# landmark'])
control_data = pd.read_csv('data/control_data.txt', names=['velocity','Yaw rate'], sep=' ')

#observation = pd.read_csv('data/observation/observations_000001.txt', names = ['X cord','Y cord'], sep=' ')


result = [(x,y, landmark) for x,y,landmark in zip(map_data['X'],map_data['Y'], map_data['# landmark'])]
landarkList=[]
for res in result:
    l = LandMark(res[0],res[1],res[2])
    landarkList.append(l)

    

a = os.listdir("data/SensorDataFiles")
a.sort()
observation=[]
for i in range(len(a)):
    fileName = 'data/SensorDataFiles/'+a[i]
    observationTmp = pd.read_csv(fileName, names = ['X cord','Y cord'], sep=' ')
    observation.append(observationTmp)
 
def calculateDistance(landmark1, landmark2):
    a =  np.sqrt((landmark1.x - landmark2.x)**2 + (landmark1.y - landmark2.y)**2)
    return a
    
 
    
def findClosestLandmark(map_landmarks, singleObs):
    
    closest_landmark = None
    min_distance = float('inf')  # Initialize with a large value
    
    for landmark in map_landmarks:
        distance = calculateDistance(landmark, singleObs)
        if distance < min_distance:
            min_distance = distance
            closest_landmark = landmark
    return closest_landmark
    
    
    
        
def getError(gt_data, bestParticle):
    error1 = np.abs(gt_data[0] - bestParticle.x)
    error2 = np.abs(gt_data[1] - bestParticle.y)
    error3 = np.abs(gt_data[2] - bestParticle.theta)
    if(error3>2*np.pi):
        error3 = 2*np.pi - error3
    return (error1, error2, error3)

def findObservationProbability(closest_landmark,map_coordinates, sigmaX, sigmaY):
    
    mew_x = closest_landmark.x
    mew_y = closest_landmark.y
    
    x = map_coordinates.x;
    y = map_coordinates.y;
    frac =  (1/ (2 * np.pi * sigmaX * sigmaY ))
    weight1 = (x-mew_x)**2/((sigmaX)**2)  +(y-mew_y)**2/(sigmaY**2)    
    ans = np.exp(-0.5*weight1)
    return ans



def mapObservationToMapCoordinates(observation, particle):
    x = observation.x
    y = observation.y

    xt = particle.x
    yt = particle.y
    theta = particle.theta

    MapX = x * np.cos(theta) - y * np.sin(theta) + xt
    MapY = x * np.sin(theta) + y * np.cos(theta) + yt

    return MapX, MapY

def mapObservationsToMapCordinatesList(observations, particle):
    
    convertedObservations=[]
    i=0
    for obs in observations.iterrows():
        singleObs = LandMark(obs[1][0],obs[1][1],1)
        mapX, mapY = mapObservationToMapCoordinates(singleObs, particle)
        tmpLandmark = LandMark(x=mapX, y=mapY, index=i)
        i+=1
        convertedObservations.append(tmpLandmark)
    return convertedObservations
class ParticleFilter:
    particles = []
    def __init__(self, intialX, initialY, std, numOfParticles):
        self.number_of_particles = numOfParticles
        for i in range(self.number_of_particles):
           # tmpParticle = Particle(intialX,initialY,0,1)
            x = random.gauss(intialX, std)
            y = random.gauss(initialY, std)
            theta = random.uniform(0, 2*np.pi)
            tmpParticle = Particle(x,y , theta,1)
            self.particles.append(tmpParticle)
            
            
            
    def moveParticles(self, velocity, yaw_rate, delta_t=0.1):
        
        for i in range(self.number_of_particles):
            if(yaw_rate!=0):
                theta = self.particles[i].theta
                newTheta = theta + delta_t*yaw_rate;
                newX =  self.particles[i].x +(velocity/yaw_rate)*(np.sin(newTheta)-np.sin(theta));
                newY =  self.particles[i].y +(velocity/yaw_rate)*(np.cos(theta)-np.cos(newTheta));
                
                #todo Add noise!!
                self.particles[i].x = newX +random.gauss(0, 0.3)
                self.particles[i].y = newY +random.gauss(0, 0.3)
                self.particles[i].theta = newTheta +random.gauss(0, 0.01)
            else:
                print("ZERO!!!")
            

        
        
    def UpdateWeight(self, observations):
        
        particle = self.particles[i]
        # Initialize weight to 1
        particle.weight = 1.0
        
        # Map observations to map coordinates for the particle
        mapped_observations = mapObservationsToMapCordinatesList(observations, particle)
        
        # Iterate over each mapped observation and find the closest landmark
        for obs in mapped_observations:
            closest_landmark = findClosestLandmark(landarkList, obs)
            
            # Calculate observation probability for the closest landmark
            observation_probability = findObservationProbability(closest_landmark, obs, self.x, self.y)
            
            # Update particle weight based on observation probability
            particle.weight *= observation_probability
            
    
    
    def getBestParticle(self):
        best_particle =  max(self.particles, key= lambda particle: particle.weight)
        return best_particle    
    
    def getBestParticleOut(self):
        x=0
        y=0
        theta=0
        for i in range(self.number_of_particles):
            x+= self.particles[i].x
            y+= self.particles[i].y
            theta+= self.particles[i].theta
        x=x/self.number_of_particles
        y=y/self.number_of_particles
        theta=theta/self.number_of_particles
        best_particle =  Particle(x,y,theta, weight=1)
        return best_particle        
    
    def PrintWeights(self):
        for i in range(self.number_of_particles):
            print("Weight:",self.particles[i].weight, self.particles[i].x,self.particles[i].y)
    
    
    def Resample(self):
         # Create an array to store the cumulative sum of weights
        cumulative_weights = [0.0] * self.number_of_particles
        cumulative_sum = 0.0
    
        # Calculate cumulative sum of weights
        for i in range(self.number_of_particles):
            cumulative_sum += self.particles[i].weight
            cumulative_weights[i] = cumulative_sum
    
        # Generate random starting point
        start_index = random.randint(0, self.number_of_particles - 1)
        step_size = cumulative_sum / self.number_of_particles
    
        # Perform resampling
        index = 0
        new_particles = []
        beta = 0.0
    
        for i in range(self.number_of_particles):
            beta += random.uniform(0, step_size * 2)
        
            while beta > cumulative_weights[index]:
                beta -= cumulative_weights[index]
                index = (index + 1) % self.number_of_particles
        
            # Add selected particle to new_particles
            new_particles.append(self.particles[index])
    
        # Update particles with the resampled particles
        self.particles = new_particles

       
        
        
sigmaY = 0.3

magicNumberOfParticles = 200


start = time.time()

def main():
    particleFilter = ParticleFilter(0 ,0 ,0.3, numOfParticles=magicNumberOfParticles)
    #articleFilter = ParticleFilter(6.2785 ,1.9598 ,0.3, numOfParticles=magicNumberOfParticles)


    for i in range(len(observation)):
        #prediction
        if(i!=0):
            velocity = control_data.iloc[i-1][0]
            yaw_rate = control_data.iloc[i-1][1]
            particleFilter.moveParticles(velocity, yaw_rate)
        a = observation[i].copy()
        particleFilter.UpdateWeight(a)
        particleFilter.Resample()
        bestP = particleFilter.getBestParticle()
        error = getError(gt_data.iloc[i], bestP)
        print(i,error)
    end = time.time()
    print(end - start)
        
main()
