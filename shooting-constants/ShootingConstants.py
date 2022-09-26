from unittest import result
import numpy as np
from joblib import Parallel, delayed
import joblib
import time
import pandas
import contextlib
from tqdm import tqdm

g = -9.81
cd = 0.53 # Chadwick, S.G. & Haake, Steve. (2000). The drag coefficient of tennis balls. Proceedings of the 3rd International Conference on the Engineering of Sport. 169-176. 
rho = 1.225 # Density of air at STP, kgm^-3
diameter = 24.13 * 0.01 # Diameter of the Cargo, m
A = 0.25 * np.pi * (diameter ** 2) # Crossetion Area of the Cargo, m^{-2}
m = 270 * 0.001 # Mass of the Cargo, kg
K = - (cd * rho * A) / (2 * m) # Combined constant K
h = 0.02 # Simulation step size

initialHeight = 0.83 # Height of the Cargo when launched, m
targetHeight = 264.0 / 100.0 # Height of the Target, m
targetRadius = 122.0 / 100.0 / 2.0 # Radius of the Target, m
heightSafeMargin = 0.20 # Height that the trajectory should be higher than the Target, m
flywheelRadius = 5 * 2.54 * 0.01 * 0.5 # Flywheel radius, m
eta = 0.40 # Efficiency of flywheel speed transmission

rpmSearchPreset =  [2400, 5000, 50]
angleSearchPreset = [20, 85, 1.0]


def sx(time, initialVelocity):
    return np.sign(initialVelocity) * -1/K * np.log(abs(1 - abs(initialVelocity) * K * time))

def sy(time, initialVelocity):
    return np.sign(initialVelocity) * -1/K * np.log(abs(1 - abs(initialVelocity) * K * time))

def vz(time, initialVelocity):
    tMaxRising = np.sqrt(1 / (g * K)) * np.arctan(np.sqrt(K / g)* initialVelocity)
    if(time >= 0 and time <= tMaxRising):
        return np.sqrt(g / K) * np.tan(-time * np.sqrt(g * K) + np.arctan(np.sqrt(K / g) * initialVelocity))
    else:
        return np.sqrt(g / K) * np.tanh(-np.sqrt(g * K) * (time - tMaxRising))

def sz(time, initialVelocity, initialHeight):
    intergrated = initialHeight
    while True:
        if(time <= 0):
            return intergrated
        intergrated += h * vz(time, initialVelocity)
        time -= h

def getTrajectoryInitialConditions(fieldVelocity, fieldAngle, shooterAngle, shooterRpm, shotHeight, shotAngle, flywheelRadius, eta):
    fieldAngle = np.radians(fieldAngle)
    shooterAngle = np.radians(shooterAngle)
    shotAngle = np.radians(shotAngle)
    
    shooterVelocity = 1 / 30 * np.pi * flywheelRadius * shooterRpm * eta
    
    vx0 = fieldVelocity * np.cos(fieldAngle) + shooterVelocity * np.cos(shooterAngle) * np.cos(shotAngle)
    vy0 = fieldVelocity * np.sin(fieldAngle) + shooterVelocity * np.sin(shooterAngle) * np.cos(shotAngle)
    vz0 = shooterVelocity * np.sin(shotAngle)
    
    return vx0, vy0, vz0, shotHeight

def isTargetReachedAtXZPlane(distance, x, z, safeConstant=0.5):
    return (z <= targetHeight) and ( (x > distance - targetRadius * safeConstant) and (x < distance + targetRadius * safeConstant))

def isOvershoot(distance, x, safeConstant=0.5):
    return x > distance + targetRadius * safeConstant

def getBestTrajectoryInitialsWithNoSideway(distance, robotForwardVelocity, step=0.05):
    bestInitial = []
    bestHeight = 3.5
    
    for lAngle in np.linspace(angleSearchPreset[0], angleSearchPreset[1], int((angleSearchPreset[1] - angleSearchPreset[0]) / angleSearchPreset[2] + 0.5)): # Search Range: min, max, resolution
        for rpm in np.linspace(rpmSearchPreset[0], rpmSearchPreset[1], int((rpmSearchPreset[1] - rpmSearchPreset[0]) / rpmSearchPreset[2] + 0.5)): # Search Range: min, max, resolution
            if(bestInitial):
                if(rpm >= bestInitial[3] and lAngle >= bestInitial[2]):
                    continue
            initialCondition = getTrajectoryInitialConditions(robotForwardVelocity, 0.0, 0.0, rpm, initialHeight, lAngle, flywheelRadius, eta)
            time = np.sqrt(1 / (g * K)) * np.arctan(np.sqrt(K / g)* initialCondition[2])
            sxInit = sx(time, initialCondition[0])
            szInit = sz(time, initialCondition[2], initialCondition[3])
            record = szInit
            if bestInitial:
                if record < bestHeight:
                    pass
                else:
                    continue
                
            if not isOvershoot(distance, sxInit) and szInit > targetHeight + heightSafeMargin and szInit < 3.5: # Prevent Trajectory Madness
                while True:
                    time += step
                    sxInit = sx(time, initialCondition[0])
                    szInit += step * vz(time, initialCondition[2])
                    if(isTargetReachedAtXZPlane(distance, sxInit, szInit)):
                        bestInitial = [distance, robotForwardVelocity, lAngle, rpm, time]
                        bestHeight = record
                        break
                    if(szInit <= targetHeight):
                        break
            else:
                continue
    return bestInitial

@contextlib.contextmanager
def tqdm_joblib(tqdm_object):
    """Context manager to patch joblib to report into tqdm progress bar given as argument"""
    class TqdmBatchCompletionCallback(joblib.parallel.BatchCompletionCallBack):
        def __call__(self, *args, **kwargs):
            tqdm_object.update(n=self.batch_size)
            return super().__call__(*args, **kwargs)

    old_batch_callback = joblib.parallel.BatchCompletionCallBack
    joblib.parallel.BatchCompletionCallBack = TqdmBatchCompletionCallback
    try:
        yield tqdm_object
    finally:
        joblib.parallel.BatchCompletionCallBack = old_batch_callback
        tqdm_object.close()

start = time.time()
with tqdm_joblib(tqdm(desc="Constant Calculation", total=121*51)) as progress_bar:
    results = Parallel(n_jobs=32)(delayed(getBestTrajectoryInitialsWithNoSideway)(distance, velocity) for distance in np.linspace(1.0,7.0,121)  for velocity in np.linspace(-2.5,2.5,51))
end = time.time()

temp = pandas.DataFrame(results, columns=["Distance", "Forward Velocity", "Angle", "RPM", "Time"]).dropna().sort_values(by=["Distance", "Forward Velocity"])
temp["Distance"] = temp["Distance"].round(2)
temp["Forward Velocity"] = temp["Forward Velocity"].round(2)
temp["Angle"] = temp["Angle"].round(2)
temp["RPM"] = temp["RPM"].round(2)
temp["Time"] = temp["Time"].round(5)
temp.to_csv("data/ShootingConstants4Decimals-0.45ETA.csv", header=None, index=None)
