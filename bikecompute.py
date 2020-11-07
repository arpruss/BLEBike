from math import *

# ignores crank moment of inertia

radius = 0.145 # meters
mass = 2.72 # kg
crankMassAdjusted = 0.25 # kg, but adjusted for moment of inertia
g = 9.81
dt = 0.00001

LINEAR = 1

if not LINEAR:
    minFriction = 10.56
    maxFriction = 40
    frictionPrecision = 0.002
else:
    minFriction = 0
    maxFriction = 75
    frictionPrecision = 0.2

def estimateTime_const(theta0, theta1, friction):
    t = 0
    theta = theta0
    angularVelocity = 0
    prevVelocity = 0
    prevAccel = 0
    
    while theta > theta1:
        angularAccel = -cos(theta) * g * mass / radius / (mass + crankMassAdjusted) + friction / radius / (mass+crankMassAdjusted)
        effectiveAccel = angularAccel if t==0 else (angularAccel + prevAccel) / 2.
        if effectiveAccel > 0:
            effectiveAccel = 0
        angularVelocity += dt * effectiveAccel
        if angularVelocity >= 0:
            return float("inf")
        theta += dt * (angularVelocity + prevVelocity) / 2.
        t += dt
        
        prevAccel = angularAccel
        prevVelocity = angularVelocity
        
    return t

def estimateTime_linear(theta0, theta1, friction):
    t = 0
    theta = theta0
    angularVelocity = 0
    prevVelocity = 0
    prevAccel = 0
    
    while theta > theta1:
        angularAccel = -cos(theta) * g * mass / radius / (mass + crankMassAdjusted) - friction * prevVelocity / radius / (mass+crankMassAdjusted) 
        effectiveAccel = angularAccel if t==0 else (angularAccel + prevAccel) / 2.
        if effectiveAccel > 0:
            effectiveAccel = 0
        angularVelocity += dt * effectiveAccel
        if angularVelocity >= 0:
            return float("inf")
        theta += dt * (angularVelocity + prevVelocity) / 2.
        t += dt
        
        prevAccel = angularAccel
        prevVelocity = angularVelocity
        
    return t
    
estimateTime = estimateTime_const if not LINEAR else estimateTime_linear

def estimateFriction(theta0, theta1, t):
    bestFriction = minFriction
    bestError = abs(estimateTime(theta0, theta1, 0)-t)
    
    friction = bestFriction + frictionPrecision
    
    while friction < maxFriction:
        est = estimateTime(theta0, theta1, friction)
        if est >= float("inf"):
            break
        error = abs(est-t)
        if error < bestError:
            bestError = error
            bestFriction = friction
        friction += frictionPrecision
                
    return bestFriction
    
def parseTime(t):
    if ',' in t:
        sec,frame = t.split(',')
        return float(sec)+float(frame)/30.
    else:
        return float(t)
    
if __name__ == '__main__':
    import sys
    
    data = {}
    setting = "(unknown)"
    rms = 0
    rmsCount = 0
    maxError = 0
    with open(sys.argv[1]) as f:
        for line in f:
            s = line.split()
            if len(s) >= 4:
                t1 = parseTime(s[0])
                t2 = parseTime(s[2])
                if t2 < t1:
                    t2 += 60       
                theta0 = pi/180. * float(s[1])
                theta1 = pi/180. * float(s[3])
                t = t2-t1
                f = estimateFriction(theta0,theta1,t)
                print("elapsed", t)
                print("friction", f)
                data[setting].append([f,theta0,theta1,t])
            elif line[0] == '#':
                setting = int(line[1:])
                if setting not in data:
                    data[setting] = []
    for s in sorted(data.keys()):
        fs = tuple(d[0] for d in data[s])
        n = len(fs)
        f = sum(fs)/float(n)
        for d in data[s]:
            t1 = estimateTime(d[1],d[2],d[0])
            e = abs(t1-d[3])
            maxError = max(e,maxError)            
            rms += e*e
            rmsCount += 1
        print(s,f)
    print("error", sqrt(rms/rmsCount),maxError)
        