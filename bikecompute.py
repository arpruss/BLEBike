from __future__ import print_function 
from math import *

radius = 0.145 # meters
mass = 2.72 # kg
moment = 10 * radius
g = 9.81
dt = 0.00005

LINEAR = 1

class Drop(object):
    def __init__(this,theta0,theta1,time):
        this.theta0 = theta0
        this.theta1 = theta1
        this.time = time

minResistance = 5
maxResistance = 75
resistancePrecision = 0.2

def estimateTime(theta0, theta1, resistanceCoeff, flywheelMoment, angularVelocity=0):
    t = 0
    theta = theta0
    prevVelocity = angularVelocity
    prevAccel = 0
    angularAccel = 0
    
    while theta > theta1:
        prevAccel = angularAccel
        torque = -cos(theta) * g * mass * radius - resistanceCoeff * prevVelocity * radius
        angularAccel = torque / (flywheelMoment + mass * radius * radius)
        effectiveAccel = angularAccel if t==0 else (angularAccel + prevAccel) / 2.
        if effectiveAccel > 0:
            effectiveAccel = 0
        prevVelocity = angularVelocity
        angularVelocity += dt * effectiveAccel
        if angularVelocity >= 0:
            return float("inf")
        theta += dt * (angularVelocity + prevVelocity) / 2.
        t += dt
        
        
    return t
    
def estimateResistance(theta0, theta1, t, resistanceMomentRatio=None):
    bestResistance = None
    bestError = None
    
    resistance = minResistance
    
    while resistance < maxResistance:
        if resistanceMomentRatio is None:
            m = moment
        else:
            m = resistance / resistanceMomentRatio
        est = estimateTime(theta0, theta1, resistance, m)
        if est >= float("inf"):
            break
        error = abs(est-t)
        if bestError is None or error < bestError:
            bestError = error
            bestResistance = resistance
        resistance += resistancePrecision
        
    return bestResistance
    
def calculateResistanceMomentRatio(angularSpeed, stopAngle):
    # d^2 r theta / dt^2 = -angularVel * coeff / m
    # d^2 theta / dt^2 = -angularVel * coeff * r / I
    # d^2 theta / dt^2 = -angularVel * ratio * r
    # angularVel(t) = A * exp(-ratio * r * t)
    # stopAngle = integral of angularVel(t) from 0 to infinity = A / (ratio * r)
    # A = angularSpeed
    # ratio * r = angularSpeed / stopAngle
    return angularSpeed / (stopAngle * radius)
    
def parseTime(t,fps=30.):
    if ',' in t:
        sec,frame = t.split(',')
        return float(sec)+float(frame)/fps
    else:
        return float(t)
    
if __name__ == '__main__':
    import sys
    
    data = {}
    maxError = 0
    
    drops = {}
    
    spinCount = 0
    ratioSum = 0
    spinRes = None

    fps = 30
    with open(sys.argv[1]) as f:
        for line in f:
            if '#' in line:
                line = line[:line.index('#')]
            line = line.strip()
            s = line.lower().split()
            if not s:
                continue
            if s[0] == 'weight':
                mass = float(s[1])
            elif s[0] == 'radius':
                radius = float(s[1])
            elif s[0] == 'minresistance':
                minResistance = float(s[1])
            elif s[0] == 'maxresistance':
                maxResistance = float(s[1])
            elif s[0] == 'fps':
                fps = float(s[1])
            elif s[0] == 'moment':
                moment = float(s[1])
            elif s[0] == 'drop':
                res = int(s[1])
                t = parseTime(s[4],fps=fps)-parseTime(s[2],fps=fps)
                if t < 0:
                    t += 60
                theta0 = pi/180. * float(s[3])
                theta1 = pi/180. * float(s[5])
                if res not in drops:
                    drops[res] = []
                drops[res].append(Drop(time=t,theta0=theta0,theta1=theta1))
            elif s[0] == 'spin':
                res = int(s[1])
                if spinRes is None:
                    spinRes = res
                elif spinRes != res:
                    print("Ignoring spin data for resistance #%d" % res)
                    continue
                
                spinRes = res
                t = parseTime(s[4],fps=fps)-parseTime(s[2],fps=fps)
                if t < 0:
                    t += 60
                theta0 = pi/180. * float(s[3])
                theta1 = pi/180. * float(s[5])
                theta2 = pi/180. * float(s[6])
                angularSpeed=abs(theta1-theta0)/t
                stopAngle=abs(theta2-theta1)
                ratio = calculateResistanceMomentRatio(angularSpeed, stopAngle)
                ratioSum += ratio                
                spinCount += 1
            else:
                print("Unrecognized data line: "+line)
                
    resistanceCoeffs = {}
                
    if spinCount:
        ratio = ratioSum / spinCount
        print("Estimated resistance to moment ratio ",ratio," at res ",spinRes)
        s = 0
        for d in drops[spinRes]:
            s += estimateResistance(d.theta0,d.theta1,d.time,resistanceMomentRatio=ratio)
        resistanceCoeffs[spinRes] = s / len(drops[spinRes])
        print("Estimated resistance coefficient ",resistanceCoeffs[spinRes])
        moment = resistanceCoeffs[spinRes] / ratio
        print("Estimated moment ",moment)        

    for res in sorted(drops):
        if res != spinRes:
            s = 0
            for d in drops[res]:
                s += estimateResistance(d.theta0,d.theta1,d.time)
            s /= len(drops[res])
            resistanceCoeffs[res] = s
            
        errors = [abs(estimateTime(d.theta0, d.theta1, resistanceCoeffs[res], moment)-d.time) for d in drops[res]]
        print(res,resistanceCoeffs[res],"errors",errors)
        