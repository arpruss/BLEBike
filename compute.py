import math

FPS = 59.97
r = 0.145 # crank radius in meters
baseForce = 0 # basic force
currentSet = None

def parseTime(d):
    parts = d.split(":")
    frames = int(parts[-1])
    sec = float(parts[-2])
    mins = 0
    hrs = 0
    if len(parts) >= 3:
        mins = float(parts[-3])
        if len(parts) >= 4:
            hrs = float(parts[-4])
    return frames/FPS + sec + mins * 60 + hrs * 3600
    
def average(data):
    n = len(data)
    def findAvailable(index, direction):
        while index != -1 and index != n:
            if data[index] is not None:
                return data[index]
            index += direction
        raise BaseException("too much missing")
    firstAvailable = findAvailable(0,1)
    lastAvailable = findAvailable(n-1,-1)
    for i in range(0,n):
        if data[i] is None:
            data[i] = firstAvailable
        else:
            break
    for i in range(n-1,-1,-1):
        if data[i] is None:
            data[i] = lastAvailable
        else:
            break
    i = 0
    while i < n:
        if data[i] == None:
            v = (findAvailable(i,-1)+findAvailable(i,1))/2.
            data[i] = v
            i = i + 1
            while data[i] == None and i < n:
                data[i] = v
                i += 1
        i += 1
    return sum(data) / n
    
def watts(rpm,k):
    distancePerSecond = 2 * math.pi * radius * (rpm / 60)
    force = k * (rpm / 60) + baseForce * 9.8
    return distancePerSecond * force
    
with open("data.txt") as f:
    for line in f:
        data = line.strip().split()
        if len(data):
            if data[0] == "FPS":
                FPS = float(data[1])
            elif data[0] == "radius":
                radius = float(data[1])
            elif data[0] == "baseforce":
                baseForce = float(data[1])
            elif data[0] == "level":
                print("level",data[1])
            elif data[0] == "start":
                startTime = parseTime(data[1])
                currentSet = []
            elif data[0] == "end":
                stopTime = parseTime(data[2])
                count = float(data[1])
                averageSpeed = count / (stopTime - startTime)
                averageForce = average(currentSet) - baseForce
                k = (averageForce*9.8)/(averageSpeed)
                print("rot/sec =",averageSpeed,"force =",averageForce*9.8,"k =",k)
                print("watts at 60RPM:",watts(60,k),"65RPM:",watts(65,k),"70RPM:",watts(70,k))
                #pr80 = (136+watts(80,k))/6.35
                #pr100 = (181+watts(100,k))/8.8
                pr60 = (86.6+watts(60,k))/3.98
                pr80 = (183.1+watts(80,k))/7.78
                pr100 = (279.1+watts(100,k))/11.73
                print("Peloton equivalent",(pr60+pr80+pr100)/3)
                currentSet = None
            else:
                if currentSet is not None:
                    if data[0] == "x":
                        currentSet.append(None)
                    else:
                        currentSet.append(float(data[0]))
                        