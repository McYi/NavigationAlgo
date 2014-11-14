import threading
import Queue
import serial
import pygame
import json
from urllib2 import urlopen
from time import sleep
from heapq import heappush, heappop
from math import sqrt, pow, atan2, degrees, log10, cos, sin, pi

# Constants
STEP_LENGTH = 71.5 # assume the step length is 0.74m
COUNT_CALI_DIRETION = 10
ONTHEWAY_DIR_ERROR = 10
FINDDIR_DIR_ERROR = 5
SONIC_MAX_DIST = 170
SONIC_OBSTACLE_LR_RANGE = 55
SONIC_OBSTACLE_FT_RANGE = 70
SONIC_ERROR_RANGE = 5

# Reading data from Arduino Mega
def ReadUART(q):
    global currData, exitFlag, read, sonicLeft, sonicRight, sonicFrontL, sonicFrontR, sonicDown
    read = read + ser.readline()
    if '\r\n' in read:
        lines = read.split('\r\n')
        last_received = lines[0]
        read = lines[1]
        data = last_received.split(',')
        # if data length is 2, accX and compass readings are received
        if len(data) == 2:
            # Catch non-integer exception
            try:
                temp = int(data[0]) + int(data[1])
            except (TypeError, ValueError):
                print "Erroneous Data Received!"
            currData = data
            dataQueue.put(data)
        # If data length is 3, ultrasonic sensor readings are received
        elif len(data) == 5:
            # Catch non-integer exception
            try:
                temp = int(data[0]) + int(data[1]) + int(data[2]) + int(data[3]) + int(data[4])
            except (TypeError, ValueError):
                print "Erroneous Data Received!"
            # SonicFrontL
            if int(data[0]) < SONIC_ERROR_RANGE or int(data[0]) > SONIC_MAX_DIST:
                sonicFrontL = SONIC_MAX_DIST
            else:
                sonicFrontL = int(data[0])
            # SonicFrontR
            if int(data[1]) < SONIC_ERROR_RANGE or int(data[1]) > SONIC_MAX_DIST:
                sonicFrontR = SONIC_MAX_DIST
            else:
                sonicFrontR = int(data[1])
            # SonicLeft
            if int(data[2]) < SONIC_ERROR_RANGE or int(data[2]) > SONIC_MAX_DIST:
                sonicLeft = SONIC_MAX_DIST
            else:
                sonicLeft = int(data[2])
            # SonicRight
            if int(data[3]) < SONIC_ERROR_RANGE or int(data[3]) > SONIC_MAX_DIST:
                sonicRight = SONIC_MAX_DIST
            else:
                sonicRight = int(data[3])
            # SonicDown
            if int(data[4]) < SONIC_ERROR_RANGE or int(data[4]) > SONIC_MAX_DIST:
                sonicDown = SONIC_MAX_DIST
            else:
                sonicDown = int(data[4])


def Process(q):
    if not q.empty():
        Navigation(q)


def playSound(str):
    while pygame.mixer.music.get_busy() == False:
        pygame.mixer.music.load(str)
        pygame.mixer.music.set_volume(1.0)
        pygame.mixer.music.play()

def playSoundInterrupt(str):
    pygame.mixer.music.load(str)
    pygame.mixer.music.set_volume(1.0)
    pygame.mixer.music.play()

def playSoundForSingleDigit(m):
    while pygame.mixer.music.get_busy() == True:
        continue
    n = int(m)
    if n == 1:
        playSound("1.wav")
    elif n == 2:
        playSound("2.wav")
    elif n == 3:
        playSound("3.wav")
    elif n == 4:
        playSound("4.wav")
    elif n == 5:
        playSound("5.wav")
    elif n == 6:
        playSound("6.wav")
    elif n == 7:
        playSound("7.wav")
    elif n == 8:
        playSound("8.wav")
    elif n == 9:
        playSound("9.wav")
    elif n == 0:
        playSound("0.wav")
    else:
        return
        
def playSoundForMultipleDigits(n):
    while pygame.mixer.music.get_busy() == True:
        continue
    for ch in n:
        playSoundForSingleDigit(ch)
        while pygame.mixer.music.get_busy() == True:
            continue

def DistCalculation(q):
    global soundOnTheWay, accList, lastSign, toggle, distCorrect

    num = q.get()
    num[0] = int(num[0])

    num[0] = num[0] - accOffset
    if num[0] > 800 or num[0] < -800:
        if accList.count(-1) + accList.count(1) is 0:
            if num[0] > 0 and lastSign is 0:
                accList.append(1)
            elif num[0] < 0 and lastSign is 0:
                accList.append(-1)
            elif num[0] > 0 and lastSign is not 1:
                accList.append(1)
            elif num[0] < 0 and lastSign is not -1:
                accList.append(-1)
        else:
            if num[0] > 0:
                accList.append(1)
            else:
                accList.append(-1)

    if accList.count(-1) + accList.count(1) is 5:
        # Set toggle if the sign has changed
        if accList.count(-1) > accList.count(1) and lastSign is not -1:
            lastSign = -1
            toggle = toggle + 1
        elif accList.count(1) > accList.count(-1) and lastSign is not 1:
            lastSign = 1
            toggle = toggle + 1

        # Preserve the different bits for the next comparison
        if accList.pop(4) is not lastSign:
            if accList.pop(3) is not lastSign:
                accList = []
                accList.append(lastSign * -1)
            else:
                accList = []
            accList.append(lastSign * -1)
        else:
            accList = []

        if toggle is 2:
            toggle = 0
            # Calculating the distance travelled and also the distance deviated from the correct direction
            distCorrect += STEP_LENGTH

            angle = readingCali(int(currData[1]))
            print angle, baseAngle

            degree = angle - baseAngle

            if degree < 0:
                degree += 360

            # Adjusting the distance
            if 180 >= degree > 5:
                soundOnTheWay = "rotate_right.wav"
                distCorrect -= STEP_LENGTH * (1 - cos(ONTHEWAY_DIR_ERROR * pi / 180))
            elif 355 > degree > 180:
                soundOnTheWay = "rotate_left.wav"
                distCorrect -= STEP_LENGTH * (1 - cos(ONTHEWAY_DIR_ERROR * pi / 180))
            else:
                soundOnTheWay = "move_forward.wav"


def Navigation(q):
    global exitFlag, distLeft, nextNode, arrived, onTrack, prevPosition, curPosition, path, curX, curY, correctDir, isNextBuilding
    nextX = "000"
    nextY = "000"
    check = 0
    if not arrived:
        onTrack = 0

        # On the way.
        if correctDir == 1:
            onTheWay(q, curX, curY, nextNode)
            onTrack = 1
            if distLeft <= STEP_LENGTH / 2:
                playSoundInterrupt("stop.wav")
                if nextNode >= 10:
                    playSoundForMultipleDigits(str(nextNode))
                else:
                    playSoundForSingleDigit(str(nextNode))
                correctDir = 0
                curPosition = nextNode
                curX = coordinate[nextNode][0]
                curY = coordinate[nextNode][1]

        # At a node, either turn to another direction or stop.
        elif correctDir == 0:
            for i in range(0, len(path) - 1):
                if curPosition == end:
                    arrived = 1
                    onTrack = 1
                    if isNextBuilding is False:
                        while pygame.mixer.music.get_busy() == True:
                            continue
                        print "we're here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                        playSound("destination_reached.wav")
                        while pygame.mixer.music.get_busy() == True:
                            continue
                        exitFlag = 1
                    else:
                        reini(q)
                        curPosition = start
                        arrived = 0
                        check = 1
                        isNextBuilding = False
                    break
                elif curPosition == path[i]:
                    onTrack = 1
                    nextNode = path[i + 1]
                    nextX = coordinate[nextNode][0]
                    nextY = coordinate[nextNode][1]
                    break
            if curPosition != end and check == 0:
                num = q.get()
                num[0] = int(num[0])
                heading = readingCali(int(num[1]))
                findDirection(q, nextY - curY, nextX - curX, heading, north)
                distLeft = findDist(coordinate[curPosition], coordinate[nextNode])

    if onTrack == 0:
        initializePath()
        dijkstra(curPosition, end)
        path = makePath(reversedPath, parent, curPosition, end)
    prevPosition = curPosition

# COM1L2 Node 29 to COM2L2 Node 2
def reini(q):
    global data, json_data, mapNode, wifiNode, north, start, end
    string = 'http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php?Building=COM'
    string = string + endBuildingNum + '&Level=' + endLevelNum

    data = urlopen(string)
    json_data = json.load(data)

    mapNode = len(json_data["map"])
    wifiNode = len(json_data["wifi"])
    north = int(json_data['info']['northAt'])

    for i in range(0, mapNode):
        data = json_data['map'][i]['nodeName'].split('-')
        if len(data) > 1 and data[0] == 'TO COM' + str(startBuildingNum):
            startNode = json_data['map'][i]['nodeId']

    endNode = endLoc
    start, end = [int(startNode), int(endNode)]

    StartUp(q)
    

def onTheWay(q, curX, curY, nextNode):
    global soundOnTheWay, distLeft, distCorrect, correctDir
    curPosition = nearNode(curX, curY)
    if curPosition == nextNode:
        playSound("stop")
        correctDir = 0

    # If the user is still on the road (at one of the edges)
    DistCalculation(q)
    distLeft = distLeft - distCorrect
    distCorrect = 0
    print "distLeft = ", distLeft

    # Obstacle detection, higher priority
    if sonicDown <= SONIC_OBSTACLE_FT_RANGE:
        print "stop"
        soundOnTheWay = "stop.wav"
    elif sonicFrontR <= SONIC_OBSTACLE_FT_RANGE and sonicFrontL <= SONIC_OBSTACLE_FT_RANGE:
        print "stop"
        soundOnTheWay = "stop.wav"
    elif sonicFrontL <= SONIC_OBSTACLE_FT_RANGE and sonicFrontR > 100:
        print "steer_right"
        soundOnTheWay = "steer_right.wav"
    elif sonicFrontR <= SONIC_OBSTACLE_FT_RANGE and sonicFrontL > 100:
        print "steer_left"
        soundOnTheWay = "steer_left.wav"
    elif sonicRight <= SONIC_OBSTACLE_LR_RANGE:
        print "steer_left"
        soundOnTheWay = "steer_left.wav"
    elif sonicLeft <= SONIC_OBSTACLE_LR_RANGE:
        print "steer_right"
        soundOnTheWay = "steer_right.wav"

    if soundOnTheWay != '':
        playSound(soundOnTheWay)
        soundOnTheWay = ''


def readingCali(num):
    num = num / 10
    # E.g. North is 315, which is 135 on the map.
    if num <= 450 - north:
        num = 450 - north - num
    else:
        num = 810 - north - num  
    return num


def findDirection(q, y, x, heading, north):
    global correctDir, baseAngle, count
    #print "y: " , y, "x: ", x
    next = degrees(atan2(y, x))

    if next < 0:
        next += 360
    baseAngle = next
    degree = next - heading
    lr = ''
    sound = "None"

    print 'heading: ', heading, '  next: ', next, '  degree:', degree
    if -359 < degree < -355:
        count += 1
    elif degree < -FINDDIR_DIR_ERROR and abs(degree) >= 180:
        count = 0
        lr = 'left'
        sound = "turn_left.wav"
    elif degree > FINDDIR_DIR_ERROR and abs(degree) >= 180:
        count = 0
        lr = 'right'
        sound = "turn_right.wav"
    elif degree > FINDDIR_DIR_ERROR and abs(degree) < 180:
        count = 0
        lr = 'left'
        sound = "turn_left.wav"
    elif degree < -FINDDIR_DIR_ERROR and abs(degree) < 180:
        count = 0
        lr = 'right'
        sound = "turn_right.wav"
    else:
        count += 1

    # When step count failed to reach the destination (travelled distance is less than expected)
    if lr == 'right' and sonicRight <= SONIC_OBSTACLE_LR_RANGE:
        if sonicFrontL > SONIC_OBSTACLE_FT_RANGE and sonicFrontR > SONIC_OBSTACLE_FT_RANGE:
            sound = "move_one_step_forward.wav"
        elif sonicFrontL <= SONIC_OBSTACLE_FT_RANGE or sonicFrontR <= SONIC_OBSTACLE_FT_RANGE:
            sound = "stop.wav"
    elif lr == 'left' and sonicLeft <= SONIC_OBSTACLE_LR_RANGE:
        if sonicFrontL > SONIC_OBSTACLE_FT_RANGE and sonicFrontR > SONIC_OBSTACLE_FT_RANGE:
            sound = "move_one_step_forward.wav"
        elif sonicFrontL <= SONIC_OBSTACLE_FT_RANGE or sonicFrontR <= SONIC_OBSTACLE_FT_RANGE:
            sound = "stop.wav"
    elif lr == '':
        if count == COUNT_CALI_DIRETION:
            sound = "move_forward.wav"
            correctDir = 1

    with q.mutex:
        q.queue.clear()

    if sound is not "None":
        playSound(sound)


def initializeMap():
    link = []
    while coordinate:
        coordinate.pop()
    while adjMatrix:
        adjMatrix.pop()
    coordinate.append((-1, -1))
    adjMatrix.append((-1, -1))
    for i in range(0, mapNode):
        coordinate.append((int(json_data['map'][i]['x']), int(json_data['map'][i]['y'])))
        link.append(json_data['map'][i]['linkTo'].split(","))
        for x in range(0, len(link[i])):
            link[i][x] = int(link[i][x].strip())
    for i in range(0, mapNode):
        edges = []
        for j in range(0, len(link[i])):
            dist = findDist(coordinate[i + 1], coordinate[link[i][j]])
            edges.append((link[i][j], dist))
        edges = sorted(edges, key=lambda x: x[0])
        adjMatrix.append(edges)


def initializePath():
    global finalDist, parent, reversedPath
    finalDist = []
    parent = []
    reversedPath = []
    finalDist.append(-1)
    parent.append(0)
    for i in range(0, mapNode):
        parent.append(0)
        finalDist.append(100000)


def dijkstra(start, end):
    finalDist[start] = 0
    heappush(pq, (start, finalDist[start]))
    while pq:
        currVtx = heappop(pq)
        if currVtx[1] <= finalDist[currVtx[0]]:
            for vtx in adjMatrix[currVtx[0]]:
                if finalDist[vtx[0]] > finalDist[currVtx[0]] + vtx[1]:
                    relax(finalDist, parent, currVtx, vtx)
                    heappush(pq, vtx)


def findDist(p1, p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    distance = findDistCoor(x1, y1, x2, y2)
    return distance


def findDistCoor(x1, y1, x2, y2):
    distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))
    return distance


def relax(finalDist, parent, currVtx, vtx):
    finalDist[vtx[0]] = finalDist[currVtx[0]] + vtx[1]
    parent[vtx[0]] = currVtx[0]


def makePath(reversedPath, parent, start, end):
    reversedPath.append(end)
    while parent[end] != 0:
        reversedPath.append(parent[end])
        end = parent[end]
    path = reversedPath[::-1]
    path = map(int, path)
    return path


def get_apdist(siglvl, freq):
    temp1 = log10(freq) * 20 + 32.45
    temp2 = siglvl - temp1
    distance = pow(10, temp2 / 20)
    return distance


def nearNode(x, y):
    for i in range(1, len(coordinate)):
        if abs(x - coordinate[i][0]) < 20 and abs(y - coordinate[i][1]) < 20:
            return i
    return 0

# Read input from keyboard and set the destination
def getDestination():
    global start, end, curX, curY, path, prevPosition, curPosition
    initializePath()
    dijkstra(start, end)
    path = makePath(reversedPath, parent, start, end)
    prevPosition = start
    curPosition = start
    curX = coordinate[start][0]
    curY = coordinate[start][1]

# Execute once when the system first starts
def Startup(q):
    q.queue.clear()
    initializeMap()
    getDestination()

# Setup & initialization
ser = serial.Serial('/dev/ttyAMA0', 19200, timeout=1)  # Target name not specified yet
pygame.mixer.init()
ser.open()
signal = ''
dataQueue = Queue.Queue(1000)
sendFlag = 0  # Flag used when there is a request to be sent
exitFlag = 0  # Flag used to stop the system

# Distance Calculation Variables
accList = []
lastSign = 0  # 0 indicates startup, 1 indicates positive, -1 indicates negative
toggle = 0
baseAngle = 90  # assume the direction of the corridor is 90 degrees(West)
distCorrect = 0
soundOnTheWay = ''

# Navigation initialization
nextNode = "000"
x, y, coordinate, path, reversedPath, finalDist, adjMatrix, pq, parent = ([] for i in range(9))
curX = 0
curY = 0
correctDir = 0
distLeft = 0
count = 0
sonicFrontL = SONIC_MAX_DIST
sonicFrontR = SONIC_MAX_DIST
sonicLeft = SONIC_MAX_DIST
sonicRight = SONIC_MAX_DIST
sonicDown = SONIC_MAX_DIST
arrived = 0
onTrack = 0

# Program STARTS from here
# Read the keypad input from the user and determine the destination and the route
# Input sequence: buiding number -> level number -> starting location -> ending location
read = ''
countKey = 0
startBuildingNum = 0
startLevelNum = 0
endBuildingNum = 0
endLevelNum = 0
startLoc = 0
endLoc = 0
isNextBuilding = False
inFlag = True

playSound("welcome_message.wav")
ser.flushInput()
ser.flushOutput()
while inFlag:
    inFlag = False
    print 'Program Started'
    while countKey is not 6:
        read = ser.readline()
        #print read
        if '\r\n' in read:
            lines = read.split('\r\n')
            last_received = lines[0]
            data = last_received.split(',')

            if len(data) == 1:
                print data
                playSoundForMultipleDigits(last_received)
                if countKey == 0:
                    startBuildingNum = last_received
                elif countKey == 1:
                    startLevelNum = last_received
                elif countKey == 2:
                    startLoc = last_received
                elif countKey == 3:
                    endBuildingNum = last_received
                elif countKey == 4:
                    endLevelNum = last_received
                else:
                    endLoc = last_received
                countKey += 1

        if countKey == 6:
            if startBuildingNum == 0 or startLevelNum == 0 or endBuildingNum == 0 or endLevelNum == 0 or startLoc == 0 or endLoc == 0:
                countKey = 0
                # Play a sound asking for re-entering the navigation routes

    if startBuildingNum == endBuildingNum and startLevelNum == endLevelNum:
        buildingNum = startBuildingNum
        levelNum = startLevelNum
        startNode = startLoc
        endNode = endLoc
    else:
        buildingNum = startBuildingNum
        levelNum = startLevelNum
        startNode = startLoc
        isNextBuilding = True

    try:
        endLoc = int(endLoc)
    except ValueError:
        print "Please input something else!"
        playSound("warning.wav")
        inFlag = True
        countKey = 0

    try:
        startLoc = int(startLoc)
    except ValueError:
        print "Please input something else!"
        playSound("warning.wav")
        inFlag = True
        countKey = 0


    string = 'http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php?Building=COM'
    string = string + endBuildingNum + '&Level=' + endLevelNum

    data = urlopen(string)
    try:
        json_data = json.load(data)
        mapNode = len(json_data["map"])
        wifiNode = len(json_data["wifi"])
        north = int(json_data['info']['northAt'])
        if endLoc > mapNode:
        #print type(endNode), endNode, type(startNode), startNode, type(mapNode), mapNode
            print "Error Encountered When Reading Data! 2"
            playSound("warning.wav")
            inFlag = True
            countKey = 0
    except (NameError, ValueError, TypeError):
        print "Error Encountered When Reading Data! 1"
        playSound("warning.wav")
        inFlag = True
        countKey = 0
    

    string = 'http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php?Building=COM'
    string = string + buildingNum + '&Level=' + levelNum

    data = urlopen(string)
    try:
        json_data = json.load(data)
        mapNode = len(json_data["map"])
        wifiNode = len(json_data["wifi"])
        north = int(json_data['info']['northAt'])
        if startLoc > mapNode:
            print "Error Encountered When Reading Data! 4"
            playSound("warning.wav")
            inFlag = True
            countKey = 0
    except (NameError, ValueError, TypeError):
        print "Error Encountered When Reading Data! 3"
        playSound("warning.wav")
        inFlag = True
        countKey = 0
    

for i in range(0, mapNode):
    data = json_data['map'][i]['nodeName'].split('-')
    if len(data) > 1 and data[0] == 'TO COM' + str(endBuildingNum):
        endNode = json_data['map'][i]['nodeId']

start, end = [int(startNode), int(endNode)]
read = ''
Startup(dataQueue)
countAcc = 0
sumAcc = 0

while countAcc < 40:
    read = read + ser.readline()
    if '\r\n' in read:
        lines = read.split('\r\n')
        last_received = lines[0]
        read = lines[1]
        data = last_received.split(',')
        # if data length is 2, accX and compass readings are received
        if len(data) == 2:
            # Catch non-integer exception
            try:
                temp = int(data[0]) + int(data[1])
            except (TypeError, ValueError):
                print "Erroneous Data Received!"
            sumAcc += int(data[0])
            countAcc += 1
accOffset = sumAcc / 40
read = ''

while exitFlag == 0:
    ReadUART(dataQueue)
    Process(dataQueue)

ser.close()
print "Destination reached, program exited."


