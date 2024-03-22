#Jehad Hamayel 1200348
#Musab Masalmah 1200078
#Abdalkarim Eiss 1200015
import numpy as np
import API
import sys
import cell
from queue import Queue
def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()
def manhattan_distance(cell1, cell2):
    return abs(cell1[0] - cell2[0]) + abs(cell1[1] - cell2[1])
def get_neighbors_costs(Cells, location):
    neighborsCost=np.zeros(4, dtype=int)
    if Cells[location[0]][location[1]].Northern_neighbor != -2 and Cells[location[0]][location[1]].Northern_neighbor != -3:
        neighborsCost[0]=Cells[location[0]][location[1] + 1].cost
    elif Cells[location[0]][location[1]].Northern_neighbor == -2 or Cells[location[0]][location[1]].Northern_neighbor == -3:
        neighborsCost[0]=10000

    if Cells[location[0]][location[1]].Eastern_neighbor != -2 and Cells[location[0]][location[1]].Eastern_neighbor != -3:
        neighborsCost[1]=Cells[location[0] + 1][location[1]].cost
    elif Cells[location[0]][location[1]].Eastern_neighbor == -2 or Cells[location[0]][location[1]].Eastern_neighbor == -3:
        neighborsCost[1]=10000

    if Cells[location[0]][location[1]].Southern_neighbor != -2 and Cells[location[0]][location[1]].Southern_neighbor != -3:
        neighborsCost[2]=Cells[location[0]][location[1] - 1].cost
    elif Cells[location[0]][location[1]].Southern_neighbor == -2 or Cells[location[0]][location[1]].Southern_neighbor == -3:
        neighborsCost[2]=10000

    if Cells[location[0]][location[1]].Western_neighbor != -2 and Cells[location[0]][location[1]].Western_neighbor != -3:
        neighborsCost[3]=Cells[location[0] - 1][location[1]].cost
    elif Cells[location[0]][location[1]].Western_neighbor == -2 or Cells[location[0]][location[1]].Western_neighbor == -3:
        neighborsCost[3]=10000

    return neighborsCost
def main():

    log("Running...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "Start")

    Cells = [[cell.cell(visited=0) for _ in range(16)] for _ in range(16)]
    location = [0,0] #x y
    Goals = [[7,7],[7,8],[8,7],[8,8]]
    diriction = ['N', 'E', 'S', 'W']
    forword = 'N'
    coun = 0
    ###########################Manhaten
    for x in range(16):
        for y in range(16):
            # Calculate the distance to all goal cells and take the minimum
            distances = [manhattan_distance((x, y), goal) for goal in Goals]
            Cells[x][y].cost = min(distances)
    for goal in Goals:
        Cells[goal[0]][goal[1]].cost=0
    for x in range(16):
        for y in range(16):
            API.setText(x, y, Cells[x][y].cost)
    ###########################

    API.setColor(location[0], location[1], "G")
    API.setWall(location[0], location[1], "s")
    if API.wallRight():
        API.setWall(location[0], location[1], "e")
    if API.wallLeft():
        API.setWall(location[0], location[1], "w")
    if API.wallFront():
        API.setWall(location[0], location[1], "n")
    for x in range(16):
        for y in range(16):
            if (x - 1) == -1 and (y - 1)==-1:
                Cells[x][y].Southern_neighbor = -3
                Cells[x][y].Western_neighbor = -3
            elif (x - 1)==-1 and (y + 1)==16:
                Cells[x][y].Northern_neighbor = -3
                Cells[x][y].Western_neighbor = -3
            elif (x + 1)==16 and (y - 1)==-1:
                Cells[x][y].Eastern_neighbor = -3
                Cells[x][y].Southern_neighbor = -3
            elif (x + 1)==16 and (y + 1)==16:
                Cells[x][y].Eastern_neighbor = -3
                Cells[x][y].Northern_neighbor = -3
            elif (x - 1)==-1 and ((y - 1)!=-1 and (y + 1)!=16):
                Cells[x][y].Western_neighbor = -3
            elif (x + 1)==16 and ((y - 1)!=-1 and (y + 1)!=16):
                Cells[x][y].Eastern_neighbor = -3
            elif (y - 1)==-1 and ((x - 1)!=-1 and (x + 1)!=16):
                Cells[x][y].Southern_neighbor = -3
            elif (y + 1)==16 and ((x - 1)!=-1 and (x + 1)!=16):
                Cells[x][y].Northern_neighbor = -3
    queue = Queue()
    while True:
        if Cells[location[0]][location[1]].visited == 0:

            coun += 1
            Cells[location[0]][location[1]].visited = 1

            if (location[0] - 1) == -1 and (location[1] - 1)==-1:

                if forword == 'N':
                    if API.wallFront():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1
                elif forword == 'E':
                    if API.wallFront():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1
                elif forword == 'S':

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1
                elif forword == 'W':
                    if API.wallRight():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1

            elif (location[0] - 1)==-1 and (location[1] + 1)==16:

                if forword == 'N':
                    Cells[location[0]][location[1]].Southern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1


                elif forword == 'W':
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
                    if API.wallLeft():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1

            elif (location[0] + 1)==16 and (location[1] - 1)==-1:

                if forword == 'E':
                    Cells[location[0]][location[1]].Western_neighbor = -1
                    if API.wallLeft():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1

                elif forword == 'S':
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1


            elif (location[0] + 1)==16 and (location[1] + 1)==16:

                if forword == 'N':
                    Cells[location[0]][location[1]].Southern_neighbor = -1
                    if API.wallLeft():

                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1

                elif forword == 'E':
                    Cells[location[0]][location[1]].Western_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1


            elif (location[0] - 1)==-1 and ((location[1] - 1)!=-1 and (location[1] + 1)!=16):

                if forword == 'N':
                    Cells[location[0]][location[1]].Southern_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1
                elif forword == 'S':
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1
                elif forword == 'W':
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1
            elif (location[0] + 1)==16 and ((location[1] - 1)!=-1 and (location[1] + 1)!=16):

                if forword == 'N':
                    Cells[location[0]][location[1]].Southern_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1
                elif forword == 'E':
                    Cells[location[0]][location[1]].Western_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1
                elif forword == 'S':
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1

            elif (location[1] - 1)==-1 and ((location[0] - 1)!=-1 and (location[0] + 1)!=16):
                # Cells[location[0]][location[1]].Southern_neighbor = -3
                if forword == 'E':
                    Cells[location[0]][location[1]].Western_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1
                elif forword == 'S':
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1
                elif forword == 'W':
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1
            elif (location[1] + 1)==16 and ((location[0] - 1)!=-1 and (location[0] + 1)!=16):

                if forword == 'N':
                    Cells[location[0]][location[1]].Southern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1
                elif forword == 'E':
                    Cells[location[0]][location[1]].Western_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1
                elif forword == 'W':
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1
            else:
                if forword == 'N':
                    Cells[location[0]][location[1]].Southern_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1
                elif forword == 'E':
                    Cells[location[0]][location[1]].Western_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1
                elif forword == 'S':
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1

                    if API.wallLeft():
                        Cells[location[0]][location[1]].Eastern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Eastern_neighbor = -1
                elif forword == 'W':
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
                    if API.wallFront():
                        Cells[location[0]][location[1]].Western_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Western_neighbor = -1
                    if API.wallRight():
                        Cells[location[0]][location[1]].Northern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Northern_neighbor = -1
                    if API.wallLeft():
                        Cells[location[0]][location[1]].Southern_neighbor = -2
                    else:
                        Cells[location[0]][location[1]].Southern_neighbor = -1

        indexOfNextMove = 0
        minimeCell = Cells[location[0]][location[1]]
        neighborsCost = get_neighbors_costs(Cells, location)

        if Cells[location[0]][location[1]].cost <= neighborsCost[3] and Cells[location[0]][location[1]].cost <= neighborsCost[2] and Cells[location[0]][location[1]].cost <= neighborsCost[1] and  Cells[location[0]][location[1]].cost <= neighborsCost[0]:

            #Add current cell to queue
            locationInQu=[location[0],location[1]]
            queue.put(locationInQu)
            while not queue.empty():

                #Take Front cell in queue out of line for cosideration
                log("==================================================")
                dequeued_item = queue.get()
                log("- Get From queue")
                neighbors_costs = get_neighbors_costs(Cells, dequeued_item)
                log("neighbors_costs: Above Neighbors" + str(neighbors_costs[0])+", Right Neighbors"+ str(neighbors_costs[1])+", Below Neighbors"+ str(neighbors_costs[2])+", Left Neighbors"+ str(neighbors_costs[3]))
                minimum = neighbors_costs[0]
                for i in neighbors_costs:
                    if i < minimum:
                        minimum=i

                if Cells[dequeued_item[0]][dequeued_item[1]].cost <= minimum:
                    Cells[dequeued_item[0]][dequeued_item[1]].cost = minimum+1
                    if Cells[dequeued_item[0]][dequeued_item[1]].visited == 1:
                        API.setColor(dequeued_item[0], dequeued_item[1], "b")
                    else:
                        API.setColor(dequeued_item[0], dequeued_item[1], "y")
                    API.setText(dequeued_item[0], dequeued_item[1], Cells[dequeued_item[0]][dequeued_item[1]].cost)
                    if Cells[dequeued_item[0]][dequeued_item[1]].Eastern_neighbor != -3 and Cells[dequeued_item[0]][dequeued_item[1]].Eastern_neighbor != -2:
                        locationInQu = [dequeued_item[0] + 1, dequeued_item[1]]
                        queue.put(locationInQu)
                    if Cells[dequeued_item[0]][dequeued_item[1]].Western_neighbor != -3 and Cells[dequeued_item[0]][dequeued_item[1]].Western_neighbor != -2:
                        locationInQu = [dequeued_item[0] - 1, dequeued_item[1]]
                        queue.put(locationInQu)
                    if Cells[dequeued_item[0]][dequeued_item[1]].Northern_neighbor != -3 and Cells[dequeued_item[0]][dequeued_item[1]].Northern_neighbor != -2:
                        locationInQu = [dequeued_item[0], dequeued_item[1] + 1]
                        queue.put(locationInQu)
                    if Cells[dequeued_item[0]][dequeued_item[1]].Southern_neighbor != -3 and Cells[dequeued_item[0]][dequeued_item[1]].Southern_neighbor != -2:
                        locationInQu = [dequeued_item[0], dequeued_item[1] - 1]
                        queue.put(locationInQu)



        elif neighborsCost[0] != 10000:
            minimeCell = Cells[location[0]][location[1]+1]
            indexOfNextMove = 0
        elif neighborsCost[1] != 10000:
            minimeCell = Cells[location[0]+1][location[1]]
            indexOfNextMove = 1
        elif neighborsCost[2] != 10000:
            minimeCell = Cells[location[0]][location[1]-1]
            indexOfNextMove = 2
        elif neighborsCost[3] != 10000:
            minimeCell = Cells[location[0]-1][location[1]]
            indexOfNextMove = 3


        for index,i in enumerate(neighborsCost):
            if i < minimeCell.cost:
                if index == 0:
                    minimeCell = Cells[location[0]][location[1]+1]
                if index == 1:
                    minimeCell = Cells[location[0]+1][location[1]]
                if index == 2:
                    minimeCell = Cells[location[0]][location[1]-1]
                if index == 3:
                    minimeCell = Cells[location[0]-1][location[1]]
                indexOfNextMove = index

        if forword == "N" and indexOfNextMove == 1:
            API.turnRight()
            index = diriction.index(forword)
            if index == 3:  #
                forword = diriction[0]
            else:
                forword = diriction[index + 1]
        elif  forword == "N" and indexOfNextMove == 3:
            API.turnLeft()
            index = diriction.index(forword)
            if index == 0:  #
                forword = diriction[3]
            else:
                forword = diriction[index - 1]
        elif forword == "N" and indexOfNextMove == 2 :
            API.turnRight()
            API.turnRight()
            for k in range(2):
                index = diriction.index(forword)
                if index == 3:  #
                    forword = diriction[0]
                else:
                    forword = diriction[index + 1]

        ######1
        if  forword == "E" and indexOfNextMove == 2:
            API.turnRight()
            index = diriction.index(forword)
            if index == 3:  #
                forword = diriction[0]
            else:
                forword = diriction[index + 1]
        elif  forword == "E" and indexOfNextMove == 0:
            API.turnLeft()
            index = diriction.index(forword)
            if index == 0:  #
                forword = diriction[3]
            else:
                forword = diriction[index - 1]
        elif forword == "E" and indexOfNextMove == 3 :
            API.turnRight()
            API.turnRight()
            for i in range(2):
                index = diriction.index(forword)
                if index == 3:  #
                    forword = diriction[0]
                else:
                    forword = diriction[index + 1]
        #####2
        if  forword == "S" and indexOfNextMove == 3:
            API.turnRight()
            index = diriction.index(forword)
            if index == 3:  #
                forword = diriction[0]
            else:
                forword = diriction[index + 1]
        elif  forword == "S" and indexOfNextMove == 1:
            API.turnLeft()
            index = diriction.index(forword)
            if index == 0:  #
                forword = diriction[3]
            else:
                forword = diriction[index - 1]
        elif forword == "S" and indexOfNextMove == 0 :
            API.turnRight()
            API.turnRight()
            for i in range(2):
                index = diriction.index(forword)
                if index == 3:  #
                    forword = diriction[0]
                else:
                    forword = diriction[index + 1]
        #####3
        if  forword == "W" and indexOfNextMove == 0:
            API.turnRight()
            index = diriction.index(forword)
            if index == 3:  #
                forword = diriction[0]
            else:
                forword = diriction[index + 1]
        elif  forword == "W" and indexOfNextMove == 2:
            API.turnLeft()
            index = diriction.index(forword)
            if index == 0:  #
                forword = diriction[3]
            else:
                forword = diriction[index - 1]
        elif forword == "W" and indexOfNextMove == 1 :
            API.turnRight()
            API.turnRight()
            for i in range(2):
                index = diriction.index(forword)
                if index == 3:  #
                    forword = diriction[0]
                else:
                    forword = diriction[index + 1]
        if Cells[location[0]][location[1]].Northern_neighbor == -2 or Cells[location[0]][location[1]].Northern_neighbor == -3:
                API.setWall(location[0], location[1], "n")
        if Cells[location[0]][location[1]].Eastern_neighbor == -2 or Cells[location[0]][location[1]].Eastern_neighbor == -3:
                API.setWall(location[0], location[1], "e")
        if Cells[location[0]][location[1]].Southern_neighbor == -2 or Cells[location[0]][location[1]].Southern_neighbor == -3:
                API.setWall(location[0], location[1], "s")
        if Cells[location[0]][location[1]].Western_neighbor == -2 or Cells[location[0]][location[1]].Western_neighbor == -3:
                API.setWall(location[0], location[1], "w")
        API.moveForward()

        if forword == 'N':
            location[1] += 1
        elif forword == 'E':
            location[0] += 1
        elif forword == 'S':
            location[1] -= 1
        elif forword == 'W':
            location[0] -= 1
        API.setColor(location[0], location[1], "G")

        if location in Goals:
            if Cells[location[0]][location[1]].Northern_neighbor == -2 or Cells[location[0]][
                location[1]].Northern_neighbor == -3:
                API.setWall(location[0], location[1], "n")
            if Cells[location[0]][location[1]].Eastern_neighbor == -2 or Cells[location[0]][
                location[1]].Eastern_neighbor == -3:
                API.setWall(location[0], location[1], "e")
            if Cells[location[0]][location[1]].Southern_neighbor == -2 or Cells[location[0]][
                location[1]].Southern_neighbor == -3:
                API.setWall(location[0], location[1], "s")
            if Cells[location[0]][location[1]].Western_neighbor == -2 or Cells[location[0]][
                location[1]].Western_neighbor == -3:
                API.setWall(location[0], location[1], "w")
            log("Finish - :)")
            exit()


if __name__ == "__main__":
    main()
