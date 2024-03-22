#Jehad Hamayel 1200348
#Musab Masalmah 1200078
#Abdalkarim Eiss 1200015
import API
import sys
import cell
def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main():

    log("Running...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "Start")

    Cells = [[cell.cell(visited=0) for _ in range(16)] for _ in range(16)]
    location = [0,0] #x y
    Goals = [[7,7],[7,8],[8,7],[8,8]]
    diriction = ['N', 'E', 'S', 'W']
    forword = 'N'

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
    while True:
        if Cells[location[0]][location[1]].visited == 0:
            log(f"Vistit Cell({location[0]},{location[1]})")
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


        if Cells[location[0]][location[1]].Northern_neighbor == -2 or Cells[location[0]][location[1]].Northern_neighbor == -3:
                API.setWall(location[0], location[1], "n")
        if Cells[location[0]][location[1]].Eastern_neighbor == -2 or Cells[location[0]][location[1]].Eastern_neighbor == -3:
                API.setWall(location[0], location[1], "e")
        if Cells[location[0]][location[1]].Southern_neighbor == -2 or Cells[location[0]][location[1]].Southern_neighbor == -3:
                API.setWall(location[0], location[1], "s")
        if Cells[location[0]][location[1]].Western_neighbor == -2 or Cells[location[0]][location[1]].Western_neighbor == -3:
                API.setWall(location[0], location[1], "w")

        if not API.wallRight():
                API.turnRight()
                index = diriction.index(forword)
                if index == 3:  #
                    forword = diriction[0]
                else:
                    forword = diriction[index + 1]
        while API.wallFront():
            API.turnLeft()
            index = diriction.index(forword)
            if index == 0:  #
                forword = diriction[3]
            else:
                    forword = diriction[index - 1]
        if forword == 'N':
            location[1] += 1
        elif forword == 'E':
            location[0] += 1
        elif forword == 'S':
            location[1] -= 1
        elif forword == 'W':
            location[0] -= 1

        API.moveForward()
        API.setColor(location[0], location[1], "G")
        if location in Goals:
            log("Finish")
            exit()



if __name__ == "__main__":
    main()
