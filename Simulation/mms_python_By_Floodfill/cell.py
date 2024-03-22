

class cell:
    Northern_neighbor = -1
    Eastern_neighbor = -1
    Southern_neighbor = -1
    Western_neighbor = -1

    cost = -1
    def __init__(self,visited):
        self.visited=visited
    def printInfo(self):
        return (f"----\nNorthern_neighbor = {self.Northern_neighbor}\n"
                f"Eastern_neighbor = {self.Eastern_neighbor}\n"
                f"Southern_neighbor = {self.Southern_neighbor}\n"
                f"Western_neighbor = {self.Western_neighbor}\n----\n")