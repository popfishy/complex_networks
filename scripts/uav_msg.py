class UAVMsg:
    def __init__(self):
        self.removeFlag = False
        self.stepCount = 0
        self.delta_msgLength = None

    def __init__(self, shortPath):
        self.shortPath = shortPath
        self.soureID = shortPath[0]
        self.nowID = shortPath[0]
        self.targetID = shortPath[-1]
        self.removeFlag = False
        self.stepCount = 0
        self.delta_msgLength = None

    def updateShortPath(self, shortPath):
        self.shortPath = shortPath
        self.nowID = shortPath[1]
        self.stepCount = self.stepCount + 1
        if self.nowID == self.targetID or self.stepCount > 10:
            self.removeFlag = True

    def isArrived(self):
        return self.removeFlag

    def getNextStationID(self):
        pass

    def moveMsg(self):
        pass

    def remove(self):
        pass
