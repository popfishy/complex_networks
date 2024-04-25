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

    def updateShortPath(self, newshortPath):
        self.shortPath = newshortPath
        self.nowID = newshortPath[0]
        self.stepCount = self.stepCount + 1
        # TODO 注意self.stepCount > 10条件
        if self.nowID == self.targetID or self.stepCount > 10:
            self.removeFlag = True

    def updateNowID(self):
        self.nowID = self.shortPath[1]

    def isArrived(self):
        return self.removeFlag
