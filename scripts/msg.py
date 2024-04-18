class Msg:
    def __init__(self):
        self.soureID = None
        self.targetID = None
        self.removeFlag = False
        self.stepCount = 0
        self.delta_msgLength = None

    def getShortPath(self):
        pass

    def getNextStationID(self):
        pass

    def moveMsg(self):
        pass

    def remove(self):
        pass
