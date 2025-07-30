import numpy as np

class JointController:

    JOINT_STEP = 0.01

    def __init__(self):
        self.targetJointValues = None
        self.waypointJointValues = None

    def onStateChange(self, client, oldState, newState):
        client.superOnStateChange(oldState, newState)

    def monitor(self, client):
        client.superMonitor()
        self.update(client)

    def waitForCommand(self, client):
        client.superWaitForCommand()
        self.update(client)

    def command(self, client):
        client.superCommand()
        self.update(client)
        client.getRobotCommand().setJointPosition(self.waypointJointValues)
    
    def update(self, client):
        if self.targetJointValues is None:
            self.targetJointValues = np.array(client.getRobotState().getMeasuredJointPosition())
            self.waypointJointValues = np.array(client.getRobotState().getMeasuredJointPosition())
        
        currentJointValues = np.array(client.getRobotState().getMeasuredJointPosition())
        jointDifferences = self.targetJointValues-currentJointValues
        maxDifference = np.max(np.abs(jointDifferences))
        scaleFactor = 1
        if maxDifference > self.JOINT_STEP:
            scaleFactor = self.JOINT_STEP/np.max(jointDifferences)
        jointDifferences = jointDifferences*scaleFactor
        self.waypointJointValues = currentJointValues+jointDifferences
    
    def setTargetJointValues(self, targetJointValues):
        self.targetJointValues = targetJointValues