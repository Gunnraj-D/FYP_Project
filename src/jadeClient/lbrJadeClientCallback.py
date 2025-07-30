from jadeClient.clientJVM import startJVM
startJVM()

from jadevep.client import LBRClientCallback
from jpype import JImplements, JOverride

@JImplements(LBRClientCallback)
class LBRJadeClientCallback:

    def __init__(self):
        self._onStateChange = None
        self._onMonitor = None
        self._onWaitForCommand = None
        self._onCommand = None

    @JOverride
    def onStateChange(self, client, oldState, newState):
        print("State changed from %s to %s" % (oldState, newState))
        if self._onStateChange:
            self._onStateChange(client, oldState, newState)

    @JOverride
    def monitor(self, client):
        if self._onMonitor:
            self._onMonitor(client)

    @JOverride
    def waitForCommand(self, client):
        if self._onWaitForCommand:
            self._onWaitForCommand(client)

    @JOverride
    def command(self, client):
        if self._onCommand:
            self._onCommand(client)

    def setOnStateChange(self, callback):
        self._onStateChange = callback
    
    def setOnMonitor(self, callback):
        self._onMonitor = callback
    
    def setOnWaitForCommand(self, callback):
        self._onWaitForCommand = callback
    
    def setOnCommand(self, callback):
        self._onCommand = callback