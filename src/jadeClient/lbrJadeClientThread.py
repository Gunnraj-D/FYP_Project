from jadeClient.clientJVM import startJVM
startJVM()

import threading

from com.kuka.connectivity.fastRobotInterface.clientSDK.connection import UdpConnection
from com.kuka.connectivity.fastRobotInterface.clientSDK.base import ClientApplication
from jadevep.client import LBRClientWrapper
import jpype
import time

class LBRJadeClientThread(threading.Thread):
    def __init__(self, hostname, port):
        threading.Thread.__init__(self)
        self.isRunning = False

        self.hostname = hostname
        self.port = port

        self.client = LBRClientWrapper()
        self.connection = UdpConnection()
        self.app = ClientApplication(self.connection, self.client)
        self.app.connect(self.port, self.hostname)

    def run(self):
        print("Starting client thread...")
        self.isRunning = True
        start_time = time.time()
        steps = 0
        while self.isRunning:
            self.step()
            steps += 1
            current_time = time.time()
            if current_time - start_time > 1.0:
                step_rate = steps / (current_time - start_time)
                print("Step rate: %.2f steps/s" % step_rate)
                start_time = current_time
                steps = 0
        self.stop()
        print("Stopped client thread.")
    
    def step(self):
        try:
            success = self.app.step()
            if not success:
                self.isRunning = False
        except jpype.JException as e:
            e.printStackTrace()
            self.isRunning = False

    def stop(self):
        if not self.isRunning: return
        print("Stopping client thread...")
        self.app.disconnect()
        self.isRunning = False

    def addClientCallback(self, callback):
        self.client.setCallback(callback)

    

