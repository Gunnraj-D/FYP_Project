import jpype
import jpype.imports

JVM_STARTED = False

def startJVM():
    global JVM_STARTED
    if JVM_STARTED: return
    print("Starting JVM...")
    jpype.startJVM(classpath=[
        "jlib/com.kuka.connectivity.fastRobotInterface.clientSDK_1.15.1.Con1-17-0_2.jar",
        "jlib/protobuf-java-2.5.0.jar",
        "jlib/LBRClientWrapper.jar"])
    JVM_STARTED = True

def stopJVM():
    global JVM_STARTED
    if not JVM_STARTED: return
    print("Stopping JVM...")
    jpype.shutdownJVM()
    JVM_STARTED = False