package application;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.*;

public class FYP_Human_Collab extends RoboticsAPIApplication {
    private LBR lbr;
    
    @Inject
    private OPCUA_Client_Manager OPCUAManager; 

    @Inject
//    @Named("RobotiqGripper")
    private Gripper_test gripper;
    
    private float currentWidth = -1;
    private float currentForce = -1;
    
    @Override
    public void initialize() {
        lbr = getContext().getDeviceFromType(LBR.class);
        float[] initialisationResults = gripper.initialise();
		currentWidth = initialisationResults[0];
	    currentForce = initialisationResults[1];
	    gripper.setOffset(3);
        gripper.close();
        
    }

    @Override
    public void run() {
    	JointPosition initialPosition = new JointPosition(
                0.0, 0.2, 0.0, 1.5, 0.0, 1.5, 0.0
            );

        // Create and activate the SmartServo motion
        SmartServo smartServoMotion = new SmartServo(initialPosition);
        lbr.moveAsync(smartServoMotion);
    	
        ISmartServoRuntime smartServoRuntime = smartServoMotion.getRuntime();
        
        while (OPCUAManager.hasStarted() && OPCUAManager.isConnected()) {
			JointPosition targetPosition = OPCUAManager.getTargetJointPosition();
			if (targetPosition != null) {
				smartServoRuntime.setDestination(targetPosition);
			}
			
			ThreadUtil.milliSleep(20);
		}
        
        smartServoRuntime.stopMotion();
        
    	// how to use gripper
//        gripper.readOffset();
//        gripper.moveTo(575);
//        gripper.readWidth();
    }
}