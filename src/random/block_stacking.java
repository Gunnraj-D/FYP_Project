package application;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.executionModel.IFiredConditionInfo;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.CartPlane;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineMotionCP;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.geometry.ObjectFrame;

import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;
 

/**
* Implementation of a robot application.
* <p>
* The application provides a {@link RoboticsAPITask#initialize()} and a 
* {@link RoboticsAPITask#run()} method, which will be called successively in 
* the application lifecycle. The application will terminate automatically after 
* the {@link RoboticsAPITask#run()} method has finished or after stopping the 
* task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
* exception is thrown during initialization or run. 
* <p>
* <b>It is imperative to call <code>super.dispose()</code> when overriding the 
* {@link RoboticsAPITask#dispose()} method.</b> 
* 
* @see UseRoboticsAPIContext
* @see #initialize()
* @see #run()
* @see #dispose()
*/
public class FYP_HRC_2025 extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	@Inject 
	private Gripper2F gripper2F1;
	@Inject
	private MediaFlangeIOGroup mF;
	@Inject
	@Named("RobotiqGripper")
	private Tool gripper;
	@Inject
	private ITaskLogger logger;
	@Inject
	private MediaFlangeIOGroup mediaFlange;
	private CartesianImpedanceControlMode springMode = new CartesianImpedanceControlMode();
 
	@Override
	public void initialize() {		
		mF.setLEDBlue(true);
		gripper.attachTo(robot.getFlange());
		gripper2F1.setSpeed(155);
		gripper2F1.setForce(155);
		springMode.parametrize(CartDOF.X,
				CartDOF.Y).setStiffness(5000);
		springMode.parametrize(CartDOF.Z).setStiffness(200);
		springMode.parametrize(CartDOF.ROT).setStiffness(300);
	}
	
	public boolean checkForBlock() {
		final double FORCE_THRESHOLD = 9;
		final double VELOCITY = 0.035; 
		
		ForceCondition forceCondition = ForceCondition.createNormalForceCondition(
				gripper.getFrame("/TCP"), 
				CoordinateAxis.Z, 
				FORCE_THRESHOLD);
		
//		ForceCondition forceCondition = ForceCondition.createSpatialForceCondition(
//				 gripper.getFrame("/TCP"),
//				 FORCE_THRESHOLD);
		
		logger.info("We are about to check for block");
		
		IMotionContainer blockCheck = gripper.move(linRel(0.0, 0.0, 30).setMode(springMode).setJointVelocityRel(VELOCITY).breakWhen(forceCondition));
		
		
		if (!blockCheck.hasFired(forceCondition)) {
			logger.info("We have checked for block and it was a failure");
			return false;
		}
		
		logger.info("We have checked for block and it was successful");
		logger.info("Fired Condition for checking: " + blockCheck.getFiredBreakConditionInfo().toString());
		
		return true;
	}
	
	public void waitForUserInput() {
		while (!mediaFlange.getUserButton()) {
			ThreadUtil.milliSleep(200);
		}
	}
	
	public void moveDownAndPickUp() {
		gripper2F1.open();
		gripper.move(linRel(0.0, 0.0, 50));
		gripper2F1.close();
	}
	
	public void placeBlock() {
		final double FORCE_THRESHOLD = 9;
		final double VELOCITY = 0.05;
		
		ForceCondition forceCondition = ForceCondition.createSpatialForceCondition(
				 gripper.getFrame("/TCP"),
				 FORCE_THRESHOLD);
		
//		ForceCondition forceCondition = ForceCondition.createNormalForceCondition(
//				gripper.getFrame("/TCP"), 
//				CoordinateAxis.Z, 
//				FORCE_THRESHOLD);
		
		robot.move(linRel(0.0, 0.0, 200).setJointVelocityRel(VELOCITY).breakWhen(forceCondition));
		
//		logger.info("Fired Condition for placing stack: " + placeMovement.getFiredBreakConditionInfo().toString());
		
		gripper2F1.open();
		robot.move(lin(getApplicationData().getFrame("/Alt_Stack")).setJointVelocityRel(0.4));
	}	
	
	public void stackBlocks(List<Frame> frames) {
		for (Frame frame : frames) {
			robot.move(lin(frame).setJointVelocityRel(0.5));
			gripper2F1.close();
			
			boolean blockExists = false;
			while (!blockExists) {
				ThreadUtil.milliSleep(200);
				blockExists = checkForBlock();
				robot.move(ptp(frame).setJointVelocityRel(0.4));
				if (!blockExists) {
					setLEDColour('r');
					waitForUserInput();
					setLEDColour('b');
					ThreadUtil.milliSleep(500);
				}
			}
		    moveDownAndPickUp();
		    robot.move(lin(frame).setJointVelocityRel(0.8));
		    robot.move(lin(getApplicationData().getFrame("/Alt_Stack")).setJointVelocityRel(0.8));
		    placeBlock();
		}
	}
	
	public List<Frame> collectFrames() {
		final int MS_BETWEEN_FRAMES = 300;
		
		
		robot.move(lin(getApplicationData().getFrame("/Alt_Start")).setJointVelocityRel(0.4));
				
		CartesianImpedanceControlMode cartImpCtrlMode = new
				CartesianImpedanceControlMode();
		
		cartImpCtrlMode.parametrize(CartDOF.X,
				CartDOF.Y).setStiffness(0).setDamping(1);
		cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(5000).setDamping(1).setAdditionalControlForce(20); 
		cartImpCtrlMode.parametrize(CartDOF.ROT).setStiffness(300).setDamping(1);
		
		IMotionContainer holdPosition = gripper.moveAsync(positionHold(cartImpCtrlMode, -1, TimeUnit.SECONDS));
		
		List<Frame> frames = new ArrayList<Frame>();
		
		int i = 3;
		setLEDColour('g');
		while (i > 0) {
			waitForUserInput();
			Frame currentPos = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
//			Frame currentPos = gripper.getFrame("/TCP");
			currentPos.setZ(250.8);
			currentPos.setAlphaRad(0);
			currentPos.setBetaRad(0);
			currentPos.setGammaRad(3.1415);
			frames.add(currentPos);
			i--;
			ThreadUtil.milliSleep(MS_BETWEEN_FRAMES);
		}
		setLEDColour('b');
		holdPosition.cancel();
		return frames;
	}
	
	public void setLEDColour(char colour) {
		if (colour == 'g') {
			mF.setLEDRed(false);
			mF.setLEDBlue(false);
			mF.setLEDGreen(true);
		}
		else if (colour == 'b') {
			mF.setLEDRed(false);
			mF.setLEDBlue(true);
			mF.setLEDGreen(false);
		}
		else {
			mF.setLEDRed(true);
			mF.setLEDBlue(false);
			mF.setLEDGreen(false);
		}
	}
	
	// block placing
	@Override
	public void run() {
	    // Move to home position with r2wed light
//		logger.info(Double.toString(getApplicationData().getFrame("/Alt_Start").getGammaRad()));
		gripper2F1.close();
		
		setLEDColour('b');
		
		List<Frame> frames = collectFrames();
		
		ThreadUtil.milliSleep(2000);
		
		stackBlocks(frames);
		
		setLEDColour('r');
		
//	    mF.setLEDRed(true);
//	    robot.move(ptp(getApplicationData().getFrame("/HOME")).setJointVelocityRel(0.4));
//
//	    // Move through pick-up sequence
	    
	    
	    
//	    robot.move(lin(getApplicationData().getFrame("/P3")).setJointVelocityRel(0.1));
//	    gripper2F1.moveTo(180);
//	    robot.move(lin(getApplicationData().getFrame("/P4")).setJointVelocityRel(0.4));
//	    gripper2F1.close();
//	    robot.move(lin(getApplicationData().getFrame("/P5")).setJointVelocityRel(0.4));
//	    robot.move(ptp(getApplicationData().getFrame("/P6")).setJointVelocityRel(0.4));
//	    robot.move(ptp(getApplicationData().getFrame("/P7")).setJointVelocityRel(0.4));
//
////	    // Get the current position of the robot (base frame or sensor)
////	    Frame currentRobotFrame = robot.getCurrentCartesianPosition(null);  // Assuming `getCurrentFrame()` gives the robot's base frame
////
////	    // Dynamically create a reference frame based on the robot's position
////	    Frame referenceFrame = currentRobotFrame;
////
////	    // Create a sensor frame with an offset from the reference frame (for example, 0.1m in the X direction)
////	    Frame sensorFrame = currentRobotFrame;  // 0.1m offset in X direction for the sensor
////
////	    // Set force condition to detect a pulling force of 15N in positive X direction
////	    ForceCondition condition = ForceCondition.createSpatialForceCondition(
////	        referenceFrame,  // Using the dynamically created reference frame
////	        sensorFrame,     // Using the dynamically created sensor frame
////	        15.0,            // force threshold in Newtons
////	        5.0              // torque threshold in Nm
////	    );
////
////	    // Move in place, waiting for pull
////	    IMotionContainer holdMotion = robot.moveAsync(
////	        linRel(0, 0, 0, World.Current.getRootFrame())  // Relative motion with respect to the root frame
////	            .setMode(springRobot)  // Applying the springRobot mode
////	            .breakWhen(condition)  // Break the motion when the force condition is met
////	    );
//
////	    // Wait for the force to be triggered
////	    holdMotion.await();
//	    Frame initialPos = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
////	    Frame initialPos = gripper.getDefaultMotionFrame().copyWithRedundancy(); // Get current tool frame
////	    gripper.move(ptp(0.0,0.785398,0.0,-1.13446,0.0,-0.436332,1.5708).setJointVelocityRel(0.2));
//
//	    m1 = gripper.moveAsync(positionHold(springRobot, -1, TimeUnit.SECONDS)); // hold position
//	    Frame currentPos = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
//	    
//	    while (true) {
//	        ThreadUtil.milliSleep(200); // check every 200ms
//
//	        currentPos = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
//	        double difference = currentPos.getX() - initialPos.getX();
//	        logger.info(String.format("=%.4f,", difference));
//	        double dx = Math.abs(currentPos.getX() - initialPos.getX());
//	        double dy = Math.abs(currentPos.getY() - initialPos.getY());
//	        double dz = Math.abs(currentPos.getZ() - initialPos.getZ());
//
//	        if (dx > 30 || dy > 30 || dz > 30) {
//	            logger.info(String.format("Motion detected: dx=%.4f, dy=%.4f, dz=%.4f", dx, dy, dz));
//	            break;
//	        }
//	    }
//
//	    m1.cancel(); // stop holding position
//
//	    // Pull detected, respond by releasing business card
//	    gripper2F1.moveTo(180); // Open gripper
//	    mF.setLEDGreen(true);   // Signal success
//	    ThreadUtil.milliSleep(2000); // Hold position
//	    gripper2F1.close();
//	    // Return to home
//	    mF.setLEDRed(true);
//	    robot.move(ptp(getApplicationData().getFrame("/HOME")).setJointVelocityRel(0.2));
//
//	    // Set light to blue to indicate reset
//	    mF.setLEDBlue(true);
	}

}