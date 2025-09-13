
package application;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
 
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;
import javax.inject.Named;
//import com.kuka.math.geometry.Vector3D;
//import com.kuka.math.geometry.Vector3D;
//import com.kuka.nav.command.CommandContainer;
//import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

//import com.kuka.roboticsAPI.capabilities.honk.IHonkCapability;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.ITriggerAction;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.CartPlane;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IMotionOverlay;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineMotionCP;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.motionModel.velocityControlled.CartesianVelocityDataSource;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
//import com.kuka.geometry.ObjectFrame;
//import com.kuka.roboticsAPI.conditionModel.ForceCondition;
 
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;
 
public class ProgramSelect extends RoboticsAPIApplication {
	
	@Inject
	private OPCUA_Client_Control2 OPCUA;
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
	CartesianImpedanceControlMode cartImpCtrlMode;
	IMotionContainer m1;
 
	@Override
	public void initialize() {
		cartImpCtrlMode = new CartesianImpedanceControlMode(); 
//		PositionControlMode positionMode = new PositionControlMode();
//		positionMode.
		
		
		// Set up robot params
		cartImpCtrlMode.parametrize(CartDOF.X,
				CartDOF.Y).setStiffness(0).setDamping(1);
		cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(5000).setDamping(1); 
		cartImpCtrlMode.parametrize(CartDOF.ROT).setStiffness(300).setDamping(1);
//		springMode.parametrize(CartDOF.ALL).setDamping(0.4);
//		springRobot.setAdditionalControlForce();
//		PositionHold positionHold = positionHold()
//		robot.moveAsync(positionHold(springRobot))
		
		mF.setLEDBlue(false);
		gripper.attachTo(robot.getFlange());
	    gripper2F1.setForce(100);
	    
	    // Connect to server
		logger.info("Connecting to OPC UA Local Server...");
		try {
			OPCUA.SetUp();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		try {
			OPCUA.ServerUpdate();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		try {
			OPCUA.setEnd(false);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void home_position() {
		double x = 430.0;  // Position X in mm
        double y = 180.0;  // Position Y in mm
        double z = 600.0;  // Position Z in mm
        double a = -3.14;    // Rotation A in radians
        double b = 0.0;    // Rotation B in radians
        double c = 3.14;    // Rotation C in radians

        Frame currentPos = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
        
        currentPos.setX(x);
        currentPos.setY(y);
		currentPos.setZ(z);
		currentPos.setAlphaRad(a);
		currentPos.setBetaRad(b);
		currentPos.setGammaRad(c);
		
		robot.move(ptp(currentPos).setJointVelocityRel(0.2));

	}
	
	public void move_to(double x_dist, double y_dist, double z_dist) {
//		z_dist = 0.0;
//		double magnitude = Math.sqrt(Math.pow(x_dist, 2) + Math.pow(y_dist, 2) + Math.pow(z_dist, 2));
//		double velocity = (max_distance - magnitude) / max_distance;
//		velocity = velocity / 2;
		try {
			if (x_dist == 0 && y_dist == 0 && z_dist == 0) {
				return;
			}

//			cartImpCtrlMode.parametrize(CartDOF.X).setAdditionalControlForce(x_dist / 50);
//			cartImpCtrlMode.parametrize(CartDOF.Y).setAdditionalControlForce(y_dist / 50);
////			springMode.parametrize(CartDOF.X).setAdditionalControlForce(z_dist / 10);
//			robot.moveAsync(positionHold(cartImpCtrlMode, 100, TimeUnit.MICROSECONDS));
			
			
//			Frame currentPos = robot.getCurrentCartesianPosition(robot.getFlange());
//			
//			currentPos.setX(currentPos.getX() + x_dist);
//			currentPos.setY(currentPos.getY() + y_dist);	
//			
//			robot.moveAsync(ptp(currentPos).setBlendingRel(1.0).setJointVelocityRel(0.1));
			
			robot.moveAsync(linRel(x_dist, y_dist, z_dist).setBlendingRel(1.0));
		}
		catch (CommandInvalidException e) {
			logger.error("Cannot move that far!!");
		}
	}
	

	public void blendTest() {
		Frame currentPos = robot.getCurrentCartesianPosition(robot.getFlange());
		for (int i = 0; i < 5; i++) {
		    Frame next = currentPos.copy();
		    next.setX(next.getX() + 10 * i);  // shift
		    next.setY(next.getY() + 10 * i);
		    robot.moveAsync(ptp(next).setBlendingRel(1.0));
		}
	}
	
	// run switch statement to select program
	@Override
	public void run() throws Exception {
		logger.info("Moving to Base Position...");
		home_position();
//		blendTest();
//		home_position();
//		double max_distance = 400.0;
		OPCUA.ServerUpdate();
		logger.info("Entering Main Program");
		
		// While loop
		while(OPCUA.Connected){
			long startTime = System.nanoTime();
			OPCUA.ServerUpdate();
			OPCUA.setReady(true);
			
			// Connection check
			if(OPCUA.Start==true){
//				logger.info("Checking program...");
				OPCUA.setReady(true);
				OPCUA.setEnd(false);
				
				// Program select
				switch(OPCUA.ProgID){
				case 1:
//					logger.info("ProgID = 1: Moving hand");
//					mF.setLEDBlue(true);
//					logger.info(String.format("X - coord: %d, Y - coord: %d, Z - coord: %d", OPCUA.PosX, OPCUA.PosY, OPCUA.PosZ));
					move_to(OPCUA.PosX, OPCUA.PosY, OPCUA.PosZ);
					break;
//				case 2:
//					logger.info("ProgID = 2: Starting open window...");
//					mF.setLEDBlue(true);
//					open_window();
//					mF.setLEDBlue(false);
//					break;
					
				default:
					break;
				} 
				
				//Program Complete Routine
//				OPCUA.setEnd(true);
//				OPCUA.setProgID(0);
//				OPCUA.setStart(false); 
//				ThreadUtil.milliSleep(20); // Formally 1500 ms
//				OPCUA.setReady(true);
////				ThreadUtil.milliSleep(20); // Formally 1500 ms
////				mF.setLEDGreen(false);
//				OPCUA.setEnd(false);
//				logger.info("Communication Signals Reset");
				
			} // End If (Start)
			if (OPCUA.Connected==false){break;} 
			

			long endTime = System.nanoTime();
			logger.info("Loop duration: " + (endTime - startTime)/1000000 + " ms");
			
		}// End While connected
		if (OPCUA.Connected==false){logger.info("Shutting Down Automatic Mode");OPCUA.clientDisconnect();}
	}//END RUN
}


