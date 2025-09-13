package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.connectivity.fastRobotInterface.*;
//import com.kuka.connectivity.fri.*;
//import com.kuka.connectivity.fastRobotInterface.*;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
//import java.util.concurrent.TimeUnit;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.task.ITaskLogger;

public class FRI_Control extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
//	@Inject 
//	private Gripper2F gripper2F1;
	@Inject
	private MediaFlangeIOGroup mF;
	@Inject
	@Named("RobotiqGripper")
	private Tool gripper;
	@Inject
	private ITaskLogger logger;
	
	private FRISession friSession;
	private String pc_ip = "172.24.30.102";
//	private int pc_port = 8080;
	private int send_period_ms = 20;
	
	@Override
	public void initialize() {
//		_lbrController = (Controller) getContext().getControllers().toArray()[0];
//        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
	}
	
	public void run() {		
		FRIConfiguration friconfiguration = FRIConfiguration.createRemoteConfiguration(robot, pc_ip);
//		friconfiguration.setPortOnRemote(pc_port);
		friconfiguration.setSendPeriodMilliSec(send_period_ms);
		
		logger.info("Creating FRI Session...");
		
		friSession = new FRISession(friconfiguration);
		FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession);
		
		try
        {
            friSession.await(20, TimeUnit.SECONDS);
        }
        catch (TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }
		
		logger.info("FRI Session creation successful. Starting streaming...");
		
		PositionControlMode ctrl = new PositionControlMode();
		
		while(true){
        	try{
        		robot.moveAsync(positionHold(ctrl, -1, TimeUnit.SECONDS).addMotionOverlay(jointOverlay));
        	} catch(Exception e){
        		getLogger().error(e.getMessage());
        		break;
        	}
        }
		
		friSession.close();
		
		logger.info("FRI Session Closed");
	}
	
}
    