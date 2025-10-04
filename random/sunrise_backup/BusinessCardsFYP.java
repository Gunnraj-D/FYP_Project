package application;
 
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;
import javax.inject.Named;
//import com.kuka.math.geometry.Vector3D;
import com.kuka.math.geometry.Vector3D;
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
public class BusinessCardsFYP extends RoboticsAPIApplication {
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
	CartesianSineImpedanceControlMode springRobot;
	IMotionContainer m1;
 
	@Override
	public void initialize() {
		//Spring motion initialisation
		springRobot = new CartesianSineImpedanceControlMode(); 
		
//		springRobot.parametrize(CartDOF.X).setStiffness(500);
//		springRobot.parametrize(CartDOF.Y).setStiffness(1000);
//		springRobot.parametrize(CartDOF.Z).setStiffness(200);
//		springRobot.parametrize(CartDOF.C).setStiffness(50);
//		springRobot.parametrize(CartDOF.B).setStiffness(50);
//		springRobot.parametrize(CartDOF.A).setStiffness(300);
//		springRobot.setReferenceSystem(World.Current.getRootFrame());
//		springRobot.parametrize(CartDOF.ALL).setDamping(0.4);
		springRobot.parametrize(CartDOF.Z).setStiffness(4000);
		springRobot.parametrize(CartDOF.Z).setBias(5);
		
		springRobot.parametrize(CartDOF.A).setFrequency(5);
		springRobot.parametrize(CartDOF.A).setAmplitude(15);
		springRobot.parametrize(CartDOF.A).setStiffness(10);
		mF.setLEDBlue(false);
		gripper.attachTo(robot.getFlange());
	}
	
	@Override
	public void run() {
//		robot.move(lin(0.0,0.785398,0.0,-1.13446,0.0,-0.436332,1.5708).setJointVelocityRel(0.2));//.setMode(springRobot));
//		robot.move(ptp(0.0,0.785398,0.0,-1.13446,0.0,-0.436332,1.5708).setJointVelocityRel(0.2));//.setMode(springRobot));
		while(true){
			robot.move(ptp(getApplicationData().getFrame("/P4")).setJointVelocityRel(0.3));//.setMode(springRobot));
			robot.move(ptp(getApplicationData().getFrame("/P5")).setJointVelocityRel(0.3));
			robot.move(ptp(getApplicationData().getFrame("/P6")).setJointVelocityRel(0.3));
			robot.move(ptp(getApplicationData().getFrame("/P7")).setJointVelocityRel(0.3));
//			robot.move(ptp(getApplicationData().getFrame("/P8")).setJointVelocityRel(0.2));
//			robot.move(ptp(getApplicationData().getFrame("/P9")).setJointVelocityRel(0.2));
//			robot.move(ptp(getApplicationData().getFrame("/P10")).setJointVelocityRel(0.2));
		}
//		gripper.getFrame("/TCP").move(linRel(0,0,10).setCartVelocity(1).setMode(springRobot));
//		robot.move(ptp(0.0,0.785398,0.0,-1.13446,0.0,-0.436332,1.5708).setJointVelocityRel(0.2));//.setMode(springRobot));

	}
}