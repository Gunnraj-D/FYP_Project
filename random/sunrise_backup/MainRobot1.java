//package application;
//
//import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
//import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
//import java.util.concurrent.TimeUnit;
//import javax.inject.Inject;
//import javax.inject.Named;
//import com.kuka.common.ThreadUtil;
//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
//import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
//import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.geometricModel.CartDOF;
//import com.kuka.roboticsAPI.geometricModel.Frame;
//import com.kuka.roboticsAPI.geometricModel.Tool;
//import com.kuka.roboticsAPI.geometricModel.World;
//import com.kuka.roboticsAPI.motionModel.IMotionContainer;
//import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
//import com.kuka.task.ITaskLogger;
//
//public class MainRobot1 extends RoboticsAPIApplication {
//	@Inject
//	private LBR robot; 
//
//	@Inject
//	private MediaFlangeIOGroup mF;
//	
////	@Inject 
////	private Gripper2F gripper2F1;
//	
//	@Inject
//	@Named("RobotiqGripper")
//	private Tool gripper;
//	
//	@Inject
//	private ITaskLogger logger;		//Printing in non background tasks
//	
//	@Inject
//	private OPCUA_Client_Control1 OPCUA;
//	
//	
//	CartesianImpedanceControlMode springRobot;
//	IMotionContainer m1;
//	
//	@Override
//	public void initialize() {
//		springRobot = new CartesianImpedanceControlMode(); 
//		
//		
//		springRobot.parametrize(CartDOF.X).setStiffness(500);
//		springRobot.parametrize(CartDOF.Y).setStiffness(1000);
//		springRobot.parametrize(CartDOF.Z).setStiffness(200);
//		springRobot.parametrize(CartDOF.C).setStiffness(50);
//		springRobot.parametrize(CartDOF.B).setStiffness(50);
//		springRobot.parametrize(CartDOF.A).setStiffness(300);
//		springRobot.setReferenceSystem(World.Current.getRootFrame());
//		springRobot.parametrize(CartDOF.ALL).setDamping(0.4);
//		
//		logger.info("Initalizing Automatic Mode");
//		logger.info("Connecting to OPC UA Local Server...");
//		try {
//			OPCUA.SetUp();
//		} catch (Exception e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		try {
//			OPCUA.ServerUpdate();
//		} catch (Exception e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
////		try {
////			OPCUA.setGripperControl(false);
////		} catch (Exception e1) {
////			// TODO Auto-generated catch block
////			e1.printStackTrace();
////		}
//		gripper.attachTo(robot.getFlange()); //Gripper attached to flange, move robot by moving gripper!
////		mF.setLEDRed(true);
////		mF.setLEDGreen(true);
//		mF.setLEDBlue(true);
//		ThreadUtil.milliSleep(1000);
////		mF.setLEDRed(false);
////		mF.setLEDGreen(false);
//		mF.setLEDBlue(false);
////		logger.info("Initalising Gripper...");
////		gripper2F1.initalise();
////		gripper2F1.setSpeed(255);
//		ThreadUtil.milliSleep(100);
//		mF.setLEDBlue(true);
//		try {
//			OPCUA.setEnd(false);
//		} catch (Exception e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//	}
//
//	@Override
//	public void run() throws Exception {
//		OPCUA.ServerUpdate();
//		logger.info("Entering Main Program");
//		while(OPCUA.Connected){
//			OPCUA.ServerUpdate();
//			OPCUA.setReady(true);
//			if (OPCUA.Connected==false){break;}
//			if(OPCUA.Start==true){
////				mF.setLEDGreen(true);
//				OPCUA.setReady(true);
//				OPCUA.setEnd(false);
//				switch(OPCUA.ProgID){
//				case 0:
//					logger.error("Program 0 Started");
//					break;
//				case 1: // Program 1 Demo Light/Gripper Communication
//					logger.info("Program 1 Started");
//					logger.info("Program 1 Complete");
//					break;
//				case 2:	// Program 2 ptp move
//					logger.info("Program 2 Started");
//					OPCUA.ServerUpdate();
//					if (OPCUA.Connected==false){break;}
////					try{
//					gripper.move(ptp(Math.toRadians(OPCUA.Joi1),Math.toRadians(OPCUA.Joi2),Math.toRadians(OPCUA.Joi3),Math.toRadians(OPCUA.Joi4),Math.toRadians(OPCUA.Joi5),Math.toRadians(OPCUA.Joi6),Math.toRadians(OPCUA.Joi7)).setJointVelocityRel(OPCUA.Vel));
////					} catch (Exception e) {
////					}
//					logger.info("Program 2 Complete");
//					break;
//				case 5:
//					OPCUA.setReady(false);
////					gripper2F1.close();
//					gripper.move(ptp(0.0,0.785398,0.0,-1.13446,0.0,-0.436332,1.5708).setJointVelocityRel(0.2));//.setMode(springRobot));
//					m1 = gripper.moveAsync(positionHold(springRobot, -1, TimeUnit.SECONDS));
//					while (OPCUA.Start == true){
//						OPCUA.ServerUpdate();
//					}
//					m1.cancel();
//					break;
//				case 6:
//					break;
//				default:
//					break;
//				} 
//				//Program Complete Routine
//				if (OPCUA.Connected==false){break;} 
//				OPCUA.setEnd(true);
//				OPCUA.setProgID(0);
//				OPCUA.setStart(false);
//				ThreadUtil.milliSleep(1500);
//				OPCUA.setReady(true);
//				ThreadUtil.milliSleep(1500);
////				mF.setLEDGreen(false);
//				OPCUA.setEnd(false);
//				logger.info("Communication Signals Reset");
//			} // End If (Start)
//			if (OPCUA.Connected==false){break;} 
//		}// End While connected
//		if (OPCUA.Connected==false){mF.setLEDBlue(false);logger.info("Shutting Down Automatic Mode");OPCUA.clientDisconnect();}
//	}//END RUN
//}
//
//
