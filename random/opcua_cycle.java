//package application;
//
//import java.util.concurrent.*; 
//import java.util.*; 
//
//import javax.inject.Inject;
//import javax.inject.Named;
//
//import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
//import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
//
//import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
//import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
//import com.kuka.roboticsAPI.conditionModel.ForceCondition;
//import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
//import com.kuka.roboticsAPI.conditionModel.ICondition;
//import com.kuka.roboticsAPI.conditionModel.ITriggerAction;
//import com.kuka.roboticsAPI.deviceModel.JointPosition;
//import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
//import com.kuka.roboticsAPI.geometricModel.CartDOF;
//import com.kuka.roboticsAPI.geometricModel.Frame;
//import com.kuka.roboticsAPI.geometricModel.Tool;
//import com.kuka.roboticsAPI.geometricModel.World;
//import com.kuka.roboticsAPI.geometricModel.math.Transformation;
//import com.kuka.roboticsAPI.geometricModel.math.Vector;
//import com.kuka.roboticsAPI.motionModel.IMotionContainer;
//import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
//import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
//import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
//import com.kuka.task.ITaskLogger;
//import com.kuka.common.Pair;
//import com.kuka.common.ThreadUtil;
//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
//
//import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;
// 
///**
//* Implementation of a robot application.
//* <p>
//* The application provides a {@link RoboticsAPITask#initialize()} and a 
//* {@link RoboticsAPITask#run()} method, which will be called successively in 
//* the application lifecycle. The application will terminate automatically after 
//* the {@link RoboticsAPITask#run()} method has finished or after stopping the 
//* task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
//* exception is thrown during initialization or run. 
//* <p>
//* <b>It is imperative to call <code>super.dispose()</code> when overriding the 
//* {@link RoboticsAPITask#dispose()} method.</b> 
//* 
//* @see UseRoboticsAPIContext
//* @see #initialize()
//* @see #run()
//* @see #dispose()
//*/
//public class OCPUA_cycle	 {
//	
// 
//	@Inject
//	private ITaskLogger logger;
//	public ScheduledFuture<?> OPCUAbackgroundCycle;
//	public ScheduledExecutorService scheduler;
//	
//	@Inject	
//	private OPCUA_Client_Background1 OPCUA;
//	//public ExecutorService scheduler;
//	
//	public Boolean initialize;
//	
//	public void initialize() {
//		initialize = true;
//		scheduler = Executors.newScheduledThreadPool(1);
//		try {
//			OPCUA.SetUp();
//		} catch (Exception e) {
//			e.printStackTrace();
//		}
//		try {
//			OPCUA.ServerUpdate();
//			OPCUA.ServerUpdate();
//		} catch (Exception e) {
//			e.printStackTrace();
//		}
//		
//	}
//	
//	public void startCycle() {
//		Runnable OPCUA_Background = new Runnable() {
//	        public void run() {
//	        	if (OPCUA.Disconnect==false) {
//	    			try {
//	    				OPCUA.ServerUpdate();
//	    			} catch (Exception e) {
//	    				// TODO Auto-generated catch block
//	    				e.printStackTrace();
//	    			}
//	    		} else if (OPCUA.Disconnect==true) {
//	    			OPCUA.clientDisconnect();
//	    			stop();
//	    			
//	    		}
//	        }
//		};
//		
//		
//		OPCUAbackgroundCycle = scheduler.scheduleAtFixedRate(OPCUA_Background, 0, 100, TimeUnit.MILLISECONDS);
//	}
//	
//	public void stop(){
////		scheduler.schedule(new Runnable() { public void run() { beeperHandle.cancel(true); }}, 0 , TimeUnit.SECONDS);
//		OPCUAbackgroundCycle.cancel(true);
//		scheduler.shutdown();
//	}
//	
//}
//
