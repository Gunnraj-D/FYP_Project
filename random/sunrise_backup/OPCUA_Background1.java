package application;

import java.util.concurrent.*; 

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;

//import com.kuka.math.geometry.Vector3D;
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
public class OPCUA_Background1 extends RoboticsAPICyclicBackgroundTask{

	@Inject	
	private OCPUA_cycle cycle1;	
	
	public void initialize(){
		ThreadUtil.milliSleep(10000);
		initializeCyclic(0, 500, TimeUnit.MILLISECONDS,	CycleBehavior.BestEffort);
		cycle1.initialize();
		cycle1.startCycle();
	}
	
	@Override
	public void runCyclic() {
	}
	
	@Override
	public void dispose() {
	    try {
	        if (cycle1 != null) {
	            cycle1.stop();
	        }
	    } catch (Exception e) {
	        getLogger().warn("Error while stopping cycle: " + e.getMessage());
	    } finally {
	        super.dispose();
	    }
	}

		
}

