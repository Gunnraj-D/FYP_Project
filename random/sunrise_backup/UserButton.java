package application;


import javax.inject.Inject;

import java.io.IOException;
import java.util.concurrent.TimeUnit;


import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;

/**
 * Implementation of a cyclic background task.
 * <p>
 * It provides the {@link RoboticsAPICyclicBackgroundTask#runCyclic} method 
 * which will be called cyclically with the specified period.<br>
 * Cycle period and initial delay can be set by calling 
 * {@link RoboticsAPICyclicBackgroundTask#initializeCyclic} method in the 
 * {@link RoboticsAPIBackgroundTask#initialize()} method of the inheriting 
 * class.<br>
 * The cyclic background task can be terminated via 
 * {@link RoboticsAPICyclicBackgroundTask#getCyclicFuture()#cancel()} method or 
 * stopping of the task.
 * @see UseRoboticsAPIContext
 * 
 */
public class UserButton extends RoboticsAPICyclicBackgroundTask {
//	@Inject
//	private Controller kUKA_Sunrise_Cabinet_1;
	
	
	@Inject 
	private Gripper_test gripper2F1;
	
	private float currentWidth = -1;
	private float currentForce = -1;
	
	@Override
	public void initialize() {
		// initialise your task here
		initializeCyclic(0, 500, TimeUnit.MILLISECONDS,	CycleBehavior.BestEffort);
		float[] initialisationResults = gripper2F1.initialise();
		currentWidth = initialisationResults[0];
	    currentForce = initialisationResults[1];
		
		
		IUserKeyBar gripperBar = getApplicationUI().createUserKeyBar("Gripper");
		IUserKeyListener openGripperListener = new IUserKeyListener(){
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Green,UserKeyLEDSize.Small);
				currentWidth = Math.min(currentWidth + 10, gripper2F1.widthMax());
				gripper2F1.open();
//				gripper2F1.grip();
				key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Grey,UserKeyLEDSize.Small);
			}
		};
		IUserKeyListener closeGripperListener = new IUserKeyListener(){
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Green,UserKeyLEDSize.Small);
				currentWidth = Math.max(0, currentWidth - 10); // Method for closing the gripper
//				try {
//					gripper2F1.setPos(currentWidth);
//				} catch (IOException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				} // Method for opening the gripper
//				gripper2F1.grip();
				gripper2F1.close();
				key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Grey,UserKeyLEDSize.Small);
			}
		};
		IUserKeyListener ForceListener = new IUserKeyListener(){
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event==UserKeyEvent.FirstKeyUp && currentForce<gripper2F1.forceMax()){
					currentForce += 10;
					
				}else if (event==UserKeyEvent.SecondKeyDown && currentForce>0){
					currentForce -= 10;
				}
				gripper2F1.setForce((int)currentForce);
				key.setText(UserKeyAlignment.Middle, Integer.toString((int)currentForce));
				
			}
		};
			IUserKey openKey = gripperBar.addUserKey(0, openGripperListener, true);
			IUserKey closeKey = gripperBar.addUserKey(1,closeGripperListener, true);
			IUserKey forceKey = gripperBar.addDoubleUserKey(2, ForceListener, false);
			openKey.setText(UserKeyAlignment.Middle, "OPEN");
			closeKey.setText(UserKeyAlignment.Middle, "CLOSE");
			forceKey.setText(UserKeyAlignment.TopMiddle, "+");
			forceKey.setText(UserKeyAlignment.BottomMiddle, "-");
			forceKey.setText(UserKeyAlignment.TopMiddle, "+");			
			forceKey.setText(UserKeyAlignment.Middle, "Force");	
			gripperBar.publish();
		
	}

	@Override
	public void runCyclic() {
		// your task execution starts here
		
	}
}