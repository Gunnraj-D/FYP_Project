package application;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;

public class startingPoint extends RoboticsAPIApplication {
    private LBR lbr;

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
        // Frames under DrinkA
        ObjectFrame drinkA = getApplicationData().getFrame("/DrinkA");
        ObjectFrame p1 = getApplicationData().getFrame("/DrinkA/P1");
        ObjectFrame p2 = getApplicationData().getFrame("/DrinkA/P2");
        ObjectFrame p3 = getApplicationData().getFrame("/DrinkA/P3");
        ObjectFrame p4 = getApplicationData().getFrame("/DrinkA/P4");
        ObjectFrame p5 = getApplicationData().getFrame("/DrinkA/P5");

        // Move sequence
        getLogger().info("Moving to DrinkA...");
        lbr.move(ptp(drinkA).setJointVelocityRel(0.2));
        
        
        getLogger().info("Moving to P1...");
        lbr.move(ptp(p1).setJointVelocityRel(0.2));
        
        gripper.readOffset();
        gripper.moveTo(575);
        gripper.readWidth();
        

        getLogger().info("Moving to P2...");
        lbr.move(ptp(p2).setJointVelocityRel(0.2));
        
        getLogger().info("Moving to P3...");
        lbr.move(ptp(p3).setJointVelocityRel(0.2));
        
        getLogger().info("Moving to P4...");
        lbr.move(ptp(p4).setJointVelocityRel(0.2));
        
        getLogger().info("Moving to P5...");
        lbr.move(ptp(p5).setJointVelocityRel(0.2));
        
//        gripper.moveTo(700);
        
    }
}
