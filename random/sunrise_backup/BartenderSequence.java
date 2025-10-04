package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

public class BartenderSequence extends RoboticsAPIApplication {
    private LBR lbr;

    // edit the frames you want to check:
    private static final String[] FRAME_NAMES = { "P1", "P13", "P14", "P5" };

    // set to true if you want the robot to move to each frame after printing
    private static final boolean MOVE_TO_FRAMES = false;
    private static final double VEL_REL = 0.2;

    @Override
    public void initialize() {
        lbr = getContext().getDeviceFromType(LBR.class);
    }

    @Override
    public void run() {
        for (String name : FRAME_NAMES) {
            String path = "/" + name;
            ObjectFrame f = null;
            try {
                f = getApplicationData().getFrame(path);
            } catch (Exception e) {
                f = null;
            }

            if (f == null) {
                getLogger().warn("Frame missing: " + path);
                continue;
            }

            // Print transform of this frame relative to its parent (often /World)
            Transformation T = f.getTransformationFromParent();
            String parent = f.getParent() != null ? f.getParent().getName() : "<no parent>";
            getLogger().info(String.format(
                "Frame %s (parent: %s)  XYZ[mm]=[%.1f, %.1f, %.1f]  ABC[deg]=[%.2f, %.2f, %.2f]",
                path, parent,
                T.getX(), T.getY(), T.getZ(),
                Math.toDegrees(T.getAlphaRad()),
                Math.toDegrees(T.getBetaRad()),
                Math.toDegrees(T.getGammaRad())
            ));

            if (MOVE_TO_FRAMES) {
                getLogger().info("Moving to " + path + "...");
                lbr.move(ptp(f).setJointVelocityRel(VEL_REL));
            }
        }
    }
}
