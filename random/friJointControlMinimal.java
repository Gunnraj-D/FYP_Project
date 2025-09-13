package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

public class FriJointControlMinimal extends RoboticsAPIApplication {

    private Controller _lbrController;
    private LBR _lbr;

    // Set this to your PC Ethernet IP
    private String _clientName = "172.24.30.150";

    @Override
    public void initialize() {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
    }

    @Override
    public void run() throws Exception {
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(10);  // 10 ms = 100 Hz

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms");

        FRISession friSession = new FRISession(friConfiguration);
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession);

        friSession.addFRISessionListener(new IFRISessionListener() {
            @Override
            public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation) {
                getLogger().info("Session State change: " + friChannelInformation.getFRISessionState());
            }

            @Override
            public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation) {
                getLogger().info("Connection Quality: " + friChannelInformation.getQuality() +
                        ", Latency: " + friChannelInformation.getLatency() +
                        ", Jitter: " + friChannelInformation.getJitter());
            }
        });

        try {
            friSession.await(30, TimeUnit.SECONDS);  // wait for session
        } catch (TimeoutException e) {
            getLogger().error("FRI session timeout: " + e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        PositionControlMode ctrl = new PositionControlMode();
        _lbr.moveAsync(positionHold(ctrl, -1, TimeUnit.SECONDS).addMotionOverlay(jointOverlay));

        // Keep the session alive until the app stops
        while (true) {
            Thread.sleep(100);
        }

        // friSession.close();  // optional, app shutdown will clean up
    }
}
