package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.net.InetAddress;

import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

public class FriJointControl  extends RoboticsAPIApplication{
	
	private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    
    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***P
        // **********************************************************************
        _clientName = "172.24.56.109";
    }
    
	@Override
	public void run() throws Exception {
		FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(10);
        
        if (InetAddress.getByName(_clientName).isReachable(5000)) {
        	getLogger().info("pc is reachable");
        } else {
        	getLogger().info("pc cannot be reached");
        }

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());
        
        while(true){
        	getLogger().info("Starting FRI Session");
	        FRISession friSession = new FRISession(friConfiguration);
	        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession);
	
	        friSession.addFRISessionListener(new IFRISessionListener(){
	        	@Override
	        	public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation) {
	        		getLogger().info("Session State change "+friChannelInformation.getFRISessionState().toString() );
	        	}
	
	        	@Override
	        	public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation) {
		        	getLogger().info("Quality change signalled "+friChannelInformation.getQuality());
		        	getLogger().info("Jitter "+friChannelInformation.getJitter());
		        	getLogger().info("Latency "+friChannelInformation.getLatency());
	        	}
	        });
	        
	        try
	        {
	            friSession.await(30, TimeUnit.SECONDS);
	        }
	        catch (final TimeoutException e)
	        {
	            getLogger().error(e.getLocalizedMessage());
	            friSession.close();
	            continue;
	        }
	
	        getLogger().info("FRI connection established.");
			
	        PositionControlMode ctrl = new PositionControlMode();
	        while(true){
	        	try{
	        		_lbr.moveAsync(positionHold(ctrl, -1, TimeUnit.SECONDS).addMotionOverlay(jointOverlay));
	        	} catch(Exception e){
	        		getLogger().error(e.getMessage());
	        		break;
	        	}
	        }
	        
	        friSession.close();
        }
	}

}
