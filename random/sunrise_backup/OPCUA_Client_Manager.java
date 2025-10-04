package application;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;
import com.prosysopc.ua.ApplicationIdentity;
import com.prosysopc.ua.ServiceException;
import com.prosysopc.ua.StatusException;
import com.prosysopc.ua.UserIdentity;
import com.prosysopc.ua.client.UaClient;
import com.prosysopc.ua.stack.builtintypes.DataValue;
import com.prosysopc.ua.stack.builtintypes.LocalizedText;
import com.prosysopc.ua.stack.builtintypes.NodeId;
import com.prosysopc.ua.stack.builtintypes.UnsignedInteger;
import com.prosysopc.ua.stack.builtintypes.Variant;
import com.prosysopc.ua.stack.core.ApplicationDescription;
import com.prosysopc.ua.stack.core.ApplicationType;
import com.prosysopc.ua.stack.core.Identifiers;
import com.prosysopc.ua.stack.core.ReferenceDescription;
import com.prosysopc.ua.stack.core.TimestampsToReturn;
import com.prosysopc.ua.stack.transport.security.SecurityMode;
import com.prosysopc.ua.stack.common.ServiceResultException;
import com.prosysopc.ua.SecureIdentityException;

public class OPCUA_Client_Manager {

    // ~~~ Injected KUKA and Logger Instances ~~~
    @Inject
    private LBR robot;

    @Inject
    @Named("RobotiqGripper")
    private Tool gripper;

    @Inject
    private ITaskLogger logger;

    // ~~~ OPC UA Configuration Constants ~~~
    private static final String OPC_SERVER_URL = "opc.tcp://172.24.200.1:4840/";
    private static final String OPC_USERNAME = "Admin";
    private static final String OPC_PASSWORD = "password";
    private static final UnsignedInteger VALUE_ATTRIBUTE_ID = UnsignedInteger.valueOf(13);

    // ~~~ OPC UA Client and State Variables ~~~
    private UaClient client;
    private NodeId disconnectNodeId;
    public boolean isDisconnectRequested;

    // A map to hold all data variables
    private final Map<String, MonitoredVariable> dataVariables = new LinkedHashMap<String, MonitoredVariable>();
    
 // A map to hold all data variables
    private final Map<String, MonitoredVariable> controlVariables = new LinkedHashMap<String, MonitoredVariable>();

    /**
     * Helper class to encapsulate all information about a single OPC UA variable.
     */
    private static class MonitoredVariable {
        final String browseName;
        NodeId nodeId;
        Variant value;

        MonitoredVariable(String browseName) {
            this.browseName = browseName;
            this.value = Variant.NULL; // Default to NULL variant
        }
    }

    /**
     * Initialises the list of variables to be monitored and written to the OPC UA server.
     * To add a new variable, just add a new line here.
     */
    private void initialiseVariableMap() {
    	// --- Data Variables ---
        // Joint Positions
        for (int i = 1; i <= 7; i++) {
            dataVariables.put("Joi" + i, new MonitoredVariable("R1d_Joi" + i));
        }
        // Joint Torques
        for (int i = 1; i <= 7; i++) {
            dataVariables.put("Tor" + i, new MonitoredVariable("R1d_Tor" + i));
        }
        // Cartesian Position
        dataVariables.put("PosX", new MonitoredVariable("R1d_PosX"));
        dataVariables.put("PosY", new MonitoredVariable("R1d_PosY"));
        dataVariables.put("PosZ", new MonitoredVariable("R1d_PosZ"));
        // Cartesian Rotation
        dataVariables.put("RotA", new MonitoredVariable("R1d_RotA"));
        dataVariables.put("RotB", new MonitoredVariable("R1d_RotB"));
        dataVariables.put("RotC", new MonitoredVariable("R1d_RotC"));
        // External Forces
        dataVariables.put("ForX", new MonitoredVariable("R1d_ForX"));
        dataVariables.put("ForY", new MonitoredVariable("R1d_ForY"));
        dataVariables.put("ForZ", new MonitoredVariable("R1d_ForZ"));
        // External Moments (Torque)
        dataVariables.put("MomX", new MonitoredVariable("R1d_MomX"));
        dataVariables.put("MomY", new MonitoredVariable("R1d_MomY"));
        dataVariables.put("MomZ", new MonitoredVariable("R1d_MomZ"));
        // Base Position
        dataVariables.put("BaseX", new MonitoredVariable("R1d_BaseX"));
        dataVariables.put("BaseY", new MonitoredVariable("R1d_BaseY"));
        dataVariables.put("BaseA", new MonitoredVariable("R1d_BaseA"));
        // Gripper Variables
        dataVariables.put("Gstatus", new MonitoredVariable("R1d_Gstatus"));
        dataVariables.put("Gpos", new MonitoredVariable("R1d_Gpos"));
        dataVariables.put("Gcur", new MonitoredVariable("R1d_Gcur"));
        
        
        // --- Control Variables ---
        // Program Control Variables
        controlVariables.put("Connected", new MonitoredVariable("R1c_Connected"));
        controlVariables.put("Start", new MonitoredVariable("R1c_Start"));
        
        // Joint Variables
        for (int i = 1; i <= 7; i++) {
            controlVariables.put("Joi" + i, new MonitoredVariable("R1c_Joi" + i));
        }
    }

    public void setUp() throws Exception {
        initialiseVariableMap();
        gripper.attachTo(robot.getFlange());

        client = new UaClient(OPC_SERVER_URL);
        client.setSecurityMode(SecurityMode.NONE);
        initializeClientApplication(client);
        client.setUserIdentity(new UserIdentity(OPC_USERNAME, OPC_PASSWORD));
        client.connect();
        logger.info("OPC UA Client connected to " + OPC_SERVER_URL);

        // Browse the server address space to find the NodeIds for our variables
        NodeId robotNode = browseToServerNode("Objects", "robot1");
        List<ReferenceDescription> references = client.getAddressSpace().browse(robotNode);

        // Find the disconnect node separately as it's an input
        disconnectNodeId = findNode("R1d_Disconnect", references);
        if (disconnectNodeId == null) {
            throw new IllegalStateException("Required OPC UA node 'R1d_Disconnect' not found on server.");
        }

        // Dynamically find all other nodes by iterating through our map
        for (MonitoredVariable variable : dataVariables.values()) {
            variable.nodeId = findNode(variable.browseName, references);
            if (variable.nodeId == null) {
                // Fail if a node is missing.
                throw new IllegalStateException("Required OPC UA node '" + variable.browseName + "' not found on server.");
            }
        }
        
        for (MonitoredVariable variable : controlVariables.values()) {
            variable.nodeId = findNode(variable.browseName, references);
            if (variable.nodeId == null) {
                // Fail if a node is missing.
                throw new IllegalStateException("Required OPC UA node '" + variable.browseName + "' not found on server.");
            }
        }
        logger.info("All OPC UA nodes found successfully.");
    }

    public void serverUpdate() throws ServiceException, ServiceResultException {
        // Get the latest data from the robot
        updateRobotData();

        // Prepare the data for batch writing
        List<NodeId> nodeIdsToWrite = new ArrayList<NodeId>();
        List<Variant> valuesToWrite = new ArrayList<Variant>();

        for (MonitoredVariable variable : dataVariables.values()) {
            nodeIdsToWrite.add(variable.nodeId);
            valuesToWrite.add(variable.value);
        }

        // Perform the batch write to the server
        try {
            client.writeValues(
                nodeIdsToWrite.toArray(new NodeId[0]),
                valuesToWrite.toArray(new Variant[0])
            );
        } catch (Exception e) {
            logger.error("Batch write to OPC UA server failed.", e);
        }
        
        // Prepare data for batch read
        List<NodeId> nodeIdsToRead = new ArrayList<NodeId>();
        
        for (MonitoredVariable variable : controlVariables.values()) {
            nodeIdsToRead.add(variable.nodeId);
        }
        
        // Perform batch read
        DataValue[] readValues = client.readValues(
	            nodeIdsToRead.toArray(new NodeId[0])
	    );
        
        // Write read values
        List<MonitoredVariable> controlVarsList = new ArrayList<MonitoredVariable>(controlVariables.values());
        for (int i = 0; i < readValues.length; i++) {
            if (readValues[i] != null && readValues[i].getValue() != null) {
                controlVarsList.get(i).value = readValues[i].getValue();
            }
        }

        // Read the disconnect signal from the server
        DataValue disconnectValue;
		try {
			disconnectValue = client.readAttribute(disconnectNodeId, VALUE_ATTRIBUTE_ID);
			isDisconnectRequested = disconnectValue.getValue().booleanValue();
		} catch (StatusException e) {
			logger.warn("Failed to read disconnect signal from OPC UA server. Assuming no disconnect requested.", e);
		}
        
    }
    
    
    public JointPosition getTargetJointPosition() {
        double[] jointValuesRad = new double[7];

        for (int i = 0; i < 7; i++) {
            String key = "Joi" + (i + 1); 
            MonitoredVariable mv = controlVariables.get(key);

            // Safety check: Is the data available and valid?
            if (mv == null || mv.value == null || mv.value.isEmpty()) {
                logger.warn("Target joint value for " + key + " is not yet available from OPC UA server.");
                return null;
            }

            try {
                // Get the value as a double. 
                jointValuesRad[i] = mv.value.doubleValue();
            } catch (ClassCastException e) {
                logger.error("OPC UA value for " + key + " is not a double as expected.", e);
                return null; // The data type is wrong, so the position is invalid.
            }
        }

        // If we successfully got all 7 values, create and return the JointPosition object.
        return new JointPosition(jointValuesRad);
    }
    
    public boolean hasStarted() {
        String key = "Start";
        MonitoredVariable mv = controlVariables.get(key);

        // Safety check with a safe default value (false)
        if (mv == null || mv.value == null || mv.value.isEmpty()) {
            return false;
        }

        return mv.value.booleanValue();
    }
    
    public boolean isConnected() {
    	String key = "Connected";
        MonitoredVariable mv = controlVariables.get(key);

        // Safety check with a safe default value (false)
        if (mv == null || mv.value == null || mv.value.isEmpty()) {
            return false;
        }

        return mv.value.booleanValue();
    }

    private void updateRobotData() {
        // Live Joint Space Information
        JointPosition position = robot.getCurrentJointPosition();
        for (int i = 0; i < 7; i++) {
            dataVariables.get("Joi" + (i + 1)).value = new Variant(Math.toDegrees(position.get(i)));
        }

        // Task and Joint Space Force/Torque Information
        TorqueSensorData measuredTorque = robot.getMeasuredTorque();
        JointEnum[] joints = JointEnum.values();
        for (int i = 0; i < 7; i++) {
            dataVariables.get("Tor" + (i + 1)).value = new Variant(measuredTorque.getSingleTorqueValue(joints[i]));
        }

        // Live Task Space Information
        Frame curFrame = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
        dataVariables.get("PosX").value = new Variant(curFrame.getX());
        dataVariables.get("PosY").value = new Variant(curFrame.getY());
        dataVariables.get("PosZ").value = new Variant(curFrame.getZ());
        dataVariables.get("RotA").value = new Variant(curFrame.getAlphaRad());
        dataVariables.get("RotB").value = new Variant(curFrame.getBetaRad());
        dataVariables.get("RotC").value = new Variant(curFrame.getGammaRad());

        ForceSensorData externalForces = robot.getExternalForceTorque(robot.getFlange(), World.Current.getRootFrame());
        dataVariables.get("ForX").value = new Variant(externalForces.getForce().getX());
        dataVariables.get("ForY").value = new Variant(externalForces.getForce().getY());
        dataVariables.get("ForZ").value = new Variant(externalForces.getForce().getZ());
        dataVariables.get("MomX").value = new Variant(externalForces.getTorque().getX());
        dataVariables.get("MomY").value = new Variant(externalForces.getTorque().getY());
        dataVariables.get("MomZ").value = new Variant(externalForces.getTorque().getZ());
        
        // Once you have the values, update the map. For now, we can set them to 0.
//        dataVariables.get("Gstatus").value = new Variant((byte) 0); // Replace with new Variant(gStatus)
//        dataVariables.get("Gpos").value = new Variant((byte) 0);    // Replace with new Variant(gPos)
//        dataVariables.get("Gcur").value = new Variant((byte) 0);    // Replace with new Variant(gCur)

    }

    private NodeId browseToServerNode(String... path) throws ServiceResultException, ServiceException, StatusException {
        NodeId currentNodeId = Identifiers.RootFolder;
        for (String nodeName : path) {
            List<ReferenceDescription> references;
			references = client.getAddressSpace().browse(currentNodeId);
			currentNodeId = findNode(nodeName, references);
            if (currentNodeId == null) {
                throw new IllegalStateException("Could not find node '" + nodeName + "' in OPC UA server path.");
            }
        }
        return currentNodeId;
    }

    private NodeId findNode(String browseName, List<ReferenceDescription> references) throws ServiceResultException {
        for (ReferenceDescription ref : references) {
            if (ref.getBrowseName().getName().equals(browseName) || ref.getDisplayName().getText().equals(browseName)) {
                return client.getAddressSpace().getNamespaceTable().toNodeId(ref.getNodeId());
            }
        }
        return null;
    }

    private void initializeClientApplication(UaClient client) throws SecureIdentityException, IOException, UnknownHostException {
        ApplicationDescription appDescription = new ApplicationDescription();
        appDescription.setApplicationName(new LocalizedText("KukaRobotClient", Locale.ENGLISH));
        appDescription.setApplicationUri("urn:kuka:robot:client");
        appDescription.setProductUri("urn:kuka:robot");
        appDescription.setApplicationType(ApplicationType.Client);

        final ApplicationIdentity identity = new ApplicationIdentity();
        identity.setApplicationDescription(appDescription);
        client.setApplicationIdentity(identity);
    }

    public void clientDisconnect() {
        if (client != null && client.isConnected()) {
            client.disconnect();
            logger.info("OPC UA Client disconnected.");
        }
        if (gripper != null) {
            gripper.detach();
        }
    }
}