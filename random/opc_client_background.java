//package application;
//
//import java.io.File;
//import java.io.IOException;
//import java.net.UnknownHostException;
//import java.util.List;
//import java.util.Locale;
//
//import javax.inject.Inject;
//import javax.inject.Named;
//
//
//import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
//import com.kuka.roboticsAPI.deviceModel.JointEnum;
//import com.kuka.roboticsAPI.deviceModel.JointPosition;
//import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.geometricModel.Frame;
//import com.kuka.roboticsAPI.geometricModel.Tool;
//import com.kuka.roboticsAPI.geometricModel.World;
//import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
//import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
//import com.kuka.task.ITaskLogger;
//import com.prosysopc.ua.ApplicationIdentity;
//import com.prosysopc.ua.SecureIdentityException;
//import com.prosysopc.ua.UserIdentity;
//import com.prosysopc.ua.client.UaClient;
//import com.prosysopc.ua.stack.builtintypes.DataValue;
//import com.prosysopc.ua.stack.builtintypes.LocalizedText;
//import com.prosysopc.ua.stack.builtintypes.NodeId;
//import com.prosysopc.ua.stack.builtintypes.UnsignedInteger;
//import com.prosysopc.ua.stack.builtintypes.Variant;
//import com.prosysopc.ua.stack.cert.DefaultCertificateValidator;
//import com.prosysopc.ua.stack.cert.DefaultCertificateValidator.IgnoredChecks;
//import com.prosysopc.ua.stack.cert.PkiDirectoryCertificateStore;
//import com.prosysopc.ua.stack.common.ServiceResultException;
//import com.prosysopc.ua.stack.core.ApplicationDescription;
//import com.prosysopc.ua.stack.core.ApplicationType;
//import com.prosysopc.ua.stack.core.Identifiers;
//import com.prosysopc.ua.stack.core.ReferenceDescription;
//import com.prosysopc.ua.stack.transport.security.SecurityMode;
//import com.prosysopc.ua.stack.builtintypes.NodeId;
//
//public class OPCUA_Client_Background1 {
//	// ~~~ Injecting other Classes ~~~ \\
//	@Inject
//	private LBR robot;				//Access to Robot Info
//	@Inject
//	@Named("RobotiqGripper")		
//	private Tool gripper;			//Access to Robot attached gripper info
//
//	
//	
//	//++++Setup OPCUA Variables (Add NodeID and Variable Here)++++\\
//	//Control (INPUTS)
//	//-OPC UA Client
//	//--Disconnect
//	public NodeId NID_Disconnect;
//	public boolean Disconnect; 
//	
//	//Data (OUTPUTS)
//	//-Robot
//	//--Joint 1
//	public NodeId NID_Joi1;
//	public double Joi1;
//	//--Joint 2
//	public NodeId NID_Joi2;
//	public double Joi2;
//	//--Joint 3
//	public NodeId NID_Joi3;
//	public double Joi3;
//	//--Joint 4
//	public NodeId NID_Joi4;
//	public double Joi4;
//	//--Joint 5
//	public NodeId NID_Joi5;
//	public double Joi5;
//	//--Joint 6
//	public NodeId NID_Joi6;
//	public double Joi6;
//	//--Joint 7
//	public NodeId NID_Joi7;
//	public double Joi7;
//	//--Torque 1
//	public NodeId NID_Tor1;
//	public double Tor1;
//	//--Torque 2
//	public NodeId NID_Tor2;
//	public double Tor2;
//	//--Torque 3
//	public NodeId NID_Tor3;
//	public double Tor3;
//	//--Torque 4
//	public NodeId NID_Tor4;
//	public double Tor4;
//	//--Torque 5
//	public NodeId NID_Tor5;
//	public double Tor5;
//	//--Torque 6
//	public NodeId NID_Tor6;
//	public double Tor6;
//	//--Torque 7
//	public NodeId NID_Tor7;
//	public double Tor7;
//	//--Position X
//	public NodeId NID_PosX;
//	public double PosX;
//	//--Position Y
//	public NodeId NID_PosY;
//	public double PosY;
//	//--Position Z
//	public NodeId NID_PosZ;
//	public double PosZ;
//	//--Rotation A(Z)
//	public NodeId NID_RotA;
//	public double RotA;
//	//--Rotation B(Y)
//	public NodeId NID_RotB;
//	public double RotB;
//	//--Rotation C(Z)
//	public NodeId NID_RotC;
//	public double RotC;
//	//--Force X
//	public NodeId NID_ForX;
//	public double ForX;
//	//--Force Y
//	public NodeId NID_ForY;
//	public double ForY;
//	//--Force Z
//	public NodeId NID_ForZ;
//	public double ForZ;
//	//--Moment X
//	public NodeId NID_MomX;
//	public double MomX;
//	//--Moment Y
//	public NodeId NID_MomY;
//	public double MomY;
//	//--Moment Z
//	public NodeId NID_MomZ;
//	public double MomZ;
//	//--Moment X
//	public NodeId NID_BaseX;
//	public double BaseX;
//	//--Moment Y
//	public NodeId NID_BaseY;
//	public double BaseY;
//	//--Moment Z
//	public NodeId NID_BaseA;
//	public double BaseA;
//	
//	//-Gripper
//	//--Status
//	public NodeId NID_Gstatus;
//	public byte Gstatus;
//	//--Position
//	public NodeId NID_Gpos;
//	public byte Gpos;
//	//--Current
//	public NodeId NID_Gcur;
//	public byte Gcur;
//
//	// Variable to Select the Data field (DO NOT CHANGE)
//	public UnsignedInteger attributeId = UnsignedInteger.valueOf(13);
//
//	
//	// Input Server Endpoint
//	public UaClient client = new UaClient("opc.tcp://172.24.200.1:4840/");
//
//	public void SetUp() throws Exception {
//		client.setSecurityMode(SecurityMode.NONE);
//		initialize(client);
//		client.setUserIdentity(new UserIdentity("Admin","password"));
//		client.connect();
//		NodeId target;		
//		NodeId nodeId = Identifiers.RootFolder;
//		client.getAddressSpace().setMaxReferencesPerNode(1000);
//		client.getAddressSpace().setReferenceTypeId(Identifiers.HierarchicalReferences);
//		
//		List<ReferenceDescription> references = client.getAddressSpace().browse(nodeId);
//		target = findNode("Objects", references);
//		nodeId = target;
//		references = client.getAddressSpace().browse(nodeId);
//		target = findNode("robot1", references);
//		nodeId = target;
//		references = client.getAddressSpace().browse(nodeId);
//		NID_Disconnect = findNode("R1d_Disconnect",references);
//		NID_Joi1 = findNode("R1d_Joi1", references);
//		NID_Joi2 = findNode("R1d_Joi2", references);
//		NID_Joi3 = findNode("R1d_Joi3", references);
//		NID_Joi4 = findNode("R1d_Joi4", references);
//		NID_Joi5 = findNode("R1d_Joi5", references);
//		NID_Joi6 = findNode("R1d_Joi6", references);
//		NID_Joi7 = findNode("R1d_Joi7", references);
//		NID_Tor1 = findNode("R1d_Tor1", references);
//		NID_Tor2 = findNode("R1d_Tor2", references);
//		NID_Tor3 = findNode("R1d_Tor3", references);
//		NID_Tor4 = findNode("R1d_Tor4", references);
//		NID_Tor5 = findNode("R1d_Tor5", references);
//		NID_Tor6 = findNode("R1d_Tor6", references);
//		NID_Tor7 = findNode("R1d_Tor7", references);
//		NID_PosX = findNode("R1d_PosX", references);
//		NID_PosY = findNode("R1d_PosY", references);
//		NID_PosZ = findNode("R1d_PosZ", references);
//		NID_RotA = findNode("R1d_RotA", references);
//		NID_RotB = findNode("R1d_RotB", references);
//		NID_RotC = findNode("R1d_RotC", references);
//		NID_ForX = findNode("R1d_ForX", references);
//		NID_ForY = findNode("R1d_ForY", references);
//		NID_ForZ = findNode("R1d_ForZ", references);
//		NID_MomX = findNode("R1d_MomX", references);
//		NID_MomY = findNode("R1d_MomY", references);
//		
//		NID_MomZ = findNode("R1d_MomZ", references);
//		NID_BaseX = findNode("R1d_BaseX", references);
//		NID_BaseY = findNode("R1d_BaseY", references);
//		NID_BaseA = findNode("R1d_BaseA", references);
//	}
//
//	public void ServerUpdate() throws Exception {
//		//Updates robot data variables
//		update();
//		//Updates OPCUA Server Data Variables
//		client.writeAttribute(NID_Joi1, attributeId,  Math.toDegrees(Joi1));
//		client.writeAttribute(NID_Joi2, attributeId,  Math.toDegrees(Joi2));
//		client.writeAttribute(NID_Joi3, attributeId,  Math.toDegrees(Joi3));
//		client.writeAttribute(NID_Joi4, attributeId,  Math.toDegrees(Joi4));
//		client.writeAttribute(NID_Joi5, attributeId,  Math.toDegrees(Joi5));
//		client.writeAttribute(NID_Joi6, attributeId,  Math.toDegrees(Joi6));
//		client.writeAttribute(NID_Joi7, attributeId,  Math.toDegrees(Joi7));
//		client.writeAttribute(NID_Tor1, attributeId,  Tor1);
//		client.writeAttribute(NID_Tor2, attributeId,  Tor2);
//		client.writeAttribute(NID_Tor3, attributeId,  Tor3);
//		client.writeAttribute(NID_Tor4, attributeId,  Tor4);
//		client.writeAttribute(NID_Tor5, attributeId,  Tor5);
//		client.writeAttribute(NID_Tor6, attributeId,  Tor6);
//		client.writeAttribute(NID_Tor7, attributeId,  Tor7);
//		client.writeAttribute(NID_PosX, attributeId,  PosX);
//		client.writeAttribute(NID_PosY, attributeId,  PosY);
//		client.writeAttribute(NID_PosZ, attributeId,  PosZ);
//		client.writeAttribute(NID_RotA, attributeId,  RotA);
//		client.writeAttribute(NID_RotB, attributeId,  RotB);
//		client.writeAttribute(NID_RotC, attributeId,  RotC);
//		client.writeAttribute(NID_ForX, attributeId,  ForX);
//		client.writeAttribute(NID_ForY, attributeId,  ForY);
//		client.writeAttribute(NID_ForZ, attributeId,  ForZ);
//		client.writeAttribute(NID_MomX, attributeId,  MomX);
//		client.writeAttribute(NID_MomY, attributeId,  MomY);
//		client.writeAttribute(NID_MomZ, attributeId,  MomZ);
//		//Control Variable to Disconnect Client
//		Disconnect = client.readAttribute(NID_Disconnect,attributeId).getValue().booleanValue();		
//	}
//
//	protected static void initialize(UaClient client) throws SecureIdentityException, IOException, UnknownHostException {
//		// *** Application Description is sent to the server
//		ApplicationDescription appDescription = new ApplicationDescription();
//		appDescription.setApplicationName(new LocalizedText("Robot1_Background", Locale.ENGLISH));
//		appDescription.setApplicationUri("Robot1_Background");
//		appDescription.setProductUri("Robot1_Background");
//		appDescription.setApplicationType(ApplicationType.Client);
//		final ApplicationIdentity identity = new ApplicationIdentity();
//		identity.setApplicationDescription(appDescription);
//		client.setApplicationIdentity(identity);
//	}
//
//	public void update(){
//		// Live Joint Space Information
//		JointPosition position = robot.getCurrentJointPosition();
//		Joi1 = position.get(0);
//		Joi2 = position.get(1);
//		Joi3 = position.get(2);
//		Joi4 = position.get(3);
//		Joi5 = position.get(4);
//		Joi6 = position.get(5);
//		Joi7 = position.get(6);
//
//		// Live Task Space Information
//		gripper.attachTo(robot.getFlange());
//		Frame curFrame = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
//		PosX = curFrame.getX();
//		PosY = curFrame.getY();
//		PosZ = curFrame.getZ();
//		RotA = curFrame.getAlphaRad();
//		RotB = curFrame.getBetaRad();
//		RotC = curFrame.getGammaRad();
//
//		// Task and Joint Space Force/Torque Information
//		TorqueSensorData measuredData = robot.getMeasuredTorque();
//		ForceSensorData data = robot.getExternalForceTorque(robot.getFlange(),World.Current.getRootFrame()); //Find most useful frame
//		
//		Tor1 = measuredData.getSingleTorqueValue(JointEnum.J1);
//		Tor2 = measuredData.getSingleTorqueValue(JointEnum.J2);
//		Tor3 = measuredData.getSingleTorqueValue(JointEnum.J3);
//		Tor4 = measuredData.getSingleTorqueValue(JointEnum.J4);
//		Tor5 = measuredData.getSingleTorqueValue(JointEnum.J5);
//		Tor6 = measuredData.getSingleTorqueValue(JointEnum.J6);
//		Tor7 = measuredData.getSingleTorqueValue(JointEnum.J7);
//		
//		ForX = data.getForce().getX();
//		ForY = data.getForce().getY();
//		ForZ = data.getForce().getZ();
//		MomX = data.getTorque().getX();
//		MomY = data.getTorque().getY();
//		MomZ = data.getTorque().getZ();
//		
//		
//		
//		// Add Gripper Info here
//		//Gstatus = (byte) gripper2F1.readObjectDetection();
//		//Gpos =  (byte) gripper2F1.readPos();
//
//	}
//
//	public NodeId findNode(String comp, List<ReferenceDescription> references) throws ServiceResultException{
//		for (int i = 0; i < references.size(); i++) {
//			if (references.get(i).getDisplayName().toString().contains((comp))){
//				return client.getAddressSpace().getNamespaceTable().toNodeId(references.get(i).getNodeId());
//			}
//		}
//		return null;
//	}
//
//
//	public void clientDisconnect(){
//		client.disconnect();
//	}
//}