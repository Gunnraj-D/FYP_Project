package application;

import java.io.File;
import java.io.IOException;
import java.net.UnknownHostException;
import java.util.List;
import java.util.Locale;

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
import com.prosysopc.ua.SecureIdentityException;
import com.prosysopc.ua.UserIdentity;
import com.prosysopc.ua.client.UaClient;
import com.prosysopc.ua.stack.builtintypes.DataValue;
import com.prosysopc.ua.stack.builtintypes.LocalizedText;
import com.prosysopc.ua.stack.builtintypes.NodeId;
import com.prosysopc.ua.stack.builtintypes.UnsignedInteger;
import com.prosysopc.ua.stack.builtintypes.Variant;
import com.prosysopc.ua.stack.cert.DefaultCertificateValidator;
import com.prosysopc.ua.stack.cert.DefaultCertificateValidator.IgnoredChecks;
import com.prosysopc.ua.stack.cert.PkiDirectoryCertificateStore;
import com.prosysopc.ua.stack.common.ServiceResultException;
import com.prosysopc.ua.stack.core.ApplicationDescription;
import com.prosysopc.ua.stack.core.ApplicationType;
import com.prosysopc.ua.stack.core.Identifiers;
import com.prosysopc.ua.stack.core.ReferenceDescription;
import com.prosysopc.ua.stack.transport.security.SecurityMode;
import com.prosysopc.ua.stack.builtintypes.NodeId;

public class OPCUA_Client_Control3 {
	// ~~~ Injecting other Classes ~~~ \\
	@Inject
	private LBR robot;				//Access to Robot Info
	@Inject
	private ITaskLogger logger;		//Printing in non background tasks
	@Inject
	@Named("RobotiqGripper")		
	private Tool gripper;			//Access to Robot attached gripper info
	@Inject 
	private Gripper2F gripper2F1;	//Access to Gripper info
	//++++Setup OPCUA Variables (Add NodeID and Variable Here)++++\\
	//Control (INPUTS)
	//-OPC UA Client
	//--Disconnect
	public boolean Connected;
	//--Start
	public NodeId NID_Start;
	public boolean Start;
	//--Program ID
	public NodeId NID_ProgID;
	public int ProgID;
	//--Count
	public NodeId NID_Count;
	public int Count;
	//--End
	public NodeId NID_End;
	public boolean End;
	//--Ready
	public NodeId NID_Ready;
	public boolean Ready;


	//-Robot
	//--Joint 1
	public NodeId NID_Joi1;
	public double Joi1;
	//--Joint 2
	public NodeId NID_Joi2;
	public double Joi2;
	//--Joint 3
	public NodeId NID_Joi3;
	public double Joi3;
	//--Joint 4
	public NodeId NID_Joi4;
	public double Joi4;
	//--Joint 5
	public NodeId NID_Joi5;
	public double Joi5;
	//--Joint 6
	public NodeId NID_Joi6;
	public double Joi6;
	//--Joint 7
	public NodeId NID_Joi7;
	public double Joi7;
	//--Vel
	public NodeId NID_Vel;
	public double Vel;

	//--PosX
	public NodeId NID_PosX;
	public double PosX;
	//--PosY
	public NodeId NID_PosY;
	public double PosY;
	//--PosZ
	public NodeId NID_PosZ;
	public double PosZ;
	//--RotA
	public NodeId NID_RotA;
	public double RotA;
	//--RotB
	public NodeId NID_RotB;
	public double RotB;
	//--RotC
	public NodeId NID_RotC;
	public double RotC;

	public NodeId NID_int1;
	public int int1;
	public NodeId NID_int2;
	public int int2;
	public NodeId NID_int3;
	public int int3;

	//--JAI Snap
	public NodeId NID_JAISnap;
	public boolean Snap;
	public NodeId NID_JAIRec;
	public boolean Rec;
	//--JAI Name
	public NodeId NID_JAIName;
	public String JAIName;
	//--JAI Ready
	public NodeId NID_JAIReady;
	public Boolean JAIReady;

	//Gripper
	//--Gripper Control
	public NodeId NID_GripperControl;
	public boolean GripperControl;
	//--Gripper Control
	public NodeId NID_GripperAct;
	public boolean GripperAct;
	//--Gripper Control
	public NodeId NID_GripperSpeed;
	public int GripperSpeed;
	//--Gripper Control
	public NodeId NID_GripperForce;
	public int GripperForce;
	//--Gripper Control
	public NodeId NID_GripperPos;
	public int GripperPos;


	public UnsignedInteger attributeId = UnsignedInteger.valueOf(13);

	public UaClient client = new UaClient("opc.tcp://172.24.200.1:4840/");

	public void SetUp() throws Exception {
		client.setSecurityMode(SecurityMode.NONE);
		initialize(client);
		client.setUserIdentity(new UserIdentity("Admin","password"));
		client.connect();
		NodeId target;
		NodeId targetC;
		NodeId targetD;
		NodeId nodeId = Identifiers.RootFolder;
		NodeId nodeIdC;
		NodeId nodeIdD;
		client.getAddressSpace().setMaxReferencesPerNode(1000);
		client.getAddressSpace().setReferenceTypeId(Identifiers.HierarchicalReferences);


		List<ReferenceDescription> references;
		target = findNode("Objects", references);
		nodeId = target;

		references = client.getAddressSpace().browse(nodeId);
		target = findNode("robot3", references);
		nodeId = target;

		targetC = findNode("Communication", references);
		nodeIdC = targetC;

		targetD = findNode("Camera1", references);
		nodeIdD = targetD;
		
		references = client.getAddressSpace().browse(nodeId);
		List<ReferenceDescription> referencesC = client.getAddressSpace().browse(nodeIdC);
		List<ReferenceDescription> referencesD = client.getAddressSpace().browse(nodeIdD);

		NID_Connected = findNode("R3c_Connected",references);
		NID_Start = findNode("R3c_Start",references);
		NID_ProgID = findNode("R3c_ProgID",references);
		NID_Count = findNode("LabCount",referencesC);
		NID_End = findNode("R3f_End",references);
		NID_Ready = findNode("R3f_Ready",references);
		
		NID_int1 = findNode("R3c_int1",references);
		NID_int2 = findNode("R3c_int2",references);
		NID_int3 = findNode("R3c_int3",references);
		
		NID_Count = findNode("LabCount",referencesC);
		NID_JAISnap = findNode("C1c_Snap",referencesD);
		NID_JAIRec = findNode("C1c_Rec",referencesD);
		NID_JAIName = findNode("C1c_Name",referencesD);
		NID_JAIReady = findNode("C1f_Ready",referencesD);
		
		NID_Joi1 = findNode("R3c_Joi1", references);
		NID_Joi2 = findNode("R3c_Joi2", references);
		NID_Joi3 = findNode("R3c_Joi3", references);
		NID_Joi4 = findNode("R3c_Joi4", references);
		NID_Joi5 = findNode("R3c_Joi5", references);
		NID_Joi6 = findNode("R3c_Joi6", references);
		NID_Joi7 = findNode("R3c_Joi7", references);
		NID_Vel = findNode("R3c_Vel", references);

		NID_PosX = findNode("R3c_PosX", references);
		NID_PosY = findNode("R3c_PosY", references);
		NID_PosZ = findNode("R3c_PosZ", references);
		NID_RotA = findNode("R3c_RotA", references);
		NID_RotB = findNode("R3c_RotB", references);
		NID_RotC = findNode("R3c_RotC", references);
		
		NID_GripperControl = findNode("R3c_GripperControl", references);
		NID_GripperAct = findNode("R3c_GripperAct", references);
		NID_GripperSpeed = findNode("R3c_GripperSpeed", references);
		NID_GripperForce = findNode("R3c_GripperForce", references);
		NID_GripperPos = findNode("R3c_GripperPos", references);
		
	}

	public void ServerUpdate() throws Exception {
		//READ Control Variables (Create function for writing)
		Connected = client.readAttribute(NID_Connected,attributeId).getValue().booleanValue();
		Start = client.readAttribute(NID_Start,attributeId).getValue().booleanValue();
		ProgID = client.readAttribute(NID_ProgID,attributeId).getValue().intValue();
		Count = client.readAttribute(NID_Count,attributeId).getValue().intValue();
		int1 = client.readAttribute(NID_int1,attributeId).getValue().intValue();
		int2 = client.readAttribute(NID_int2,attributeId).getValue().intValue();
		int3 = client.readAttribute(NID_int3,attributeId).getValue().intValue();
		JAIReady = client.readAttribute(NID_JAIReady,attributeId).getValue().booleanValue();

		
		Joi1 = client.readAttribute(NID_Joi1,attributeId).getValue().doubleValue();
		Joi2 = client.readAttribute(NID_Joi2,attributeId).getValue().doubleValue();
		Joi3 = client.readAttribute(NID_Joi3,attributeId).getValue().doubleValue();
		Joi4 = client.readAttribute(NID_Joi4,attributeId).getValue().doubleValue();
		Joi5 = client.readAttribute(NID_Joi5,attributeId).getValue().doubleValue();
		Joi6 = client.readAttribute(NID_Joi6,attributeId).getValue().doubleValue();
		Joi7 = client.readAttribute(NID_Joi7,attributeId).getValue().doubleValue();
		Vel = client.readAttribute(NID_Vel,attributeId).getValue().doubleValue();
		PosX = client.readAttribute(NID_PosX,attributeId).getValue().doubleValue();
		PosY = client.readAttribute(NID_PosY,attributeId).getValue().doubleValue();
		PosZ = client.readAttribute(NID_PosZ,attributeId).getValue().doubleValue();
		RotA = client.readAttribute(NID_RotA,attributeId).getValue().doubleValue();
		RotB = client.readAttribute(NID_RotB,attributeId).getValue().doubleValue();
		RotC = client.readAttribute(NID_RotC,attributeId).getValue().doubleValue();
	
		GripperControl = client.readAttribute(NID_GripperControl,attributeId).getValue().booleanValue();
		GripperAct = client.readAttribute(NID_GripperAct,attributeId).getValue().booleanValue();
		GripperSpeed = client.readAttribute(NID_GripperSpeed,attributeId).getValue().intValue();
		GripperPos = client.readAttribute(NID_GripperPos,attributeId).getValue().intValue();
		GripperForce = client.readAttribute(NID_GripperPos,attributeId).getValue().intValue();
		
		if (GripperControl == true){
			gripper2F1.setForce(GripperForce);
			gripper2F1.setSpeed(GripperSpeed);
			if(GripperAct == true){
				gripper2F1.setPos(GripperPos);
			}
		}
	}
	//Set Write Functions for Flags/Outputs
	public void setCount(int A) throws Exception {
		client.writeAttribute(NID_Count, attributeId,  A);
	}

	public void setStart(boolean A) throws Exception {
		client.writeAttribute(NID_Start, attributeId,  A);
	}
	
	public void setProgID(int A) throws Exception {
		client.writeAttribute(NID_ProgID, attributeId,  A);
	}

	public void setReady(boolean A) throws Exception {
		client.writeAttribute(NID_Ready, attributeId,  A);
	}

	public void setEnd(boolean A) throws Exception {
		client.writeAttribute(NID_End, attributeId,  A);
	}

	public void setSnap(boolean A) throws Exception {
		client.writeAttribute(NID_JAISnap, attributeId,  A);
	}
	
	public void setRec(boolean A) throws Exception {
		client.writeAttribute(NID_JAIRec, attributeId,  A);
	}

	public void setSnapName(String A) throws Exception {
		client.writeAttribute(NID_JAIName, attributeId,  A);
	}
	
	public void setGripperControl(boolean A) throws Exception {
		client.writeAttribute(NID_GripperControl, attributeId, A);
	}
	
	protected static void initialize(UaClient client) throws SecureIdentityException, IOException, UnknownHostException {
		ApplicationDescription appDescription = new ApplicationDescription();
		appDescription.setApplicationName(new LocalizedText("Robot4_Control", Locale.ENGLISH));
		appDescription.setApplicationUri("Robot4_Control");
		appDescription.setProductUri("Robot4_Control");
		appDescription.setApplicationType(ApplicationType.Client);
		final ApplicationIdentity identity = new ApplicationIdentity();
		identity.setApplicationDescription(appDescription);
		client.setApplicationIdentity(identity);
	}

	public NodeId findNode(String comp, List<ReferenceDescription> references) throws ServiceResultException{
		for (int i = 0; i < references.size(); i++) {
			if (references.get(i).getDisplayName().toString().contains((comp))){
				return client.getAddressSpace().getNamespaceTable().toNodeId(references.get(i).getNodeId());
			}
		}
		return null;
	}


	public void clientDisconnect(){
		client.disconnect();
	}

}