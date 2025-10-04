package application;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.task.ITaskLogger;

import java.io.InputStream;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.net.Socket;

// Documentation: Modbus TCP section in User_Manual_for_TECHMAN_OMRON_TM_v1.05_EN_0.pdf, search internet or MA drive (not yet)

// INPUT ALL POSITION VALUES IN MM, ALL FORCES IN N
// SET FORCE AND WIDTH MAX VALUES BEFORE RUNNING CODE !!
// ALWAYS USE INITIALISE FIRST


@Singleton 
public class Gripper_test {
//	@Inject
//	private Gripper2FIOGroup gripperIO;
	@Inject
	private ITaskLogger logger;
	
	private static String SERVER_ADDRESS = "172.24.200.100"; // DONT CHANGE, IP FOR ONROBOT COMPUTE BOX
    private static int SERVER_PORT = 502;

	private static boolean offsetSet = false; // Flag to check if offset is set
    
	// NECESSARY FUNCTIONS FOR COMM. // 248 - 253
    public int readRegister(int slaveId, int registerAddress, boolean verbose) throws IOException {
        // Modbus TCP header: Transaction ID (2 bytes), Protocol ID (2 bytes), Length (2 bytes), Unit ID (1 byte)
        // Modbus PDU: Function code (1 byte), Starting Address (2 bytes), Quantity of Registers (2 bytes)
        byte[] request = new byte[12];
        request[0] = 0; // Transaction ID high
        request[1] = 1; // Transaction ID low
        request[2] = 0; // Protocol ID high
        request[3] = 0; // Protocol ID low
        request[4] = 0; // Length high
        request[5] = 6; // Length low (6 bytes after this)
        request[6] = (byte) slaveId; // Unit ID
        request[7] = 3; // Function code: Read Holding Registers
        request[8] = (byte) (registerAddress >> 8); // Starting address high
        request[9] = (byte) (registerAddress & 0xFF); // Starting address low
        request[10] = 0; // Quantity high
        request[11] = 1; // Quantity low (read 1 register)
        
        Socket socket = null;
        BufferedOutputStream outStream = null;
        BufferedReader inStream = null;
        int value = -1;

        try {
            socket = new Socket(SERVER_ADDRESS, SERVER_PORT);
            outStream = new BufferedOutputStream(socket.getOutputStream());
            inStream = new BufferedReader(new InputStreamReader(socket.getInputStream()));

            // Original code had these lines commented out, but they are typically
            // necessary for sending a request before reading a response.
            // If your protocol doesn't require sending a request, then keep them commented.
            outStream.write(request);
            outStream.flush();

            byte[] response = new byte[11]; // Expected response length for 1 register
            int bytesRead = socket.getInputStream().read(response);

            if (bytesRead < 11) {
                throw new IOException("Incomplete response: Expected 11 bytes, got " + bytesRead);
            }

            // The register value is in response[9] (high byte) and response[10] (low byte)
            value = ((response[9] & 0xFF) << 8) | (response[10] & 0xFF);
            if (verbose == true){
            	System.out.println("Register value: " + value);
            }
            
            return value;

        } finally {
            // This 'finally' block ensures resources are closed, regardless of
            // whether an exception occurred in the 'try' block.
            
            // Close resources in reverse order of opening to maintain dependencies,
            // and always check for null before closing to prevent NullPointerExceptions.
            // Each close operation needs its own try-catch block to ensure that
            // if closing one stream fails, it doesn't prevent others from being closed.

            if (inStream != null) {
                try {
                    inStream.close();
                } catch (IOException e) {
                    System.err.println("Error closing input stream: " + e.getMessage());
                    // In a real application, you'd log this more robustly.
                }
            }
            if (outStream != null) {
                try {
                    outStream.close();
                } catch (IOException e) {
                    System.err.println("Error closing output stream: " + e.getMessage());
                }
            }
            if (socket != null) {
                try {
                    socket.close();
                } catch (IOException e) {
                    System.err.println("Error closing socket: " + e.getMessage());
                }
            }
        }
    };
	
	public void writeRegister(int slaveId, int registerAddress, int value) throws IOException {
	    // Modbus TCP header: Transaction ID (2 bytes), Protocol ID (2 bytes) always low, Length (2 bytes), Unit ID (1 byte)
	    // Modbus PDU: Function code (1 byte), Starting Address (2 bytes), Register Value (2 bytes)
	    byte[] request = new byte[12];
	    request[0] = 0; // Transaction ID high
	    request[1] = 1; // Transaction ID low
	    request[2] = 0; // Protocol ID high
	    request[3] = 0; // Protocol ID low
	    request[4] = 0; // Length high
	    request[5] = 6; // Length low (6 bytes after this)
	    request[6] = (byte) slaveId; // Unit ID
	    request[7] = 6; // Function code: Write Single Register
	    request[8] = (byte) (registerAddress >> 8); // Register address high
	    request[9] = (byte) (registerAddress & 0xFF); // Register address low
	    request[10] = (byte) (value >> 8); // Value high
	    request[11] = (byte) (value & 0xFF); // Value low
	    
	    Socket socket = null;
	    BufferedOutputStream outStream = null;
	    InputStream inStream = null;
	
	    try {
	        socket = new Socket(SERVER_ADDRESS, SERVER_PORT);
	        outStream = new BufferedOutputStream(socket.getOutputStream());
	        inStream = socket.getInputStream(); // Get the input stream directly from the socket

	        outStream.write(request);
	        outStream.flush();

	        byte[] response = new byte[12]; // Response should echo the request
	        int bytesRead = inStream.read(response); // Read from the obtained InputStream

	        if (bytesRead < 12) {
	            throw new IOException("Incomplete response: Expected 12 bytes, got " + bytesRead);
	        }

	        // Optionally, verify the response matches the request
	        if (response[7] != 6) {
	            throw new IOException("Invalid response function code: Expected 6, got " + response[7]);
	        }

	        System.out.println("Register written successfully.");

	    } finally {
	        // Close resources in reverse order of opening (conceptually),
	        // and always with null checks and individual try-catch blocks.

	        if (inStream != null) {
	            try {
	                inStream.close();
	            } catch (IOException e) {
	                System.err.println("Error closing input stream: " + e.getMessage());
	                // Log this error if using a logging framework
	            }
	        }
	        if (outStream != null) {
	            try {
	                outStream.close();
	            } catch (IOException e) {
	                System.err.println("Error closing output stream: " + e.getMessage());
	            }
	        }
	        if (socket != null) {
	            try {
	                socket.close();
	            } catch (IOException e) {
	                System.err.println("Error closing socket: " + e.getMessage());
	            }
	        }
	    }
	}
	
	
	// SET FORCE AND WIDTH MAXES HERE
	public int forceMax(){
		return 1200;
	}
	
	public int widthMax(){
		return 1600;
	}
	
	// LOWER LEVEL FUNCTIONS
	
	// moveTo requires only the position value, and will move gripper to position. Use setForce to specify 
	// how much force.
	public void moveTo(int position) {
        try {
//            writeRegister(65, 0, force);
            writeRegister(65, 1, position);

            if (offsetSet) {
                gripWithOffset();
            } else {
                grip();
            }

            logger.info("Moving to position " + position);
//            System.out.println("Moving to position " + position + " with force " + force);
            int busy = 1;
            while (busy == 1) {
                busy = readStatus(false);
            }
            readStatus(true);
        } catch (IOException e) {
//            System.err.println("Failed to write value to slave: " + e.getMessage());
            logger.error("Failed to write value: " + e.getMessage());
        }
    }
	
	// Sets force value in register
    public void setForce(int force) {
        // For RG6, valid range is 0 to 1200, so 0 to 120 N
    	try {
            writeRegister(65, 0, force*10);
//            writeRegister(65, 2, 1);
//            System.out.println("Setting force " + force);
            logger.info("Setting force " + force);
            
        } catch (IOException e) {
//            System.err.println("Failed to write value to slave: " + e.getMessage());
            logger.error("Failed to write value: " + e.getMessage());
        }
    }
    
    // Sets width value in register
    public void setPos(int pos) throws IOException {
    	try {
            writeRegister(65, 1, pos);
//            writeRegister(65, 2, 1);
            System.out.println("Setting position " + pos);
            logger.info("Setting position " + pos);
            
        } catch (IOException e) {
            System.err.println("Failed to write value to slave: " + e.getMessage());
            logger.error("Failed to write value to slave: " + e.getMessage());
        }
	}

    // actuates movement to values set in position and force registers
    public void grip(){
        try {
            writeRegister(65, 2, 1);
            // System.out.println("Setting position " + pos);
            logger.info("Gripping");
            
        } catch (IOException e) {
            // System.err.println("Failed to grip:" + e.getMessage());
            logger.error("Failed to grip: " + e.getMessage());
        }
    }
    
    // uses preset grip offset to move
    public void gripWithOffset(){
        try {
            writeRegister(65, 2, 16);
            // System.out.println("Setting position " + pos);
            logger.info("Gripping with offset");
            
        } catch (IOException e) {
            // System.err.println("Failed to grip:" + e.getMessage());
            logger.error("Failed to grip no offset: " + e.getMessage());
        }
    }
	
    // stops all movement (i think like an e-stop, need to test)
	public void stop(){
		try {
            writeRegister(65, 1, 8);
//            writeRegister(65, 2, 1);
            // System.out.println("Setting position " + pos);
            logger.info("Stopping gripper");
            
        } catch (IOException e) {
            // System.err.println("Failed to stop:" + e.getMessage());
            logger.error("Failed to stop: " + e.getMessage());
        }
	}
	
	// sets grip offset
    public void setOffset(int offset) {
        try {
            writeRegister(65, 1031, offset*10); // Convert mm to 1/10 mm for the register
            if (!offsetSet) {
                offsetSet = true; // Set the flag to true after the first successful offset set
            }
            logger.info("Setting offset " + offset);
        } catch (IOException e) {
            logger.error("Failed to set offset: " + e.getMessage());
        }
    }
	
    // Resets force and position settings
    public void reset(){
        try {
            writeRegister(63, 0, 2); // Reset force
            writeRegister(63, 0, 0); // Reset position

            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                System.err.println("Sleep interrupted: " + e.getMessage());
            }

            logger.info("Gripper power cycled.");

        } catch (IOException e) {
            logger.error("Failed to reset gripper: " + e.getMessage());
        }
    }
	
	
	// READ FUNCTIONS

    // Status Values:
//1 - Busy
//2 - Grip detected
//4 - S1 pushed
//8 - S1 triggered
//16 - S2 pushed
//32 - S2 triggered
//64 - Safety error

    public float[] initialise() {

    	float offset = -1;
    	float depth = -1;
    	float width = -1;
    	int status = -1;
    	
        try {
            // Checking all gripper values
        	offset = readOffset();
            System.out.println("Offset: " + offset + " mm");
            depth = readDepth();
            System.out.println("Depth: " + depth + " mm");
            width = readWidth()/10.0f;
            System.out.println("Width: " + width + " mm");
            status = readStatus(true);
            setForce(0);
            System.out.println("Set Force to: 0N");

            // Optional
            // setPos(0);
            
        } catch (Exception e) {
            logger.error("Failed to activate gripper: " + e.getMessage());
        } finally {
        	logger.info("Gripper activated.");
        	logger.info("Current Width: " + width + "mm");
        }
        return new float[]{width,0};
    }

	public int readStatus(boolean verbose) {
		int status = 0;
        try {
            status = readRegister(65, 268, false);
            if (verbose){
                if (status == 0) {
                    logger.info("Gripper is idle.");
                } else if (status == 1) {
                    logger.info("Gripper is busy.");
                } else if (status == 2) {
                    logger.info("Grip detected.");
                } else if (status == 4) {
                    logger.info("S1 pushed.");
                } else if (status == 8) {
                    logger.info("S1 triggered.");
                } else if (status == 16) {
                    logger.info("S2 pushed.");
                } else if (status == 32) {
                    logger.info("S2 triggered.");
                } else if (status == 64) {
                    logger.error("Safety error detected.");
                } else {
                    logger.info("Unknown status: " + status);
                }
            }
        } catch (IOException e) {
            logger.error("Failed to read status: " + e.getMessage());
        }

        return status;
	}
	
	// public int readObjectDetection(){
	// 	boolean status1 = gripperIO.getGOBJ1();
	// 	boolean status0 = gripperIO.getGOBJ0();
	// 	if (status1&&status0){
	// 		//logger.info("Fingers in Requested Position");
	// 		return 3; //Activation Complete
	// 	}else if (status1&&!status0){
	// 		//logger.info("Object Deteted, Fingers Stopped while closing");
	// 		return 2;
	// 	}else if (!status1&&status0){
	// 		//logger.info("Object Deteted, Fingers Stopped while opening");
	// 		return 1;
	// 	}else if (!status1&&!status0){
	// 		//logger.info("Fingers in motion");
	// 		return 0;
	// 	}else{
	// 		//logger.error("Object Deteted ERROR");
	// 		return 10; //Activation in Progress
	// 	}
	// }
	
	public int readOffset(){
        int offset = -1;
        try {
            offset = readRegister(65, 258, false);
            logger.info("Offset: " + offset);
        } catch (IOException e) {
            logger.error("Failed to read offset: " + e.getMessage());
        }
        return offset/10; // In mm
	}

    public int readDepth(){
        int depth = -1;
        try {
            depth = readRegister(65, 263, false);
            logger.info("Depth: " + depth);
        } catch (IOException e) {
            logger.error("Failed to read depth: " + e.getMessage());
        }
        return depth/10; // In mm
    }

    public int readDepthRelative(){
        int depthRelative = -1;
        try {
            depthRelative = readRegister(65, 264, false);
            logger.info("Depth Relative: " + depthRelative);
        } catch (IOException e) {
            logger.error("Failed to read depth relative: " + e.getMessage());
        }
        return depthRelative/10; // In mm
    }

    public int readWidth(){
        // Current width between fingers (as measured between the two original pads)
        int width = -1;
        try {
            width = readRegister(65, 267, false);
            logger.info("Width: " + width);
        } catch (IOException e) {
            logger.error("Failed to read width: " + e.getMessage());
        }
        return width; // In mm
    }
	
	public float readWidthOffset(){
        // Current width between fingers considering fingertip offset
        float widthActual = -1;
        try {
            widthActual = readRegister(65, 275, false);
            logger.info("Width Actual: " + widthActual);
        } catch (IOException e) {
            logger.error("Failed to read width actual: " + e.getMessage());
        }
        return widthActual/10; // In mm
    }	
	

//Level 2 Functions - Combined

	
	public void open(int force) {
        openWithForce(force);
    }

    public void open() {
        openWithForce(1200);
    }

    private void openWithForce(int force) {
        try {
            writeRegister(65, 0, force);
            writeRegister(65, 1, 1600);
            writeRegister(65, 2, 1);
            System.out.println("Opening with force " + force);
            int busy = 1;
            while (busy == 1) {
                busy = readStatus(false);
            }
            readStatus(true);

        } catch (IOException e) {
            System.err.println("Failed to write value to slave: " + e.getMessage());
        }
    }
	
	// Close, with or without a given force (defaults to max force)
	public void close(int force) {
        closeWithForce(force);
    }

    public void close() {
        closeWithForce(1200);
    }
        

    public void closeWithForce(int force) {
        try {
            
            writeRegister(65, 0, force);
            writeRegister(65, 1, 0);
            writeRegister(65, 2, 1);
            System.out.println("CLosing with force " + force);
            int busy = 1;
            while (busy == 1) {
                busy = readStatus(false);
            }
            readStatus(true);
        } catch (IOException e) {
            System.err.println("Failed to write value to slave: " + e.getMessage());
        }
    }

    public static void main(String[] args) {
        Gripper_test gripper = new Gripper_test();
        
        // Create a simple logger implementation for standalone use
        gripper.logger = new ITaskLogger() {
            @Override
            public void info(String message) {
                System.out.println("INFO: " + message);
            }
            
            @Override
            public void error(String message) {
                System.err.println("ERROR: " + message);
            }
            
            @Override
            public void error(String message, Throwable throwable) {
                System.err.println("ERROR: " + message);
                if (throwable != null) {
                    throwable.printStackTrace();
                }
            }
            
            @Override
            public void warn(String message) {
                System.out.println("WARN: " + message);
            }
            
            @Override
            public void warn(String message, Throwable throwable) {
                System.out.println("WARN: " + message);
                if (throwable != null) {
                    throwable.printStackTrace();
                }
            }
            
            @Override
            public void fine(String message) {
                System.out.println("FINE: " + message);
            }
        };
        
        try {
            // Initialize the gripper
            System.out.println("Initializing gripper...");
            gripper.initialise();
            
            // Wait a moment after initialization
            Thread.sleep(2000);
            
            // Open the gripper
            // System.out.println("Opening gripper...");
            // gripper.open();
            
            // // Wait a moment
            Thread.sleep(2000);
            
            // // Close the gripper
            // System.out.println("Closing gripper...");
            // gripper.close();
            
            System.out.println("Gripper test completed successfully!");
            
        } catch (InterruptedException e) {
            System.err.println("Thread interrupted: " + e.getMessage());
            Thread.currentThread().interrupt();
        } catch (Exception e) {
            System.err.println("Error during gripper test: " + e.getMessage());
        }
    }

}


