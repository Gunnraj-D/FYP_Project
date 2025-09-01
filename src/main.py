"""
Main control loop for robot arm hand tracking system.
Coordinates between hand detection, kinematics, and OPC UA communication.
"""
from hand_detection_module import HandTracker
import numpy as np
from shared_state import SharedState
from config import LOOP_RATE_MS, URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
from camera_transform_module import transform_camera_to_base
from kinematics_solver import InverseKinematicsSolver
from opc_communication import RobotCommunication
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RobotHandTrackingSystem:
    """Main system coordinator for hand tracking robot control."""
    
    def __init__(self):
        # Initialize shared state
        self.shared_state = SharedState()
        
        # Initialize modules
        self.hand_tracker = HandTracker(self.shared_state)
        self.robot_comm = RobotCommunication(self.shared_state)
        self.kinematics_solver = InverseKinematicsSolver(
            URDF_FILEPATH, BASE_ELEMENT, ACTIVE_LINKS
        )
        
        # Control flags
        self.running = False
        self.mode = "IDLE"  # IDLE, TRACKING, PICKUP, PLACE
        
    def start(self):
        """Start all system components."""
        logger.info("Starting Robot Hand Tracking System...")
        self.running = True
        
        # Start hand tracker
        self.hand_tracker.start()
        
        # Start OPC UA communication in separate thread
        self.robot_comm.start()
        
        # Start main control loop
        self.run_control_loop()
        
    def stop(self):
        """Stop all system components."""
        logger.info("Stopping Robot Hand Tracking System...")
        self.running = False
        
        # Stop components
        self.hand_tracker.stop()
        self.robot_comm.stop()
        
        logger.info("System stopped.")
        
    def run_control_loop(self):
        """Main control loop for processing hand tracking and robot control."""
        loop_start_time = 0
        
        try:
            logger.info("Control loop started. Press Ctrl+C to stop.")
            
            while self.running:
                loop_start_time = time.time()
                
                # Process based on current mode
                if self.mode == "TRACKING":
                    self.process_hand_tracking()
                elif self.mode == "PICKUP":
                    self.process_pickup_sequence()
                elif self.mode == "PLACE":
                    self.process_place_sequence()
                else:  # IDLE
                    self.process_idle()
                
                # Maintain loop rate
                self.maintain_loop_rate(loop_start_time)
                
        except KeyboardInterrupt:
            logger.info("Stopping due to keyboard interrupt...")
        except Exception as e:
            logger.error(f"Error in control loop: {e}")
        finally:
            self.stop()
            
    def process_hand_tracking(self):
        """Process hand tracking mode - follow detected hand."""
        camera_vector = self.shared_state.get_camera_vector()
        
        # Skip if no hand detected
        if camera_vector == [0.0, 0.0, 0.0]:
            logger.debug("No hand detected")
            return
            
        # Get current joint positions from robot
        current_joints = self.shared_state.get_current_joints()
        
        # Add base and end-effector dummy joints for kinematics
        current_joints_full = np.insert(current_joints, 0, 0.0)
        current_joints_full = np.append(current_joints_full, 0.0)
        
        # Calculate current TCP pose
        tcp_pose = self.kinematics_solver.solve_tcp(current_joints_full)
        
        # Transform hand position from camera to base frame
        hand_position_base = transform_camera_to_base(camera_vector, tcp_pose)
        
        # Solve inverse kinematics for target position
        target_joints_full = self.kinematics_solver.solve_XYZ(
            hand_position_base, current_joints_full
        )
        
        # Remove base and end-effector joints
        target_joints = target_joints_full[1:8]
        
        # Update shared state with target joints
        self.shared_state.update_target_joints(target_joints)
        
        logger.debug(f"Target joints updated: {target_joints}")
        
    def process_pickup_sequence(self):
        """Process pickup sequence - move to pickup location and grasp."""
        # TODO: Implement pickup sequence
        # 1. Move to predefined pickup location
        # 2. Engage object detection
        # 3. Generate grasp pose
        # 4. Execute grasp
        # 5. Transition to PLACE mode
        logger.info("Pickup sequence not yet implemented")
        self.mode = "IDLE"
        
    def process_place_sequence(self):
        """Process place sequence - track hand and place when stable."""
        # TODO: Implement place sequence
        # 1. Track hand position
        # 2. Check if hand is stable (stationary for X seconds)
        # 3. Move to hand position
        # 4. Release object
        # 5. Retract and transition to IDLE
        logger.info("Place sequence not yet implemented")
        self.mode = "IDLE"
        
    def process_idle(self):
        """Process idle mode - maintain current position."""
        # In idle mode, robot maintains current position
        # Could add home position logic here if needed
        pass
        
    def maintain_loop_rate(self, loop_start_time):
        """Maintain consistent loop rate."""
        elapsed_ms = (time.time() - loop_start_time) * 1000
        remaining_ms = LOOP_RATE_MS - elapsed_ms
        
        if remaining_ms > 0:
            time.sleep(remaining_ms / 1000)
        else:
            logger.warning(f"Loop exceeded {LOOP_RATE_MS}ms by {-remaining_ms:.1f}ms")
            
    def set_mode(self, mode):
        """Set the operating mode of the system."""
        valid_modes = ["IDLE", "TRACKING", "PICKUP", "PLACE"]
        if mode in valid_modes:
            self.mode = mode
            logger.info(f"Mode changed to: {mode}")
        else:
            logger.error(f"Invalid mode: {mode}")


def main():
    """Main entry point for the application."""
    system = RobotHandTrackingSystem()
    
    try:
        # For now, automatically start in tracking mode
        system.mode = "TRACKING"
        system.start()
        
    except Exception as e:
        logger.error(f"System error: {e}")
        system.stop()


if __name__ == "__main__":
    main()