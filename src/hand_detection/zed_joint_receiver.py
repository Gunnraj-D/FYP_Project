"""
ZED Joint Data Receiver
Receives skeleton joint tracking data from Unity ZED Body Tracking via TCP.

The Unity script sends newline-delimited JSON messages containing:
- Frame number
- Multiple skeletons (each with ID and 38 joint positions)
- Joint names from ZED SDK BODY_38_PARTS enum

Network Configuration:
- Default host '0.0.0.0' accepts connections from any device on the network
- Use host='127.0.0.1' to accept only local connections
- Ensure firewall allows incoming connections on the specified port

Usage:
    # Accept connections from network devices (default)
    receiver = ZEDJointReceiver(host='0.0.0.0', port=5005)
    receiver.start()
    
    # Accept only local connections
    receiver = ZEDJointReceiver(host='127.0.0.1', port=5005)
    receiver.start()
    
    # Get latest data
    frame_data = receiver.get_latest_frame()
    
    # Or use callback
    def on_frame(frame_data):
        print(f"Frame {frame_data['frame']}: {len(frame_data['skeletons'])} skeletons")
    
    receiver = ZEDJointReceiver(callback=on_frame)
    receiver.start()
"""

import socket
import json
import threading
import logging
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, field
import time

logger = logging.getLogger(__name__)

from config import ZED_MANUAL_OFFSET

@dataclass
class JointData:
    """Represents a single joint position."""
    joint_name: str
    x: float
    y: float
    z: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            'joint_name': self.joint_name,
            'x': self.x,
            'y': self.y,
            'z': self.z
        }

    def to_position(self) -> List[float]:
        """Return as [x, y, z] list."""
        return [self.x, self.y, self.z]


@dataclass
class SkeletonData:
    """Represents a tracked skeleton with all its joints."""
    skeleton_id: int
    joints: List[JointData] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'skeleton_id': self.skeleton_id,
            'joints': [joint.to_dict() for joint in self.joints]
        }

    def get_joint_by_name(self, joint_name: str) -> Optional[JointData]:
        """Get a specific joint by name."""
        for joint in self.joints:
            if joint.joint_name == joint_name:
                return joint
        return None

    def get_joint_position(self, joint_name: str) -> Optional[List[float]]:
        """Get joint position as [x, y, z] or None if not found."""
        joint = self.get_joint_by_name(joint_name)
        return joint.to_position() if joint else None


@dataclass
class FrameData:
    """Represents a complete frame of skeleton tracking data."""
    frame: int
    skeletons: List[SkeletonData] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'frame': self.frame,
            'timestamp': self.timestamp,
            'skeletons': [skeleton.to_dict() for skeleton in self.skeletons]
        }

    def get_skeleton(self, skeleton_id: int) -> Optional[SkeletonData]:
        """Get a specific skeleton by ID."""
        for skeleton in self.skeletons:
            if skeleton.skeleton_id == skeleton_id:
                return skeleton
        return None


class ZEDJointReceiver:
    """
    TCP server that receives skeleton joint data from Unity ZED Body Tracking.
    Runs in a separate thread and provides latest frame data.
    """

    def __init__(
        self,
        host: str = '0.0.0.0',
        port: int = 5005,
        callback: Optional[Callable[[FrameData], None]] = None,
        buffer_size: int = 4096,
        remap_to_z_up: bool = True,
        smoothing_factor: float = 0.3,
        tracking_loss_frames: int = 5
    ):
        """
        Initialize ZED Joint Receiver.

        Args:
            host: Host address to bind to ('0.0.0.0' for all network interfaces, 
                  '127.0.0.1' for localhost only)
            port: Port to listen on
            callback: Optional callback function called for each received frame
            buffer_size: Size of receive buffer in bytes
            remap_to_z_up: If True, remap axes (x,y,z)->(z,x,y) and negate y
            smoothing_factor: EMA smoothing factor (0-1, higher = less smooth)
            tracking_loss_frames: Frames to wait before declaring tracking lost
        """
        self.host = host
        self.port = port
        self.callback = callback
        self.buffer_size = buffer_size
        self.remap_to_z_up = remap_to_z_up
        self.smoothing_factor = smoothing_factor
        self.tracking_loss_frames = tracking_loss_frames

        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None

        self.latest_frame: Optional[FrameData] = None
        self.frame_lock = threading.Lock()

        self.total_frames_received = 0
        self.connection_active = False

        # Buffer for incomplete messages
        self.message_buffer = ""

        # Smoothing state: {skeleton_id: {joint_name: [x, y, z]}}
        self.smoothed_positions: Dict[int, Dict[str, List[float]]] = {}
        # Track frames since last detection: {skeleton_id: frame_count}
        self.frames_since_detection: Dict[int, int] = {}

    def start(self):
        """Start the receiver in a background thread."""
        if self.running:
            logger.warning("Receiver already running")
            return

        self.running = True
        self.thread = threading.Thread(target=self._run_server, daemon=True)
        self.thread.start()
        logger.info(f"ZED Joint Receiver started on {self.host}:{self.port}")

    def stop(self):
        """Stop the receiver and close connections."""
        logger.info("Stopping ZED Joint Receiver...")
        self.running = False

        if self.client_socket:
            try:
                self.client_socket.close()
            except Exception:
                pass

        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass

        if self.thread:
            self.thread.join(timeout=2.0)

        logger.info("ZED Joint Receiver stopped")

    def _run_server(self):
        """Main server loop running in background thread."""
        try:
            # Create and bind server socket
            self.server_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            # Timeout for checking self.running
            self.server_socket.settimeout(1.0)

            print("\n" + "="*60)
            print(f"📡 ZED JOINT RECEIVER: WAITING FOR CONNECTION")
            print(f"   Listening on: {self.host}:{self.port}")
            print(
                f"   Coordinate Remapping: {'ENABLED (Z-up)' if self.remap_to_z_up else 'DISABLED'}")
            print("="*60 + "\n")
            logger.info(
                f"Listening for Unity connection on {self.host}:{self.port}")

            while self.running:
                try:
                    # Accept client connection (Unity)
                    self.client_socket, addr = self.server_socket.accept()
                    print("\n" + "="*60)
                    print(f"🔗 ZED JOINT RECEIVER: CONNECTION ESTABLISHED")
                    print(f"   Client Address: {addr[0]}:{addr[1]}")
                    print(f"   Server Address: {self.host}:{self.port}")
                    print("="*60 + "\n")
                    logger.info(f"Unity client connected from {addr}")
                    self.connection_active = True

                    # Handle client connection
                    self._handle_client()

                except socket.timeout:
                    # Normal timeout, continue loop to check self.running
                    continue
                except Exception as e:
                    if self.running:
                        logger.error(f"Error accepting connection: {e}")
                    time.sleep(1)

        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            self.connection_active = False

    def _handle_client(self):
        """Handle connected client (Unity) and receive data."""
        try:
            self.client_socket.settimeout(5.0)  # Timeout for recv
            self.message_buffer = ""

            while self.running and self.client_socket:
                try:
                    # Receive data
                    data = self.client_socket.recv(self.buffer_size)

                    if not data:
                        print("\n" + "="*60)
                        print(f"❌ ZED JOINT RECEIVER: CLIENT DISCONNECTED")
                        print("="*60 + "\n")
                        logger.info("Client disconnected")
                        break

                    # Decode and add to buffer (utf-8-sig automatically handles BOM)
                    self.message_buffer += data.decode('utf-8-sig')

                    # Process complete messages (newline-delimited)
                    self._process_buffer()

                except socket.timeout:
                    # Normal timeout, check if still running
                    continue
                except Exception as e:
                    logger.error(f"Error receiving data: {e}")
                    break

        finally:
            if self.client_socket:
                try:
                    self.client_socket.close()
                except Exception:
                    pass
            self.client_socket = None
            self.connection_active = False

    def _process_buffer(self):
        """Process message buffer and extract complete JSON messages."""
        while '\n' in self.message_buffer:
            # Extract one complete message
            message, self.message_buffer = self.message_buffer.split('\n', 1)
            message = message.strip()

            if message:
                try:
                    self._parse_and_store_frame(message)
                except Exception as e:
                    logger.error(f"Error parsing message: {e}")

    def _parse_and_store_frame(self, json_string: str):
        """Parse JSON message and store frame data."""
        try:
            # Parse JSON
            raw_data = json.loads(json_string)

            # Convert to structured data
            frame_data = self._convert_to_frame_data(raw_data)

            # Store latest frame
            with self.frame_lock:
                self.latest_frame = frame_data
                self.total_frames_received += 1

            # Call callback if provided
            if self.callback:
                try:
                    self.callback(frame_data)
                except Exception as e:
                    logger.error(f"Error in callback: {e}")

        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON: {e}")
        except Exception as e:
            logger.error(f"Error processing frame: {e}")

    def _convert_to_frame_data(self, raw_data: Dict[str, Any]) -> FrameData:
        """Convert raw JSON dict to FrameData object."""
        frame = raw_data.get('frame', 0)
        skeletons = []

        for raw_skeleton in raw_data.get('skeletons', []):
            skeleton_id = raw_skeleton.get('skeletonID', 0)
            joints = []

            for raw_joint in raw_skeleton.get('joints', []):
                # Get raw coordinates from Unity
                x_raw = raw_joint.get('x', 0.0)
                y_raw = raw_joint.get('y', 0.0)
                z_raw = raw_joint.get('z', 0.0)

                # Remap axes to Z-up, right-handed if requested: (x,y,z)->(z,x,y)
                if self.remap_to_z_up:
                    x_out = z_raw
                    y_out = x_raw
                    z_out = y_raw
                else:
                    x_out = x_raw
                    y_out = y_raw
                    z_out = z_raw

                # Negate Y-axis for robot base frame
                y_out = -y_out

                # Apply manual calibration offset AFTER coordinate transforms
                
                x_out += ZED_MANUAL_OFFSET.get('x', 0.0)
                y_out += ZED_MANUAL_OFFSET.get('y', 0.0)
                z_out += ZED_MANUAL_OFFSET.get('z', 0.0)

                # Apply exponential smoothing to reduce jitter
                joint_name = raw_joint.get('jointName', 'UNKNOWN')
                x_smooth, y_smooth, z_smooth = self._apply_smoothing(
                    skeleton_id, joint_name, x_out, y_out, z_out
                )

                joint = JointData(
                    joint_name=joint_name,
                    x=x_smooth,
                    y=y_smooth,  # Y-axis negated before offset/smoothing
                    z=z_smooth
                )
                joints.append(joint)

            # Mark skeleton as detected this frame
            self.frames_since_detection[skeleton_id] = 0

            skeleton = SkeletonData(skeleton_id=skeleton_id, joints=joints)
            skeletons.append(skeleton)

        # Handle tracking loss tolerance - add skeletons with last known positions
        detected_ids = {s.skeleton_id for s in skeletons}
        for skeleton_id in list(self.frames_since_detection.keys()):
            if skeleton_id not in detected_ids:
                self.frames_since_detection[skeleton_id] += 1

                # If within tolerance, use last smoothed positions
                if self.frames_since_detection[skeleton_id] <= self.tracking_loss_frames:
                    if skeleton_id in self.smoothed_positions:
                        logger.debug(
                            f"Skeleton {skeleton_id} lost for {self.frames_since_detection[skeleton_id]} frames - using smoothed position")
                        # Create skeleton from smoothed positions
                        joints = []
                        for joint_name, pos in self.smoothed_positions[skeleton_id].items():
                            joints.append(JointData(
                                joint_name=joint_name,
                                x=pos[0],
                                y=pos[1],
                                z=pos[2]
                            ))
                        if joints:  # Only add if we have joints
                            skeletons.append(SkeletonData(
                                skeleton_id=skeleton_id, joints=joints))
                else:
                    # Exceeded tolerance - clear smoothing state
                    logger.info(
                        f"Skeleton {skeleton_id} lost tracking (>{self.tracking_loss_frames} frames)")
                    self.smoothed_positions.pop(skeleton_id, None)
                    self.frames_since_detection.pop(skeleton_id, None)

        return FrameData(frame=frame, skeletons=skeletons)

    def _apply_smoothing(self, skeleton_id: int, joint_name: str, x: float, y: float, z: float) -> tuple:
        """
        Apply exponential moving average smoothing to joint position.

        Args:
            skeleton_id: ID of the skeleton
            joint_name: Name of the joint
            x, y, z: New position

        Returns:
            Smoothed (x, y, z) tuple
        """
        # Initialize smoothing state for this skeleton if needed
        if skeleton_id not in self.smoothed_positions:
            self.smoothed_positions[skeleton_id] = {}
            self.frames_since_detection[skeleton_id] = 0

        # Get previous smoothed position or initialize
        if joint_name in self.smoothed_positions[skeleton_id]:
            prev = self.smoothed_positions[skeleton_id][joint_name]
            # Exponential moving average: new = alpha * current + (1-alpha) * previous
            alpha = self.smoothing_factor
            x_smooth = alpha * x + (1 - alpha) * prev[0]
            y_smooth = alpha * y + (1 - alpha) * prev[1]
            z_smooth = alpha * z + (1 - alpha) * prev[2]
        else:
            # First time seeing this joint - no smoothing
            x_smooth, y_smooth, z_smooth = x, y, z

        # Store smoothed position
        self.smoothed_positions[skeleton_id][joint_name] = [
            x_smooth, y_smooth, z_smooth]

        return x_smooth, y_smooth, z_smooth

    def get_latest_frame(self) -> Optional[FrameData]:
        """
        Get the most recently received frame data.
        Thread-safe.

        Returns:
            Latest FrameData or None if no data received yet
        """
        with self.frame_lock:
            return self.latest_frame

    def is_connected(self) -> bool:
        """Check if Unity client is currently connected."""
        return self.connection_active

    def get_stats(self) -> Dict[str, Any]:
        """Get receiver statistics."""
        return {
            'connected': self.connection_active,
            'total_frames': self.total_frames_received,
            'latest_frame_number': self.latest_frame.frame if self.latest_frame else None,
            'running': self.running
        }

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()


# Example usage
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    def on_frame_received(frame_data: FrameData):
        """Callback when new frame is received - shows all joint data."""
        print(f"\n{'='*80}")
        print(
            f"FRAME {frame_data.frame} | Skeletons: {len(frame_data.skeletons)}")
        print('='*80)

        for skeleton in frame_data.skeletons:
            print(
                f"\n  🧍 Skeleton ID: {skeleton.skeleton_id} | Joints: {len(skeleton.joints)}")
            print(f"  {'-'*76}")

            # Print all joint positions in a formatted way
            for i, joint in enumerate(skeleton.joints):
                print(f"  [{i:2d}] {joint.joint_name:25s} | "
                      f"x:{joint.x:7.3f} | y:{joint.y:7.3f} | z:{joint.z:7.3f}")

        if len(frame_data.skeletons) == 0:
            print("  ⚠️  No skeletons detected in this frame")

    # Start receiver with callback
    receiver = ZEDJointReceiver(callback=on_frame_received)
    receiver.start()

    print("Waiting for Unity connection on port 5005...")
    print("Press Ctrl+C to stop")

    try:
        while True:
            time.sleep(1)

            # Print stats every 5 seconds
            if receiver.total_frames_received > 0 and receiver.total_frames_received % 50 == 0:
                stats = receiver.get_stats()
                print(f"\nStats: {stats}")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        receiver.stop()
