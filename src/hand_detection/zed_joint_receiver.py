"""
ZED Joint Data Receiver
Receives skeleton joint tracking data from Unity ZED Body Tracking via TCP.

The Unity script sends newline-delimited JSON messages containing:
- Frame number
- Multiple skeletons (each with ID and 38 joint positions)
- Joint names from ZED SDK BODY_38_PARTS enum

Usage:
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
        host: str = '127.0.0.1',
        port: int = 5005,
        callback: Optional[Callable[[FrameData], None]] = None,
        buffer_size: int = 4096
    ):
        """
        Initialize ZED Joint Receiver.

        Args:
            host: Host address to bind to
            port: Port to listen on
            callback: Optional callback function called for each received frame
            buffer_size: Size of receive buffer in bytes
        """
        self.host = host
        self.port = port
        self.callback = callback
        self.buffer_size = buffer_size

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

            logger.info(
                f"Listening for Unity connection on {self.host}:{self.port}")

            while self.running:
                try:
                    # Accept client connection (Unity)
                    self.client_socket, addr = self.server_socket.accept()
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
                        logger.info("Client disconnected")
                        break

                    # Decode and add to buffer
                    self.message_buffer += data.decode('utf-8')

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
                joint = JointData(
                    joint_name=raw_joint.get('jointName', 'UNKNOWN'),
                    x=raw_joint.get('x', 0.0),
                    y=raw_joint.get('y', 0.0),
                    z=raw_joint.get('z', 0.0)
                )
                joints.append(joint)

            skeleton = SkeletonData(skeleton_id=skeleton_id, joints=joints)
            skeletons.append(skeleton)

        return FrameData(frame=frame, skeletons=skeletons)

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
        """Callback when new frame is received."""
        print(f"\n--- Frame {frame_data.frame} ---")
        print(f"Skeletons: {len(frame_data.skeletons)}")

        for skeleton in frame_data.skeletons:
            print(
                f"  Skeleton ID: {skeleton.skeleton_id}, Joints: {len(skeleton.joints)}")

            # Example: Print specific joint positions
            if skeleton.joints:
                first_joint = skeleton.joints[0]
                print(f"    First joint ({first_joint.joint_name}): "
                      f"({first_joint.x:.3f}, {first_joint.y:.3f}, {first_joint.z:.3f})")

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
