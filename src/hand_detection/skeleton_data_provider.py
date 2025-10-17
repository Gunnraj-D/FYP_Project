"""
Skeleton Data Provider - Unified interface for skeleton data sources.

Provides a consistent interface for getting skeleton data from either:
1. ZED Joint Receiver (live data)
2. Static configuration data (for testing/development)

This allows easy switching between data sources via configuration.
"""

import logging
from typing import Optional, Dict, List, Any
from dataclasses import dataclass, field
import time

from hand_detection.zed_joint_receiver import ZEDJointReceiver, SkeletonData, JointData, FrameData
from config import PATH_PLANNING_CONFIG

logger = logging.getLogger(__name__)


@dataclass
class StaticSkeletonData:
    """Represents static skeleton data from configuration."""
    skeleton_id: int = 0
    joints: List[JointData] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)

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

    def to_dict(self) -> Dict[str, Any]:
        return {
            'skeleton_id': self.skeleton_id,
            'joints': [joint.to_dict() for joint in self.joints],
            'timestamp': self.timestamp
        }


class SkeletonDataProvider:
    """
    Unified interface for skeleton data sources.

    Automatically switches between ZED receiver and static data based on configuration.
    """

    def __init__(self, zed_receiver: Optional[ZEDJointReceiver] = None):
        """
        Initialize skeleton data provider.

        Args:
            zed_receiver: ZED joint receiver instance (can be None if using static data)
        """
        self.zed_receiver = zed_receiver
        self.use_static_data = PATH_PLANNING_CONFIG.get(
            'use_static_skeleton_data', False)
        self.static_data = PATH_PLANNING_CONFIG.get('static_skeleton_data', {})

        if self.use_static_data:
            logger.info("Using static skeleton data from configuration")
            self._static_skeleton = self._create_static_skeleton()
        else:
            logger.info("Using live ZED skeleton data")
            self._static_skeleton = None

    def get_latest_frame(self) -> Optional[FrameData]:
        """
        Get the most recently available skeleton frame.

        Returns:
            FrameData with skeleton information, or None if no data available
        """
        if self.use_static_data:
            return self._get_static_frame()
        else:
            if self.zed_receiver is None:
                logger.error(
                    "ZED receiver not available and static data disabled")
                return None
            return self.zed_receiver.get_latest_frame()

    def _get_static_frame(self) -> Optional[FrameData]:
        """Get static skeleton frame from configuration."""
        if self._static_skeleton is None:
            return None

        # Create a FrameData object with the static skeleton
        return FrameData(
            frame=0,  # Static frame number
            skeletons=[self._static_skeleton],
            timestamp=time.time()
        )

    def _create_static_skeleton(self) -> Optional[StaticSkeletonData]:
        """Create static skeleton from configuration data with offset applied."""
        if not self.static_data:
            logger.error("No static skeleton data configured")
            return None

        # Get offset configuration
        offset = PATH_PLANNING_CONFIG.get(
            'static_data_offset', {'x': 0.0, 'y': 0.0, 'z': 0.0})
        offset_x = offset.get('x', 0.0)
        offset_y = offset.get('y', 0.0)
        offset_z = offset.get('z', 0.0)

        joints = []
        for joint_name, position in self.static_data.items():
            if len(position) != 3:
                logger.warning(
                    f"Invalid position data for {joint_name}: {position}")
                continue

            # Apply offset to position
            adjusted_x = position[0] + offset_x
            adjusted_y = position[1] + offset_y
            adjusted_z = position[2] + offset_z

            joint = JointData(
                joint_name=joint_name,
                x=adjusted_x,
                y=adjusted_y,
                z=adjusted_z
            )
            joints.append(joint)

        if not joints:
            logger.error("No valid joint data found in static configuration")
            return None

        skeleton = StaticSkeletonData(
            skeleton_id=0,
            joints=joints,
            timestamp=time.time()
        )

        logger.info(f"Created static skeleton with {len(joints)} joints")
        return skeleton

    def is_using_static_data(self) -> bool:
        """Check if currently using static data."""
        return self.use_static_data

    def get_data_source_info(self) -> Dict[str, Any]:
        """Get information about the current data source."""
        return {
            'use_static_data': self.use_static_data,
            'zed_receiver_available': self.zed_receiver is not None,
            'static_joints_count': len(self.static_data) if self.use_static_data else 0,
            'data_source': 'static_config' if self.use_static_data else 'zed_receiver'
        }
