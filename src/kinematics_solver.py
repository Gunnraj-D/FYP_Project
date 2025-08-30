from ikpy.chain import Chain


class InverseKinematicsSolver:
    def __init__(self, urdf_filpath: str, base_elements: list[str], active_links_mask: list[bool]):
        self.chain = Chain.from_urdf_file(
            urdf_filpath,
            base_elements=base_elements,
            active_links_mask=active_links_mask
        )

    def solve_tcp(self, current_joint_angles: list[int]):
        return self.chain.forward_kinematics(current_joint_angles)

    def solve_XYZ(self, target_position: list[int], current_joint_angles: list[int]):
        return self.chain.inverse_kinematics(
            target_position,
            initial_position=current_joint_angles
        )
