from threading import Lock


class SharedState:
    def __init__(self):
        self.target_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.radius = 0
        self.camera_vector = [0, 0, 0]
        self.lock = Lock()

    def update_camera_vector(self, vector: list[int]):
        with self.lock:
            self.camera_vector = vector

    def update_target_joints(self, joint_list: list[int]):
        with self.lock:
            self.target_joints = joint_list

    def update_radius(self, radius):
        with self.lock:
            self.radius = radius / 2

    def get_camera_vector(self) -> list[int, int, int]:
        with self.lock:
            return self.camera_vector
