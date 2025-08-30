
import math 

class SharedState:
    def __init__(self):
        self.vector = (0.0, 0.0, 0.0)  # vector should be in mm
        self.radius = 0
        self.z_tolerance = 20

    def update_camera_vector(self, vector):
        z_comp = vector[2]
        # print(z_comp)
        if z_comp != 0:
            z_comp = vector[2] - 300
        
        xy_mag = math.sqrt(vector[0]**2 + vector[1]**2)
        total_magnitude = math.sqrt(vector[0]**2 + vector[1]**2 + z_comp**2)
        if xy_mag < self.radius:
            if z_comp < self.z_tolerance:
                self.vector = (0.0, 0.0, 0.0)
            else:
                self.vector = (0.0, 0.0, z_comp)
            return
        
        # TODO: janky check, should be improved
        # if vector[0] > 

        # self.vector = vector
        vector_size = 7

        if total_magnitude <= vector_size:
            self.vector = vector
        else:
            self.vector = (vector[0] / total_magnitude * vector_size, vector[1] / total_magnitude * vector_size, z_comp / total_magnitude * vector_size)
        

    def update_radius(self, radius):
        self.radius = radius / 2