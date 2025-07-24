import asyncio
from asyncua import Client, ua
from shared_state import SharedState
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotCommunication:
    def __init__(self, shared_vector: SharedState):
        self.url = "opc.tcp://172.24.200.1:4840/"
        self.objects_name = "0:Objects"
        self.robot_name = "22:robot2"
        self.shared_vector = shared_vector
        self.running = False
        self.update_interval = 0.005

    def start(self):
        self.running = True
        asyncio.run(self.main_loop())

    def stop(self):
        self.running = False

    async def main_loop(self):
        async with Client(url=self.url) as client:
            root = client.get_root_node()

            objects = await root.get_child([self.objects_name])

            robot2 = await objects.get_child(self.robot_name)

            r2c_start_node = await robot2.get_child("R2c_Start")
            r2c_prog_id_node = await robot2.get_child("R2c_ProgID")
            r2c_posx_node = await robot2.get_child("R2c_PosX")
            r2c_posy_node = await robot2.get_child("R2c_PosY")
            r2c_posz_node = await robot2.get_child("R2c_PosZ")

            program_id = ua.Variant(1, ua.VariantType.Int32)
            start = ua.Variant(True, ua.VariantType.Boolean)

            # previous_vector = None

            while self.running:

                current_vector = self.shared_vector.vector

                # logger.info(f"Current vector: {current_vector}")

                # if the 

                # robot_running = await r2c_start_node.read_value()

                # logger.info(f"Vector: {current_vector}")
                # logger.info(f"Robot Running?: {robot_running}")

                # if current_vector != previous_vector and not robot_running:
                # if current_vector != previous_vector:
                await r2c_posx_node.write_value(-current_vector[0])
                await r2c_posy_node.write_value(-current_vector[1])   
                await r2c_posz_node.write_value(current_vector[2])

                await r2c_prog_id_node.write_value(program_id)
                await r2c_start_node.write_value(start)

                    # previous_vector = current_vector

                await asyncio.sleep(self.update_interval)

            program_id = ua.Variant(0, ua.VariantType.Int32)
            start = ua.Variant(False, ua.VariantType.Boolean)

            await r2c_prog_id_node.write_value(program_id)
            await r2c_start_node.write_value(start)