import carla
import argparse
import random
import math
import pandas as pd
from pure_persuit import pure_persuit_control,calc_target_index
import _thread
import time

class VechicleState:
    def __init__(self,transform,velocity):
        self.x = transform.location.x
        self.y = transform.location.y
        self.z = transform.location.z
        self.pitch = transform.rotation.pitch
        self.yaw = transform.rotation.yaw
        self.roll = transform.rotation.roll
        self.v =  math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        self.wheelAngle = 0
        self.gear = 0

    def update(self,transform,velocity,vehicleControl):
        self.x = transform.location.x
        self.y = transform.location.y
        self.z = transform.location.z
        self.pitch = transform.rotation.pitch
        self.yaw = transform.rotation.yaw
        self.roll = transform.rotation.roll
        self.v = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        self.wheelAngle = vehicleControl.steer
        self.gear = vehicleControl.gear

    def get_status(self):
        status = {}
        status["timestamp"] = int(time.time())
        status["pose"] = {
            "position": {"x":self.x,"y":self.y,"z":self.z},
            "orientation": {"roll":self.roll,"pitch":self.pitch,"yaw":self.yaw}
        }
        status["v"] = self.v
        status["wheelAngle"] = self.wheelAngle
        status["gear"] = self.gear
        status["nodes"] = []
        return status

class Carla_Client:
    def __init__(self):
        argparser = argparse.ArgumentParser(
            description=__doc__)
        argparser.add_argument(
            '--host',
            metavar='H',
            default='10.214.143.7',
            help='IP of the host server (default: 127.0.0.1)')
        argparser.add_argument(
            '-p', '--port',
            metavar='P',
            default=2000,
            type=int,
            help='TCP port to listen to (default: 2000)')
        argparser.add_argument(
            '-n', '--number-of-vehicles',
            metavar='N',
            default=10,
            type=int,
            help='number of vehicles (default: 10)')
        argparser.add_argument(
            '-d', '--delay',
            metavar='D',
            default=2.0,
            type=float,
            help='delay in seconds between spawns (default: 2.0)')
        argparser.add_argument(
            '--safe',
            action='store_true',
            help='avoid spawning vehicles prone to accidents')
        self.args = argparser.parse_args()
        self.vControl = carla.VehicleControl()

    def myReadCsv(self,filePath):
        obj = pd.read_csv(filePath, encoding='utf-8', header=None, sep='\t')
        cx = []
        cy = []
        for indexs in obj.index:
            if indexs > 4:
                tmp = obj.loc[indexs].values[0].split(",")
                cx.append(float(tmp[0]))
                cy.append(float(tmp[1]))
        return cx, cy

    def get_car_status(self):
        return self.state.get_status()

    def init(self):
        self.actor_list = []
        client = carla.Client(self.args.host, self.args.port)
        client.set_timeout(2.0)
        world = client.get_world()
        vehicle_blueprints = world.get_blueprint_library().filter('vehicle.tesla.*')
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')  # 获得色彩镜头的蓝图
        # 在transform的位置产生车辆
        def try_spawn_random_vehicle_at(transform):
            blueprint = random.choice(vehicle_blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            blueprint.set_attribute('role_name', 'autopilot')
            # 这边设置了自动驾驶
            vehicle = world.try_spawn_actor(blueprint, transform)
            if vehicle is not None:
                self.actor_list.append(vehicle)
                print('spawned %r at %s' % (vehicle.type_id, transform.location))
                return vehicle
            return None

        self.my_vehicle = try_spawn_random_vehicle_at(
            carla.Transform(carla.Location(x=93.1965, y=0.914151, z=3), carla.Rotation(pitch=0, yaw=180, roll=0)))
        self.state = VechicleState(self.my_vehicle.get_transform(), self.my_vehicle.get_velocity())
        self.cx = []
        self.cy = []
        lastIndex = len(self.cx) - 1
        self.target_ind = 0

    def update(self):
        # delta返回的是弧度 范围为-PI/2 ~ PI/2 为前轮转向 要转化为 900度为 5PI
        self.state.update(self.my_vehicle.get_transform(), self.my_vehicle.get_velocity(), self.my_vehicle.get_control())
        if len(self.cx)>10:
            delta, target_ind = pure_persuit_control(self.state, self.cx, self.cy, self.target_ind)
            self.target_ind = target_ind
            # print(self.target_ind)
            # 方向盘转角和前轮夹角的关系一般为10:1 [-2.5PI , 2.5PI] ---> [-1.0,1.0]
            self.vControl.steer = delta * 10 * 2.0 / (5 * math.pi)
            self.vControl.throttle = 0.5
            print(self.vControl.steer, target_ind)
            self.my_vehicle.apply_control(self.vControl)

    def shutdown(self):
        print('\ndestroying %d actors' % len(self.actor_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])


if __name__ == '__main__':
    try:
        client = Carla_Client()
        client.init()
        while True:
            client.update()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
