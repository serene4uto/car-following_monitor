import carla
import argparse
import logging
import numpy as np
import time
from PIL import Image 

from carla import ColorConverter as cc

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()

class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None
    
class SensorMonitorManager:
    def __init__(self, vehicle_monitor, display_man, sensor_role_name, display_pos, color_converter=None):
        self.surface = None
        self.vehicle_monitor = vehicle_monitor
        self.display_man = display_man
        self.display_pos = display_pos

        self.color_converter = color_converter

        self._sensor = None
        # get the sensor with the specified role name
        for sensor in vehicle_monitor.sensors:
            if sensor_role_name in sensor.attributes['role_name']:
                self._sensor = sensor
                break

        self.init_sensor_monitor()
        self.timer = CustomTimer()
        
        self.time_processing = 0.0
        self.tics_processing = 0.0

        self.display_man.add_sensor(self)
    
    def init_sensor_monitor(self):
        if 'RGBCamera' or 'DepthCamera' in self._sensor.attributes['role_name']:
            self._sensor.listen(self.process_camera_sensor)
    
    def get_sensor(self):
        return self.sensor
    

    def process_camera_sensor(self, image):
        t_start = self.timer.time()

        image.convert(self.color_converter)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        # Resize the image to fit the display
        array = self.fit_display(array)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def fit_display(self, image_array, stretch=False):

        
        pil_image = Image.fromarray(image_array)
        
        if stretch:
            target_size = self.display_man.get_display_size()  # Get the display size
        else:
            display_size = self.display_man.get_display_size()  # Get the display size
            pil_image_size = pil_image.size  # Get the image size
            ratio = min(display_size[0] / pil_image_size[0], display_size[1] / pil_image_size[1])
            target_size = (int(pil_image_size[0] * ratio), int(pil_image_size[1] * ratio))
            

        # Resize the image using Pillow
        pil_image = pil_image.resize(target_size)

        # Convert back to NumPy array for Pygame
        return np.array(pil_image)
    
    
    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)
    
    def destroy(self):
        pass


    
class VehicleMonitor:
    def __init__(self, world, vehicle_role_name):
        self.world = world
        self.vehicle_role_name = vehicle_role_name
        self.sensors = []

        # Get the vehicle with the specified role name
        for vehicle in self.world.get_actors().filter('vehicle.*'):
            if vehicle.attributes['role_name'] == vehicle_role_name:
                self.vehicle = vehicle
                break
        
        if self.vehicle is None:
            raise ValueError("No vehicle found with role name {}".format(vehicle_role_name))
        
        # Get all sensors attached to the vehicle
        for sensor in self.world.get_actors().filter('sensor.*'):
            if self.vehicle_role_name in sensor.attributes['role_name']:
                self.sensors.append(sensor)
    
    def get_sensor(self, sensor_role_name):
        for sensor in self.sensors:
            if sensor_role_name in sensor.attributes['role_name']:
                return sensor
        return None
        
    


        



def argparser():
    argparser = argparse.ArgumentParser(
        description="Monitor a vehicle")
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='Name of the vehicle role to monitor (default: "hero")')
    
    return argparser.parse_args()



def monitor_loop(args):
    pygame.init()
    pygame.font.init()


    try:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        sim_world = client.get_world()

        # display = pygame.display.set_mode(
        # (1280, 720),
        #     pygame.HWSURFACE | pygame.DOUBLEBUF)
        # display.fill((0,0,0))
        # pygame.display.flip()

        # initialize 
        vehicle_monitor = VehicleMonitor(sim_world, args.rolename)
        display_manager = DisplayManager(grid_size=[2, 2], window_size=[args.width, args.height])

        # add sensor monitor to the display manager
        SensorMonitorManager(vehicle_monitor, display_manager, 'RGBCamera_Driver_Seat', display_pos=[0, 0], color_converter=cc.Raw)
        SensorMonitorManager(vehicle_monitor, display_manager, 'DepthCamera_Bumper', display_pos=[0, 1], color_converter=cc.Raw)
        SensorMonitorManager(vehicle_monitor, display_manager, 'DepthCamera_Rear', display_pos=[1, 0], color_converter=cc.Depth)
        # SensorMonitorManager(vehicle_monitor, display_manager, 'DepthCamera_Bumper', display_pos=[1, 1], color_converter=cc.LogarithmicDepth)



        call_exit = False

        print(display_manager.get_display_size())

        while True:
            if args.sync:
                sim_world.tick()
            else:
                sim_world.wait_for_tick()

            # Render received data
            display_manager.render()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break

            if call_exit:
                break

        

    finally:
        pygame.quit()


def main():
    args = argparser()
    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    try:
        monitor_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':
    main()





