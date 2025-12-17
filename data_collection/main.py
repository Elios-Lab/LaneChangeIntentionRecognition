#!/usr/bin/env python


"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

import os
import re
import glob
import os
import subprocess
import sys
import time
import psutil
import platform
import pandas as pd
import threading
import pygame.mixer 
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import ColorConverter as cc
import cv2
import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import csv
import queue
import shutil

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_f
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

from SpeedGauge import GaugeWidget
from RPMgauge import RPMGauge
from pydub import AudioSegment
from pydub.playback import play
import threading
import math
import time
import json

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================
def generate_log_filename():
	"""
	Generate a log filename with progressive numbering in the './logs' directory.
	The filenames will be in the format 'log_001.txt', 'log_002.txt', and so on, always using 3 digits.
	
	:return: The absolute path to the new file
	"""
	directory = './logs'
	
	# Ensure the directory exists
	if not os.path.exists(directory):
		os.makedirs(directory)
	
	# Find the next available filename
	file_number = 1
	while True:
		numbered_filename = f"log_{file_number:03}.txt"
		file_path = os.path.join(directory, numbered_filename)
		if not os.path.exists(file_path):
			break
		file_number += 1
	
	# Convert to absolute path
	absolute_path = os.path.abspath(file_path)
	
	return absolute_path

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if not bps:
        print(f"No blueprints found for filter: {filter}")  # Debugging line

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.set_clear_noon_weather() 
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        blueprint_list = get_actor_blueprints(self.world, 'vehicle.ford.crown', self._actor_generation)
        if not blueprint_list:
            raise ValueError("Couldn't find any blueprints with the specified filters")
        blueprint = random.choice(blueprint_list)
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)

            # Standard starting position
            wp = self.map.get_waypoint(carla.Location(x=-3451.6, y=-1796.9, z=0.018910)).next(2000)[0]

            spawn_point = wp.transform
            spawn_point.location.z += 1.0

            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            if self.player is None:
                print("Failed to spawn player at location:", spawn_point.location)
                print("Blueprint used:", blueprint)
                sys.exit(1)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
            
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        # self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

        print(self.player)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def set_clear_noon_weather(self):
        clear_noon_weather = carla.WeatherParameters.ClearNoon
        self.world.set_weather(clear_noon_weather)
        self.hud.notification('Weather: ClearNoon')

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensorF.destroy()
        self.camera_manager.sensorF = None
        self.camera_manager.sensorB.destroy()
        self.camera_manager.sensorB = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensorF,
            self.camera_manager.sensorB,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


if sys.version_info >= (3, 0):
    from configparser import ConfigParser

else:
    from ConfigParser import RawConfigParser as ConfigParser
    
class DualControl(object):
    def __init__(self, world, start_in_autopilot, wheel_path, speed_limit_mps =36):  # 36 
        self._autopilot_enabled = start_in_autopilot
        self._speed_limit_mps = speed_limit_mps
        self.left_blinker_manual = False  
        self.right_blinker_manual = False  
        self.blinker_auto_off = False  # Flag to reset manual blinker state after auto off
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # Initialize steering wheel
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()
        self._X_button_index = 5  # Left Blinker
        self._B_button_index = 4  # Right  Blinker
        self.throttle_state = 0

        self._parser = ConfigParser()
        self._parser.read(wheel_path)
        self._steer_idx = int(
            self._parser.get('G29 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('G29 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G29 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('G29 Racing Wheel', 'handbrake'))
    
    
    def set_manual_blinker_state(self, left_state, right_state):
        self.left_blinker_manual = left_state
        self.right_blinker_manual = right_state

    def parse_events(self, world, clock, sync_mode):
        if isinstance(self._control, carla.VehicleControl):
            self.current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    world.reset()
                elif event.button == 1:  # Reverse
                    world.hud.toggle_info()
                elif event.button == 3:
                    world.next_weather()
                elif event.button == self._reverse_idx:
                    world.camera_manager.toggle_camera()
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == 23:
                    world.camera_manager.next_sensor()
                elif event.button == self._X_button_index:
                    self.left_blinker_manual = not self.left_blinker_manual
                    self.blinker_auto_off = False  # Reset flag
                    if self.left_blinker_manual:
                        self._lights |= carla.VehicleLightState.LeftBlinker
                        self._lights &= ~carla.VehicleLightState.RightBlinker  # Ensure the right blinker is off
                        print("Left Blinker turned on")
                    else:
                        self._lights &= ~carla.VehicleLightState.LeftBlinker
                        print("Left Blinker turned off")
                elif event.button == self._B_button_index:
                    self.right_blinker_manual = not self.right_blinker_manual
                    self.blinker_auto_off = False  # Reset flag
                    if self.right_blinker_manual:
                        self._lights |= carla.VehicleLightState.RightBlinker
                        self._lights &= ~carla.VehicleLightState.LeftBlinker  # Ensure the left blinker is off
                        print("Right Blinker turned on")
                    else:
                        self._lights &= ~carla.VehicleLightState.RightBlinker
                        print("Right Blinker turned off")

                light_state = carla.VehicleLightState(self._lights)
                world.player.set_light_state(light_state)
                world.player.apply_control(self._control)

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p:
                        if not self._autopilot_enabled and not sync_mode:
                            print("WARNING: You are currently in asynchronous mode and could "
                                  "experience some issues with the traffic simulation")
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                        
        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                current_speed = world.player.get_velocity().length()
                self._parse_vehicle_wheel()
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
                        # Adjust throttle based on the speed limit
            if current_speed > self._speed_limit_mps:
                # If current speed exceeds the speed limit, reduce throttle
                self._control.throttle = 0.0
            else:
                # If current speed is within the speed limit, use existing throttle control
                self._control.throttle = self.get_throttle_state()    
                
                
            world.player.apply_control(self._control)

    def update_blinker_auto_off(self, steer, world):
        steering_center_threshold = 0.2  # Adjust this value for center threshold
        # steering_off_threshold = 0.04  # Adjust this value for off threshold, should be greater for left and right turns

        # Check if the blinkers are manually turned on, if yes, don't turn them off automatically
        if self.left_blinker_manual or self.right_blinker_manual:
            self.blinker_auto_off = False
        else:
            # Apply automatic blinker control logic
            if -steering_center_threshold < steer < steering_center_threshold:
                # Vehicle is driving straight, turn off both blinkers
                self._lights &= ~carla.VehicleLightState.LeftBlinker
                self._lights &= ~carla.VehicleLightState.RightBlinker
                self.blinker_auto_off = True

        # Apply the updated light state
        light_state = carla.VehicleLightState(self._lights)
        world.player.set_light_state(light_state)
        world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = 1.0
        elif keys[K_b]:  # If 'b' key is pressed
            self._control.brake = min(1.0, self._control.brake + 47)  # Increase braking force by 0.1, up to a maximum of 1.0
        else:
            self._control.brake = 0.0
        self._control.hand_brake = keys[K_SPACE]
        
    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 0.5 # 0.55
        steerCmd = K1 * math.tan(1.5 * jsInputs[self._steer_idx])

        # steerCmd = 0.8 * jsInputs[self._steer_idx]

        K2 = 1.4  # 1.6
        throttleCmd = K2 + (2.5 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        # brakeCmd = 1.6 + (2.05 * math.log10(-0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        brakeCmd = 1.6 + (2.05 * math.log10(-0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        brakeCmd*=20

        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1
        
        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])
        
    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()
    
    def get_throttle_state(self):
        return self._control.throttle

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._ackermann_enabled = False
        self.blinker_auto_off = False  # Flag to reset manual blinker state after auto offa
        self._ackermann_reverse = 1
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._ackermann_control = carla.VehicleAckermannControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0)) 
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key == K_o:
                    try:
                        if world.doors_are_open:
                            world.hud.notification("Closing Doors")
                            world.doors_are_open = False
                            world.player.close_door(carla.VehicleDoor.All)
                        else:
                            world.hud.notification("Opening doors")
                            world.doors_are_open = True
                            world.player.open_door(carla.VehicleDoor.All)
                    except Exception:
                        pass
                elif event.key == K_t:
                    if world.show_vehicle_telemetry:
                        world.player.show_debug_telemetry(False)
                        world.show_vehicle_telemetry = False
                        world.hud.notification("Disabled Vehicle Telemetry")
                    else:
                        try:
                            world.player.show_debug_telemetry(True)
                            world.show_vehicle_telemetry = True
                            world.hud.notification("Enabled Vehicle Telemetry")
                        except Exception:
                            pass
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_f:
                        # Toggle ackermann controller
                        self._ackermann_enabled = not self._ackermann_enabled
                        world.hud.show_ackermann_info(self._ackermann_enabled)
                        world.hud.notification("Ackermann Controller %s" %
                                               ("Enabled" if self._ackermann_enabled else "Disabled"))
                    if event.key == K_q:
                        if not self._ackermann_enabled:
                            self._control.gear = 1 if self._control.reverse else -1
                        else:
                            self._ackermann_reverse *= -1
                            # Reset ackermann control
                            self._ackermann_control = carla.VehicleAckermannControl()
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if not self._autopilot_enabled and not sync_mode:
                            print("WARNING: You are currently in asynchronous mode and could "
                                  "experience some issues with the traffic simulation")
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
                # Apply control
                if not self._ackermann_enabled:
                    world.player.apply_control(self._control)
                else:
                    world.player.apply_ackermann_control(self._ackermann_control)
                    # Update control to the last one applied by the ackermann controller.
                    self._control = world.player.get_control()
                    # Update hud with the newest ackermann control
                    world.hud.update_ackermann_control(self._ackermann_control)

            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
                world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            if not self._ackermann_enabled:
                self._control.throttle = min(self._control.throttle + 0.1, 1.00)
            else:
                self._ackermann_control.speed += round(milliseconds * 0.005, 2) * self._ackermann_reverse
        else:
            if not self._ackermann_enabled:
                self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            if not self._ackermann_enabled:
                self._control.brake = min(self._control.brake + 0.2, 1)
            else:
                self._ackermann_control.speed -= min(abs(self._ackermann_control.speed), round(milliseconds * 0.005, 2)) * self._ackermann_reverse
                self._ackermann_control.speed = max(0, abs(self._ackermann_control.speed)) * self._ackermann_reverse
        else:
            if not self._ackermann_enabled:
                self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        if not self._ackermann_enabled:
            self._control.steer = round(self._steer_cache, 1)
            self._control.hand_brake = keys[K_SPACE]
        else:
            self._ackermann_control.steer = round(self._steer_cache, 1)

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()
        
    def get_throttle_state(self):
        return self._control.throttle  
     
    def update_blinker_auto_off(self):
        pass
    
    def set_manual_blinker_state(self, left_state, right_state):
        self.left_blinker_manual = left_state
        self.right_blinker_manual = right_state

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._font_speed = pygame.font.Font(mono, 80 if os.name == 'nt' else 40)  # New font for speed
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

        self._show_ackermann_info = False
        self._ackermann_control = carla.VehicleAckermannControl()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')

        # Calculate engine RPM
        c = world.player.get_control()
        p = world.player.get_physics_control()

        # Detect gear change
        if hasattr(world.player, 'previous_gear') and c.gear != world.player.previous_gear:
            gear_change_reduction_factor = 0.001  # Adjust this value as needed
        else:
            gear_change_reduction_factor = 1

        world.player.previous_gear = c.gear  # Store current gear for next iteration

        # Introduce throttle response factor
        throttle_response = 0.01  # Adjust this value as needed

        # Set a minimum RPM to simulate engine idling when it's on
        idle_rpm = 1000  # Adjust this value as needed

        # Calculate desired RPM based on throttle and gear
        desired_rpm = p.max_rpm * c.throttle * gear_change_reduction_factor

        # Gradually adjust engine RPM towards desired RPM
        if hasattr(world.player, 'engine_rpm'):
            if c.throttle == 0:  # If throttle is 0, decrease engine RPM more slowly
                engine_rpm = world.player.engine_rpm - (world.player.engine_rpm - idle_rpm) * 0.01
            else:
                engine_rpm = world.player.engine_rpm + (desired_rpm - world.player.engine_rpm) * throttle_response
        else:
            engine_rpm = desired_rpm

        world.player.engine_rpm = engine_rpm  # Store current RPM for next iteration

        if c.throttle != 0 and c.gear > 0 and c.gear < len(p.forward_gears):
            gear = p.forward_gears[c.gear]
            engine_rpm *= gear.ratio

        engine_rpm = max(engine_rpm, idle_rpm)

        # Ensure engine_rpm doesn't exceed 7000
        engine_rpm = min(engine_rpm, 7000)
                   

        self._info_text = [
             'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            # '',
            # 'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            # 'Map:     % 20s' % world.map.name.split('/')[-1],
             'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
             'Throttle: % 12.2f' % c.throttle,
             'Brake: % 12.2f' % c.brake,
            'Engine RPM: % 12.1f' % engine_rpm, 
            # '',
            # u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            # 'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            # 'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            # 'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            # 'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            # 'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)
                ]
            if self._show_ackermann_info:
                self._info_text += [
                    '',
                    'Ackermann Controller:',
                    '  Target speed: % 8.0f km/h' % (3.6*self._ackermann_control.speed),
                ]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
          'Number of vehicles: % 8d' % len(vehicles),
          'Current pos: (% 5.1f, % 5.1f)' % (t.location.x, t.location.y),]

    def show_ackermann_info(self, enabled):
        self._show_ackermann_info = enabled

    def update_ackermann_control(self, ackermann_control):
        self._ackermann_control = ackermann_control

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            #display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    if 'Speed:' in item:  # 
                        surface = self._font_speed.render(item, True, (255, 255, 255))
                    else:
                        surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35)) #35
        bp.set_attribute('vertical_fov', str(20)) #20
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensorF = None
        self.sensorB = None
        self.sensorL = None  # Left rearview mirror sensor
        self.surfaceF = None
        self.surfaceB = None
        self.surfaceL = None  # Left rearview mirror surface
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        Attachment = carla.AttachmentType
        self.needs_respawn = True

        self._camera_transforms = []

        # If the entity is not a pedestrian, set an alternative camera transformation
        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=0.30, y=-0.3, z=1.17), carla.Rotation(pitch=-10)), Attachment.Rigid),  # Front
                (carla.Transform(carla.Location(x=-2.5, y=0, z=1.4), carla.Rotation(yaw=180)), Attachment.Rigid),  # Back
                (carla.Transform(carla.Location(x=0.35, y=-0.9, z=1.14), carla.Rotation(yaw=210)), Attachment.Rigid)  # Left
            ]

        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=-0.3, y=-0.7, z=1.4), carla.Rotation(yaw=90)), Attachment.Rigid)  # Left mirror
            ]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.rgb', cc.Raw, 'Back Camera RGB', {}],
            ['sensor.camera.rgb', cc.Raw, 'Left Mirror Camera RGB', {}]
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('fov') and item[2] != 'Left Mirror Camera RGB':
                    bp.set_attribute('fov', '105')  # Set the FOV to 140 degrees 
                if bp.has_attribute('fov') and item[2] == 'Left Mirror Camera RGB':
                    bp.set_attribute('fov', '90')  # Set the FOV to 90 degrees for the left mirror
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)

    def toggle_camera(self):
        # self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        # self.set_sensor(self.index, notify=False, force_respawn=True)
        pass

    def set_sensor(self, index=None, notify=True, force_respawn=False):
        # index = index % len(self.sensors)
        # needs_respawn = True if self.sensors[0][2] == None or self.sensors[1][2] == None else False
        print(f'needs_respawn: {self.needs_respawn}')
        if self.needs_respawn:
            if self.sensorF is not None:
                self.sensorF.destroy()
                self.surfaceF = None
            self.sensorF = self._parent.get_world().spawn_actor(
                self.sensors[0][-1],
                self._camera_transforms[0][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[0][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_selfF = weakref.ref(self)
            self.sensorF.listen(lambda image: CameraManager._parse_image(weak_selfF, image, 'F'))

            if self.sensorB is not None:
                self.sensorB.destroy()
                self.surfaceB = None
            self.sensorB = self._parent.get_world().spawn_actor(
                self.sensors[1][-1],
                self._camera_transforms[1][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[1][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_selfB = weakref.ref(self)
            self.sensorB.listen(lambda image: CameraManager._parse_image(weak_selfB, image, 'B'))

            if self.sensorL is not None:
                self.sensorL.destroy()
                self.surfaceL = None
            self.sensorL = self._parent.get_world().spawn_actor(
                self.sensors[2][-1],
                self._camera_transforms[2][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[2][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_selfL = weakref.ref(self)
            self.sensorL.listen(lambda image: CameraManager._parse_image(weak_selfL, image, 'L'))

            self.needs_respawn = False

    def next_sensor(self):
        # self.set_sensor(self.index + 1)
        pass

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render_rear_view(self):
        # Change the size of the rear view surface here
        rear_view_surface = pygame.Surface((self.hud.dim[0] // 5, self.hud.dim[1] // 5), pygame.SRCALPHA)
        rear_view_surface.fill((0, 0, 0, 0))  # Fully transparent

        # Draw the black frame first
        frame_rect = rear_view_surface.get_rect()
        self.draw_rounded_rect(rear_view_surface, (0, 0, 0, 255), frame_rect, corner_radius=110)

        if self.surfaceB is not None:
            # Create the inner surface with a margin for the frame
            inner_width = rear_view_surface.get_width() - 20
            inner_height = rear_view_surface.get_height() - 20
            inner_surface = pygame.Surface((inner_width, inner_height), pygame.SRCALPHA)
            inner_surface.fill((0, 0, 0, 0))

            resized_surfaceB = pygame.transform.scale(self.surfaceB, (inner_surface.get_width(), inner_surface.get_height()))
            inner_surface.blit(resized_surfaceB, (0, 0))

            # Create a mask to shape the surface to the mirror's shape
            mask = pygame.Surface(inner_surface.get_size(), pygame.SRCALPHA)
            self.draw_rounded_rect(mask, (255, 255, 255, 255), mask.get_rect(), corner_radius=110)
            
            # Apply the mask to the inner surface
            inner_surface.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)

            # Blit the inner surface onto the rear view surface
            rear_view_surface.blit(inner_surface, (10, 10))
        
        return rear_view_surface

    def draw_rounded_rect(self, surface, color, rect, corner_radius):
        """ Draw a rectangle with rounded corners.
        Args:
            surface: The surface to draw on.
            color: The color of the rectangle.
            rect: A tuple (x, y, width, height) representing the rectangle.
            corner_radius: The radius of the rounded corners.
        """
        x, y, width, height = rect
        pygame.draw.rect(surface, color, (x + corner_radius, y, width - 2 * corner_radius, height))
        pygame.draw.rect(surface, color, (x, y + corner_radius, width, height - 2 * corner_radius))

        pygame.draw.circle(surface, color, (x + corner_radius, y + corner_radius), corner_radius)
        pygame.draw.circle(surface, color, (x + width - corner_radius, y + corner_radius), corner_radius)
        pygame.draw.circle(surface, color, (x + corner_radius, y + height - corner_radius), corner_radius)
        pygame.draw.circle(surface, color, (x + width - corner_radius, y + height - corner_radius), corner_radius)

    def render_left_view(self):
        # Change the size of the left view surface here
        left_view_surface = pygame.Surface((500, 280), pygame.SRCALPHA)
        left_view_surface.fill((0, 0, 0, 0))  # Fully transparent

        # Draw the black frame first
        frame_rect = left_view_surface.get_rect()
        self.draw_rounded_rect(left_view_surface, (0, 0, 0, 255), frame_rect, corner_radius=110)

        if self.surfaceL is not None:
            # Create the inner surface with a margin for the frame
            inner_width = left_view_surface.get_width() - 20
            inner_height = left_view_surface.get_height() - 20
            inner_surface = pygame.Surface((inner_width, inner_height), pygame.SRCALPHA)
            inner_surface.fill((0, 0, 0, 0))

            resized_surfaceL = pygame.transform.scale(self.surfaceL, (inner_surface.get_width(), inner_surface.get_height()))
            inner_surface.blit(resized_surfaceL, (0, 0))

            # Create a mask to shape the surface to the mirror's shape
            mask = pygame.Surface(inner_surface.get_size(), pygame.SRCALPHA)
            self.draw_rounded_rect(mask, (255, 255, 255, 255), mask.get_rect(), corner_radius=110)
            
            # Apply the mask to the inner surface
            inner_surface.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)

            # Blit the inner surface onto the left view surface
            left_view_surface.blit(inner_surface, (10, 10))
        
        return left_view_surface

    def render(self, display):
        if self.surfaceF is not None:
            display.blit(self.surfaceF, (0, 0))
        rear_view_surface = self.render_rear_view()
        left_view_surface = self.render_left_view()
        # Calculate the x position for center alignment
        x_position_rear = (self.hud.dim[0] - rear_view_surface.get_width()) // 2
        # Adjust the position to fit into the left rearview mirror's actual location
        x_position_left = 57  # Adjust these values to fit the left mirror
        y_position_left = 750  # Adjust these values to fit the left mirror
        display.blit(rear_view_surface, (x_position_rear, 0))
        display.blit(left_view_surface, (x_position_left, y_position_left))

    @staticmethod
    def _parse_image(weak_self, image, index):
        self = weak_self()
        if not self:
            return
        if index == 'F':
            image.convert(self.sensors[0][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surfaceF = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        elif index == 'L':
            image.convert(self.sensors[2][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))          
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            array = np.fliplr(array)  # Flip the image horizontally
            self.surfaceL = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        elif index == 'B':
            image.convert(self.sensors[1][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))          
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            array = np.fliplr(array)  # Flip the image horizontally
            
            self.surfaceB = pygame.surfarray.make_surface(array.swapaxes(0, 1))


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


class ImageProcessor:
    def __init__(self):
        self.image_list = []

    def process_image(self, image):
        image_data = np.array(image.raw_data, dtype=np.uint8)
        image_data = image_data.reshape((image.height, image.width, 4))
        image_data = image_data[:, :, :3]
        #image_data = image_data[:, :, ::-1]  # Convert BGR to RGB
        self.image_list.append(image_data)

    def store_images(self, folder_path):
        # Delete the folder if it exists
        if os.path.exists(folder_path):
            shutil.rmtree(folder_path)

        # Create the folder
        os.makedirs(folder_path)

        # Write each image to disk
        for i, image_data in enumerate(self.image_list):
            image_path = os.path.join(folder_path, f'image_{i}.png')
            cv2.imwrite(image_path, image_data)

def attach_camera_to_vehicle(world, vehicle, camera_transform):
    # Create a directory to save the images
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(1920))
    camera_bp.set_attribute('image_size_y', str(1080))
    if camera_bp.has_attribute('fov'):
        camera_bp.set_attribute('fov', str(110))
    
    # Adjust the camera's orientation
    camera_transform.rotation.yaw += 180  # Rotate the camera by 180 degrees
    
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    processor = ImageProcessor()
    camera.listen(lambda image: processor.process_image(image))
    
    return camera, processor

def process_image(image, image_list):
    image_data = np.array(image.raw_data, dtype=np.uint8)
    print(image.raw_data)
    print(image_data.shape)
    image_data = image_data.reshape((image.height, image.width, 4))

    image_data = image_data[:, :, :3]

    image_list.append(image_data)

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def accelerate_actor(actor, acceleration_steps, delay=0.1):
    x_values = np.linspace(-5.0, 5.0, acceleration_steps)  # Values for the sigmoid function
    throttle_values = sigmoid(x_values)  # Apply the sigmoid function to get the throttle values
    for throttle_control in throttle_values:
        control = carla.VehicleControl(throttle=throttle_control, steer=0.0)
        try:
            actor.apply_control(control)
            #time.sleep(delay)  # Add delay between each step
        except Exception as e:
            logging.error(f"Error applying control to actor {actor.id}: {e}")
            return False  # Return False if there was an error
    return True  # Return True if everything went well

def create_traffic(world, traffic_manager, ego_location, offset=100, num_vehicles=10, actor_list=[]):

    for id in actor_list:
        world.get_actor(id).destroy()
        actor_list.remove(id)
        
    excluded_vehicles = ['gazelle.omafiets', 'bh.crossbike', 'diamondback.century', 'mitsubishi.fusorosa', 'vehicle.micro.microlino', 'vehicle.carlamotors.carlacola', 'vehicle.carlamotors.european_hgv', 'vehicle.carlamotors.firetruck', 'vehicle.tesla.cybertruck', 'vehicle.ford.ambulance', 'vehicle.mercedes.sprinter', 'vehicle.volkswagen.t2', 'vehicle.mitsubishi.fusorosa', 'vehicle.harley-davidson.low_rider', 'vehicle.kawasaki.ninja','vehicle.vespa.zx125', 'vehicle.yamaha.yzf', 'vehicle.bh.crossbike', 'vehicle.diamondback.century', 'vehicle.gazelle.omafiets'    ]
    bps = world.get_blueprint_library().filter("vehicle.*")
    bps = [bp for bp in bps if not any(vehicle in bp.id for vehicle in excluded_vehicles)]

    map = world.get_map()
    
    actors = []

    for i in range(num_vehicles):
        waypoint = map.get_waypoint(ego_location).next(offset * (i+1))[0]
        # spawn traffic always in the rightmost lane
        while True: 
            right_lane = waypoint.get_right_lane()
            second_right_lane = right_lane.get_right_lane() if right_lane is not None else None
            if right_lane is None:
                break
            elif second_right_lane is None:
                break
            else:
                waypoint = right_lane
        traffic_transform = waypoint.transform
        traffic_transform.rotation.pitch = 0 
        traffic_transform.location.z += 1.0  # Add a small height offset to avoid collision with the ground
        blueprint = np.random.choice(bps)
        # Set color before spawning actor
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        traffic_vehicle = world.try_spawn_actor(blueprint, traffic_transform)
        actors.append(traffic_vehicle.id) if traffic_vehicle is not None else None and print('Failed to spawn vehicle', i)
        traffic_vehicle.set_autopilot(True, traffic_manager.get_port())  # Enable autopilot after acceleration

    return actors

def create_traffic_behaviour(actor_list, traffic_manager, world, spacing , baseline):
    vehicles_id_list = [v for v in actor_list if v is not None]
    vehicles_list = [(id,world.get_actor(id)) for id in vehicles_id_list]
    
    for vehicle in vehicles_list:
        traffic_manager.auto_lane_change(vehicle[1], False)  # enables or disables automatic lane changes for the vehicle.

    if baseline:
        traffic_manager.global_percentage_speed_difference(-200) # -350 means ~135km/h, -200 means ~90km/h, -250 means ~100km/h
    else:
        traffic_manager.global_percentage_speed_difference(-200)

    traffic_manager.set_global_distance_to_leading_vehicle(spacing) # 150

def draw_arrow(surface, color, position, direction):
    # Define arrow dimensions
    ARROW_WIDTH = 30
    ARROW_HEIGHT = 40

    # Calculate the points of the arrow polygon based on the direction
    x, y = position
    if direction == -1:  # Left arrow
        points = [(x, y), (x + ARROW_HEIGHT, y + ARROW_WIDTH / 2), (x + ARROW_HEIGHT, y - ARROW_WIDTH / 2)]
    elif direction == 1:  # Right arrow
        points = [(x, y), (x - ARROW_HEIGHT, y + ARROW_WIDTH / 2), (x - ARROW_HEIGHT, y - ARROW_WIDTH / 2)]
    else:
        return  # Invalid direction

    pygame.draw.polygon(surface, color, points)

def calculate_waypoint_distance(waypoint1, waypoint2, ahead=True, max_distance=1000):
    distance = 0.0
    current_waypoint = waypoint1
    # Calculate the distance along the waypoints until the second waypoint is reached
    if ahead:
        while current_waypoint.transform.location.distance(waypoint2.transform.location) > 25 and distance < max_distance:
            next_waypoint = current_waypoint.next(20)[0]  # Move to the next waypoint with a step size of 20 m
            distance += current_waypoint.transform.location.distance(next_waypoint.transform.location)
            current_waypoint = next_waypoint
    else:
        while current_waypoint.transform.location.distance(waypoint2.transform.location) > 25 and distance < max_distance:
            next_waypoint = current_waypoint.previous(20)[0]  # Move to the previous waypoint with a step size of 10 m
            distance += current_waypoint.transform.location.distance(next_waypoint.transform.location)
            current_waypoint = next_waypoint
    return distance

def is_location_clear(world, map, npv_list, distance, waypoint):
    for id in npv_list:
        vehicle = world.get_actor(id)
        wp = map.get_waypoint(vehicle.get_location())

        # Check if the vehicle is within the minimum distance
        if min(calculate_waypoint_distance(waypoint, wp, ahead=True, max_distance=distance), calculate_waypoint_distance(waypoint, wp, ahead=False, max_distance=distance)) < distance:
            return False  # If the vehicle is within the minimum distance, location is not clear
    return True

def teleport_vehicle(world, map, actor_id, snpv_list, waypoint, offset=100, right=True, velocity=carla.Vector3D(0, 0, 0), angular_velocity=carla.Vector3D(0, 0, 0)):
    vehicle = world.get_actor(actor_id)
    # spawn traffic always in the rightmost lane
    if right:
        while True: 
            right_lane = waypoint.get_right_lane()
            second_right_lane = right_lane.get_right_lane() if right_lane is not None else None
            if right_lane is None:
                break
            elif second_right_lane is None:
                break
            else:
                waypoint = right_lane
    try:
        print('Trying to teleport vehicle')
        # Check if the spawn position is available
        while not is_location_clear(world, map, snpv_list, offset, waypoint):
            print('Location is not clear. Trying a different location...')
            # Teleport offset m ahead until a clear location is found
            delta = np.random.randint(offset, offset + offset/8)
            waypoint = waypoint.next(delta)[0]

        transform = waypoint.transform
        transform.rotation.pitch = 0
        transform.location.z += 1  # Add a small height offset to avoid collision with the ground
        vehicle.set_transform(transform)
        vehicle.set_target_velocity(velocity)
        vehicle.set_target_angular_velocity(angular_velocity)
        print('Vehicle teleported to', transform.location)  
        world.tick()  # Tick the world to update the vehicle's position

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(f"An error occurred while teleporting: {e}")
        print(exc_type, fname, exc_tb.tb_lineno)

def change_pitch(sound, playback_speed):
    # Check if the sound object is an instance of AudioSegment
    if isinstance(sound, AudioSegment):
        new_sample_rate = int(sound.frame_rate * playback_speed)
        pitched_sound = sound._spawn(sound.raw_data, overrides={'frame_rate': new_sample_rate})
        return pitched_sound.set_frame_rate(44100)
    else:
        print("The sound object is not an instance of AudioSegment. Cannot change pitch.")
        return sound

# Function to play the engine sound continuously with pitch changes
def play_engine_sound(engine_sound, max_rpm, rpm_queue):
    while True:
        if not rpm_queue.empty():
            engine_rpm = rpm_queue.get()
            # Use a linear scale for the playback speed
            playback_speed = (engine_rpm / max_rpm) * 10e6  # Increase the factor for more noticeable changes
            pitched_sound = change_pitch(engine_sound, playback_speed)
            play(pitched_sound)

def calculate_distance(location1, location2):
    dx = location1.x - location2.x
    dy = location1.y - location2.y
    dz = location1.z - location2.z
    return math.sqrt(dx**2 + dy**2 + dz**2)

def game_loop(args, config):

    baseline = args.baseline

    pygame.mixer.init()

    # Load the audio file
    engine_sound = pygame.mixer.Sound(config["audio"]["engine_sound"])

    # Set the initial volume
    engine_sound.set_volume(1)
    
    # Start playing the engine sound in a loop
    engine_channel = engine_sound.play(loops=-1)

    screen_info = pygame.display.Info()
    SCREEN_WIDTH = screen_info.current_w
    SCREEN_HEIGHT = screen_info.current_h
    GREEN_COLOR = (0, 255, 0)
    BLINKER_DISTANCE = 140  # Adjust this value to set the distance between blinkers
    BLINKER_Y_POSITION = SCREEN_HEIGHT - 450  # Move it 100 pixels from the bottom

    # Change the positions of the blinkers on x axis
    LEFT_BLINKER_X_POSITION = SCREEN_WIDTH // 2 - 20
    RIGHT_BLINKER_X_POSITION = SCREEN_WIDTH // 3 + 850
        
    # Change size of the speed gauge (x,y)
    speed_gauge = GaugeWidget(1300, 470, position=(SCREEN_WIDTH -50, 500))
        #  Change size of the RPM gaug
        
    rpm_gauge = RPMGauge(650, 350, position=(SCREEN_WIDTH -50, 700))

    # Load the PNG image
    image = pygame.image.load(config["video"]["bg_dashboard"])
    # Resize the image (width, height)
    image = pygame.transform.scale(image, (1800, 700))

    # Define the warning message and its display condition
    warning_display_distance = 350  # Distance at which the warning should be displayed
    warning_font = pygame.font.Font(None, 80)  # Set font and size for the warning message


    pygame.font.init()
    world = None
    original_settings = None
    spawning_positions = []

    
    try:
        # Launch the Carla server
        print('Launching carla server...')
        launch_carla_server(config["gameplay"]["server"], args.port, render=False)

        # Give the server a moment to start
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        # Load the traffic manager
        tm = client.get_trafficmanager(8000)
        print('Traffic manager loaded ...\n')         
        # Get the world
        sim_world = client.load_world(config["gameplay"]["map"])
        map = sim_world.get_map()
        

        # Get the current settings
        settings = sim_world.get_settings()
    
        # Apply the settings
        sim_world.apply_settings(settings)
        print('World loaded ...\n')

        # Set the synchronous mode
        settings = sim_world.get_settings()
        # settings.synchronous_mode = True
        print('Settings loaded ...\n')

    
        if args.sync:
            if not settings.synchronous_mode:                
                settings.synchronous_mode = True

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)
            settings.fixed_delta_seconds = 0.051 # 0.045. 20 fps 
            sim_world.apply_settings(settings)
            print('Synchronous mode activated ...\n')

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        print('HUD loaded ...\n')
        
        world = World(sim_world, hud, args)
        print('World loaded ...\n')

        # Call restart to initialize the player
        world.restart()
        print('Player initialized ...\n')
        
        if args.wheel:
            controller = DualControl(world, args.autopilot, config["gameplay"]["wheel_path"])  
        else:
            controller = KeyboardControl(world, args.autopilot)

        print('Controller loaded ...\n')
        # Create traffic for each town should be modified. Otherwise it will be the same traffic for all towns
        try:
            vehicles_dist = config["traffic"]["SNPV"]["vehicles_dist"]
            num_vehicles = 1 if baseline else config["traffic"]["SNPV"]["num_vehicles"]
            snpv_list = create_traffic(client.get_world(), tm, world.player.get_transform().location, num_vehicles=num_vehicles, offset=vehicles_dist)
            min_distance_behind = config["traffic"]["SNPV"]["min_distance_behind"]
            max_distance_behind = config["traffic"]["SNPV"]["max_distance_behind"]
            under_bridge_waypoint = map.get_waypoint(carla.Location(x=2234.5, y=123.1, z=0))
            on_bridge_waypoint = map.get_waypoint(carla.Location(x=1402.6, y=1228.3, z=0))
            under_bridge_length = 500
            on_bridge_length = 500
            front_vehicle_speeds = config["traffic"]["SNPV"]["front_vehicle_speeds"]
            front_vehicle_traveled_dist = front_vehicle_speed_change_interval = config["traffic"]["SNPV"]["front_vehicle_traveled_dist"]
            front_vehicle_speed_idx = 0
            print('Traffic created ...\n')

        except Exception as e:
            print(f"Failed to create traffic: {e}")
        create_traffic_behaviour(snpv_list, tm, client.get_world(), vehicles_dist, baseline)
        print('Behaviour created ...\n')

        # Find Trigger Friction Blueprint
        friction_bp = sim_world.get_blueprint_library().find('static.trigger.friction')

        extent = carla.Location(700.0, 700.0, 700.0)

        friction_bp.set_attribute('friction', str(0.0))  # Consider changing this to a non-zero value
        friction_bp.set_attribute('extent_x', str(extent.x))
        friction_bp.set_attribute('extent_y', str(extent.y))
        friction_bp.set_attribute('extent_z', str(extent.z))

        # Spawn Trigger Friction
        transform = carla.Transform()
        transform.location = carla.Location(100.0, 0.0, 0.0)
        sim_world.spawn_actor(friction_bp, transform)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        last_blink_time = time.time()
        blink_interval = 0.5  # Blink interval in seconds
        blink_on = True

        print('Camera attached ...\n')

        # Start to record the log
        client.start_recorder(generate_log_filename() , True)
    
        frame_counter = 0

        # Define the hysteresis thresholds
        steering_center_threshold = 0.03  # Adjust this value for center threshold
        steering_off_threshold = 0.05  # Adjust this value for off threshold

        # Track blinker states
        left_blinker_on = False
        right_blinker_on = False
        
        # Change the position of the speed gauge (x,y)
        speed_gauge.set_position((1220, 930)) # Set the position of the gauge on the screen 1120
        
        # Change the position of the RPM gauge (x,y)
        rpm_gauge.set_position((1050, 1055)) # Set the position of the gauge on the screen 1100

        # Create a hidden surface (double buffer)
        double_buffer = pygame.Surface(display.get_size())

        t_hnpv = time.time()
        hnpv_present = False
        hnpv_back_distance = config["traffic"]["HNPV"]["back_distance"]
        hnpv_front_distance = config["traffic"]["HNPV"]["front_distance"]
        hnpv_tm_speed = config["traffic"]["HNPV"]["speed"]
        hnpv_time = config["traffic"]["HNPV"]["time_int"]
        delta_time_hnpv = config["traffic"]["HNPV"]["delta_time"]
        excluded_vehicles = ['gazelle.omafiets', 'bh.crossbike', 'diamondback.century', 'mitsubishi.fusorosa', 'vehicle.micro.microlino', 'vehicle.carlamotors.carlacola', 'vehicle.carlamotors.european_hgv', 'vehicle.carlamotors.firetruck', 'vehicle.tesla.cybertruck', 'vehicle.ford.ambulance', 'vehicle.mercedes.sprinter', 'vehicle.volkswagen.t2', 'vehicle.mitsubishi.fusorosa', 'vehicle.harley-davidson.low_rider', 'vehicle.kawasaki.ninja','vehicle.vespa.zx125', 'vehicle.yamaha.yzf', 'vehicle.bh.crossbike', 'vehicle.diamondback.century', 'vehicle.gazelle.omafiets'    ]
        hnpv_bps = sim_world.get_blueprint_library().filter("vehicle.*")
        hnpv_bps = [bp for bp in hnpv_bps if not any(vehicle in bp.id for vehicle in excluded_vehicles)]

        total_distance_km = 0  # Initialize odometer
        prev_position = world.player.get_location()  # Initial vehicle position

        while True:  
            #calculate current speed
            if world.player:
                current_speed = math.sqrt(world.player.get_velocity().x**2 + world.player.get_velocity().y**2)
                # Update the speed value in the GaugeWidget object
                current_speed = current_speed * 3.6
                speed_gauge.update_speed(current_speed)
            else:
                print("world.player is None")
                continue

            # Handle traffic
            ego_waypoint = map.get_waypoint(world.player.get_location())
            distance_on_bridge = calculate_waypoint_distance(ego_waypoint, on_bridge_waypoint, ahead=True, max_distance=warning_display_distance)
            distance_under_bridge = calculate_waypoint_distance(ego_waypoint, under_bridge_waypoint, ahead=True, max_distance=warning_display_distance)

            # Check if the player is approaching the teleport area
            if distance_under_bridge < warning_display_distance:
                # Draw the warning message for under bridge on the screen
                warning_text = warning_font.render("You are about to teleport", True, (255, 0, 0))  # Red color for the warning text
                warning_rect = warning_text.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2))
                display.blit(warning_text, warning_rect)

            if distance_on_bridge < warning_display_distance:
                # Draw the warning message for on bridge on the screen
                warning_text = warning_font.render("You are about to teleport", True, (255, 0, 0))  # Red color for the warning text
                warning_rect = warning_text.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2))
                display.blit(warning_text, warning_rect)

            if distance_under_bridge < 100:
                under_bridge_next_waypoint = map.get_waypoint(world.player.get_location()).next(under_bridge_length*.9)[0]
                teleport_vehicle(sim_world, map, world.player.id, snpv_list, under_bridge_next_waypoint, offset=vehicles_dist, right=False)
                print('Vehicle teleported after bridge')

            if distance_on_bridge < 100:
                on_bridge_next_waypoint =  map.get_waypoint(world.player.get_location()).next(under_bridge_length*4.1)[0]
                teleport_vehicle(sim_world, map, world.player.id, snpv_list, on_bridge_next_waypoint, offset=vehicles_dist, right=False)
                print('Vehicle teleported after bridge')

            for actor_id in snpv_list:
                npv = sim_world.get_actor(actor_id)
                npv_waypoint = map.get_waypoint(npv.get_location())
                distance_under_bridge = calculate_waypoint_distance(npv_waypoint, under_bridge_waypoint, ahead=True, max_distance=100)
                if distance_under_bridge < 100:
                    under_bridge_next_waypoint = under_bridge_waypoint.next(under_bridge_length)[0]
                    teleport_vehicle(sim_world, map, actor_id, snpv_list, under_bridge_next_waypoint, offset=vehicles_dist)
                if distance_on_bridge < 100:
                    on_bridge_next_waypoint = on_bridge_waypoint.next(on_bridge_length)[0]
                    teleport_vehicle(sim_world, map, actor_id, snpv_list, on_bridge_next_waypoint, offset=vehicles_dist)

                else:
                    distance_behind = calculate_waypoint_distance(ego_waypoint, npv_waypoint, ahead=False, max_distance=max_distance_behind)
                    if min_distance_behind < distance_behind < max_distance_behind:
                        new_waypoint = ego_waypoint.next(vehicles_dist)[0]
                        teleport_vehicle(sim_world, map, actor_id, snpv_list, new_waypoint, offset=vehicles_dist)

            # Create a hnpv vehicle that spawns on the left lane and has a speed greater than the ego vehicle
            if time.time() - t_hnpv > hnpv_time + random.randint(-delta_time_hnpv,delta_time_hnpv) and not hnpv_present:
                hnpv_present = True
                waypoint = map.get_waypoint(world.player.get_location()).previous(hnpv_back_distance)[0]
                while True:
                    left_lane = waypoint.get_left_lane()
                    second_left_lane = left_lane.get_left_lane() if left_lane is not None else None
                    if left_lane is None:
                        break
                    elif second_left_lane is None:
                        break
                    else:
                        waypoint = left_lane

                hnpv_vehicle_bp = sim_world.get_blueprint_library().find('vehicle.audi.tt')
                # Spawn in the scene
                hnpv_spawn_point = waypoint.transform
                hnpv_spawn_point.location.z += 0.5
                print('Spawning hnpv vehicle at', hnpv_spawn_point, 'while ego is in', world.player.get_transform())
                hnpv_vehicle = sim_world.try_spawn_actor(hnpv_vehicle_bp, hnpv_spawn_point)
                if hnpv_vehicle is None:
                    print('Failed to spawn hnpv vehicle')
                    continue
                # Set the speed of the hnpv vehicle
                sim_world.tick()
                # hnpv_vehicle.set_target_velocity(carla.Vector3D(vx, vy, 0))
                hnpv_vehicle.set_autopilot(True)
                tm.auto_lane_change(hnpv_vehicle, False)
                tm.set_desired_speed(hnpv_vehicle, hnpv_tm_speed)
                tm.ignore_vehicles_percentage(hnpv_vehicle,50)

            # Check if hnpv has overtaken the ego vehicle. If so, destroy it
            if hnpv_present and hnpv_vehicle is not None and hnpv_vehicle.is_alive:
                try:
                    hnpv_location = map.get_waypoint(hnpv_vehicle.get_location())
                    ego_location = map.get_waypoint(world.player.get_location())
                    distance = calculate_waypoint_distance(ego_location, hnpv_location, ahead=True, max_distance=hnpv_front_distance+100)
                    if  hnpv_front_distance <= distance < hnpv_front_distance+100:
                        print('HNPV vehicle has overtaken the ego vehicle and is far enough. Destroying hnpv vehicle...')
                        hnpv_vehicle.destroy()
                        hnpv_present = False
                        t_hnpv = time.time()
                except:
                    print('HNPV vehicle already destroyed')
                    hnpv_present = False
                    t_hnpv = time.time()
            elif hnpv_present and hnpv_vehicle is not None and not hnpv_vehicle.is_alive:
                hnpv_present = False
                t_hnpv = time.time()
            # In baseline mode, if some time has passed, change the speed of the front vehicle
            if baseline and front_vehicle_traveled_dist >= front_vehicle_speed_change_interval:
                front_vehicle = sim_world.get_actor(snpv_list[0])  # IN baseline mode, the first (and only) vehicle is the front vehicle
                traffic_manager.set_desired_speed(front_vehicle, front_vehicle_speeds[front_vehicle_speed_idx])
                front_vehicle_speed_idx = (front_vehicle_speed_idx + 1) % len(front_vehicle_speeds)
                front_vehicle_traveled_dist = 0
                
            state = pygame.key.get_pressed()
            frame_counter += 1

            # Calculate engine RPM
            c = world.player.get_control()
            p = world.player.get_physics_control()

            # Detect gear change
            if hasattr(world.player, 'previous_gear') and c.gear != world.player.previous_gear:
                gear_change_reduction_factor = 0.08  # Adjust this value as needed
            else:
                gear_change_reduction_factor = 1
            
            world.player.previous_gear = c.gear  # Store current gear for next iteration
            
            # Set a minimum RPM to simulate engine idling when it's on
            idle_rpm = 1000  # Adjust this value as needed
            
            # Calculate desired RPM based on throttle and gear
            desired_rpm = p.max_rpm * c.throttle * gear_change_reduction_factor
            
            # Gradually adjust engine RPM towards desired RPM
            rpm_increase_rate = 0.01  # Adjust this value as needed
            rpm_decrease_rate = 0.005  # Adjust this value as needed
            
            if hasattr(world.player, 'engine_rpm'):
                if desired_rpm > world.player.engine_rpm:
                    engine_rpm = world.player.engine_rpm + (desired_rpm - world.player.engine_rpm) * rpm_increase_rate
                else:
                    engine_rpm = world.player.engine_rpm - (world.player.engine_rpm - desired_rpm) * rpm_decrease_rate
            else:
                engine_rpm = desired_rpm
            
            world.player.engine_rpm = engine_rpm  # Store current RPM for next iteration
            
            if c.throttle != 0 and c.gear > 0 and c.gear < len(p.forward_gears):
                gear = p.forward_gears[c.gear]
                engine_rpm *= gear.ratio
            
            engine_rpm = max(engine_rpm, idle_rpm)
            
            # Ensure engine_rpm doesn't exceed 7000
            engine_rpm = min(engine_rpm, 7000)
            
            # Adjust the sound pitch
            # engine_channel.set_volume(0.5) #(playback_speed)
            if engine_rpm > 5000:
                engine_channel.set_volume(1)
            elif engine_rpm > 4750:
                engine_channel.set_volume(0.85)
            elif engine_rpm > 4500:
                engine_channel.set_volume(0.8)
            elif engine_rpm > 4250:
                engine_channel.set_volume(0.75)
            elif engine_rpm > 4000:
                engine_channel.set_volume(0.7)
            elif engine_rpm > 3750:
                engine_channel.set_volume(0.65)
            elif engine_rpm > 3500:
                engine_channel.set_volume(0.6)
            elif engine_rpm > 3250:
                engine_channel.set_volume(0.55)
            elif engine_rpm > 3000:
                engine_channel.set_volume(0.5)
            elif engine_rpm > 2750:
                engine_channel.set_volume(0.45)
            elif engine_rpm > 2500:
                engine_channel.set_volume(0.4)
            elif engine_rpm > 2250:
                engine_channel.set_volume(0.35)
            elif engine_rpm > 2000:
                engine_channel.set_volume(0.3)
            elif engine_rpm > 1750:
                engine_channel.set_volume(0.25)
            elif engine_rpm > 1500:
                engine_channel.set_volume(0.2)
            elif engine_rpm > 1000:
                engine_channel.set_volume(0.1)
            elif engine_rpm > 500:
                engine_channel.set_volume(0.05)

            rpm_queue = queue.Queue()
            rpm_queue.put(engine_rpm)

            # Update the RPM value in the RPMGauge object
            rpm_gauge.update_rpm(engine_rpm)

            # Get all traffic lights and set them to green
            traffic_lights = [actor for actor in world.world.get_actors() if 'traffic_light' in actor.type_id]
            for traffic_light in traffic_lights:
                traffic_light.set_state(carla.TrafficLightState.Green)
                
            if state[pygame.K_SPACE]:
                print(world.player.get_transform())
                spawning_positions.append((world.player.get_transform().location.x,
                                          world.player.get_transform().location.y,
                                          world.player.get_transform().location.z,
                                          world.player.get_transform().rotation.pitch,
                                          world.player.get_transform().rotation.yaw,
                                          world.player.get_transform().rotation.roll))
                
            if state[pygame.K_r]:
                print('Restarting the player position')
                nearest_waypoint = map.get_waypoint(world.player.get_location())
                teleport_vehicle(sim_world, map, world.player.id, snpv_list, nearest_waypoint, offset=vehicles_dist, right=False)

            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
    
            if args.wheel:
                if controller.parse_events(world, clock, args.sync):
                    break
            else:
                if controller.parse_events(client, world, clock, args.sync):
                    break

            world.tick(clock)
            world.render(double_buffer)

            # Chang the position of the image on the screen (x,y)
            double_buffer.blit(image, (792, 800))

            # Render the gauges on the double buffer
            speed_gauge.screen = double_buffer
            speed_gauge.render()

            rpm_gauge.screen = double_buffer
            rpm_gauge.render()

            # Blit the double buffer onto the display
            display.blit(double_buffer, (0, 0))
            
            # Get the light state of the player vehicle
            light_state = world.player.get_light_state()
            current_time = time.time()
            if current_time - last_blink_time > blink_interval:
                blink_on = not blink_on
                last_blink_time = current_time

            if blink_on:
                if light_state & carla.VehicleLightState.LeftBlinker:
                    draw_arrow(display, GREEN_COLOR, (LEFT_BLINKER_X_POSITION - BLINKER_DISTANCE, BLINKER_Y_POSITION), direction=-1)
                if light_state & carla.VehicleLightState.RightBlinker:
                    draw_arrow(display, GREEN_COLOR, (RIGHT_BLINKER_X_POSITION + BLINKER_DISTANCE, BLINKER_Y_POSITION), direction=1)

            # Automatically manage blinker states with hysteresis logic
            steer = round(world.player.get_control().steer, 2)
            # controller.update_blinker_auto_off(steer, world)

            # Calculate the distance traveled
            current_position = world.player.get_location()
            distance_traveled = calculate_distance(current_position, prev_position)
            front_vehicle_traveled_dist += distance_traveled # Update the distance traveled by the front vehicle
            total_distance_km += distance_traveled / 1000  # Convert meters to kilometers
            prev_position = current_position  # Update for next frame

            # Display odometer
            if baseline:
                odometer_text = f'{total_distance_km:.2f} / 15 km'
            else:
                odometer_text = f'{total_distance_km:.2f} / 32 km'    
            font = pygame.font.Font(None, 36)  # Initialize font
            odometer_surface = font.render(odometer_text, True, (255, 255, 255))  # Render text
            display.blit(odometer_surface, (900, 1250))  # Display on the screen at the top-left corner

            """
            # Then, handle automatic blinker turning off
            if controller.blinker_auto_off:
                world.player.set_light_state(carla.VehicleLightState.NONE)
                left_blinker_on = False
                right_blinker_on = False
                controller.set_manual_blinker_state(False, False)  # Reset manual state
                blink_on = False  # Reset blink_on variable
                last_blink_time = time.time()  # Reset last_blink_time

            if -steering_center_threshold < steer < steering_center_threshold:
                if left_blinker_on or right_blinker_on:
                    world.player.set_light_state(carla.VehicleLightState.NONE)
                    left_blinker_on = False
                    right_blinker_on = False
                    controller.set_manual_blinker_state(False, False)  # Reset manual state
                    blink_on = False  # Reset blink_on variable
                    last_blink_time = time.time()  # Reset last_blink_time
            elif steer <= -steering_off_threshold:
                if not left_blinker_on:  # Turn on left blinker if it's not already on
                    world.player.set_light_state(carla.VehicleLightState.LeftBlinker)
                    left_blinker_on = True
                    right_blinker_on = False  # Ensure right blinker is off
                if right_blinker_on:  # If right blinker is on, turn it off
                    world.player.set_light_state(carla.VehicleLightState.NONE)
                    right_blinker_on = False
            elif steer >= steering_off_threshold:
                if not right_blinker_on:  # Turn on right blinker if it's not already on
                    world.player.set_light_state(carla.VehicleLightState.RightBlinker)
                    right_blinker_on = True
                    left_blinker_on = False  # Ensure left blinker is off
                if left_blinker_on:  # If left blinker is on, turn it off
                    world.player.set_light_state(carla.VehicleLightState.NONE)
                    left_blinker_on = False
            """
            
    except Exception as e:
        exc_type, _, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(f"An error occurred: {e}")
        print(exc_type, fname, exc_tb.tb_lineno)

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()
        
        if world is not None:
            world.destroy()
        
        client.stop_recorder()
        kill_old_instances(args.port, stop_execution=True)
        pygame.quit()
        
def launch_carla_server(carla_path, port, render=False):

    if platform.system() == 'Windows':
        #cmd = [r'.\0705map\WindowsNoEditor\CarlaUE4.exe', "-benchmark", f"-carla-rpc-port={port}"]
        cmd = [carla_path, "-benchmark", f"-carla-rpc-port={port}"]
    else:
        #platform.system() == 'Linux':  # Changed from 'Linux' to 'Windows'
        cmd = [carla_path, "-benchmark", f"-carla-rpc-port={port}"]
        
    if not render:
        cmd.append("-RenderOffScreen")

    subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(3) if not render else time.sleep(5)
        
        
def kill_old_instances(port, stop_execution=False):
    
    PROCNAMES = ['CarlaUE4-Win64-Shipping.exe', 'CARLA UE4', 'CarlaUE4-Linux-Shipping']  # Added 'CARLA UE4'
    processes = [proc for proc in psutil.process_iter() if any(procname in proc.name() for procname in PROCNAMES)]
    
    for proc in processes:
        for c in proc.connections():
            if c.laddr.port == port and c.status == 'LISTEN':
                proc.kill()
                if stop_execution:
                    exit()
                return 
            
    print('No old instances found')
    
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    
    # Load config file
    with open('config.json') as config_file:
        config = json.load(config_file)
    
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
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
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='3440x1440', 
        help='window resolution (default: 3440x1440)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution',
        default=True)
    argparser.add_argument(
        '--wheel',
        default=False,
        help='Activate steering wheel controller simulation',
    )
    argparser.add_argument(
        '--record',
        action='store_true',
        default=True,
        help='Enable recording frames of the run'
    )
    argparser.add_argument(
        '--savevideo',
        action='store_true',
        default=False,
        help='Enable saving video of the run'
    )
    argparser.add_argument(
        '--baseline',
        action='store_true',
        default=config.get('gameplay', {}).get('baseline_mode', True),
        help='Enable baseline mode. If false, it will be treatment mode'
    )
    
    argparser.add_argument(
        '--treatment',
        action='store_false',
        dest='baseline',
        help='Disable baseline mode. Overrides --baseline if both are provided'
    )
    
    args = argparser.parse_args()

    # Kill old instances of the CARLA server
    print(f'Killing active CARLA server on {args.port}')
    kill_old_instances(args.port)

    args.width, args.height = [int(x) for x in args.res.split('x')]
    x, y = 0, 0
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x,y)
    pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('Listening to CARLA server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args, config)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()