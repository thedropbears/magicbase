#!/usr/bin/env python3
import magicbot
import wpilib

from pyswervedrive.swervechassis import SwerveChassis
from pyswervedrive.swervemodule import SwerveModule, SwerveModuleConfig
from utilities.bno055 import BNO055

from networktables import NetworkTable

import logging

import math


class Robot(magicbot.MagicRobot):

    module_a: SwerveModule
    module_a_cfg = SwerveModuleConfig(
        steer_talon_id=8, drive_talon_id=13, steer_enc_offset=0.401,
        reverse_steer_direction=True, reverse_steer_encoder=True,
        reverse_drive_direction=False, reverse_drive_encoder=False,
        drive_enc_gear_reduction=5.43956, wheel_diameter_meters=0.0254 * 3,
        drive_motor_free_speed=700,
        x_pos=0.3, y_pos=0.3)
    module_b: SwerveModule
    module_b_cfg = SwerveModuleConfig(
        steer_talon_id=2, drive_talon_id=9, steer_enc_offset=0.249,
        reverse_steer_direction=True, reverse_steer_encoder=True,
        reverse_drive_direction=False, reverse_drive_encoder=False,
        drive_enc_gear_reduction=5.43956, wheel_diameter_meters=0.0254 * 3,
        drive_motor_free_speed=700,
        x_pos=-0.3, y_pos=0.3)
    module_c: SwerveModule
    module_c_cfg = SwerveModuleConfig(
        steer_talon_id=4, drive_talon_id=14, steer_enc_offset=0.797,
        reverse_steer_direction=True, reverse_steer_encoder=True,
        reverse_drive_direction=False, reverse_drive_encoder=False,
        drive_enc_gear_reduction=5.43956, wheel_diameter_meters=0.0254 * 3,
        drive_motor_free_speed=700,
        x_pos=-0.3, y_pos=-0.3)
    module_d: SwerveModule
    module_d_cfg = SwerveModuleConfig(
        steer_talon_id=11, drive_talon_id=6, steer_enc_offset=0.079,
        reverse_steer_direction=True, reverse_steer_encoder=True,
        reverse_drive_direction=False, reverse_drive_encoder=False,
        drive_enc_gear_reduction=5.43956, wheel_diameter_meters=0.0254 * 3,
        drive_motor_free_speed=700,
        x_pos=0.3, y_pos=-0.3)

    chassis: SwerveChassis

    def createObjects(self):
        '''Create motors and stuff here'''

        # Objects that are created here are shared with all classes
        # that declare them. For example, if I had:
        # self.elevator_motor = wpilib.TalonSRX(2)
        # here, then I could have
        # class Elevator:
        #     elevator_motor = wpilib.TalonSRX
        # and that variable would be available to both the MyRobot
        # class and the Elevator class. This "variable injection"
        # is especially useful if you want to certain objects with
        # multiple different classes.

        # create the imu object
        self.bno055 = BNO055()

        # the "logger" - allows you to print to the logging screen
        # of the control computer
        self.logger = logging.getLogger("robot")
        # the SmartDashboard network table allows you to send
        # information to a html dashboard. useful for data display
        # for drivers, and also for plotting variables over time
        # while debugging
        self.sd = NetworkTable.getTable('SmartDashboard')

        # boilerplate setup for the joystick
        self.joystick = wpilib.Joystick(0)
        self.pressed_buttons_js = set()
        self.pressed_buttons_gp = set()
        self.spin_rate = 5

    def putData(self):
        # update the data on the smart dashboard
        # put the inputs to the dashboard
        """self.sd.putNumber("i_x", self.chassis.inputs[0])
        self.sd.putNumber("i_y", self.chassis.inputs[1])
        self.sd.putNumber("i_z", self.chassis.inputs[2])
        self.sd.putNumber("i_t", self.chassis.inputs[3])
        self.sd.putNumber("heading", self.bno055.getHeading())"""

    def teleopInit(self):
        '''Called when teleop starts; optional'''
        self.bno055.resetHeading()

    def teleopPeriodic(self):
        '''Called on each iteration of the control loop'''
        self.putData()

        # if you want to get access to the buttons, you should be doing it like so:
        try:
            if self.debounce(1):
                # perform some action
                pass
        except:
            self.onException()

        # this is where the joystick inputs get converted to numbers that are sent
        # to the chassis component. we rescale them using the rescale_js function,
        # in order to make their response exponential, and to set a dead zone -
        # which just means if it is under a certain value a 0 will be sent
        # TODO: Tune these constants forvwhatever robot they are on
        vx = -rescale_js(self.joystick.getY(), deadzone=0.05, exponential=1.2, rate=4)
        vy = -rescale_js(self.joystick.getX(), deadzone=0.05, exponential=1.2, rate=4)
        vz = -rescale_js(self.joystick.getZ(), deadzone=0.2, exponential=15.0, rate=self.spin_rate)
        self.chassis.set_inputs(vx, vy, vz)
        # x = throttle*-rescale_js(self.joystick.getY(), deadzone=0.05, exponential=1.2, rate=4)
        # y = throttle*-rescale_js(self.joystick.getX(), deadzone=0.05, exponential=1.2, rate=4)
        # z = throttle*-rescale_js(self.joystick.getZ(), deadzone=0.2, exponential=15.0, rate=self.spin_rate)
        # self.module_b.set_velocity(x, y)

    # the 'debounce' function keeps tracks of which buttons have been pressed
    def debounce(self, button, gamepad=False):
        device = None
        if gamepad:
            pressed_buttons = self.pressed_buttons_gp
            device = self.gamepad
        else:
            pressed_buttons = self.pressed_buttons_js
            device = self.joystick
        if device.getRawButton(button):
            if button in pressed_buttons:
                return False
            else:
                pressed_buttons.add(button)
                return True
        else:
            pressed_buttons.discard(button)
            return False


# see comment in teleopPeriodic for information
def rescale_js(value, deadzone=0.0, exponential=0.0, rate=1.0):
    value_negative = 1.0
    if value < 0:
        value_negative = -1.0
        value = -value
    # Cap to be +/-1
    if abs(value) > 1.0:
        value /= abs(value)
    # Apply deadzone
    if abs(value) < deadzone:
        return 0.0
    elif exponential == 0.0:
        value = (value - deadzone) / (1 - deadzone)
    else:
        a = math.log(exponential + 1) / (1 - deadzone)
        value = (math.exp(a * (value - deadzone)) - 1) / exponential
    return value * value_negative * rate


if __name__ == '__main__':
    wpilib.run(Robot)
