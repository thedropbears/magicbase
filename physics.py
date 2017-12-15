import math
from pyswervedrive.swervemodule import SwerveModule
from robot import Robot


class PhysicsEngine:

    X_WHEELBASE = 0.6
    Y_WHEELBASE = 0.6

    def __init__(self, controller):
        self.controller = controller

        self.drive_counts_per_rev = \
            SwerveModule.CIMCODER_COUNTS_PER_REV*SwerveModule.DRIVE_ENCODER_GEAR_REDUCTION
        self.drive_counts_per_meter = \
            self.drive_counts_per_rev / (math.pi * SwerveModule.WHEEL_DIAMETER)

        # factor by which to scale velocities in m/s to give to our drive talon.
        # 0.1 is because SRX velocities are measured in ticks/100ms
        self.drive_velocity_to_native_units = self.drive_counts_per_meter*0.1

        # for modules [a, b, c, d]. used to iterate over them
        self.module_steer_can_ids = [8, 2, 4, 11]
        self.module_drive_can_ids = [13, 9, 14, 6]
        self.module_steer_offsets = [0.401, 0.249, 0.797, 0.079]

        self.controller.add_device_gyro_channel('bno055')

    def initialize(self, hal_data):
        pass

    def update_sim(self, hal_data, now, tm_diff):
        """
        Update pyfrc simulator.
        :param hal_data: Data about motors and other components
        :param now: Current time in ms
        :param tm_diff: Difference between current time and time when last checked
        """

        motors_percentvbus = []
        for can_id in self.module_drive_can_ids:
            # max speed of motor in ticks/sec (native vel is ticks/100ms)
            max_speed = Robot.module_drive_free_speed*10
            # divide value by 1023 as it gets multiplied by 1023 in talon code
            percentvbus = hal_data['CAN'][can_id]['value']/1023
            # multiply by dt as we are calculating encoder delta
            hal_data['CAN'][can_id]['enc_position'] += tm_diff*max_speed*percentvbus
            motors_percentvbus.append(percentvbus)

        lf_motor, lr_motor, rr_motor, rf_motor = motors_percentvbus

        steer_positions = []
        for can_id, offset in zip(self.module_steer_can_ids, self.module_steer_offsets):
            value = hal_data['CAN'][can_id]['value']
            hal_data['CAN'][can_id]['enc_position'] = value/4096
            position = (hal_data['CAN'][can_id]['enc_position']-offset) / SwerveModule.STEER_COUNTS_PER_RADIAN
            position_degrees = math.degrees(-position) % 360
            steer_positions.append(position_degrees)

        lf_angle, lr_angle, rr_angle, rf_angle = steer_positions
        x_wheelbase = self.X_WHEELBASE/0.3048  # convert m to feet
        y_wheelbase = self.Y_WHEELBASE/0.3048  # convert m to feet
        vx, vy, vw = \
            four_motor_swerve_drivetrain(lr_motor, rr_motor, lf_motor, rf_motor,
                                         lr_angle, rr_angle, lf_angle, rf_angle,
                                         x_wheelbase=x_wheelbase, y_wheelbase=y_wheelbase,
                                         speed=9)
        self.controller.vector_drive(vx, vy, vw, tm_diff)
        print("vx %s, vy %s, vw %s" % (vx, vy, vw))


def four_motor_swerve_drivetrain(lr_motor, rr_motor, lf_motor, rf_motor, lr_angle, rr_angle, lf_angle, rf_angle,
                                 x_wheelbase=2, y_wheelbase=2, speed=5):
    '''
        Four motors that can be rotated in any direction

        If any motors are inverted, then you will need to multiply that motor's
        value by -1.

        :param lr_motor:   Left rear motor value (-1 to 1); 1 is forward
        :param rr_motor:   Right rear motor value (-1 to 1); 1 is forward
        :param lf_motor:   Left front motor value (-1 to 1); 1 is forward
        :param rf_motor:   Right front motor value (-1 to 1); 1 is forward

        :param lr_angle:   Left rear motor angle in degrees (0 to 360 measured clockwise from forward position)
        :param rr_angle:   Right rear motor angle in degrees (0 to 360 measured clockwise from forward position)
        :param lf_angle:   Left front motor angle in degrees (0 to 360 measured clockwise from forward position)
        :param rf_angle:   Right front motor angle in degrees (0 to 360 measured clockwise from forward position)

        :param x_wheelbase: The distance in feet between right and left wheels.
        :param y_wheelbase: The distance in feet between forward and rear wheels.
        :param speed:      Speed of robot in feet per second (see above)

        :returns: Speed of robot in x (ft/s), Speed of robot in y (ft/s),
                  clockwise rotation of robot (radians/s)
    '''
    # Calculate speed of each wheel
    lr = lr_motor * speed
    rr = rr_motor * speed
    lf = lf_motor * speed
    rf = rf_motor * speed

    # Calculate angle in radians
    lr_rad = math.radians(lr_angle)
    rr_rad = math.radians(rr_angle)
    lf_rad = math.radians(lf_angle)
    rf_rad = math.radians(rf_angle)

    # Calculate wheelbase radius
    wheelbase_radius = math.hypot(x_wheelbase / 2, y_wheelbase / 2)

    # Calculates the Vx and Vy components
    # Sin an Cos inverted because forward is 0 on swerve wheels
    Vx = (math.sin(lr_rad) * lr) + (math.sin(rr_rad) * rr) + (math.sin(lf_rad) * lf) + (math.sin(rf_rad) * rf)
    Vy = (math.cos(lr_rad) * lr) + (math.cos(rr_rad) * rr) + (math.cos(lf_rad) * lf) + (math.cos(rf_rad) * rf)

    # Adjusts the angle corresponding to a diameter that is perpendicular to the radius (add or subtract 45deg)
    lr_rad = (lr_rad + (math.pi / 4)) % (2 * math.pi)
    rr_rad = (rr_rad - (math.pi / 4)) % (2 * math.pi)
    lf_rad = (lf_rad - (math.pi / 4)) % (2 * math.pi)
    rf_rad = (rf_rad + (math.pi / 4)) % (2 * math.pi)

    # Finds the rotational velocity by finding the torque and adding them up
    Vw = wheelbase_radius * (
    (math.cos(lr_rad) * lr) + (math.cos(rr_rad) * -rr) + (math.cos(lf_rad) * lf) + (math.cos(rf_rad) * -rf))

    Vx *= 0.25
    Vy *= 0.25
    Vw *= 0.25


    return Vx, Vy, Vw