from lowpass import LowPassFilter
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,  accel_limit,
                wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0      # minimum throttle value
        mx = 0.2    # maximum throttle value
        self.throttle_pid = PID(kp, ki, kd, mn, mx)

        tau = .5    # cutoff frequency (1 / 2 * pi * tau))
        ts = .02    # Sample time
        self.velocity_lowpass = LowPassFilter(tau, ts)

        # vehicle parameters for calculation
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

    def control(self, target_lin_vel, target_ang_vel, current_lin_vel, current_ang_vel, dbw_enabled):
        # TODO: lowpass for current velocity to reduce noise
        # TODO: pid to control steering, throttle and maybe brake

        # Return throttle, brake, steer
        if dbw_enabled:
            return 1., 0., 0.
        else:
            return 0., 0., 0.
