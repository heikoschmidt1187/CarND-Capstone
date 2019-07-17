import rospy

from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,  accel_limit,
                wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # hyperparameters for throttle PID controller
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.      # minimum throttle value
        mx = 0.2    # maximum throttle value
        self.throttle_pid = PID(kp, ki, kd, mn, mx)

        # hyperparameters for velocity low pass filter
        tau = .5    # cutoff frequency (1 / 2 * pi * tau))
        ts = .02    # Sample time
        self.velocity_lowpass = LowPassFilter(tau, ts)

        # steerin controller based on the givven YawController class
        self.steering_controller = YawController(wheel_base,
                steer_ratio,
                0.1,    # TODO: what is a good value for min speed?
                max_lat_accel,
                max_steer_angle)

        # member to holt last call timestamp
        self.last_timestamp = rospy.get_time()

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

        # TODO: pid to control steering, throttle and maybe brake

        # Return throttle, brake, steer
        if dbw_enabled and (target_lin_vel is not None) and (target_ang_vel is not None) and (current_lin_vel is not None) and (current_ang_vel is not None):

            # init control values before calculation
            throttle = 0.
            brake = 0.
            steering = 0.

            # lowpassfilter on velocity to reduce noise
            filtered_velocity = self.velocity_lowpass.filt(current_lin_vel)

            # calculate sample time to use in PID controller
            sample_time = rospy.get_time() - self.last_timestamp

            # calculate CTE for PID controller
            cte = target_lin_vel - filtered_velocity

            # get throttle from PID controlelr
            throttle = self.throttle_pid.step(cte, sample_time)

            # save current timestamp for next round
            last_timestamp = rospy.get_time();

            # calculate brake torque
            if (target_lin_vel < .01) and (filtered_velocity < .1):
                # car is standing still, avoid throttle and brake with enough force to hold car
                throttle = 0.
                brake = 700.    # Nm according to Udacity project information

            if (throttle < .1) and ((target_lin_vel - filtered_velocity) < 0):
                # we are driving to fast and need to brake
                throttle = 0.

                # calculate deceleration with respect to the limit parameter for comfort
                # first use a linear decrease proportional to the current error
                decel = min(self.decel_limit, abs(cte / sample_time))

                # calculate the brake torque through decelleration, mass and wheel physics
                # TODO: maybe use better mass calculation?
                brake = abs(decel) * self.vehicle_mass * self.wheel_radius

                # if brake force is below a weak value, cancel braking
                if brake < .1:
                    brake = 0.

            steering = self.steering_controller.get_steering(target_lin_vel,
                    target_ang_vel,
                    filtered_velocity)
        else:
            # ensure to reset controller to avoid accumulating error
            self.throttle_pid.reset();

            throttle = 0.
            brake = 0.
            steering = 0.

        return throttle, brake, steering
