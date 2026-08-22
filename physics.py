from wpilib.simulation import SimDeviceSim, AnalogGyroSim
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains

from src.subsystems.drive.drive_train_constants import FRONT_LEFT_ID, FRONT_RIGHT_ID, REAR_LEFT_ID, REAR_RIGHT_ID, MAX_ANGULAR_SPEED, WHEEL_CIRCUMFERENCE, WHEEL_GEAR_RATIO


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller
        self.robot = robot
        
        self.kinematics = MecanumDriveKinematics(
        
        sparkmax_str = "SPARK MAX [{}]"
        self.front_left_sim = SimDeviceSim(sparkmax_str.format(FRONT_LEFT_ID))
        self.front_right_sim = SimDeviceSim(sparkmax_str.format(FRONT_RIGHT_ID))
        self.rear_left_sim = SimDeviceSim(sparkmax_str.format(REAR_LEFT_ID))
        self.rear_right_sim = SimDeviceSim(sparkmax_str.format(REAR_RIGHT_ID))
        
        self.front_left_output = self.front_left_sim.getDouble("Applied Output")
        self.front_right_output = self.front_right_sim.getDouble("Applied Output")
        self.rear_left_output = self.rear_left_sim.getDouble("Applied Output")
        self.rear_right_output = self.rear_right_sim.getDouble("Applied Output")
        
        self.gyro_sim = AnalogGyroSim(0)
    
    def update_sim(self, now: float, tm_diff: float):
        # Get wheel speeds in radians per second
        fl_radps = self.front_left_output.get() * MAX_ANGULAR_SPEED
        fr_radps = self.front_right_output.get() * MAX_ANGULAR_SPEED
        rl_radps = self.rear_left_output.get() * MAX_ANGULAR_SPEED
        rr_radps = self.rear_right_output.get() * MAX_ANGULAR_SPEED
        
        # Set velocity (rpm)
        radps_to_rpm = 60 / pi
        self.front_left_sim.setVelocity(fl_radps * radps_to_rpm)
        self.front_right_sim.setVelocity(fr_radps * radps_to_rpm)
        self.rear_left_sim.setVelocity(rl_radps * radps_to_rpm)
        self.rear_right_sim.setVelocity(rr_radps * radps_to_rpm)
        
        # Advance position (rotations per simualtion frame)
        radps_to_rpf = tm_diff / pi
        self.front_left_sim.addPosition(fl_radps * radps_to_rpf)
        self.front_right_sim.addPosition(fr_radps * radps_to_rpf)
        self.rear_left_sim.addPosition(rl_radps * radps_to_rpf)
        self.rear_right_sim.addPosition(rr_radps * radps_to_rpf)
        
        # Compute wheel speeds (m/s), chassis speeds and drive simulation
        radps_to_mps = (1/pi) * WHEEL_GEAR_RATIO * WHEEL_CIRCUMFERENCE
        wheel_speeds = MecanumDriveWheelSpeeds(
            fl_radps * radps_to_mps,
            fr_speed * radps_to_mps,
            rl_speed * radps_to_mps,
            rr_speed * radps_to_mps
        )
        chassis_speeds = self.robot.robot_container.drive.kinematics.toChassisSpeeds(wheel_speeds)
        self.physics_controller.drive(chassis_speeds, tm_diff)
        
        # Advance simulated gyro heading
        self.gyro_sim.setAngle(self.gyro_sim.getAngle() + chassis_speeds.omega * tm_diff)
