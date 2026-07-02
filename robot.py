import wpilib
import wpilib.drive as drive


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        print("INFO: Robot initiation sequence")

        self.left_front_drive: wpilib.PWMSparkMax = wpilib.PWMSparkMax(0)
        self.right_front_drive: wpilib.PWMSparkMax = wpilib.PWMSparkMax(1)
        self.left_rear_drive: wpilib.PWMSparkMax = wpilib.PWMSparkMax(2)
        self.right_rear_drive: wpilib.PWMSparkMax = wpilib.PWMSparkMax(3)

        self.robot_drive: drive.MecanumDrive = drive.MecanumDrive(
            self.left_front_drive,
            self.left_rear_drive,
            self.right_front_drive,
            self.right_rear_drive,
        )

        self.controller: wpilib.Joystick = wpilib.Joystick(0)

        self.timer: wpilib.Timer = wpilib.Timer()

        self.i: int = 0

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        forwardSpeed: float = -self.controller.getRawAxis(1)
        strafeSpeed: float = self.controller.getRawAxis(0)
        turnSpeed: float = self.controller.getRawAxis(2)

        if not self.i:
            print(f"{forwardSpeed=}\n{strafeSpeed=}\n{turnSpeed}")
            self.i = 10

        else:
            self.i -= 1

        self.robot_drive.driveCartesian(forwardSpeed, strafeSpeed, turnSpeed)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def disabledPeriodic(self):
        pass

    def robotPeriodic(self):
        # print("INFO: Robot periodic sequence")
        pass

    def simulationPeriodic(self):
        print("INFO: Robot simulation sequence")
