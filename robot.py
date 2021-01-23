import wpilib
import wpilib.drive
import ctre

print('Successfully ran robotpy')

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        gyroChannel = 0  # analog input

        self.left = wpilib.Spark(0)
        self.right = wpilib.Spark(1)

        self.gyro = ctre.PigeonIMU(gyroChannel)
    
        self.myRobot = wpilib.drive.DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)
    
        self.stick = wpilib.XboxController(0)

    def teleopInit(self):
        """
        Runs at the beginning of the teleop period
        """
        print(self.gyro.getFusedHeading())
        self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """
        Sets the gyro sensitivity and drives the robot when the joystick is pushed. The
        motor speed is set from the joystick while the RobotDrive turning value is assigned
        from the error between the setpoint and the gyro angle.
        """
        # Multiplying by 0.5 to reduce speed
        self.myRobot.arcadeDrive(
            self.stick.getX(self.stick.Hand.kLeftHand) * 0.5, self.stick.getY(self.stick.Hand.kLeftHand) * 0.5, True
        )

if __name__ == "__main__":
    wpilib.run(MyRobot)