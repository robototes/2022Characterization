/**
* This is a very simple robot program that can be used to send telemetry to
* the data_logger script to characterize your drivetrain. If you wish to use
* your actual robot code, you only need to implement the simple logic in the
* autonomousPeriodic function and change the NetworkTables update rate
*/

package dc;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
// WPI_Talon* imports are needed in case a user has a Pigeon on a Talon
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public class Robot extends TimedRobot {

  // TODO: change these because i forgor 💀 what the real values are
  static private double ENCODER_EDGES_PER_REV = 2048;
  static private int PIDIDX = 0;
  static private int ENCODER_EPR = 2048;
  static private double GEARING = 1/ ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));

  private double encoderConstant = (1 / GEARING) * (1 / ENCODER_EDGES_PER_REV);

  Joystick stick;
  DifferentialDrive drive;

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;

  WPI_TalonFX[] steerMotors = { new WPI_TalonFX(2), new WPI_TalonFX(5), new WPI_TalonFX(8), new WPI_TalonFX(11) };

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  public static final double[] ENCODER_OFFSETS = {-Math.toRadians(67.852), -Math.toRadians(221.924), -Math.toRadians(214.980), -Math.toRadians(168.398)};
  public static final double STEER_RATIO = (15.0 / 32.0) * (10.0 / 60.0);

  public static final boolean IS_COMPETITION = true;
  private static final boolean USE_MODULES = true;
  private static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 1, DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 4,
          DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 7, DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 10;
  private static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 2, DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 5,
          DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 8, DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 11;
  private static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 3, DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 6,
          DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 9, DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 12;
  private static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(67.852);
  private static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(221.924);
  private static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(214.980);
  private static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(168.398);

  // Changes swerve modules to correct gearing
  private static final Mk4SwerveModuleHelper.GearRatio GEAR_RATIO = IS_COMPETITION ? Mk4SwerveModuleHelper.GearRatio.L2
            : Mk4SwerveModuleHelper.GearRatio.L1;

  private static final Mk4Configuration FRONT_LEFT_CONFIG = new Mk4Configuration(
          GEAR_RATIO,
          DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
          DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
          DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
          DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET,
          "DrivebaseIntake");
  private static final Mk4Configuration FRONT_RIGHT_CONFIG = new Mk4Configuration(
          GEAR_RATIO,
          DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
          DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
          DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
          DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET,
          "DrivebaseIntake");
  private static final Mk4Configuration BACK_LEFT_CONFIG = new Mk4Configuration(
          GEAR_RATIO,
          DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
          DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
          DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
          DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET,
          "DrivebaseIntake");
  private static final Mk4Configuration BACK_RIGHT_CONFIG = new Mk4Configuration(
          GEAR_RATIO,
          DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
          DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
          DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
          DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET,
          "DrivebaseIntake");

  private SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

  String data = "";

  int counter = 0;
  double startTime = 0;
  double priorAutospeed = 0;

  double[] numberArray = new double[10];
  ArrayList<Double> entries = new ArrayList<Double>();

  public Robot() {
    super(.005);
    LiveWindow.disableAllTelemetry();
  }

  public enum Sides {
    LEFT,
    RIGHT,
    FOLLOWER
  }

  // methods to create and setup motors (reduce redundancy)
  public WPI_TalonFX setupWPI_TalonFX(int port, Sides side, boolean inverted) {
    // create new motor and set neutral modes (if needed)
    WPI_TalonFX motor = new WPI_TalonFX(port);
    // setup talon
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(inverted);

    // setup encoder if motor isn't a follower
    if (side != Sides.FOLLOWER) {

      motor.configSelectedFeedbackSensor(
          FeedbackDevice.IntegratedSensor,
          PIDIDX, 10);

      switch (side) {
        // setup encoder and data collecting methods

        case RIGHT:
          // set right side methods = encoder methods

          motor.setSensorPhase(false);
          rightEncoderPosition = () -> motor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
          rightEncoderRate = () -> motor.getSelectedSensorVelocity(PIDIDX) * encoderConstant *
              10;

          break;
        case LEFT:
          motor.setSensorPhase(false);

          leftEncoderPosition = () -> motor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
          leftEncoderRate = () -> motor.getSelectedSensorVelocity(PIDIDX) * encoderConstant *
              10;

          break;
        default:
          // probably do nothing
          break;

      }

    }

    return motor;

  }

  @Override
  public void robotInit() {
    if (!isReal())
      SmartDashboard.putData(new SimEnabler());

    stick = new Joystick(0);

    if (USE_MODULES) {
      frontLeftModule = FRONT_LEFT_CONFIG.create(IS_COMPETITION);
      frontRightModule = FRONT_RIGHT_CONFIG.create(IS_COMPETITION);
      backLeftModule = BACK_LEFT_CONFIG.create(IS_COMPETITION);
      backRightModule = BACK_RIGHT_CONFIG.create(IS_COMPETITION);

      WPI_TalonFX frontRightMotor = (WPI_TalonFX) frontRightModule.getDriveMotor();
      frontRightMotor.setSensorPhase(false);
      rightEncoderPosition = () -> frontRightMotor.getSelectedSensorPosition(PIDIDX) * FRONT_LEFT_CONFIG.getRatio().getConfiguration().getDriveReduction() / ENCODER_EDGES_PER_REV;
      rightEncoderRate = () -> frontRightMotor.getSelectedSensorVelocity(PIDIDX) * 10.0 * FRONT_LEFT_CONFIG.getRatio().getConfiguration().getDriveReduction() / ENCODER_EDGES_PER_REV;

      WPI_TalonFX frontLeftMotor = (WPI_TalonFX) frontLeftModule.getDriveMotor();
      frontLeftMotor.setSensorPhase(false);
      leftEncoderPosition = () -> frontLeftMotor.getSelectedSensorPosition(PIDIDX) * FRONT_LEFT_CONFIG.getRatio().getConfiguration().getDriveReduction() / ENCODER_EDGES_PER_REV;
      leftEncoderRate = () -> frontLeftMotor.getSelectedSensorVelocity(PIDIDX) * 10.0 * FRONT_LEFT_CONFIG.getRatio().getConfiguration().getDriveReduction() / ENCODER_EDGES_PER_REV;
    } else {
      // create left motor
      WPI_TalonFX leftMotor = setupWPI_TalonFX(1, Sides.LEFT, true);

      WPI_TalonFX leftFollowerID7 = setupWPI_TalonFX(7, Sides.FOLLOWER, true);
      leftFollowerID7.follow(leftMotor);

      WPI_TalonFX rightMotor = setupWPI_TalonFX(4, Sides.RIGHT, true);
      WPI_TalonFX rightFollowerID10 = setupWPI_TalonFX(10, Sides.FOLLOWER, true);
      rightFollowerID10.follow(rightMotor);

      drive = new DifferentialDrive(leftMotor, rightMotor);
      drive.setDeadband(0);

      // config steer motors
      for (WPI_TalonFX motor : steerMotors) {
        motor.setNeutralMode(NeutralMode.Brake);
      }
    }

    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of WPILib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    gyroAngleRadians = () -> 0.0;

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  @Override
  public void disabledInit() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Robot disabled");
    if (USE_MODULES) {
      frontLeftModule.set(0.0, 0.0);
      frontRightModule.set(0.0, 0.0);
      backLeftModule.set(0.0, 0.0);
      backRightModule.set(0.0, 0.0);
    } else {
      drive.tankDrive(0, 0);
    }

    // data processing step
    data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    entries.clear();
    System.out.println("Robot disabled");
    System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
    data = "";
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    // feedback for users, but not used by the control program
    SmartDashboard.putNumber("l_encoder_pos", leftEncoderPosition.get());
    SmartDashboard.putNumber("l_encoder_rate", leftEncoderRate.get());
    SmartDashboard.putNumber("r_encoder_pos", rightEncoderPosition.get());
    SmartDashboard.putNumber("r_encoder_rate", rightEncoderRate.get());
  }

  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");

    if (USE_MODULES) {
    } else {
      for (int i = 0; i < steerMotors.length; i++) {
        //steerMotors[i].set(ControlMode.Position, ENCODER_OFFSETS[i] * 2048.0 / (Math.PI * 2.0 * STEER_RATIO));
        steerMotors[i].set(ControlMode.Position, 0);
      }
    }
  }

  @Override
  public void teleopPeriodic() {
    if (USE_MODULES) {
      double speed = -stick.getY();
      frontLeftModule.set(speed, 0.0);
      frontRightModule.set(speed, 0.0);
      backLeftModule.set(speed, 0.0);
      backRightModule.set(speed, 0.0);
    } else {
      drive.arcadeDrive(-stick.getY(), stick.getX());
    }
  }

  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
    if (USE_MODULES) {
    } else {
      for (int i = 0; i < steerMotors.length; i++) {
        steerMotors[i].set(ControlMode.Position, 0);
      }
    }

    startTime = Timer.getFPGATimestamp();
    counter = 0;
  }

  /**
   * If you wish to just use your own robot program to use with the data logging
   * program, you only need to copy/paste the logic below into your code and
   * ensure it gets called periodically in autonomous mode
   * 
   * Additionally, you need to set NetworkTables update rate to 10ms using the
   * setUpdateRate call.
   */
  @Override
  public void autonomousPeriodic() {
    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();
    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);
    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;
    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    double leftPosition = leftEncoderPosition.get();
    double leftRate = leftEncoderRate.get();

    double rightPosition = rightEncoderPosition.get();
    double rightRate = rightEncoderRate.get();

    // command motors to do things
    if (USE_MODULES) {
      frontLeftModule.set(autospeed * battery, 0.0);
      frontRightModule.set(autospeed * battery, 0.0);
      backLeftModule.set(autospeed * battery, 0.0);
      backRightModule.set(autospeed * battery, 0.0);
    } else {
      drive.tankDrive(
              (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
              false);
    }

    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();

    // Add data to a string that is uploaded to NT
    for (double num : numberArray) {
      entries.add(num);
    }
    counter++;
  }
}
