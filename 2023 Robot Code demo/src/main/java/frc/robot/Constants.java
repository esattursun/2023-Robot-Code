package frc.robot;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ModuleConfiguration;

import static edu.wpi.first.math.util.Units.degreesToRadians;



public final class Constants{
  public static final class DriveConstants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = inchesToMeters(20.5);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = inchesToMeters(20.5);

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

   

    // SPARK MAX CAN IDs 
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
  ModuleConfiguration.MK4_L1.getDriveReduction() *
  ModuleConfiguration.MK4_L1.getWheelDiameter() * Math.PI;
        
  public static class VisionConstants {
    

    public static final Transform3d PHOTONVISION_TRANSFORM = new Transform3d(
            new Translation3d(0.205697, -0.244475, 0.267365),
            new Rotation3d(0, Units.degreesToRadians(15), 0)
    );

    public static final Vector<N3> PHOTONVISION_STD_DEV = VecBuilder.fill(0.7, 0.7, 0.5);

    public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);

    public static final double AMBIGUITY_FILTER = 0.05;
  }
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 
  (DriveConstants.MAX_VELOCITY_METERS_PER_SECOND /
  Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));


    //büyük
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 10;
    public static final int kRearRightDrivingCanId = 8;
    //küçük
    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 1;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;
  }
  public static final class TeleopDriveConstants {

    public static final double DEADBAND = 0.1;

    public static final double X_RATE_LIMIT = 6.0;
    public static final double Y_RATE_LIMIT = 6.0;
    public static final double ROTATION_RATE_LIMIT = 5.0 * Math.PI;

    public static final double HEADING_MAX_VELOCITY = Math.PI * 2;
    public static final double HEADING_MAX_ACCELERATION = Math.PI * 2;
    
    public static final double HEADING_kP = 2.0;
    public static final double HEADING_kI = 0.0;
    public static final double HEADING_kD = 0.0;

    public static final double HEADING_TOLERANCE = degreesToRadians(1.5);

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;

    
  }
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
 
  public static class Elevator{
    public static final int kElevatorMotorPort = 0;
    public static final boolean kElevatorEncoderReverseDirection = true;
    public static final double setpoint = 0.5; 
    public static final int elevatorjoystickaxis =3;
  }

  public static class IntakeConstants{
  public static final double IntakeInSpeed = 0.4;
  public static final int IntakeMotorId = 9; 
  }
 public static class LimelightConstants{
  public static double goalHeightInches=30.31496;
 }
 public static class VisionConstants {

  /**
   * Physical location of the camera on the robot, relative to the center of the robot.
   */
  public static final Transform3d CAMERA_TO_ROBOT =
      new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
  public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
}

 }
