// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.io.ObjectInputFilter.Config;
import java.util.function.DoublePredicate;
import com.studica.frc.jni.AHRSJNI;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.ImmutableLinearVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constanst;
import frc.robot.Subsystems.SwerveModule;



/** Represents a swerve drive style drivetrain. */
public class DriveTrain extends SubsystemBase {
  public static final double kMaxSpeed = 4.47; // was 3 meters per second
  public static final double kMaxAngularSpeed = 4.41 * 2 * Math.PI; // was Math.PI for 1/2 rotation per second
  
  double Ptranslate = 10.0;
  double Itranslate = 0.0;
  double Dtranslate = 0.0;
  double Protate = 15.0;
  double Irotate = 2.0;
  double Drotate = 0.0;
       
private final Translation2d m_frontLeftLocation = Constanst.SwerveConstants.m_frontLeftLocation;
private final Translation2d m_frontRightLocation = Constanst.SwerveConstants.m_frontRightLocation;
private final Translation2d m_backLeftLocation = Constanst.SwerveConstants.m_backLeftLocation;
private final Translation2d m_backRightLocation = Constanst.SwerveConstants.m_backRightLocation;


private final SwerveModule m_frontLeft = new SwerveModule(Constanst.SwerveConstants.Mod3.constants);
private final SwerveModule m_frontRight = new SwerveModule(Constanst.SwerveConstants.Mod2.constants);
private final SwerveModule m_backLeft = new SwerveModule(Constanst.SwerveConstants.Mod0.constants);
private final SwerveModule m_backRight = new SwerveModule(Constanst.SwerveConstants.Mod1.constants);

//  private final Gyro_EPRA m_gyro = new Gyro_EPRA();
private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kMXP);

private double xSpeed_cur;
private double ySpeed_cur;
private double rot_cur;
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          new Pose2d(new Translation2d(),new Rotation2d(Units.degreesToRadians(180)))
          );

  public DriveTrain() {
    // SmartDashboard.putNumber("P rotate", Protate);
    // SmartDashboard.putNumber("D rotate", Drotate);
    // SmartDashboard.putNumber("I rotate", Irotate);

    // SmartDashboard.putNumber("P translate", Ptranslate);
    // SmartDashboard.putNumber("D translate", Dtranslate);
    // SmartDashboard.putNumber("I translate", Itranslate);

    m_gyro.reset();
    try{
      RobotConfig config = RobotConfig.fromGUISettings();
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
    this // Reference to this subsystem to set requirements
);
}catch(Exception e){
  DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
}
  }

  public void ResetDrives () {

    /* 
    m_frontLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_backLeft.resetEncoder();
    m_backRight.resetEncoder();
    */
    m_gyro.reset();

    SmartDashboard.putString("Gyro has been reset", java.time.LocalTime.now().toString());
  }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  
  public void driveRobotRelative(ChassisSpeeds chassisSpeedsIn) {
    drive(chassisSpeedsIn.vxMetersPerSecond, chassisSpeedsIn.vyMetersPerSecond, chassisSpeedsIn.omegaRadiansPerSecond, false);
  }
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

  /*
SmartDashboard.putString("gyro", m_gyro.getRotation2d().toString());


    SmartDashboard.putString("module 0", swerveModuleStates[0].toString());
    SmartDashboard.putString("module 1", swerveModuleStates[1].toString());
    SmartDashboard.putString("module 2", swerveModuleStates[2].toString());
    SmartDashboard.putString("module 3", swerveModuleStates[3].toString());
*/
    xSpeed_cur = xSpeed;
    ySpeed_cur = ySpeed;
    rot_cur = rot;

    //TODO we probably need to publish the updated location from here once we get it to work
    //updateOdometry();
    //MetricsProvider.updateLocation(getPose());
  }

  public ChassisSpeeds getSpeed() {
    return new ChassisSpeeds( xSpeed_cur, ySpeed_cur, rot_cur);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters() ;
  } 

  public void resetPose(Pose2d aPose2d) {
    m_odometry.resetPosition(m_gyro.getRotation2d(),
         new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        }, aPose2d );
  
  }
  
  /* adding some gyro output so other classes don't need to access it directly */
  public double getYaw() {
    return (m_gyro.getAngle());
  }
}
