// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.springframework.stereotype.Component;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AdvancerJiggle;
import frc.robot.Commands.AutoRevShooter;
import frc.robot.Commands.ChangeShot;
import frc.robot.Commands.DriveTrain;
import frc.robot.Commands.IntakeLoad;
import frc.robot.Commands.IntakeOut;
import frc.robot.Commands.RetractToBottom;
import frc.robot.Commands.ShooterIntake;
import frc.robot.Commands.ShooterLob;
import frc.robot.Commands.SpeakerShoot;
import frc.robot.Commands.TelopSwerve;
import frc.robot.Constanst.JoystickConstants;
import frc.robot.Subsystems.*;

@Component
public class RobotContainer {
  
  /* Controllers */
    private final Joystick driver = new Joystick(JoystickConstants.DRIVER_USB);
    private final Joystick operator = new Joystick(JoystickConstants.OPERATOR_USB);

  /* Drive Controls */
    private final int translationAxis = JoystickConstants.LEFT_Y_AXIS;
    private final int strafeAxis = JoystickConstants.LEFT_X_AXIS;
    private final int rotationAxis = JoystickConstants.RIGHT_X_AXIS;

  /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, JoystickConstants.BACK_BUTTON);

  /* Subsystems */
    private final LimelightSubsystem limelight = new LimelightSubsystem();
    private final AdvancerSubsystem advancer = new AdvancerSubsystem();
    private final ShooterSubsystem shooter_ = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final DriveTrain s_swerve = new DriveTrain();

  /* Pathplanner stuff */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    double jiggle_count = SmartDashboard.getNumber("Advancer Jiggle Number Auto", 5);
    s_swerve.setDefaultCommand(
      new TelopSwerve(
        s_swerve,
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis), 
        () -> -driver.getRawAxis(rotationAxis)
        )
    );
    
    configureBindings();
      /* Intake */
        NamedCommands.registerCommand("Intake", new IntakeLoad(intake, advancer));
        NamedCommands.registerCommand("Outtake", new IntakeOut(intake, advancer));
      /* Shooter Commands*/
        NamedCommands.registerCommand("Speaker Shot", new SpeakerShoot(shooter_, advancer));
        NamedCommands.registerCommand("Lob shot", new ShooterLob(shooter_, advancer));
        NamedCommands.registerCommand("Rev Shooter", new AutoRevShooter(shooter_));
      /* Advancer Commands */   
        NamedCommands.registerCommand("Advancer Jiggle", new AdvancerJiggle(advancer, jiggle_count));

        SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void teleopInit() {
    shooter_.Stop();
    intake.Stop();
    advancer.Stop();
  }

  private void configureBindings() {
    /* Driver Controls */
      zeroGyro.onTrue (new InstantCommand(() -> s_swerve.ResetDrives()));

    /* Operator Controls */
      /* Shooter Controls */
        new JoystickButton (operator, JoystickConstants.GREEN_BUTTON).onTrue(new SpeakerShoot(shooter_, advancer));
        new Trigger (new JoystickButton(operator, JoystickConstants.GREEN_BUTTON)).and(
          new Trigger(() -> (advancer.LowerBeam.get()))).onTrue(new ParallelCommandGroup(new ShooterIntake(shooter_), new RetractToBottom(advancer)));
        new Trigger(() -> limelight.InRange(5)).and(new Trigger(() -> (advancer.LowerBeam.get())))
          .onTrue(new AutoRevShooter(shooter_));
        new JoystickButton (operator, JoystickConstants.BLUE_BUTTON).onTrue(new ShooterLob(shooter_, advancer));
        new JoystickButton (operator, JoystickConstants.YELLOW_BUTTON).onTrue(new ChangeShot(shooter_, advancer));
      /* Intake Controls */
        new JoystickButton (operator, JoystickConstants.RIGHT_BUMPER).onTrue(new IntakeLoad(intake, advancer));
        new JoystickButton (operator, JoystickConstants.LEFT_BUMPER).onTrue(new IntakeOut(intake, advancer));
      /* Advancer stuff */
        new JoystickButton (operator, JoystickConstants.START_BUTTON).onTrue(new AdvancerJiggle(advancer, SmartDashboard.getNumber("Advancer Jiggle Number Teleop", 5)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
