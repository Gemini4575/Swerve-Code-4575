package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constanst;


public class TelopSwerve extends Command {    
    private DriveTrain s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier slowMode;
    private BooleanSupplier middleMode;
    private final double slowSpeed = 0.2;
    private final double slowRotation = 0.2;
    private final double middleSpeed = 0.65;
    private final double middleRotation = 0.65;

    public TelopSwerve(DriveTrain s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.slowMode = slowMode;
        // this.targetingUtil = targetingUtil; 
        this.middleMode = middleMode;
    }

    @Override
    public void execute() {
        String mode = "normal";
        double speedMulti;
        double rotMulti;
        // double teleOpMaxSpeed = Constants.Swerve.maxSpeed;
        // double teleOpMaxAngularVelocity = Constants.Swerve.maxAngularVelocity;

        /*speedMulti = teleOpMaxSpeed * .90;
        rotMulti = teleOpMaxAngularVelocity * .90;

        if (slowMode.getAsBoolean()) {
            speedMulti = slowSpeed * teleOpMaxSpeed;
            rotMulti = slowRotation * teleOpMaxAngularVelocity;
            mode="slow";
        }

        if (middleMode.getAsBoolean()) {
            speedMulti = middleSpeed * teleOpMaxSpeed;
            rotMulti = middleRotation * teleOpMaxAngularVelocity;
            mode="middle";
        }
*/

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constanst.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constanst.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constanst.stickDeadband);

        // if(autoTargeting.getAsBoolean()) {
        //     rotationVal = targetingUtil.calculateRotation();
        //     rotMulti = teleOpMaxAngularVelocity;
        //     mode="auto";
        // }


        /* Drive */
        s_Swerve.drive(
           strafeVal,
           translationVal,
           rotationVal,
           true
        );
        SmartDashboard.putString("swerve mode", mode);
    }
}