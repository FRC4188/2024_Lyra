package frc.robot.commands.groups;

import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.sensors.Localization;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class StillShot extends ParallelCommandGroup {
    private Swerve drive = Swerve.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Shooter shooter = Shooter.getInstance();

    /** Creates a new ShootOnReady. 
     * 
    */
    public StillShot(DoubleSupplier xInput, DoubleSupplier yInput) {
        addCommands(
            
        );
    }
}
