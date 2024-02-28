package frc.robot.commands.groups;

import frc.robot.Constants;
import frc.robot.Constants.sensors;
import frc.robot.commands.drivetrain.TrackingDrive;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.sensors.Sensors;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shoulder.Shoulder;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootOnReady extends ParallelCommandGroup {
    private Swerve drive = Swerve.getInstance();
    private Shoulder shoulder = Shoulder.getInstance();
    private Flywheel shooter = Flywheel.getInstance();
    private Feeder feeder = Feeder.getInstance();
    private Sensors sensors = Sensors.getInstance();

    /** Creates a new ShootOnReady. */
    public ShootOnReady(DoubleSupplier xInput, DoubleSupplier yInput, Translation3d goal) {
        addCommands(
            new ParallelCommandGroup(
                new SetShooterMPS(() -> sensors.getFormulaShooterRPM()),
                new SetShoulderAngle(() -> sensors.getFormulaShoulderAngle()),
                new TrackingDrive(xInput, yInput, () -> sensors.getFormulaDriveAngle())
            )
        );
    }
}
