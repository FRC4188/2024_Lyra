package frc.robot.commands.groups;

import frc.robot.Constants.field.Goal;
import frc.robot.subsystems.sensors.Sensors;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SpeakerStillShot extends ParallelCommandGroup {
    private Sensors sensors = Sensors.getInstance();

    /** Creates a new ShootOnReady. 
     * 
    */
    public SpeakerStillShot(DoubleSupplier xInput, DoubleSupplier yInput) {
        addCommands(
            new InstantCommand(() -> sensors.setGoal(Goal.SPEAKER)),
            new ShootOnReady(
                xInput, 
                yInput,
                () -> sensors.getFormulaShooterRPM(), 
                () -> sensors.getFormulaShoulderAngle().getDegrees(), 
                () -> sensors.getFormulaDriveAngle().getDegrees())
        );
    }
}
