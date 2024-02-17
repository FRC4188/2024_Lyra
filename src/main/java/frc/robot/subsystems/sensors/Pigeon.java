package frc.robot.subsystems.sensors;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon extends Pigeon2 {
  double offsetDegrees;

  public Pigeon(int canID, double offsetDegrees) {
    super(canID, "canivore");
    super.getConfigurator().apply(new Pigeon2Configuration());
    super.clearStickyFaults();
    this.offsetDegrees = offsetDegrees;
    reset();
  }

  public void reset() {
    super.setYaw(offsetDegrees);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(((super.getYaw().getValueAsDouble() + 180) % 360) - 180);
  }

  public double getRollAsDouble() {
    return super.getRoll().getValueAsDouble();
  }

  public double getPitchAsDouble() {
    return super.getPitch().getValueAsDouble();
  }
}
