package frc.robot;

public enum BlindShots {
    SPEAKER_DIRECT(10.0),
    SPEAKER_DEFENDED(80.0);

    private final double velocity;
    private BlindShots(double velocity) {
        this.velocity = velocity;
    }

    public double getVelocity() {
        return velocity;
    }
}
