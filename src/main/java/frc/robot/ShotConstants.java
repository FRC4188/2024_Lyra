package frc.robot;

public final class ShotConstants {


    public enum BlindShots {
        SPEAKER_DIRECT(13.0, 32.5),
        SPEAKER_DIRECT_REVERSE(13.0, -32.5),
        SPEAKER_DEFENDED(12.0, 45.0),
        SPEAKER_DEFENDED_REVERSE(12.0, -45.0);

        private final double velocity;
        private final double angle;
        private BlindShots(double velocity, double angle) {
            this.velocity = velocity;
            this.angle = angle;
        }

        public double getVelocity() {
            return velocity;
        }

        public double getAngle() {
            return angle;
        }
    }   

}