package CSP_Lib.motors;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class CSP_CANcoder extends CANcoder {

    public boolean inverted = false;
    public double zero = 0.0;

    public CSP_CANcoder(int id, String canBus) {
        super(id, canBus);
        init();
    }

    public CSP_CANcoder(int id) {
        this(id, "rio");
    }

    /** Configures the encoder for typical use */
    public void init() {
        super.getConfigurator().apply(new CANcoderConfiguration());
        super.clearStickyFaults();

        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        sensorConfigs.MagnetOffset = 0.0;
        sensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        super.getConfigurator().apply(sensorConfigs);
    }

    public void setPositionRads(double position) {
        super.setPosition(position / (2.0 * Math.PI));
    }

    public void resetPosition() {
        super.setPosition(0.0);
    }

    /**
     * 
     * @return Position of encoder from [-pi to pi].
     */
    public double getPositionRads() {
        if(!inverted) {
            return (super.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI - zero + Math.PI) % (2.0 * Math.PI) - Math.PI;
        } else {
            return (-super.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI - zero + Math.PI) % (2.0 * Math.PI) - Math.PI;
        }
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public boolean getInverted() {
        return inverted;
    }

    public void setZero(double zero) {
        this.zero = zero;
    }
}