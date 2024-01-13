package org.team340.lib.swerve.hardware.encoders.vendors;

import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import org.team340.lib.swerve.hardware.encoders.SwerveEncoder;

/**
 * Wrapper for an absolute encoder attached directly to a REV Spark for swerve (Through bor, MagEncoder with adapter board, CANandcoder, etc).
 */
public class SwerveSparkEncoder implements SwerveEncoder {

    private final SparkAbsoluteEncoder sparkEncoder;

    /**
     * Create the Spark Attached Encoder wrapper.
     * @param sparkEncoder The encoder to wrap.
     * @param config The general swerve config.
     * @param moduleConfig The encoder's module's config.
     */
    public SwerveSparkEncoder(SparkAbsoluteEncoder sparkEncoder) {
        this.sparkEncoder = sparkEncoder;
        // Config options are applied in SwerveSparkMax / SwerveSparkFlex.
    }

    @Override
    public double getPosition() {
        return MathUtil.angleModulus(sparkEncoder.getPosition());
    }
}
