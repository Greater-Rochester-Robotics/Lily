package org.team340.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.swerve.Perspective;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final SwerveModuleConfig kFrontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(0.27305, 0.27305)
        .setMoveMotor(SwerveMotors.sparkMax(RobotMap.kFlMove, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkMax(RobotMap.kFlTurn, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkMaxEncoder(0.856, false));

    private static final SwerveModuleConfig kFrontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(0.27305, -0.27305)
        .setMoveMotor(SwerveMotors.sparkMax(RobotMap.kFrMove, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkMax(RobotMap.kFrTurn, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkMaxEncoder(0.463, false));

    private static final SwerveModuleConfig kBackLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-0.27305, 0.27305)
        .setMoveMotor(SwerveMotors.sparkMax(RobotMap.kBlMove, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkMax(RobotMap.kBlTurn, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkMaxEncoder(0.802, false));

    private static final SwerveModuleConfig kBackRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-0.27305, -0.27305)
        .setMoveMotor(SwerveMotors.sparkMax(RobotMap.kBrMove, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkMax(RobotMap.kBrTurn, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkMaxEncoder(0.170, false));

    private static final SwerveConfig kConfig = new SwerveConfig()
        .setTimings(TimedRobot.kDefaultPeriod, TimedRobot.kDefaultPeriod, 0.04)
        .setMovePID(0.01, 0.0, 0.0)
        .setMoveFF(0.05, 0.127)
        .setTurnPID(10.0, 0.0, 0.2)
        .setBrakeMode(false, true)
        .setLimits(4.2, 13.5, 11.5, 28.0)
        .setDriverProfile(4.0, 1.0, 0.15, 4.2, 2.0, 0.1)
        .setPowerProperties(Constants.kVoltage, 80.0, 60.0)
        .setMechanicalProperties(7.13, 13.71, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.1)
        .setIMU(SwerveIMUs.adis16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Port.kOnboardCS0, CalibrationTime._4s))
        .setModules(kFrontLeft, kFrontRight, kBackLeft, kBackRight);

    private final SwerveAPI api;

    public Swerve() {
        api = new SwerveAPI(kConfig);
        api.enableTunables("swerve/api");
    }

    @Override
    public void periodic() {
        api.refresh();
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(Perspective.kOperator))
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        return commandBuilder("Swerve.drive()").onExecute(() ->
            api.applyDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                Perspective.kOperator,
                true,
                true
            )
        );
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }
}
