package org.team340.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team340.lib.util.GRRDashboard;
import org.team340.lib.util.Profiler;
import org.team340.lib.util.Tunable;
import org.team340.robot.subsystems.Swerve;

@Logged
public final class Robot extends TimedRobot {

    public final Swerve swerve;

    private final CommandXboxController driver;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SignalLogger.enableAutoLogging(false);
        Epilogue.getConfig().root = "/Telemetry";

        // Initialize subsystems
        swerve = new Swerve();

        // Initialize controllers
        driver = new CommandXboxController(Constants.kDriver);

        // Set default commands
        swerve.setDefaultCommand(
            swerve.drive(
                driver::getLeftX,
                driver::getLeftY,
                () -> driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()
            )
        );

        // Driver bindings
        driver.povLeft().onTrue(swerve.tareRotation());
    }

    @Override
    public void robotPeriodic() {
        Profiler.start("RobotPeriodic");
        Profiler.run("CommandScheduler", () -> CommandScheduler.getInstance().run());
        Profiler.run("Epilogue", () -> Epilogue.update(this));
        Profiler.run("GRRDashboard", GRRDashboard::update);
        Profiler.run("Tunables", Tunable::update);
        Profiler.end();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}
}
