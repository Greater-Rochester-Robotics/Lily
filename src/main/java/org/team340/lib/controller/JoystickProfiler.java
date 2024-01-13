package org.team340.lib.controller;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import org.team340.lib.commands.CommandBuilder;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Polar2d;

/**
 * Records a controller's joystick outputs and generates a corresponding JSON string.
 * For use in correcting irregular joystick output shape.
 */
public class JoystickProfiler {

    private final GenericHID controller;
    private final InterpolatingDoubleTreeMap data = new InterpolatingDoubleTreeMap();
    private final int xAxis;
    private final int yAxis;

    /**
     * Create the profiler.
     * @param controller The controller to profile.
     * @param xAxis The index of the controller's X axis.
     * @param yAxis The index of the controller's Y axis.
     */
    public JoystickProfiler(GenericHID controller, int xAxis, int yAxis) {
        this.controller = controller;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
    }

    /**
     * Clears currently saved data.
     */
    public void clearData() {
        data.clear();
    }

    /**
     * Collects data from the joystick. Should be run periodically.
     */
    public void pollData() {
        double x = controller.getRawAxis(xAxis);
        double y = controller.getRawAxis(yAxis);
        double theta = Math.atan2(y, x);
        double r = Math.hypot(x, y);
        System.out.println("T: " + Math2.toFixed(theta) + " R: " + Math2.toFixed(r));
        data.put(theta, r);
    }

    /**
     * Generates the profile using data collected by {@link JoystickProfiler#pollData() pollData()}.
     * @param samplePoints Number of points to sample.
     * @return The joystick profile.
     */
    public List<Polar2d> generateProfile(int samplePoints) {
        List<Polar2d> profile = new ArrayList<>();

        for (int i = 0; i < samplePoints; i++) {
            double theta = ((double) i / (double) samplePoints * Math2.TWO_PI) - Math.PI;
            profile.add(new Polar2d(theta, data.get(theta)));
        }
        return profile;
    }

    /**
     * Write profile generated by {@link JoystickProfiler#generateProfile(int samplePoints) generateProfile()} to console as a JSON string.
     * The output can be saved to a file and loaded by {@link JoystickProfile}.
     * @param rawProfile Profile to write to file.
     * @return The JSON string.
     */
    public static String writeToConsole(List<Polar2d> rawProfile) {
        List<List<Double>> profile = new ArrayList<List<Double>>();
        for (Polar2d polar2d : rawProfile) {
            profile.add(List.of(polar2d.getTheta(), polar2d.getR()));
        }

        try {
            String json = new ObjectMapper().writeValueAsString(profile);
            System.out.println(json);
            return json;
        } catch (Exception e) {
            e.printStackTrace();
            return "[]";
        }
    }

    /**
     * Command to profile a controller's joystick and print to console.
     * @param controller Controller to profile.
     * @param xAxis X-Axis of controller joystick.
     * @param yAxis Y-Axis of controller joystick.
     * @param samplePoints Number of points to sample.
     */
    public static Command run(GenericHID controller, int xAxis, int yAxis, int samplePoints) {
        JoystickProfiler profiler = new JoystickProfiler(controller, xAxis, yAxis);
        return new CommandBuilder()
            .onInitialize(profiler::clearData)
            .onExecute(profiler::pollData)
            .onEnd(() -> writeToConsole(profiler.generateProfile(samplePoints)))
            .ignoringDisable(true);
    }
}