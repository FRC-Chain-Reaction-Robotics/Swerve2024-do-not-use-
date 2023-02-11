package frc.robot.lib;

public final class Utils {
    
    public static double applyDeadband(double input, double deadband) {
        return (Math.abs(input) < deadband) ? 0.0 : Math.copySign((Math.abs(input) - deadband) / 0.9, input);
    }

}
