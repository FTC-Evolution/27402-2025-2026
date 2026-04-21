package org.firstinspires.ftc.teamcode.utility;

public class SimplePID {
    private double kP;
    private double kI;
    private double kD;

    private double integral, lastError;
    private double min, max;
    private double band;

    public double error;

    public SimplePID(double kP, double kI, double kD,
                     double min, double max, double band) {

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.min = min;
        this.max = max;
        this.band = band;
    }

    public double update(double error) {
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        double clampedOutput = clamp(output);
        double bandOutput = deadband(clampedOutput);

        this.error = bandOutput;

        return bandOutput;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }

    private double clamp(double value) {
        return Math.max(min, Math.min(max, value));
    }

    private double deadband(double value) {
        double banded;
        if (Math.abs(value) < this.band) {
            banded = 0;
        } else {
            banded = value;
        }
        return banded;
    }
}
