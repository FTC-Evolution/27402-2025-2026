package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shooter {
    protected final DcMotorEx shooter1;
    protected final DcMotorEx shooter2;

    static final double     SHOOTER_P              = 90;
    static final double     SHOOTER_I              = 1.95;
    static final double     SHOOTER_D              = 0.1;
    static final double     SHOOTER_F              = 0;
    static final double     SHOOTER_TICKS_PER_REV  = 28;

    static final double     SHOOTER_READY_THRESHOLD= 41;

    public static final double DEFAULT_SHOOTER_POWER = 32;
    public static final double DEFAULT_LONG_SHOOTER_POWER = 39.5;
    public static final double DEFAULT_LONG_AUTONOMOUS_SHOOTER_POWER = DEFAULT_LONG_SHOOTER_POWER;
    public static final double DEFAULT_SIDE_SHOOTER_POWER = DEFAULT_SHOOTER_POWER - 1;

    public Shooter(DcMotorEx shooter1, DcMotorEx shooter2) {
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;

        this.shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        this.shooter1.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        this.shooter2.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        // Reset encoders
        this.shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void modPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public void modVelocity(double velocity) {
        shooter1.setVelocity(velocity);
        shooter2.setVelocity(velocity);
    }

    public void stop() {
        modVelocity(0);
    }

    public double[] getPower() {
        return new double[]{shooter1.getPower(), shooter2.getPower()};
    }

    public double[] getVelocity() {
        return new double[]{shooter1.getVelocity(), shooter2.getVelocity()};
    }

    public double speed(double speed) {
        return speed * SHOOTER_TICKS_PER_REV;
    }

    public double[] readyRange(double goalTps) {
        return new double[]{goalTps - SHOOTER_READY_THRESHOLD, goalTps + SHOOTER_READY_THRESHOLD};
    }

    public boolean inReadyRange(double goalTps) {
        double[] shooterReadyRange = readyRange(goalTps);

        boolean shooter1InRange = shooter1.getVelocity() >= shooterReadyRange[0]
                && shooter1.getVelocity() <= shooterReadyRange[1];
        boolean shooter2InRange = shooter2.getVelocity() >= shooterReadyRange[0]
                && shooter2.getVelocity() <= shooterReadyRange[1];

        boolean shooter1NotEmpty = shooter1.getVelocity() > 0;
        boolean shooter2NotEmpty = shooter2.getVelocity() > 0;

        return shooter1NotEmpty && shooter1InRange && shooter2NotEmpty && shooter2InRange;
    }

    public void fireUp(double shooterTPS, Led led) {
        modVelocity(shooterTPS);

        while (!inReadyRange(shooterTPS)) {
            led.setColour(Led.Colour.WHITE);
            // telemetry.addLine("Waiting for shooters to fire up");
            // telemetry.update();
        }

        led.setColour(Led.Colour.GREEN);
        //telemetry.addLine("Shooters ready ... Shooting");
        //telemetry.update();
    }
}