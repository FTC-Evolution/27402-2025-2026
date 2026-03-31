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

    public double[] getPower() {
        return new double[]{shooter1.getPower(), shooter2.getPower()};
    }

    public double[] getVelocity() {
        return new double[]{shooter1.getVelocity(), shooter2.getVelocity()};
    }
}