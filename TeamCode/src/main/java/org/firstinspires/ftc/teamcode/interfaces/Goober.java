package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Goober {
    protected final DcMotor bottomGoober; // goober 2
    protected final DcMotor topGoober;

    public enum Type {
        TOP,
        BOTTOM,
    }

    public Goober(DcMotor topGoober, DcMotor bottomGoober) {
        this.bottomGoober = bottomGoober;
        this.topGoober = topGoober;
    }

    public void modPower(double power) {
        topGoober.setPower(power);
        bottomGoober.setPower(-power);
    }

    public void modSoloPower(double power, Type goober) {
        switch (goober) {
            case TOP:
                topGoober.setPower(power);
                break;
            case BOTTOM:
                bottomGoober.setPower(power);
                break;
            default:
                this.modPower(power);
                break;
        }
    }

    public double[] getPower() {
        return new double[]{topGoober.getPower(),bottomGoober.getPower()};
    }
}
