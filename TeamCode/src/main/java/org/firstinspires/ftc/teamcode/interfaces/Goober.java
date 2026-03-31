package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Goober {
    protected final DcMotor bottomGoober; // goober 2
    protected final DcMotor topGoober;
    public Goober(DcMotor topGoober, DcMotor bottomGoober) {
        this.bottomGoober = bottomGoober;
        this.topGoober = topGoober;
    }

    public void modPower(double power) {
        topGoober.setPower(power);
        bottomGoober.setPower(-power);
    }

    public void modSoloPower(double power, String goober) {
        switch (goober) {
            case "top":
            topGoober.setPower(power);
            case "bottom":
            bottomGoober.setPower(power);
            default:
                this.modPower(power);
        }
    }

    public double[] getPower() {
        return new double[]{topGoober.getPower(),bottomGoober.getPower()};
    }
}
