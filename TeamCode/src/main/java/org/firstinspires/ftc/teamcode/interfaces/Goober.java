package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;

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
                bottomGoober.setPower(0);
                break;
            case BOTTOM:
                bottomGoober.setPower(power);
                topGoober.setPower(0);
                break;
        }
    }

    public HashMap<Type, Double> getPower() {
        HashMap<Type, Double> gooberPowerMap = new HashMap<>();
        gooberPowerMap.put(Type.TOP, topGoober.getPower());
        gooberPowerMap.put(Type.BOTTOM, bottomGoober.getPower());
        return gooberPowerMap;
    }
}
