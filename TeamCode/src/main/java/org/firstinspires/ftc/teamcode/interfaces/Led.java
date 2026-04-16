package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.Servo;

public class Led {
    private final Servo led;
    public Led(Servo led){
        this.led = led;
    }

    public void setColour(double colour) {
        led.setPosition(colour);
    }
}
