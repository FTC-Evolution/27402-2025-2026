package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.Servo;

public class Led {
    private final Servo led;

    public class Colour {
        public static final double WHITE = 1;
        public static final double GREEN = 0.5;
    }
    public Led(Servo led){
        this.led = led;
    }

    public void setColour(double colour) {
        led.setPosition(colour);
    }
}
