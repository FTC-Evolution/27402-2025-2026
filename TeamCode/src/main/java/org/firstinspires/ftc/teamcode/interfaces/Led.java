package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.Servo;

public class Led {
    private final Servo led;

    public static class Colour {
        public static final double RED        = 0.28;
        public static final double ORANGE     = 0.33;
        public static final double YELLOW     = 0.39;
        public static final double SAGE       = 0.44;
        public static final double GREEN      = 0.50;
        public static final double AZURE      = 0.56;
        public static final double BLUE       = 0.61;
        public static final double INDIGO     = 0.67;
        public static final double VIOLET     = 0.72;
        public static final double WHITE      = 1.00;
        public static final double OFF        = 0.00;
    }
    public Led(Servo led){
        this.led = led;
    }

    public void setColour(double colour) {
        led.setPosition(colour);
    }
}
