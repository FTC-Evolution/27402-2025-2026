package org.firstinspires.ftc.teamcode.interfaces;

import static android.os.SystemClock.sleep;

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

        public void Confetti() {
            setColour(Colour.RED);
            sleep(50);
            setColour(Colour.ORANGE);
            sleep(50);
            setColour(Colour.YELLOW);
            sleep(50);
            setColour(Colour.SAGE);
            sleep(50);
            setColour(Colour.GREEN);
            sleep(50);
            setColour(Colour.AZURE);
            sleep(50);
            setColour(Colour.BLUE);
            sleep(50);
            setColour(Colour.INDIGO);
            sleep(50);
            setColour(Colour.VIOLET);
            sleep(50);
            setColour(Colour.WHITE);
        }
        public void TeamSpirit() {
            setColour(Colour.ORANGE);
            sleep(100);
            setColour(Colour.YELLOW);
            sleep(100);
            setColour(Colour.WHITE);
            sleep(100);
            setColour(Colour.YELLOW);
            sleep(100);
            setColour(Colour.ORANGE);
            sleep(100);
            setColour(Colour.WHITE);
            sleep(100);
            setColour(Colour.ORANGE);
            sleep(100);
            setColour(Colour.YELLOW);
            sleep(100);
            setColour(Colour.WHITE);
            sleep(100);
            setColour(Colour.YELLOW);
            sleep(100);
            setColour(Colour.ORANGE);
            sleep(100);
            setColour(Colour.WHITE);
        }

    public void TeamSpiritBlitz() {
        setColour(Colour.ORANGE);
        sleep(100);
        setColour(Colour.YELLOW);
        sleep(100);
        setColour(Colour.WHITE);
        sleep(100);
        setColour(Colour.YELLOW);
        sleep(100);
        setColour(Colour.ORANGE);
        sleep(100);
        setColour(Colour.WHITE);
    }


    public void setColour(double colour) {
        led.setPosition(colour);
    }
}
