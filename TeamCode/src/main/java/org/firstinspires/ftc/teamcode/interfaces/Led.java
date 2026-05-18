package org.firstinspires.ftc.teamcode.interfaces;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Led {
    private final Servo led;

    private final ElapsedTime confettiTimer = new ElapsedTime();
    private int currentColourIndex = 0;
    private boolean isConfettiRunning = false;

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

    private final double[] confettiColours = {
            Colour.RED, Colour.ORANGE, Colour.YELLOW, Colour.SAGE,
            Colour.GREEN, Colour.AZURE, Colour.BLUE, Colour.INDIGO,
            Colour.VIOLET, Colour.WHITE
    };

    public void startConfetti() {
        if (!isConfettiRunning) {
            isConfettiRunning = true;
            currentColourIndex = 0;
            confettiTimer.reset();
            setColour(confettiColours[currentColourIndex]);
        }
    }

    public void updateConfetti(double wait_ms) {
        if (!isConfettiRunning) return;

        if (confettiTimer.milliseconds() >= wait_ms) {
            currentColourIndex++;

            if (currentColourIndex >= confettiColours.length) {
                isConfettiRunning = false;
                setColour(Colour.WHITE);
            } else {
                setColour(confettiColours[currentColourIndex]);
                confettiTimer.reset();
            }
        }
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
