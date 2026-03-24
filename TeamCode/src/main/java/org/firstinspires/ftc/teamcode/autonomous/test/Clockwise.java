package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="Clockwise", group = "Test-Autonomous")
public class Clockwise extends BaseAutonomous {
    @Override
    public void Run() {
        drive(TURN_SPEED,  inchValue,  -inchValue, 5.0);
    }
}
