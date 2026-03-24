package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="Reculer",group="Test-Autonomous")
public class Reculer extends BaseAutonomous {
    @Override
    public void Run() {
        drive(DRIVE_SPEED,  -inchValue,  -inchValue, 5.0);
    }
}
