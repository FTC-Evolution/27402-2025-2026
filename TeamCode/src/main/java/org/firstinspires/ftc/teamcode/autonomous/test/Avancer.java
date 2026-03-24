package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="Avancer", group="Test-Autonomous")
public class Avancer extends BaseAutonomous {
    @Override
    public void Run() {
        drive(DRIVE_SPEED,  inchValue,  inchValue, 5.0);
    }
}
