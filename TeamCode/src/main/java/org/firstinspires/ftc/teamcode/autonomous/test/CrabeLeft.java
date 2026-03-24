package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="Crabe Left", group="Test-Autonomous")
public class CrabeLeft extends BaseAutonomous {
    @Override
    public void Run() {
        driveCrabe(DRIVE_SPEED, inchValue, -inchValue, 5.0);
    }
}
