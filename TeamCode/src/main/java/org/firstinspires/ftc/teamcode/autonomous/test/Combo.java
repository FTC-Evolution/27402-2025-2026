package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="Test Combo",group="Test-Autonomous")
public class Combo extends BaseAutonomous {
    @Override
    public void Run() {
        drive(DRIVE_SPEED, 48, 48, 5.0);
        drive(TURN_SPEED, 12, -12, 4.0);
        drive(DRIVE_SPEED, -24, -24, 4.0);
    }
}
