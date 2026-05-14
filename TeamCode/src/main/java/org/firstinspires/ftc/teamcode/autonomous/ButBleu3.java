package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="But Bleu - Richel 2")
@Disabled
public class ButBleu3 extends BaseAutonomous {
    @Override
    public void Run() {
        drive(DRIVE_SPEED, 77.5, 77.5, 0.5);
        drive(TURN_SPEED, 12, -12, 0.5);
    }
}
