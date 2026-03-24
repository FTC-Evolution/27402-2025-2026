package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="But Bleu - Richel 1")
public class ButBleu1 extends BaseAutonomous {
    @Override
    public void Run() {
        drive(DRIVE_SPEED,  65.80,  65.8, 3.0);
        drive(TURN_SPEED,   24, -24, 4.0);
        drive(DRIVE_SPEED, -24, -24, 4.0);
    }
}
