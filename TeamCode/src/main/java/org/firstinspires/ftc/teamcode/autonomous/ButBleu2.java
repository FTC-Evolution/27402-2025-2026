package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="But Bleu - Rayane")
public class ButBleu2 extends BaseAutonomous {
    @Override
    public void Run() {
        drive(DRIVE_SPEED, 54, 54, 0.0);
        drive(TURN_SPEED, 20, -20, 0.0);
        drive(DRIVE_SPEED, 30, 30, 0.0);
        drive(TURN_SPEED, 8, -8, 0.0);
        drive(DRIVE_SPEED, 19, 19, 1.5);
        drive(TURN_SPEED, 6, -6, 0.0);
        drive(DRIVE_SPEED, 44, 44, 0.0);
        drive(TURN_SPEED, 10, -10, 2.5);
        drive(DRIVE_SPEED, 70, 70, 0.0);
        drive(TURN_SPEED, 7, -7, 6.7);
    }
}
