package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="Avancer",group="Fallback-Autonomous")
public class FallbackAvancer extends BaseAutonomous {
    @Override
    public void Run() {
        drive(DRIVE_SPEED, 6.3, 6.3, 0.5);
    }
}
