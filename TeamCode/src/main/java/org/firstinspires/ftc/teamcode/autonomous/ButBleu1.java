package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="But Bleu - Richel 1")
public class ButBleu1 extends BaseAutonomous {
    @Override
    public void Run() {
        drive(DRIVE_SPEED,  -45,  -45, 3.0); // adjust from 51
        // drive(TURN_SPEED,   , -24, 4.0);

        vision.updateGoalAprilTag();

        double target_distance = 45;

        // alignAprilTag(target_distance);

        shooterTPS = shooter.speed(shooterPower);

        shooter.fireUp(shooterTPS, led);

        sleep(500);
        goober.modPower(-1);
        sleep(1500);
        goober.modPower(0);
        sleep(500);
        shooter.modVelocity(0);
    }
}
