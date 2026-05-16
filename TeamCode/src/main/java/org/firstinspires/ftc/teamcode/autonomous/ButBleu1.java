package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;
import org.firstinspires.ftc.teamcode.interfaces.Shooter;
import org.firstinspires.ftc.teamcode.interfaces.Vision;

@Autonomous(name="But Bleu - Richel 1")
public class ButBleu1 extends BaseAutonomous {
    @Override
    public void Run() {
        brain.resetYaw();
        drive(DRIVE_SPEED,  -45,  -45, 3.0); // adjust from 51
        // drive(TURN_SPEED,   , -24, 4.0);

        alignFieldGoal(2500, 45, Vision.UpdateGoalAprilTagGoal.BLUE, 0);

        shooterTPS = shooter.speed(Shooter.DEFAULT_SHOOTER_POWER);

        shooter.fireUp(shooterTPS, led, telemetry);

        sleep(1000);
        intake(2500);
        sleep(500);
        shooter.stop();

        rotate(2500, 35);

        sleep(750);

        drive(DRIVE_SPEED, 45, 45, 5.0);

        led.TeamSpirit();

        // requestOpModeStop();
    }
}