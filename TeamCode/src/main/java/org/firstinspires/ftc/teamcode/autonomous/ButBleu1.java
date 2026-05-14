package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;
import org.firstinspires.ftc.teamcode.interfaces.Vision;

@Autonomous(name="But Bleu - Richel 1")
public class ButBleu1 extends BaseAutonomous {
    @Override
    public void Run() {
        brain.resetYaw();
        drive(DRIVE_SPEED,  -45,  -45, 3.0); // adjust from 51
        // drive(TURN_SPEED,   , -24, 4.0);

        alignFieldGoal(2500, 45, Vision.UpdateGoalAprilTagGoal.BLUE, 0);

        shooterTPS = shooter.speed(shooterPower);

        shooter.fireUp(shooterTPS, led);

        sleep(500);
        goober.modPower(-1);
        sleep(2500);
        goober.modPower(0);
        sleep(500);
        shooter.modVelocity(0);

        drive(DRIVE_SPEED,  20,  20, 3.0);

        rotate(2500, 35);

        // driveCrabe(DRIVE_SPEED, 32, -32, 5.0);

        // requestOpModeStop();
    }
}