package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;
import org.firstinspires.ftc.teamcode.interfaces.Shooter;
import org.firstinspires.ftc.teamcode.interfaces.Vision;

@Autonomous(name="Petit Triangle Bleu")
public class PetitTriangleBleu extends BaseAutonomous {
    @Override
    public void Run() {
        brain.resetYaw();
        // alignFieldGoal(2500, 115, Vision.UpdateGoalAprilTagGoal.BLUE, 34.5);

        shooterTPS = shooter.speed(Shooter.DEFAULT_LONG_AUTONOMOUS_SHOOTER_POWER);

        shooter.fireUp(shooterTPS, led, telemetry);

        sleep(3000);
        intake(500);
        shooter.fireUp(shooterTPS, led, telemetry);
        sleep(1500);
        intake(700);
        shooter.fireUp(shooterTPS, led, telemetry);
        sleep(1500);
        intake(1000);
        sleep(500);
        shooter.stop();


        rotate(2500, 70);

        sleep(750);

        drive(DRIVE_SPEED, 15, 15, 5.0);

        led.TeamSpirit();

        // requestOpModeStop();
    }
}
