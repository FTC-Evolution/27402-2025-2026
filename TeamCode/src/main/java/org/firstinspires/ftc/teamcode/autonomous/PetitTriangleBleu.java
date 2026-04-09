package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;
import org.firstinspires.ftc.teamcode.interfaces.Vision;

import java.util.Date;

@Autonomous(name="Petit Triangle Bleu")
public class PetitTriangleBleu extends BaseAutonomous {
    @Override
    public void Run() {
        driveCrabe(DRIVE_SPEED, 72, -72, 5.0);
        drive(TURN_SPEED, degreesToInches(45), -degreesToInches(45), 5.0);

        vision.updateGoalAprilTag();

        double target_distance = 60;

        double goal_distance;

        while (vision.absoluteFieldGoalDistance() > Vision.APRILTAG_TOLERANCE_DISTANCE + target_distance) {
            goal_distance = vision.absoluteFieldGoalDistance() - target_distance;
            drive(DRIVE_SPEED, goal_distance, goal_distance, 100.0);
        }

        shooter.modVelocity(shooter.speed(40));
        sleep(500);
        goober.modPower(1);
        sleep(850);
        goober.modPower(0);
        sleep(500);
        shooter.modVelocity(0);
    }
}
