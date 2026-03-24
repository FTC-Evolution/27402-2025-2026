package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="Petit Triangle Bleu")
public class PetitTriangleBleu extends BaseAutonomous {
    @Override
    public void Run() {
        driveCrabe(DRIVE_SPEED, -72, 72, 5.0);
        drive(TURN_SPEED, degreesToInches(45), -degreesToInches(45), 5.0);
        modShooter(40);
    }
}
