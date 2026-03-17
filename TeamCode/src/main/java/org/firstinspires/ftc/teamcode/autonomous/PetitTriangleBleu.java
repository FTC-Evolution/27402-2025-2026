package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Petit Triangle Bleu")
public class PetitTriangleBleu extends BaseAutonomous {
    @Override
    public void runAutonomous() {
        boucleDriveCrabe(DRIVE_SPEED, -72, 72, 5.0);
        boucleDrive(TURN_SPEED, degreesToInches(45), -degreesToInches(45), 5.0);
        boucleShoot(40, 5.0);
    }
}
