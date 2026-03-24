package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAutonomous;

@Autonomous(name="Turning test",group="Test-Autonomous")
public class DegreesTurnTest extends BaseAutonomous {

    double degreeValue = 0;
    @Override
    public void setAutonomousAsLooping() {
        super.setAutonomousAsLooping();
    }

    @Override
    public void Run() {

        if (gamepad1.xWasPressed()) {
            degreeValue -= 15;
        }

        if (gamepad1.yWasPressed()) {
            degreeValue += 15;
        }

        if (gamepad1.bWasPressed()) {
            drive(TURN_SPEED,  degreesToInches(degreeValue),  -degreesToInches(degreeValue), 5.0);
        }

        telemetryLoop();
    }
}
