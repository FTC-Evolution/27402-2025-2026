package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="exemple")

public class Prototyp extends LinearOpMode {
    private DcMotor shooter1;
    private DcMotor shooter2;

    @Override
    public void runOpMode() {

        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");


        waitForStart();
        // run until the end of the match (driver presses STOP)
             while (opModeIsActive()) {

                shooter1.setPower(gamepad1.right_stick_y / 2);
                shooter2.setPower(-gamepad1.right_stick_y / 2);

           }
    }
        }