package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="exemple")

public class Prototyp extends LinearOpMode {
    private DcMotor shooter1;
    private DcMotor shooter2;

    private Integer divider = 1;

    @Override
    public void runOpMode() {

        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");


        waitForStart();
        // run until the end of the match (driver presses STOP)
             while (opModeIsActive()) {

                //shooter1.setPower(gamepad1.right_stick_y / 2);
                //shooter2.setPower(-gamepad1.right_stick_y / 2);

                 shooter1.setPower(gamepad1.right_stick_y / divider);
                 shooter2.setPower(-gamepad1.right_stick_y / divider);

                 if (gamepad1.x) {
                     divider = 1;
                 }
                 if (gamepad1.y) {
                     divider = 2;
                 }
                 if (gamepad1.b) {
                     divider = 3;
                 }
                 if (gamepad1.a) {
                    divider = 4;
                 }

           }
    }
        }