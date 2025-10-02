package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="exemple")

public class Exemple extends LinearOpMode {
    private DcMotor shooter1;
    private DcMotor shooter2;

    private Servo pince;
    private DcMotor intake;
    private Double forceshooter=0.6;

    @Override
    public void runOpMode() {

        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        // intake = hardwareMap.get(DcMotor.class, "intake");
        // pince = hardwareMap.get(Servo.class, "pince");
        waitForStart();
        // run until the end of the match (driver presses STOP)
             while (opModeIsActive()) {

            if (gamepad1.left_trigger > 0.3) {
                shooter1.setPower(forceshooter);
                shooter2.setPower(-forceshooter);
            } else if (gamepad1.right_trigger > 0.3) {
                shooter1.setPower(-forceshooter);
                shooter2.setPower(forceshooter);
            } else {
                shooter1.setPower(0);
                shooter2.setPower(0);
            }

            if (gamepad1.y) {
                if (forceshooter == 1) {
                    return;
                }
                forceshooter += 0.1;
            }

            if (gamepad1.a) {
                if (forceshooter == 0) {
                    return;
                }
                forceshooter -= 0.1;
            }

       // intake.setPower(gamepad1.right_trigger);
            // intake.setPower(-gamepad1.left_trigger);
            // pince.setPosition(gamepad1.left_stick_x);
        }
    }
        }