package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.RobotTeleopMecanumFieldRelativeDrive;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Prototype des Sec 2 27402")

public class Prototyp extends LinearOpMode {
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor goober;
    private Servo shooteraim;

    private ElapsedTime runtime = new ElapsedTime();

    private Integer divider = 1;

    double counter1 = 0;
    double counter2 = 0;
    boolean end = false;
    String winner = "none";

    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
    AngleUnit angle = AngleUnit.DEGREES;
    IMU.Parameters imup = new IMU.Parameters(orientationOnRobot);




    @Override
    public void runOpMode() {

        // InitDrive();
        // shooterInit();
        // aimInit();
        // buttoninnit();
        gooberInit();
        telemetryInit();
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            // buttonLoop();
            //     boucleDrive();
            //   aimLoop();
            // shooterLoop();
            gooberLoop();
            telemetryLoop();
        }

    }

    public void shooterInit() {
        shooter1 = hardwareMap.get(DcMotor.class, "leftshooter");
        shooter2 = hardwareMap.get(DcMotor.class, "rightshooter");
    }

    public void aimInit() {
        shooteraim = hardwareMap.get(Servo.class, "servo0");
    }

    public void aimLoop() {
        shooteraim.setPosition(gamepad2.left_stick_y);
    }

    public void gooberInit(){
        goober = hardwareMap.get(DcMotor.class, "goober");

    }

    public void gooberLoop(){
        if (gamepad2.rightBumperWasPressed()) {
            goober.setPower(-1);
        } else if (gamepad2.leftBumperWasPressed()) {
            goober.setPower(1);
        } else {
            goober.setPower(0);
        }

    }
    public void shooterLoop() {

        shooter1.setPower(gamepad2.right_stick_y / divider);
        shooter2.setPower(-gamepad2.right_stick_y / divider);

        if (gamepad2.x) {
            divider = 1;
        }
        if (gamepad2.y) {
            divider = 2;
        }
        if (gamepad2.b) {
            divider = 3;
        }
        if (gamepad2.a) {
            divider = 4;
        }
    }





    public void InitDrive() {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftDrive = hardwareMap.dcMotor.get("motor0");
        backLeftDrive = hardwareMap.dcMotor.get("motor2");
        frontRightDrive = hardwareMap.dcMotor.get("motor1");
        backRightDrive = hardwareMap.dcMotor.get("motor3");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void boucleDrive() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }
    public void buttoninnit() {


    }

    public void buttonLoop() {


        // telemetry.addLine("get 50 to win");

        if (end)
        {
            return;
        }
        if (gamepad1.xWasPressed()) {
            counter1 +=1;
        }

        if (gamepad2.xWasPressed()) {
            counter2 +=1;
        }


        else if (counter1 >= 50) {
            winner = "player 1";
            end = true;

        }
        else if (counter2 >= 50) {
            winner = "player 2";
            end = true;
        }

        telemetry.addData("player 1 score", counter1);
        telemetry.addData("player 2 score", counter2);
        telemetry.addData("winner", winner);
        telemetry.addData("game over", end);

        telemetry.update();
    }
    public void telemetryInit(){
        telemetry.addData("status", "initialized");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imup);
        imu.resetYaw();
    }

    public void telemetryLoop(){

        telemetry.addData("status", "runtime: " + runtime.toString());

        telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(angle));
        telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(angle));
        telemetry.addData("roll", imu.getRobotYawPitchRollAngles().getRoll(angle));

        telemetry.addData("angular velocity x", imu.getRobotAngularVelocity(angle).xRotationRate);
        telemetry.addData("angular velocity y", imu.getRobotAngularVelocity(angle).yRotationRate);
        telemetry.addData("angular velocity z", imu.getRobotAngularVelocity(angle).zRotationRate);


        telemetry.update();
    }

}