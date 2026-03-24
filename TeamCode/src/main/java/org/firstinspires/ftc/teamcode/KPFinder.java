package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name="KP Finder", group="Utility")

public class KPFinder extends LinearOpMode {
    boolean showImuTelemetry = false;
    boolean showShooterTelemetry = false;
    boolean showColorTelemetry = false;
    boolean showGooberTelemetry = false;
    boolean showCameraTelemetry = false;

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    final double SHOOTER_TICKS_PER_REV = 28;


    private DcMotor goober;
    private DcMotor goober2;
    private NormalizedColorSensor sensor;

    private final ElapsedTime runtime = new ElapsedTime();

    private double shooterPower = 0; // Perfect speed to throw the ball when at the point of the launch triangle on the field
    double shooterTPS = shooterPower * SHOOTER_TICKS_PER_REV;


    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


    // This declares the IMU needed to get the current direction the robot is faci ng
    IMU imu;
    AngleUnit angle = AngleUnit.DEGREES;
    IMU.Parameters imup = new IMU.Parameters(orientationOnRobot);

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_DISTANCE_INCHES   = 12.25;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     INCHES_PER_DEGREE       = (WHEEL_DISTANCE_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    double     SHOOTER_P              = 90;
    double     SHOOTER_I              = 1.95;
    double     SHOOTER_D              = 0.1;
    // 70, 2.05, 5 (Rough, might go down by 20 or up by 20 regularly)
    // 130, 1, 2 (Idk)
    // 250, 2, 2 (Oscillates a bit
    // 90, 1.95, 0.1 (Osciallates basically none, stays steady and goes up a little or down a little very sparsely
     double     SHOOTER_F              = 0;



    @Override
    public void runOpMode() {

        //InitDrive();
        shooterInit();
        //gooberInit();
        // colorSensorInit();
        telemetryInit();
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // buttonLoop();
            //boucleDrive();
            shooterLoop();
            // colorSensorLoop();
            kpFindLoop();
            telemetryLoop();
        }

    }

    public void shooterInit() {
        showShooterTelemetry = true;
        shooter1 = hardwareMap.get(DcMotorEx.class, "leftshooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "rightshooter");

        shooter1.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooter2.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        // Reset encoders
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gooberInit(){
        showGooberTelemetry = true;
        goober = hardwareMap.get(DcMotorEx.class, "goober");
        goober2 = hardwareMap.get(DcMotorEx.class, "goober2");
    }
    public void updatePID() {
        shooter1.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooter2.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);


    }

    public void updateVelocity() {
        shooterTPS = shooterPower * SHOOTER_TICKS_PER_REV;
        // double shooterMulti = gamepad2.right_trigger * shooterTPS;

        shooter1.setVelocity(-shooterTPS);
        shooter2.setVelocity(shooterTPS);
    }

    public void kpFindLoop() {
        if (gamepad1.bWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                SHOOTER_P += 50;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                SHOOTER_P += 1;
                updatePID();
                return;
            }
            SHOOTER_P += 10;
            updatePID();
        };
        if (gamepad1.aWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                SHOOTER_P -= 50;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                SHOOTER_P -= 1;
                updatePID();
                return;
            }
            SHOOTER_P -= 10;
            updatePID();
        };
        if (gamepad1.yWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                SHOOTER_D += 1;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                SHOOTER_D += 0.01;
                updatePID();
                return;
            }
            SHOOTER_D += 0.05;
            updatePID();
        };
        if (gamepad1.xWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                SHOOTER_D -= 1;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                SHOOTER_D -= 0.01;
                updatePID();
                return;
            }
            SHOOTER_D -= 0.05;
            updatePID();
        };
        if (gamepad1.dpadUpWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                SHOOTER_I += 1;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                SHOOTER_I += 0.01;
                updatePID();
                return;
            }
            SHOOTER_I += 0.05;
        };
        if (gamepad1.dpadDownWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                SHOOTER_I -= 1;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                SHOOTER_I -= 0.01;
                updatePID();
                return;
            }
            SHOOTER_I -= 0.05;
            updatePID();
        };
    }
    public void shooterLoop() {



        if (gamepad1.dpadLeftWasPressed()) {
            if (shooterPower <= 0){ return; }
            shooterPower -= 5;
            updateVelocity();
        }
        /*if (gamepad2.yWasPressed()) {
            if (shooterPower >= 100){ return; }
            if (shooterPower == 95) {
                shooterPower += 5;
                return;
            }
            shooterPower += 10;
        }*/
        if (gamepad1.dpadRightWasPressed()) {
            if (shooterPower >= 100){ return; }
            shooterPower += 5;
            updateVelocity();
        }
        /*if (gamepad2.aWasPressed()) {
            if (shooterPower <= 0){ return; }
            if (shooterPower <= 5) {
                shooterPower -= 5;
            }
            shooterPower -= 10;
        }*/
        if (gamepad1.rightBumperWasPressed()) {
            shooterPower = 0;
            updateVelocity();
        }
    }


    public void telemetryInit(){
        telemetry.addData("status", "initialized");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imup);
        imu.resetYaw();
    }

    public void telemetryLoop(){

        telemetry.addData("status", "runtime: " + runtime);



        if (showImuTelemetry) {
            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(angle));
            telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(angle));
            telemetry.addData("roll", imu.getRobotYawPitchRollAngles().getRoll(angle));
        }

        telemetry.addLine("Increase P until oscillations occur");
        telemetry.addLine("Increase D until no overshoot occurs");
        telemetry.addLine("If there is steady state error, increase I until corrected");

        telemetry.addData("B","P UP");
        telemetry.addData("A","P DOWN");
        telemetry.addData("Y","D UP");
        telemetry.addData("X","D DOWN");
        telemetry.addData("DpadUP","I UP");
        telemetry.addData("DpadDown","I DOWN");

        if (showShooterTelemetry) {

            telemetry.addData("Shooter P", SHOOTER_P);
            telemetry.addData("Shooter I", SHOOTER_I);
            telemetry.addData("Shooter D", SHOOTER_D);
            telemetry.addData("Shooter F", SHOOTER_F);
            telemetry.addData("shooter Power variable", shooterPower);
            telemetry.addData("shooter TPS (Supposed velocity)", shooterPower*SHOOTER_TICKS_PER_REV);
            telemetry.addData("left Shooter Velocity", shooter1.getVelocity());
            telemetry.addData("right Shooter Velocity", shooter2.getVelocity());
        }

        telemetry.update();
    }
}




//get a job./