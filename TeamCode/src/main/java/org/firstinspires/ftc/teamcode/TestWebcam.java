package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name="Test de la camera USB GoBILDA")

public class TestWebcam extends LinearOpMode {


    private Boolean showCameraTelemetry;

    Map<Integer, String> obeliskPositions;
    Map<Integer, String> fieldSidePositions;
    private String obelisk;
    private String fieldSide;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    IMU imu;
    AngleUnit angle = AngleUnit.DEGREES;
    IMU.Parameters imup = new IMU.Parameters(orientationOnRobot);


    public void runOpMode(){
        telemetry.addData("status", "initialized");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imup);
        imu.resetYaw();
        initAprilTag();
        waitForStart();
        while(opModeIsActive()){
            for (AprilTagDetection detection : aprilTag.getDetections()) {
                if (detection.metadata != null) {
                    if (detection.metadata.name.contains("Obelisk")) {
                        obelisk = obeliskPositions.get(detection.id);
                    } else {
                        fieldSide = fieldSidePositions.get(detection.id);
                    }

                }

            }

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("number of apriltags", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (%d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("key:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            telemetry.addData("Last seen Pattern", obelisk);
            telemetry.addData("Last seen Target", fieldSide);

            telemetry.update();
        }
    }

    public void initAprilTag() {

        showCameraTelemetry = true;

        obeliskPositions = new HashMap<Integer, String>();
        obeliskPositions.put(21,"GPP");
        obeliskPositions.put(22,"PGP");
        obeliskPositions.put(23,"PPG");


        fieldSidePositions = new HashMap<Integer, String>();
        fieldSidePositions.put(20,"blue");
        fieldSidePositions.put(24,"red");

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        // if (USE_WEBCAM) {
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "webcam"), aprilTag);

    }
}
