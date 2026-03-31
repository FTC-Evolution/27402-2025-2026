package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

public class Brain extends LinearOpMode {
    public static IMU imu;
    public static void Init(){
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters params = new IMU.Parameters(orientationOnRobot);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);
        imu.resetYaw();
    }

    public void runOpMode() {}
}
