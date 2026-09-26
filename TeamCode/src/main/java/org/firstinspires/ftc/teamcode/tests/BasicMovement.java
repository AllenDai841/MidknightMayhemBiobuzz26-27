package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class BasicMovement extends OpMode {//using imu rn don't have any imu so this dont work
//if remove all imu code then it will run
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private IMU imu = null;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // <-- these lines might not work, revise if robot moves weird
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

    //    IMU imu = hardwareMap.get(IMU.class, "imu");
     //   IMU.Parameters parameters = new IMU.Parameters(
  //              new RevHubOrientationOnRobot(
  //                      RevHubOrientationOnRobot.LogoFacingDirection.UP,
  //                      RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
   //     imu.initialize(parameters);

        runtime.reset();
    }

    @Override
    public void loop() {

        double max;

        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x * 1.1; // "* 1.1" edit adopted from gm0.org.
        double yaw = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double frontLeftPower = (axial + lateral + yaw) / denominator;
        double frontRightPower = (axial - lateral - yaw) / denominator;
        double backLeftPower = (axial - lateral + yaw) / denominator;
        double backRightPower = (axial + lateral - yaw) / denominator;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = axial * Math.cos(-botHeading) - lateral * Math.sin(-botHeading);
        double rotY = axial * Math.sin(-botHeading) + lateral * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }
}