package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("fl");
                c.backLeftName.set("bl");
                c.frontRightName.set("fr");
                c.backRightName.set("br");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static MecanumConfig driveConfig = drivetrainConfig;

    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(
            c -> {
                c.xPodName.set("xPod");
                c.yPodName.set("yPod");
                c.imuName.set("imu");
                c.xPodOffset.set(0.0);
                c.yPodOffset.set(0.0);
                c.forwardTicksToInches.set(Math.PI * 32 / 25.4 / 2000);
                c.strafeTicksToInches.set(Math.PI * 32 / 25.4 / 2000);
                c.xPodDirection.set(Encoder.FORWARD);
                c.yPodDirection.set(Encoder.FORWARD);
                c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                c.forwardTranslational.set(Controller.proportional(0.1));
                c.strafeTranslational.set(Controller.proportional(0.1));
                c.coast.set(Controller.proportionalFeedforward(0.1));
                c.brake.set(Controller.proportionalFeedforward(0.1));
                c.headingFeedback.set(Controller.proportional(1.0));

                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));
                c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0, 0.0));

                c.maxAchievableForwardVelocity.set(30.0);
                c.maxAchievableStrafeVelocity.set(30.0);
                c.naturalForwardDeceleration.set(30.0);
                c.naturalStrafeDeceleration.set(30.0);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new TwoWheelLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
