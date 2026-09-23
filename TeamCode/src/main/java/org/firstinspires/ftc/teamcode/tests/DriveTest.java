package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;

public class DriveTest {
    double circ = 3.474;
    double diameter = 1.0515743172;
    double wheel_to_wheel = 16;
    double wheel_to_other_wheel = 18;
    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;

    public DriveTest(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void drive(double forward, double strafe, double turn) {
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(flPower),
                        Math.max(
                                Math.abs(frPower),
                                Math.max(Math.abs(blPower), Math.abs(brPower))
                        )
                )
        );

        fl.setPower(flPower / max);
        fr.setPower(frPower / max);
        bl.setPower(blPower / max);
        br.setPower(brPower / max);
    }
    public void four_wheel_turn(double degrees, double left_or_not) {
        double degs = ((3.1415*(wheel_to_wheel/2*Math.sqrt(2)))*360/circ)/degrees;
        double flPower = 0;
        double frPower = 0;
        double blPower = 0;
        double brPower = 0;
        if (left_or_not == 0) {
            double flPower = degs;
            double frPower = -degs;
            double blPower = degs;
            double brPower = -degs;
        }
        if (left_or_not == 1) {
            double flPower = -degs;
            double frPower = degs;
            double blPower = -degs;
            double brPower = degs;
        }

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(flPower),
                        Math.max(
                                Math.abs(frPower),
                                Math.max(Math.abs(blPower), Math.abs(brPower))
                        )
                )
        );
        fl.setPower(flPower/max);
        fr.setPower(frPower/max);
        bl.setPower(blPower/max);
        br.setPower(brPower/max);

    }
    
    public void stop() {
        drive(0, 0, 0);
    }
}