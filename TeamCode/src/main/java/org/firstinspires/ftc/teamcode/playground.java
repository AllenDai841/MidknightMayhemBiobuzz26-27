package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class playground extends OpMode{
    @Override
    public void init() {
    }

    double squareInputWithSign(double input){
        double output = input * input;
        if(input<0){
            output = output * -1;
        }
        return output;
    }

    @Override
    public void loop() {
        double leftAmount = gamepad1.left_stick_x;
        double fwdAmount = -gamepad1.left_stick_y;

        telemetry.addData("Before X", leftAmount);
        telemetry.addData("Before Y", fwdAmount);

        leftAmount = squareInputWithSign(leftAmount);
        fwdAmount = squareInputWithSign(leftAmount);

    }
}
