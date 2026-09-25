package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Motor;

@Configurable
public class motorTest extends OpMode {
    Motor motor;
    @Override
    public void init() {
        motor = new Motor(hardwareMap.get(DcMotorEx.class, "motor"), 28);
    }

    @Override
    public void loop() {
        motor.setRPM(300);
    }
}
