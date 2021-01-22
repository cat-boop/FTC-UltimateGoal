package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class AS5600 extends LinearOpMode {

    AnalogInput magneticEncoder;

    double positionMagneticEncoder;

    @Override
    public void runOpMode() {

        magneticEncoder = hardwareMap.get(AnalogInput.class, "magneticEncoder");

        waitForStart();

        while (opModeIsActive()) {
            positionMagneticEncoder = magneticEncoder.getVoltage();

            telemetry.addData("Position of magnetic encoder:", positionMagneticEncoder);
            telemetry.update();
        }
    }
}
