package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class ButtonTest extends LinearOpMode {
    // Define variables for our potentiometer and motor
    TouchSensor button;

    // Define variable for the current voltage
    boolean currentVoltage;

    @Override
    public void runOpMode() {
        // Get the potentiometer and motor from hardwareMap
        button = hardwareMap.get(TouchSensor.class, "button");

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // Update currentVoltage from the potentiometer
            currentVoltage = button.isPressed();

            // Show the potentiometer’s voltage in telemetry
            telemetry.addData("Potentiometer voltage", currentVoltage);
            telemetry.update();
        }
    }
}