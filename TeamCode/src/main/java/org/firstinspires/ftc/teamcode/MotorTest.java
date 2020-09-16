package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MotorTest", group="Linear Opmode")

public class MotorTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        DcMotor test_motor = null;

        test_motor = hardwareMap.get(DcMotor.class, "test_motor");


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;

            test_motor.setPower(drive);
        }
    }
}
