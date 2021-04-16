package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;

import java.util.Objects;
import static org.firstinspires.ftc.teamcode.Hardware.encoders;


@TeleOp(name = "TestEncoder")

public class TestEncoder extends LinearOpMode {
    public void runOpMode(){
        Hardware robot = new Hardware(); // Use a Pushbot's hardware
        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()){

            int currentPositionX = Objects.requireNonNull(encoders.get("encoder")).getCurrentPosition();
            int currentLeftPosition = Objects.requireNonNull(encoders.get("leftEncoder")).getCurrentPosition();
            int currentRightPosition = Objects.requireNonNull(encoders.get("rightEncoder")).getCurrentPosition();

            telemetry.addData("encoderX:", currentPositionX);
            telemetry.addData("encoderYL:", currentLeftPosition);
            telemetry.addData("encoderYR:", currentRightPosition);

            telemetry.update();
        }
    }
}