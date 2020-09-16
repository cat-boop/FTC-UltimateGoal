package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="test", group="Linear Opmode")

public class MotorTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        DcMotor testmotor2 = null;
        DcMotor lrsmotor = null;
        testmotor2 = hardwareMap.get(DcMotor.class, "testmotor2");
        lrsmotor = hardwareMap.get(DcMotor.class, "lrsmotor");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double left, right;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;


            left   = Range.clip(drive + turn, -1.0, 1.0) ;
            right  = Range.clip(drive - turn, -1.0, 1.0) ;

            testmotor2.setPower(drive);
            lrsmotor.setPower(drive);
        }
        }

}
