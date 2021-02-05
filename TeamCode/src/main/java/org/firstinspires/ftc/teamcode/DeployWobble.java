package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class DeployWobble extends LinearOpMode {
    public void runOpMode() {

        DcMotor wobble = null;
        wobble = hardwareMap.get(DcMotor.class, "wobble");

        double EP = 0;
        double K = 0;

        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobble.setTargetPosition(-80);
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //wobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        while (opModeIsActive()) {
            //tick -248

            if (gamepad1.right_bumper) K += 0.1;
            else if (gamepad1.left_bumper) K -= 0.1;

            EP = wobble.getCurrentPosition() - 58;

            wobble.setPower(EP * K);

            telemetry.addData("K:", K);
            telemetry.addData("tick", wobble.getCurrentPosition());
            telemetry.update();
        }
    }
}

