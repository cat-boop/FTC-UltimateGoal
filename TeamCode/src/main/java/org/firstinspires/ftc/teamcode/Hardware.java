package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    private DcMotor leftFront  = null;
    private DcMotor leftRear   = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear  = null;

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);;
    }

    public void setPower(double move, double turn, double sideways) {
        double[] powers = new double[4];
        powers[0] = -move + turn + sideways;
        powers[1] = -move + turn - sideways;
        powers[2] = -move - turn - sideways;
        powers[3] = -move - turn + sideways;

        powers = normalize(powers);

        leftFront.setPower(powers[0]);
        leftRear.setPower(powers[1]);
        rightFront.setPower(powers[2]);
        rightRear.setPower(powers[3]);
    }

    private double[] normalize(double[] powers) {
        double max_value = Math.abs(powers[0]);
        for (double power : powers) {
            max_value = Math.max(max_value, power);
        }

        if (max_value > 1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max_value;
            }
        }
        return powers;
    }
}
