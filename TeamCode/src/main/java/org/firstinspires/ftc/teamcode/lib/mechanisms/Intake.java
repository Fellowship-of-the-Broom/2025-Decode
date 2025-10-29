package org.firstinspires.ftc.teamcode.lib.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    private LinearOpMode opMode = null;
    private DcMotor motor = null;
    public Intake(LinearOpMode OpMode) {
        opMode = OpMode;
        this.motor = opMode.hardwareMap.get(DcMotor.class, "Jimy");

    }
}
