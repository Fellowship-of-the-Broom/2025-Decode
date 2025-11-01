package org.firstinspires.ftc.teamcode.lib.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher implements Runnable {
    private LinearOpMode opMode = null;
    private DcMotor flywheelMotor = null;
    private Servo hoodServo = null;
    private double power = 0;
    public Launcher(LinearOpMode OpMode) {
        opMode = OpMode;
        this.flywheelMotor = opMode.hardwareMap.get(DcMotor.class, "Timmy");
        this.hoodServo = opMode.hardwareMap.get(Servo.class, "Kimmy");
        //Timmy and Kimmy are placeholders

    }

    public void start() {
        Thread thread = new Thread(this);
        thread.start();
    }

    @Override
    public void run() {
        while (this.opMode.opModeIsActive()) {

            // Flywheel motor
            // Not sure about the trigger and how this is going to be used on the gamepad
            this.flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            power = this.opMode.gamepad2.right_trigger;
            this.flywheelMotor.setPower(power);

            //Hood servo

            

        }
    }
}
