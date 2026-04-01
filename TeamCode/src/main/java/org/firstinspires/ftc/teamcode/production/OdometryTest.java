package org.firstinspires.ftc.teamcode.production;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp(name = "Odometry Test", group = "Test")
public class OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            drive.update();

            Pose2d pose = drive.getPoseEstimate();

            telemetry.addData("x ticks", pose.getX());
            telemetry.addData("y ticks", pose.getY());
            telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
            telemetry.addData("x cm", (pose.getX()/1.9894));
            telemetry.addData("y cm", (pose.getY()/1.9894));
            telemetry.update();
        }
    }
}
