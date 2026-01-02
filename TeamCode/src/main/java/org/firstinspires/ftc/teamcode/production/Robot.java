package org.firstinspires.ftc.teamcode.production;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.lib.mechanisms.AutoGateState;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Chassis;
import org.firstinspires.ftc.teamcode.lib.mechanisms.FakeIntakeImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.FakeLauncherImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.FakeTransferSystemImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.lib.mechanisms.IntakeImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.lib.mechanisms.LauncherImpl;
import org.firstinspires.ftc.teamcode.lib.mechanisms.TransferSystem;
import org.firstinspires.ftc.teamcode.lib.mechanisms.TransferSystemImpl;

public class Robot implements Runnable{

    private final LinearOpMode opMode;
    private final Chassis chassis;

    private final AprilTag aprilTag;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Launcher launcher;
    private final Intake intake;
    private final TransferSystem transferSystem;
    private final AllianceColor allianceColor;
    private boolean runChassis = true;


    public Robot(LinearOpMode opMode, boolean useReal, AllianceColor allianceColor){
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        this.allianceColor = allianceColor;

        aprilTag = new AprilTag (opMode);
        chassis = new Chassis (opMode, aprilTag);

        //TODO reverse intake to correct directions (not in this file tho)
        if (useReal) {
            intake = new IntakeImpl(opMode);
            launcher = new LauncherImpl(opMode, aprilTag);
            transferSystem = new TransferSystemImpl(opMode);
        }
        else {
            intake = new FakeIntakeImpl();
            launcher = new FakeLauncherImpl();
            transferSystem = new FakeTransferSystemImpl();
        }



    }
    public void init(AllianceColor allianceColor){
        chassis.init();
        aprilTag.init(allianceColor);

    }
    public void start(){
//        launcher.start();
//        intake.start();
//       // aprilTag.start();
//        chassis.start():
        if (allianceColor == AllianceColor.RED_ALLIANCE){
            strafeMultiplier = -1;
            turnMultiplier = -1;
        }

        while (this.opMode.opModeIsActive()){
            intake.run();

            if(runChassis){
                chassis.run();
            }

            launcher.run();
            transferSystem.run();


            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                //throw new RuntimeException(e);
            }
        }


    }


    public void autoSleep(long timeMS){
        try {
            Thread.sleep(timeMS);
        } catch (InterruptedException e) {
            //throw new RuntimeException(e);
        }
    }

    double strafeMultiplier = 1;
    double turnMultiplier = 1;



    public void rollout() {

        this.chassis.moveRobot(1, 0,0);
        autoSleep(300);
        this.chassis.moveRobot(0, 0,0);
        autoSleep(200);
        this.chassis.moveRobot(0,-1 * strafeMultiplier,0);
        autoSleep(500);
    }

    public void autoFarLaunch() {

        //Defaults to blue alliance values

        //TODO Tune these values
        
        // Move forward to see april tag

        runChassis = false;

        this.chassis.moveRobot(-0.5, 0,0);
        autoSleep(1500);
        this.chassis.moveRobot(0, 0,0);
        autoSleep(500);

        //Turn to see april tag

        this.chassis.moveRobot(0,0,1 * turnMultiplier);
        autoSleep(100);

        this.chassis.moveRobot(0, 0,0);
        autoSleep(500);

        // Start Launcher

        ((LauncherImpl)launcher).autoFarLaunch = true;

        // Detect april tag & drive to it

        runChassis = true;

        aprilTag.autoAprilTagDetect = true;
        autoSleep(10000);
        aprilTag.autoAprilTagDetect = false;

        this.chassis.moveRobot(0, 0,0);
        autoSleep(500);

        //launch

        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_OPEN;
        autoSleep(250);
        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_CLOSE;
        autoSleep(1000);

        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_OPEN;
        autoSleep(250);
        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_CLOSE;
        autoSleep(1000);

        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_OPEN;
        autoSleep(250);
        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_CLOSE;
        autoSleep(1000);



        ((LauncherImpl)launcher).autoFarLaunch = false;

        // Possibly move back to start and/or out of the way
    }

    public void autoCloseLaunch() {
        
        //TODO Tune these values
        //TODO Make opMode that used this method
        
        //Slightly move away from the goal
        this.chassis.moveRobot(-0.5, 0,0);
        autoSleep(250);
        this.chassis.moveRobot(0, 0,0);
        autoSleep(500);

        // Start Launcher
        ((LauncherImpl)launcher).autoCloseLaunch = true;

        //Launch x3
        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_OPEN;
        autoSleep(250);
        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_CLOSE;
        autoSleep(1000);

        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_OPEN;
        autoSleep(250);
        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_CLOSE;
        autoSleep(1000);

        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_OPEN;
        autoSleep(250);
        ((TransferSystemImpl)transferSystem).autoGateOpen = AutoGateState.AUTO_GATE_CLOSE;
        autoSleep(1000);

        ((LauncherImpl)launcher).autoCloseLaunch = false;

    }

    @Override
    public void run() {
        start();
    }
}
