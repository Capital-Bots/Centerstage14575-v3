package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "BlueAutoFar", group = "BlueSide")
public class AutoBlueFar extends LinearOpMode{
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor leftBackDrive    = null;
    public DcMotor rightBackDrive   = null;
    @Override

    public void runOpMode(){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            for (long stop = System.nanoTime()+ TimeUnit.SECONDS.toNanos(1); stop>System.nanoTime();) {
                leftFrontDrive.setPower(0.5);
                rightFrontDrive.setPower(0.5);
                leftBackDrive.setPower(0.5);
                rightBackDrive.setPower(0.5);
            }
        }

    }
}
