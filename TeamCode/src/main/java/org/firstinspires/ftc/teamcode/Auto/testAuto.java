package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "testAuto", group = "BlueSide")
public class testAuto extends LinearOpMode{
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
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {
            long t= System.currentTimeMillis();
            long end = t+2625;
            long ender = t + 3000;
            long endest = t + 6000;
            long endester = t + 6375;
            long endestest = t + 30000;
            while(System.currentTimeMillis() < end) {
                leftFrontDrive.setPower(0.25);
                leftBackDrive.setPower(0.25);
                rightFrontDrive.setPower(0.25);
                rightBackDrive.setPower(0.3);
            }
            while (System.currentTimeMillis() < ender){
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
            }
            while (System.currentTimeMillis() < endest){
                leftFrontDrive.setPower(-1 * 0.7);
                leftBackDrive.setPower(-1 * 0.7);
                rightFrontDrive.setPower(-1 * 0.7);
                rightBackDrive.setPower(-1 * 0.7);
            }
            while (System.currentTimeMillis() < endester){
                leftFrontDrive.setPower(0.25);
                leftBackDrive.setPower(0.25);
                rightFrontDrive.setPower(0.25);
                rightBackDrive.setPower(0.25);
            }
            while (System.currentTimeMillis() < endestest){
                opModeIsActive();
                leftFrontDrive.setPower(-1 * 0.2675);
                rightFrontDrive.setPower(-1 * 0.2675);
                leftBackDrive.setPower(0.2675);
                rightBackDrive.setPower(0.2675);
            }
        }

    }
}
