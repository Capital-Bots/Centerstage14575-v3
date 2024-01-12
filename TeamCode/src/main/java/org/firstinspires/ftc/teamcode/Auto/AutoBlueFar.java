package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "BlueAutoFar", group = "BlueSide")
public class AutoBlueFar extends LinearOpMode{
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor leftBackDrive    = null;
    public DcMotor rightBackDrive   = null;
    public DcMotorEx leftEncoder    = null;
    public DcMotorEx rightEncoder  = null;
    @Override

    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            for (long stop = System.nanoTime()+ TimeUnit.SECONDS.toNanos(1); stop>System.nanoTime();) {
                leftFrontDrive.setPower(0.45);
                rightFrontDrive.setPower(0.5);
                leftBackDrive.setPower(0.45);
                rightBackDrive.setPower(0.5);
                telemetry.addData("Time: ", stop);
                telemetry.update();
            }
            Thread.sleep(4000);
            leftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            for (int i = 0; i < 999999999; i++){
                if (rightEncoder.getCurrentPosition() < 75000){
                    rightFrontDrive.setPower(0.5);
                    leftBackDrive.setPower(0.45);
                    rightBackDrive.setPower(0.5);
                    leftFrontDrive.setPower(0.45);
                    telemetry.addData("leftEnc", leftEncoder.getCurrentPosition());
                    telemetry.update();
                }
            }
        }

    }
}