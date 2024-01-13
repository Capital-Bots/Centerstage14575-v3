package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.opencv.OpenCVRed.getDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.OpenCVRed;
import org.firstinspires.ftc.teamcode.opencv.OpenCVRed.RedBlobDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Red Auto Close", group = "RedSide")
public class AutoRedClose extends LinearOpMode{
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor leftBackDrive    = null;
    public DcMotor rightBackDrive   = null;
    public DcMotor rightSlideRotate = null;
    public DcMotor leftSlideRotate = null;
    public DcMotorEx leftEncoder    = null;
    public DcMotorEx rightEncoder   = null;
    public int location = 0;
    @Override

    public void runOpMode(){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        rightSlideRotate = hardwareMap.get(DcMotor.class, "rightSlideRotate");
        leftSlideRotate = hardwareMap.get(DcMotor.class, "leftSlideRotate");

        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        while (!opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) OpenCVRed.cX + ", " + (int) OpenCVRed.cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(OpenCVRed.width)));

            if(OpenCVRed.cX < 300 && OpenCVRed.cX >= 0){
                location = 0;
            } else if(OpenCVRed.cX > 300){
                location = 1;
            } else{
                location = 2;
            }
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();



        waitForStart();



        if (opModeIsActive()) {
            if (location ==1){
                for (long stop = System.nanoTime()+ 220000000; stop>System.nanoTime();) {
                    leftFrontDrive.setPower(-0.4);
                    rightFrontDrive.setPower(-0.5);
                    leftBackDrive.setPower(-0.4);
                    rightBackDrive.setPower(-0.5);
                    rightSlideRotate.setPower(.35);
                    leftSlideRotate.setPower(.35);
                    telemetry.addData("Time: ", stop);
                    telemetry.update();
                    opModeIsActive();
                }
            }
            else{
                for (long stop = System.nanoTime()+ 1950000000; stop>System.nanoTime();) {
                    leftFrontDrive.setPower(-0.4);
                    rightFrontDrive.setPower(-0.5);
                    leftBackDrive.setPower(-0.4);
                    rightBackDrive.setPower(-0.5);
                    rightSlideRotate.setPower(.35);
                    leftSlideRotate.setPower(.35);
                    telemetry.addData("Time: ", stop);
                    telemetry.update();
                    opModeIsActive();
                }
            }


            for (long stop = System.nanoTime() + 1850000000; stop>System.nanoTime();) {
                if(location == 0) {
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(-0.5);
                    rightBackDrive.setPower(0.5);
                    leftFrontDrive.setPower(0);
                    rightSlideRotate.setPower(.35);
                    leftSlideRotate.setPower(.35);
                    telemetry.update();
                    opModeIsActive();
                }else if(location == 1){
                    break;
                }else {
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(0.5);
                    rightBackDrive.setPower(-0.5);
                    leftFrontDrive.setPower(0);
                    rightSlideRotate.setPower(.35);
                    leftSlideRotate.setPower(.35);
                    telemetry.update();
                    opModeIsActive();;
                }
            }
        }

    }
    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new RedBlobDetectionPipeline());

        controlHubCam.openCameraDevice(); //I don't know why this is deprecated
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
}
