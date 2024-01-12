package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.opencv.OpenCVBoth.getDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.OpenCVBoth;
import org.firstinspires.ftc.teamcode.opencv.OpenCVBoth.bothBlobDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "OpenCV Both Autonomous")
public class BothTest extends LinearOpMode{
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution


    @Override
    public void runOpMode() {
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) OpenCVBoth.cX + ", " + (int) OpenCVBoth.cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(OpenCVBoth.width)));

            if(OpenCVBoth.cX < 320 && OpenCVBoth.cX >= 0){
                telemetry.addData("Location:", "Left");
            } else if(OpenCVBoth.cX > 320){
                telemetry.addData("Location:", "Middle");
            } else{
                telemetry.addData("Location:", "Right");
            }
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new bothBlobDetectionPipeline());

        controlHubCam.openCameraDevice(); //I don't know why this is deprecated
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

}
