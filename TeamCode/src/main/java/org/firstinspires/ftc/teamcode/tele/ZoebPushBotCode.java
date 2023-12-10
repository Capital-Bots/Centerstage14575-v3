/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareClasses.HardwareFourWheel;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Zoeb Pushbot Code", group="Linear Opmode")

public class ZoebPushBotCode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareFourWheel robot = new HardwareFourWheel();
    public final double SPEED_MULTIPLIER_DRIVE = 0.6;
    public final double SPEED_MULTIPLIER_SLIDE_ROTATION = 0.2;
    public final double STRAFING_CORRECTION = 1.05;
    double verticalComponent;
    double lateralComponent;
    double turnComponent;
    double normalizingFactor;
    double slidePositive;
    double slideNegative;
    double fl = 0;
    double fr = 0;
    double bl = 0;
    double br = 0;
    double sr = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            verticalComponent = -gamepad1.left_stick_y;
            lateralComponent = gamepad1.left_stick_x * STRAFING_CORRECTION;
            turnComponent = gamepad1.right_stick_x;
            slidePositive = gamepad1.left_trigger;
            slideNegative = gamepad1.right_trigger;

            //This one liner makes sure that the powers dont go over 1 and are in the same ratio.
            normalizingFactor = Math.max(Math.abs(verticalComponent)
                    + Math.abs(lateralComponent) + Math.abs(turnComponent), 1);

            fl = SPEED_MULTIPLIER_DRIVE * (verticalComponent + lateralComponent + turnComponent) / normalizingFactor;
            fr = SPEED_MULTIPLIER_DRIVE * (verticalComponent - lateralComponent - turnComponent) / normalizingFactor;
            bl = SPEED_MULTIPLIER_DRIVE * (verticalComponent - lateralComponent + turnComponent) / normalizingFactor;
            br = SPEED_MULTIPLIER_DRIVE * (verticalComponent + lateralComponent - turnComponent) / normalizingFactor;
            sr = SPEED_MULTIPLIER_SLIDE_ROTATION * (slidePositive - slideNegative);


            robot.leftFrontDrive.setPower(fl);
            robot.rightFrontDrive.setPower(fr);
            robot.leftBackDrive.setPower(bl);
            robot.rightBackDrive.setPower(br);
            robot.slideRotation.setPower(sr);

            telemetry.addData("Front Left Power", fl);
            telemetry.addData("Front Right Power", fr);
            telemetry.addData("Back Right Power", br);
            telemetry.addData("Back Left Power", bl);
            telemetry.update();
        }
    }
}
