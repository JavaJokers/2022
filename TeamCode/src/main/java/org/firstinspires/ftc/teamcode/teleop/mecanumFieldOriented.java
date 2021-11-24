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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;

import java.util.Locale;


@TeleOp(name = "MecanumFieldOriented", group = "Competition")
//@Disabled
public class mecanumFieldOriented extends LinearOpMode {

    public static Orientation angles;
    public static Acceleration gravity;

    // Declare OpMode members.
    DcMotor lF = hardwareMap.dcMotor.get("front_left");
    DcMotor lB = hardwareMap.dcMotor.get("back_left");
    DcMotor rF = hardwareMap.dcMotor.get("front_right");
    DcMotor rB = hardwareMap.dcMotor.get("back_right");
    DcMotor arm1 = hardwareMap.dcMotor.get("arm1");
    DcMotor duckies = hardwareMap.dcMotor.get("duckies");
    Servo wrist1 = hardwareMap.servo.get("wrist");
    Servo grabber = hardwareMap.servo.get("grabber");
    BNO055IMU imu; // = hardwareMap.get(BNO055IMU.class, "imu");

    public void initIMU(HardwareMap hwm) {
        imu = hwm.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters1);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    @Override
    public void runOpMode() {
        initIMU(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Set motor directions
        lF.setDirection(DcMotor.Direction.FORWARD);
        rF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;
            double duckiesPower;


            //set gamepad values
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double t = gamepad1.right_stick_x;

            //get angle


            // rotation
            double x_rotated = x * Math.cos(angles.firstAngle) - y * Math.sin(angles.firstAngle);
            double y_rotated = x * Math.sin(angles.firstAngle) + y * Math.cos(angles.firstAngle);

            // x, y, theta input mixing
            frontLeftPower = x_rotated + y_rotated + t;
            backLeftPower = x_rotated - y_rotated + t;
            frontRightPower = x_rotated - y_rotated - t;
            backRightPower = x_rotated + y_rotated - t;

            //Set carousel motor power
            if (gamepad1.a) {
                duckiesPower = 1;
            } else {
                duckiesPower = 0;
            }

            // Send calculated power to motors
            if (gamepad1.right_bumper) {
                lF.setPower(frontLeftPower * 0.25);
                rF.setPower(frontRightPower * 0.25);
                lB.setPower(backLeftPower * 0.25);
                rB.setPower(backRightPower * 0.25);
                duckies.setPower(duckiesPower * 0.25);
            } else if (gamepad1.left_bumper) {
                lF.setPower(frontLeftPower * 0.25);
                rF.setPower(frontRightPower * 0.25);
                lB.setPower(backLeftPower * 0.25);
                rB.setPower(backRightPower * 0.25);
                duckies.setPower(duckiesPower * 0.25);
            } else {
                lF.setPower(frontLeftPower);
                rF.setPower(frontRightPower);
                lB.setPower(backLeftPower);
                rB.setPower(backRightPower);
                duckies.setPower(duckiesPower);
            }

            //set grabber positions
            if (gamepad2.a) {
                grabber.setPosition(0.7);
                telemetry.addLine("position 20");
                telemetry.update();
            } else if (gamepad2.b) {
                grabber.setPosition(0.45);
                telemetry.addLine("position 30");
                telemetry.update();
            } else if (gamepad2.y) {
                grabber.setPosition(0.1);
                telemetry.addLine("position 50");
                telemetry.update();
            }

            //set wrist position
            wrist1.setPosition(gamepad2.right_trigger);

            //TODO
            //set wrist position option 2
            /*
            if(gamepad2.dpad_up){
                wrist1.setPosition(1);
            } else if(gamepad2.dpad_right){
                wrist1.setPosition(0.7);
            } else if(gamepad2.dpad_down){
                wrist1.setPosition(0.5);
            } else if(gamepad2.dpad_left){
                wrist1.setPosition(0.3);
            }
            */

            //TODO
            //set arm positions
            int ticks = 0;
            ticks += -(int) gamepad2.left_stick_y * 2;
            arm1.setPower(1);
            arm1.setTargetPosition(ticks);
            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //control carousel wheel
            if (gamepad1.x || gamepad2.x) {
                duckies.setPower(-0.5);
            } else {
                duckies.setPower(0);
            }

            // Show the wheel power.
            telemetry.addData("Motors", "carousel (%.2f)", duckiesPower);
            telemetry.update();
        }
    }

    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}