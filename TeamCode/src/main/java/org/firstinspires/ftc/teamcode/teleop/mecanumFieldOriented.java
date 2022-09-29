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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.reflect.Parameter;
import java.util.Locale;




@TeleOp(name = "mecanumFieldOriented", group = "Competition")
//@Disabled
public class mecanumFieldOriented extends LinearOpMode {

    public static Orientation angles;
    public static Acceleration gravity;
    int ticksLift = 0;

    int gridX = 1;
    int gridY = 1;
    char gridX_Converted = "A";
    
    //dpad vars
    private boolean isDpadLeft = false;
    private boolean isDpadRight = false;
    private boolean isDpadUp = false;
    private boolean isDpadDown = false;

    private boolean wasDpadLeft = false;
    private boolean wasDpadRight = false;
    private boolean wasDpadUp = false;
    private boolean wasDpadDown = false;

    

    BNO055IMU imu;

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

        // Declare OpMode members.
        DcMotor lF = hardwareMap.dcMotor.get("front_left");
        DcMotor lB = hardwareMap.dcMotor.get("back_left");
        DcMotor rF = hardwareMap.dcMotor.get("front_right");
        DcMotor rB = hardwareMap.dcMotor.get("back_right");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        CRServo intake = hardwareMap.crservo.get("intake");
        DcMotor dr4b = hardwareMap.dcMotor.get("dr4b");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");


        initIMU(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Set motor directions
        lF.setDirection(DcMotor.Direction.FORWARD);
        rF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        upDown0.setDirection(DcMotor.Direction.FORWARD);
        upDown1.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upDown0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upDown1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;


            //set gamepad values
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double t = gamepad1.right_stick_x;


            // rotation
            double x_rotated = x * Math.cos(angles.firstAngle) - y * Math.sin(angles.firstAngle);
            double y_rotated = x * Math.sin(angles.firstAngle) + y * Math.cos(angles.firstAngle);

            // x, y, theta input mixing
            frontLeftPower = x_rotated + y_rotated + t;
            backLeftPower = x_rotated - y_rotated + t;
            frontRightPower = x_rotated - y_rotated - t;
            backRightPower = x_rotated + y_rotated - t;


            // Send calculated power to motors
            if (gamepad1.right_bumper) {
                lF.setPower(frontLeftPower * 0.75);
                rF.setPower(frontRightPower * 0.75);
                lB.setPower(backLeftPower * 0.75);
                rB.setPower(backRightPower * 0.75);

            } else if (gamepad1.left_bumper) {
                lF.setPower(frontLeftPower * 0.25);
                rF.setPower(frontRightPower * 0.25);
                lB.setPower(backLeftPower * 0.25);
                rB.setPower(backRightPower * 0.25);

            } else {
                lF.setPower(frontLeftPower);
                rF.setPower(frontRightPower);
                lB.setPower(backLeftPower);
                rB.setPower(backRightPower);
            }

            //reinitialize field oriented
            if (gamepad1.a && gamepad1.x) {
                initIMU(hardwareMap);
            }

            //grid control
            if((isDpadLeft = gamepad1.dpad_left) && !wasDpadLeft){
                gridX --;
            }

            if((isDpadRight = gamepad1.dpad_right) && !wasDpadRight){
                gridX ++;
            }

            if((isDpadUp = gamepad1.dpad_up) && !wasDpadUp){
                gridY --;
            }

            if((isDpadDown = gamepad1.dpad_down) && !wasDpadDown){
                gridY ++;
            }


            //limit grid values
            if(gridX < 1){
                gridX = 1;
            } else if(gridX > 5){
                gridX = 5;
            }

            if(gridY < 1){
                gridY = 1;
            }else if(gridY > 5){
                gridY = 5;
            }
            
            //gridX numbers --> letters
            switch(gridX){
            case 1:  gridX_Converted = "A";
                     break;
            case 2:  gridX_Converted = "B";
                     break;
            case 3:  gridX_Converted = "C";
                     break;
            case 4:  gridX_Converted = "D";
                     break;
            case 5:  gridX_Converted = "E";
                     break;
            default: gridX_Converted = "0";
                     break;
            }
            //linear slide
            if(gamepad2.dpad_down){
                slide.setPower(0.4);
            }else if(gamepad2.dpad_up){
                slide.setPower(-0.4);
            }else{
                slide.setPower(0);
            }


            

            //intake
            if(gamepad1.right_trigger == 1 || gamepad2.right_trigger == 1){
                intake.setPower(0.6);
            }else if(gamepad1.left_trigger == 1 || gamepad2.left_trigger == 1){
                intake.setPower(-0.6);
            }else{
                intake.setPower(0);
            }




            //lift control
            ticksLift = ticksLift - (-(int) gamepad2.left_stick_y * 2);
            dr4b.setTargetPosition(ticksLift);
            dr4b.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Grid X", gridX_Converted);
            telemetry.addData("Grid Y", gridY);
            telemetry.addData("Lift Position", ticksLift);
            telemetry.update();

            wasDpadLeft = isDpadLeft;
            wasDpadRight = isDpadRight;
            wasDpadUp = isDpadUp;
            wasDpadDown = isDpadDown;
        }
        
    }

    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
} 
