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
 import org.firstinspires.ftc.teamcode.PIDController;
 
 import java.lang.reflect.Parameter;
 import java.util.Locale;
 
 @TeleOp(name = "mecanumOnly", group = "Competition")
 // @Disabled
 public class mecanumOnly extends LinearOpMode {
 
     public static Orientation angles;
     public static Acceleration gravity;

 
 
   
 
     BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
     BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
 
 
     @Override
     public void runOpMode() throws InterruptedException {
 
         // Declare OpMode members.
         DcMotor lF = hardwareMap.dcMotor.get("front_left");
         DcMotor lB = hardwareMap.dcMotor.get("back_left");
         DcMotor rF = hardwareMap.dcMotor.get("front_right");
         DcMotor rB = hardwareMap.dcMotor.get("back_right");
 
         imu.initialize(parameters);
 
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
         
 
        
 
 
         //variables
     
 
 
         // Wait for the game to start (driver presses PLAY)
         waitForStart();
 
         // run until the end of the match (driver presses STOP)
         while (opModeIsActive()) {
 
         
 
           
    
           
 
             // Setup a variable for each drive wheel to save power level for telemetry
             double frontLeftPower;
             double frontRightPower;
             double backLeftPower;
             double backRightPower;
 
             // set gamepad values
             double x = gamepad1.left_stick_x * 1.1;
             double y = -gamepad1.left_stick_y;
             double t = gamepad1.right_stick_x;
 
             // rotation
             double x_rotated = x * Math.cos(angles.firstAngle) - y * Math.sin(angles.firstAngle);
             double y_rotated = x * Math.sin(angles.firstAngle) + y * Math.cos(angles.firstAngle);
 
             // x, y, theta input mixing
             double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(t), 1);
             frontLeftPower = (y + x + t) / denominator;
             backLeftPower = (y - x + t) / denominator;
             frontRightPower = (y - x - t) / denominator;
             backRightPower = (y + x - t) / denominator;
 
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
 
 
             // reinitialize field oriented
             if (gamepad1.left_trigger == 1 && gamepad1.right_trigger == 1) {
                 imu.initialize(parameters);
             }
 
             
         }
 
     }
 
 }
 