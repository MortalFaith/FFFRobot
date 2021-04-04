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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "ManualFdfz", group = "Fdfz")
public class TeamFdfzOpModeManual extends LinearOpMode {


    /*底盘马达*/
    protected DcMotor leftFront = null;
    protected DcMotor rightFront = null;
    protected DcMotor leftRear = null;
    protected DcMotor rightRear = null;
    // 1挂钩上升 -1挂钩下降
    protected DcMotor elevator = null;
    protected DcMotor elbow = null;
    protected DcMotor shoulder = null;
    protected DcMotor roller = null;

    protected Servo camHor = null;
    protected Servo camVer = null;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime speedTimer = new ElapsedTime();
    private ElapsedTime lockTimer = new ElapsedTime();

    static final double     CLAQ_OPEN = 0;
    static final double     CLAQ_CLOSE = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // =====初始化阶段============================= 不用改，调0，给他名字//


        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        //rear：后面
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        roller = hardwareMap.get(DcMotor.class, "roller");

        elevator.setDirection(DcMotor.Direction.REVERSE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // test
        camHor = hardwareMap.get(Servo.class, "camera_hor");
        camVer = hardwareMap.get(Servo.class, "camera_ver");

        // 底盘电机方向设置
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // 底盘电机运行模式设置
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 机械臂肩部电机
        shoulder.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 机械臂肘部电机
        elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        camHor.scaleRange(0.4,0.6);
        camHor.setPosition(0.5);

        camVer.scaleRange(0.35,0.5);
        camVer.setPosition(0.5);

        //
        telemetry.update();

        boolean driveFullPower = true;

        waitForStart();
        // =====启动自动阶段============================= //
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // 定义变量
            double leftPower;
            double rightPower;
            double elevatePower = 0.0;
            double shoulderPower = 0.0;
            double elbowPower = 0.0;
            double rollerPower = 0.0;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // 手柄1
            // left_stick_y：前后运动
            // right_stick_x：左右运动
            //读入~~算出power
            double drive = -gamepad1.left_stick_y * 0.6;
            double turn  =  gamepad1.left_stick_x * 0.2;

            leftPower = Range.clip( (drive + turn), -1.0, 1.0) ;
            rightPower = Range.clip((drive - turn), -1.0, 1.0) ;

            // 手柄1
            // x：切换速度；轮换切换快速或者慢速；快速为全速，慢速为0.3倍速度用于微操。
            //左右、前后平移，旋转
            if(gamepad1.x && speedTimer.seconds()>0.5){
                driveFullPower = !driveFullPower;
                speedTimer.reset();
            }
            // 计算速度
            if(!driveFullPower){
                leftPower *= 0.3;
                rightPower *= 0.3;
            }
            // 手柄1
            // y：升降机器人
            if (gamepad1.y){
                elevatePower = 1;
            } else if (gamepad1.a){
                elevatePower = -1;
            }
            // 手柄1
            // dpad_left：辅助功能，摄像头水平旋转
            // dpad_right：辅助功能，摄像头水平旋转
            if (gamepad1.dpad_left){
                camHor.setPosition(camHor.getPosition()-0.1);
            } else if (gamepad1.dpad_right){
                camHor.setPosition(camHor.getPosition()+0.1);
            }
            // 手柄1
            // dpad_up：辅助功能，摄像头垂直旋转
            // dpad_up：辅助功能，摄像头垂直旋转
            if (gamepad1.dpad_up){
                camVer.setPosition(1);
            } else if (gamepad1.dpad_down){
                camVer.setPosition(0);
            }

            // 手柄2
            // left_stick_y：机械臂，肩部电机，控制上臂推出或者收回。上推则机械臂推出，下拉则机械臂收回。
            shoulderPower = gamepad2.left_stick_y;

            // 手柄2
            // right_stick_y：机械臂，肘部电机，控制小臂抬起或者落下。上推则机械臂抬起，下拉则机械臂落下。
            elbowPower = gamepad2.right_stick_y;

            // 手柄2
            // left_bumper：控制收球滚筒旋转。
            // right_bumper：控制收球滚筒反向旋转。
            if(gamepad2.left_bumper ){
                rollerPower = 1;
            }
            if(gamepad2.right_bumper){
                rollerPower = -1;
            }
            //给地盘电机供能
            leftFront.setPower(leftPower);
            leftRear.setPower(leftPower);

            rightFront.setPower(rightPower);
            rightRear.setPower(rightPower);

            elevator.setPower(elevatePower);

            shoulder.setPower(shoulderPower);
            elbow.setPower(elbowPower);
            roller.setPower(rollerPower);


            //
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), elevate (%.2f), "
                    , leftPower, rightPower, elevatePower);
            telemetry.update();
        }
    }
}
