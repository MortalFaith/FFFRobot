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

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.ImuReader2;

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

@Autonomous(name = "Speed Test", group = "Test")
public class TeamFdfzSpeedTestOpModeAuto extends LinearOpMode {

    /* Declare OpMode members. */
    protected DcMotor leftFront = null;
    protected DcMotor rightFront = null;
    protected DcMotor leftRear = null;
    protected DcMotor rightRear = null;
    // 1挂钩上升 -1挂钩下降
    protected DcMotor elevator = null;
    protected DcMotor elbow = null;
    protected DcMotor shoulder = null;

    // test
    private BNO055IMU imu = null;
    private BNO055IMU imu2 = null;
    protected ImuReader2 imuReader = new ImuReader2();

    static final double     LOCK_CLOSE = 1;
    static final double     CAM_SERVO_LEFT = 0;

    static final double     CLAW_CLOSE = 1;

    protected ElapsedTime runtime = new ElapsedTime();
    protected ElapsedTime runtimeTemp = new ElapsedTime();


    static final double FORWARD_SPEED = 0.5;
    static final double FORWARD_SLOW_SPEED = 0.3;
    static final double TURN_SPEED = 0.25;

    static final double COUNTS_PER_MOTOR_REV = 288;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 9 / 2.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                     (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double ROBOT_LENGTH_WHEEL = 29 / 2.54;    // 前后轮的轴距
    static final double ROBOT_WIDTH_WHEEL = 37 / 2.54;    // 左右轮的轴距
    static final double ROBOT_MOTION_CIRCUM = Math.PI * Math.sqrt(ROBOT_LENGTH_WHEEL * ROBOT_LENGTH_WHEEL + ROBOT_WIDTH_WHEEL * ROBOT_WIDTH_WHEEL);

    // 1厘米5齿；传动2：1
    @Override
    public void runOpMode() {

        // =====初始化==================== //
        //设置摄像头位置
        initHardware();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "READY TO GO ");    //
        telemetry.update();

        waitForStart();
        // =====开始执行==================== //
        runtime.reset();

        double timeFull40 = 0;
        double timeFull80 = 0;
        double timeHalf40 = 0;
        double timeHalf80 = 0;

        runtimeTemp.reset();

        telemetry.addData("[40 half]distance=",40);
        Position pos = imuReader.getPosition();
        telemetry.addData("[40 half]pos 0：","[%.2f,%.2f]",pos.x,pos.y);
        goDistance(FORWARD_SPEED, 20);
        pos = imuReader.getPosition();
        telemetry.addData("[40 half]pos 1：","[%.2f,%.2f]",pos.x,pos.y);
        goDistance(FORWARD_SPEED, -20);
        pos = imuReader.getPosition();
        timeHalf40 = runtimeTemp.milliseconds();
        telemetry.addData("[40 half]: ","[%.2f:%.2f] time=%.2f,speed=%.2f",pos.x,pos.y,timeHalf40,40/timeHalf40/2.54);

        runtimeTemp.reset();
        goDistance(FORWARD_SPEED, 40);
        goDistance(FORWARD_SPEED, -40);
        timeHalf80 = runtimeTemp.milliseconds();
        pos = imuReader.getPosition();
        telemetry.addData("[80 half]","[%.2f:%.2f]  time=%.2f,speed=%.2f",pos.x,pos.y,timeHalf80,80/timeHalf80/2.54);


//        runtimeTemp.reset();
//        goDistance(1, 20);
//        goDistance(1, -20);
//        timeFull40 = runtimeTemp.seconds();
//        pos = imuReader.getPosition();
//        telemetry.addData("[40 full]","[%.2f:%.2f] time=%.2f,speed=%.2f",pos.x,pos.y,"",timeFull40,80/timeFull40/2.54);
//
//        runtimeTemp.reset();
//        goDistance(1, 40);
//        goDistance(1, -40);
//        timeFull80 = runtimeTemp.seconds();
//        pos = imuReader.getPosition();
//        telemetry.addData("[80 full]","[%.2f:%.2f] time=%.2f,speed=%.2f",pos.x,pos.y,timeFull80,80/timeFull80/2.54);

        telemetry.update();

        while (opModeIsActive()){
            idle();
        }
    }

    /**
     * 初始化硬件
     */
    private void initHardware() {

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");

        elevator.setDirection(DcMotor.Direction.REVERSE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setDirection(DcMotor.Direction.REVERSE);

        // 设置imu硬件
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");


        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // 初始化imu角度读取对象
        imuReader.initImu(imu,imu2);

        if (Vuforia.isInitialized()) {
            Vuforia.deinit();
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current camPosition.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired camPosition
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void goDistance(double speed,
                           double distance) {
        int newLeftFrontTarget;
        int newLeftRareTarget;
        int newRightFrontTarget;
        int newRightRareTarget;

        // 用一个简单的公式来尝试根据距离来计算行进时间
        // 逻辑是按距离除以速度。速度按照最大速度为15英寸测算；即如果speed为0.5，则按15×0.5=7.5英寸/秒计算时间。
        // 这个15，在编码器正常的情况下可以不管。
        // 如果编码器不正常，需要按时间控制，可以根据具体观测值进行调整
        // 方法：写个测试程序，让机器人跑3秒，测量起止点距离，算出此值

        double time = (distance>0)?distance/(15*speed):0;
        // 为保证即便上述数值不准确也不会真的提前停止，额外追加0.5秒时间。
        time+=0.5;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //设置四个马达使用runToPosition模式运行
            runModeRunToPosition();

            // 在当前刻度的基础上，加上行进距离对应的刻度
            newLeftFrontTarget = leftFront .getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftRareTarget = leftRear.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightRareTarget = rightRear.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            // 将数值设置给马达
            leftFront.setTargetPosition(newLeftFrontTarget);
            leftRear.setTargetPosition(newLeftRareTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            rightRear.setTargetPosition(newRightRareTarget);

            // 重置计时器
            runtime.reset();
            power(Math.abs(speed));

            //四个马达中只要有还没停止的，就等待。
            //如果马达行进中，isBusy返回true；行进到指定的targetPosition之后，isBusy返回false
            while (opModeIsActive() &&
                    (leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy()) &&
                    (runtime.seconds() < time)) {
                idle();
            }

            // 停止动作
            power(0);
        }

    }


    /**
     * @param angle angle为正左转，angle为负右转
     */
    public void turnAngle(double angle) {
        // 新方法，使用imu传感器的信号进行转向
        turnByIMU(angle, TURN_SPEED,false);
    }

    /**
     * 使用IMU进行转向控制
     *
     * @param degrees 转向角度, 正数表示向左转，负数表示向右转，0不做处理
     */
    private void turnByIMU(double degrees, double power, boolean isResetAngle) {
        double currentDegrees = imuReader.getAngle();
        if(Math.abs(degrees - currentDegrees) < 1)
            return;
        // 不使用RunToPosition模式，而是使用RunUsingEncoder模式。
        // 此时给的power值，实际上代表的是速度。额定最高速度为1，如power=0.5则恒定半速巡航
        runModeUsingEncoder();

        // 根据IMU读数，进行旋转
        // 如果旋转过程中读取到的当前角度与目标角度的偏差大于1度，则继续旋转；小于等于1度则认为已经结束。
        // 此数值可以根据实践观察调整。
        //
        // 可以自动根据角度差确定向左还是向右调整
        // 如果调整过程超过5秒则停止
        // 根据实践经验，旋转速度需要控制不要太高，如太高可能有旋转太快导致角度超出目标角，此时会出现来回调整的情况。
        // 另，如果想要更精确，4个驱动马达应该设置setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 这样当旋转角度达到目标角之后，马达power设置0之后，会自动锁死，防止受惯性影响多转的情况
        runtimeTemp.reset();
        // 重置IMU中的角度，会把当前方向作为0度
        if(isResetAngle)
            imuReader.resetAngle();
        while (opModeIsActive() &&
                Math.abs(degrees - currentDegrees) > 1 &&
                runtimeTemp.seconds() < 5) {
            // 根据imu角度读数和目标角数值，计算下次调整方向
            // 若degrees为正，则目标角度为左侧的某个角度。此时currentDegrees从0开始。
            //    减法结果为正，大于0，返回1，则左侧power为负，右侧power为正，机器人左转
            //    若机器人旋转超出degrees，减法结果为负，返回-1，则左侧power为正，右侧power为负，机器人右转
            // 若degrees为负，则目标角度为右侧的某个角度。此时currentDegrees从0开始。
            //    减法结果为负，小于0，返回-1，则左侧power为正，右侧power为负，机器人右转
            //    若机器人旋转超出degrees，减法结果为正，返回1，则左侧power为负，右侧power为正，机器人左转
            double direction = (degrees - currentDegrees) > 0 ? 1 : -1;
            if(Math.abs(degrees - currentDegrees) < 6){
                direction *= 0.6;
            }
            // clockwise为正则向左调整，为负则向右调整
            powerLeft(power * direction);
            powerRight(-power * direction);
            currentDegrees = imuReader.getAngle();
            idle();

        }

        // 结束旋转，power设置为0
        power(0);

    }

    /**
     * 设置机器人使用RunToPosition模式
     */
    public void runModeRunToPosition() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void runModeUsingEncoder() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void power(double speed) {
        powerLeft(speed);
        powerRight(speed);
    }

    private void powerLeft(double speed) {
        leftFront.setPower(speed);
        leftRear.setPower(speed);
    }

    private void powerRight(double speed) {
        rightFront.setPower(speed);
        rightRear.setPower(speed);
    }


    /**
     * 等待指定的时间，期间软件系统不做其他事情
     *
     * @param second 秒数，可以是小数
     */
    private void waitForComplete(double second) {
        runtimeTemp.reset();
        while (opModeIsActive() && runtimeTemp.seconds() < second) {
            idle();
        }
    }

}
