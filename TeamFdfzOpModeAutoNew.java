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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "Fdfz NewCrater", group = "Fdfz")
public class TeamFdfzOpModeAutoNew extends LinearOpMode {

    /* Declare OpMode members. */
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

    private BNO055IMU imu = null;
    private BNO055IMU imu2 = null;
    protected ImuReader2 imuReader = new ImuReader2();

    static final double FORWARD_SPEED = 0.3;
    static final double FORWARD_SLOW_SPEED = 0.1;
    static final double TURN_SPEED = 0.05;

    static final double COUNTS_PER_MOTOR_REV = 288;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 9 / 2.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double ROBOT_LENGTH_WHEEL = 29 / 2.54;    // 前后轮的轴距
    static final double ROBOT_WIDTH_WHEEL = 37 / 2.54;    // 左右轮的轴距
    static final double ROBOT_MOTION_CIRCUM = Math.PI * Math.sqrt(ROBOT_LENGTH_WHEEL * ROBOT_LENGTH_WHEEL + ROBOT_WIDTH_WHEEL * ROBOT_WIDTH_WHEEL);

    protected String goldMinePosition = "No";

    static final int START_FROM_CRATER = 1;
    static final int START_FROM_STORAGE = 2;

    protected int defaultStartPoint = START_FROM_CRATER;
    protected int startPoint = 0;

    protected ElapsedTime runtime = new ElapsedTime();
    protected ElapsedTime runtimeTemp = new ElapsedTime();

    MineralDetector detection = null;

    //1厘米5齿；传动2：1
    @Override
    public void runOpMode() {

        //设置摄像头位置
        initHardware();

        waitForComplete(3);
        //检测金矿
        goldMineralDectectInit();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "READY TO GO " + goldMinePosition);    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        goldMineralDectect();
        //着陆
        landing();

        // 设置默认值，如果找不到金矿则默认中间点
        if (goldMinePosition.equals("No")) {
            goldMinePosition = "Center";
        }
        // 设置默认值，如果没有图像识别定位，则使用默认点
        if (startPoint == 0) {
            startPoint = defaultStartPoint;
        }

        // 如果出发点是面向仓库
        if (startPoint == START_FROM_STORAGE) {
            storePath();
        } else {
            craterPathFull();
        }
        telemetry.update();
    }

    /**
     * 初始化硬件
     */
    private void initHardware() {

//        if (Vuforia.isInitialized()) {
//            Vuforia.deinit();
//        }

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator = hardwareMap.get(DcMotor.class, "elevator");

        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        elbow = hardwareMap.get(DcMotor.class, "elbow");

        roller = hardwareMap.get(DcMotor.class, "roller");

        camHor = hardwareMap.get(Servo.class, "camera_hor");
        camVer = hardwareMap.get(Servo.class, "camera_ver");

        // 设置imu硬件
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");

        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        roller.setDirection(DcMotor.Direction.FORWARD);

        // 调整摄像头云台水平伺服的运行区间。目前调整为0.6-1。
        // 此时，如setPosition(0)，代表实际设置值为0.6；如setPosition(0.5)，代表实际设置值为0.8；
        camHor.scaleRange(0.4,0.6);
        camHor.setPosition(0.5);

        camVer.scaleRange(0.38,0.5);
        camVer.setPosition(0);

        // 初始化imu角度读取对象
        imuReader.initImu(imu,imu2);


    }
    /**
     * 机器人着陆
     */
    private void landing() {
        // 重置角度为0
        imuReader.resetAngle();

        // 机器人下降到触地
        elevator.setPower(1);
        waitForComplete(6.8);
        elevator.setPower(0);

        // 机器人扭一下，保证脱钩
        turnAngle(8);
        goDistance(TURN_SPEED,5);

    }


    /**
     * 检测金矿
     * 左右旋转伺服来寻找金矿
     *
     */
    private void goldMineralDectectInit() {
        detection = new MineralDetector();
        detection.init(hardwareMap, telemetry);
        detection.start();
    }

    /**
     * 检测金矿
     * 左右旋转伺服来寻找金矿
     *
     */
    private void goldMineralDectect() {
        runtimeTemp.reset();
        // 循环移动云台，扫描金矿，不超过4秒
        while (opModeIsActive() && runtimeTemp.seconds() < 2) {
            // 让程序等待一会，给伺服转动留时间
            // 调用Tensorflow去做金矿识别，
            goldMinePosition = detection.searchGoldMine();
            telemetry.addData("gold dect:",goldMinePosition);
            // 如果返回的不是No，意味着识别到了，就立即退出循环
            if (!"No".equals(goldMinePosition)) {
                break;
            }
            sleep(50);
        }
        detection.stop();
    }

    /**
     * 从面向陨石坑的点出发，金矿位于中间情况下的完整执行路径
     * 此路径还未做完全测试，数值需要继续测试
     */
    private void craterPathFull() {
        double distanceFix = 0;
        goDistance(FORWARD_SPEED, 5);
        if (goldMinePosition.equals("Left")) {          // 金矿在左侧
            turnAngle(30);
            distanceFix = -5;
            goDistance(FORWARD_SPEED, 22);
            goDistance(FORWARD_SPEED,-12);
        } else if (goldMinePosition.equals("Right")) {  // 金矿在右侧
            turnAngle(-30);
            distanceFix = 10;
            goDistance(FORWARD_SPEED, 22);
            goDistance(FORWARD_SPEED,-12);
        } else {                                        // 金矿在中键
            turnAngle(0);
            goDistance(FORWARD_SPEED, 27);
            goDistance(FORWARD_SPEED,-12);
        }


//        // 转向，背向墙壁，准备倒退驶向墙壁
//        turnAngle(80);
//        // 驶向墙壁
//        goDistance(FORWARD_SPEED, 42+distanceFix);
//        // 转向，面向仓库
//        turnAngle(130);
//        // 驶向仓库
//        goDistance(FORWARD_SPEED, 25);
//        // 释放标志物
//        releaseTeamMark();
//        // 驶向陨石坑
//        goDistance(FORWARD_SPEED, -50);
//        turnAngle(135);
//        goDistance(FORWARD_SLOW_SPEED, -10);

    }

    /**
     * 从面向仓库的点出发，金矿位于中间情况下的完整执行路径
     * 向前撞击金矿
     * 然后直接释放标志物
     * 略微后退
     * 左转直接从立柱和矿之间穿过驶向敌方陨石坑
     * 此路径还未做完全测试，数值需要继续测试
     */
    private void storePath() {
        double distanceFix =0;
        goDistance(FORWARD_SPEED, 5);
        if (goldMinePosition.equals("Left")) {          // 金矿在左侧
            turnAngle(25);
            goDistance(FORWARD_SPEED, 22);
            goDistance(FORWARD_SPEED,-12);
            distanceFix = -5;
        } else if (goldMinePosition.equals("Right")) {  // 金矿在右侧
            turnAngle(-25);
            goDistance(FORWARD_SPEED, 22);
            goDistance(FORWARD_SPEED,-12);
            distanceFix = 5;
        } else {                                        // 金矿在中键
            turnAngle(0);
            goDistance(FORWARD_SPEED, 22);
            goDistance(FORWARD_SPEED,-12);
        }
//        turnAngle(80);
//        goDistance(FORWARD_SPEED, 30+distanceFix);
//        // 左转，将姿态调整为面向敌对方陨石坑
//        turnAngle(40);
//        goDistance(FORWARD_SPEED, 10);
//        turnAngle(-34);
//        goDistance(FORWARD_SPEED, 35);
//        releaseTeamMark();
//        goDistance(FORWARD_SPEED, -50);
//        turnAngle(-40);
//        // 直行到陨石坑
//        goDistance(FORWARD_SLOW_SPEED, -10);

    }


    /**
     * 机器人出发后，走到距仓库40英寸左右距离时，面向仓库执行本方法中的动作
     * 1、释放机械臂，降落到地面附近
     * 2、打开机械爪，松开标志物
     * 3、收回机械臂
     */
    private void releaseTeamMark() {

        // 伸出机械臂的方案速度太慢，放弃
        // 伸出机械臂
//        elbow.setPower(1);
//        shoulder.setPower(1);
//        waitForComplete(3);
//        elbow.setPower(0);
//        waitForComplete(2);
//        shoulder.setPower(0);

        // 旋转滚筒，释放标志物

        roller.setPower(1);
        waitForComplete(1);
        roller.setPower(0);

//        // 收回机械臂
//        shoulder.setPower(-1);
//        elbow.setPower(-1);
//        waitForComplete(3);
//        elbow.setPower(0);
//        waitForComplete(2);
//        shoulder.setPower(0);
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
        double absDistance = Math.abs(time);
        double time = (absDistance>0)?absDistance/(15*speed):0;
        // 为保证即便上述数值不准确也不会真的提前停止，额外追加0.5秒时间。
        time=Math.abs(time)+0.5;

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
        // 如果调整过程超过3秒则停止
        // 根据实践经验，旋转速度需要控制不要太高，如太高可能有旋转太快导致角度超出目标角，此时会出现来回调整的情况。
        // 另，如果想要更精确，4个驱动马达应该设置setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 这样当旋转角度达到目标角之后，马达power设置0之后，会自动锁死，防止受惯性影响多转的情况
        runtimeTemp.reset();
        // 重置IMU中的角度，会把当前方向作为0度
        if(isResetAngle)
            imuReader.resetAngle();
        while (opModeIsActive() &&
                Math.abs(degrees - currentDegrees) > 1 &&
                runtimeTemp.seconds() < 3) {
            // 根据imu角度读数和目标角数值，计算下次调整方向
            // 若degrees为正，则目标角度为左侧的某个角度。此时currentDegrees从0开始。
            //    减法结果为正，大于0，返回1，则左侧power为负，右侧power为正，机器人左转
            //    若机器人旋转超出degrees，减法结果为负，返回-1，则左侧power为正，右侧power为负，机器人右转
            // 若degrees为负，则目标角度为右侧的某个角度。此时currentDegrees从0开始。
            //    减法结果为负，小于0，返回-1，则左侧power为正，右侧power为负，机器人右转
            //    若机器人旋转超出degrees，减法结果为正，返回1，则左侧power为负，右侧power为正，机器人左转
            double direction = (degrees - currentDegrees) > 0 ? 1 : -1;
            if(Math.abs(degrees - currentDegrees) < 10){
                direction *= 0.6;
            }
            // clockwise为正则向左调整，为负则向右调整
            powerLeft(-power * direction);
            powerRight(power * direction);
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
