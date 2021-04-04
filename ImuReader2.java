package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


public class ImuReader2 {

    private BNO055IMU imu1 = null;
    private BNO055IMU imu2 = null;

    private Orientation startAngles1 = new Orientation();
    private Orientation startAngles2 = new Orientation();

    private Position startPosition1 = new Position();
    private Position startPosition2 = new Position();

    private Position currPosition1 = new Position();
    private Position currPosition2 = new Position();

    private double globalAngle = 0;
    private double globalAngle1 = 0;
    private double globalAngle2 = 0;
    //默认
    private AxesOrder axesOrder = AxesOrder.ZYX;

    private ElapsedTime runtimeTemp = new ElapsedTime();

    public ImuReader2(){}

    public ImuReader2(BNO055IMU imu1){
        this.imu1 = imu1;
        initImu();
    }

    public void initImu() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu1.initialize(parameters);

        if(imu2!=null){
            imu2.initialize(parameters.clone());
        }

        //校准im
        runtimeTemp.reset();

        while (!imu1.isGyroCalibrated() && runtimeTemp.seconds()<2)
        {
            sleep(50);
        }
    }

    public void initImu(BNO055IMU imuSensor) {
        this.imu1 = imuSensor;
        initImu();
        resetAngle();
    }

    public void initImu(BNO055IMU imuSensor, BNO055IMU imuSensor2) {
        this.imu1 = imuSensor;
        this.imu2 = imuSensor2;
        initImu();
        resetAngle();
    }

    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        startAngles1 = imu1.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.DEGREES);
        startPosition1 = imu1.getPosition();
        startAngles2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.DEGREES);
        startPosition2 = imu2.getPosition();

        currPosition1 = new Position();
        currPosition2 = new Position();

        globalAngle = 0;
        globalAngle1 = 0;
        globalAngle2 = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu1 works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles1 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle1 = angles1.firstAngle - startAngles1.firstAngle;
        startAngles1 = angles1;
        double deltaAngle2 = angles2.firstAngle - startAngles2.firstAngle;
        startAngles2 = angles2;

        double deltaAngle = (deltaAngle1 + deltaAngle2) /2;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;

        if (deltaAngle1 < -180)
            deltaAngle1 += 360;
        else if (deltaAngle1 > 180)
            deltaAngle1 -= 360;
        globalAngle1 += deltaAngle1;

        if (deltaAngle2 < -180)
            deltaAngle2 += 360;
        else if (deltaAngle2 > 180)
            deltaAngle2 -= 360;
        globalAngle2 += deltaAngle2;

        return globalAngle;
    }

    public Position getPosition(){
        Position pos1 = getCurrPosition1();
        Position pos2 = getCurrPosition2();

        return new Position(DistanceUnit.MM,
                (pos1.x + pos2.x)/2,
                (pos1.y + pos2.y)/2,
                (pos1.z + pos2.z)/2,
                0);
    }

    public Position getCurrPosition1(){
        Position absPosition = imu1.getPosition();
        currPosition1.x = (absPosition.x - startPosition1.x)/2;
        currPosition1.y = (absPosition.y - startPosition1.y)/2;
        currPosition1.z = (absPosition.z - startPosition1.z)/2;
        currPosition1.acquisitionTime = absPosition.acquisitionTime;
        return currPosition1;
    }

    public Position getCurrPosition2(){
        Position absPosition = imu2.getPosition();
        currPosition2.x = (absPosition.x - startPosition2.x)/2;
        currPosition2.y = (absPosition.y - startPosition2.y)/2;
        currPosition2.z = (absPosition.z - startPosition2.z)/2;
        currPosition2.acquisitionTime = absPosition.acquisitionTime;
        return currPosition2;
    }
    /**
     * 检查是否直线运行。
     * 如果不是直线，那么返回一个power矫正值。
     *
     * @return Power矫正值，+ 表示向左矫正 - 表示向右矫正.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction = 0;
        double gain = .10;

        double angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }


    public double getAngle1() {
        return globalAngle1;
    }

    public double getAngle2() {
        return globalAngle2;
    }
}
