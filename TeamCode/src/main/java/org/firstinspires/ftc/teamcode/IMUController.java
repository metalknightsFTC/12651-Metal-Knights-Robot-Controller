package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class IMUController {


    private BNO055IMU imu1;
    public Blinker Expansion_Hub_1;
    public Blinker Expansion_Hub_2;
    double lastAngles;
    double globalAngle;
    HardwareMap hardwareMap;
    public double heading;

    public IMUController(HardwareMap gHardwareMap){
        hardwareMap = gHardwareMap;
        IMUInit();
    }

    public double AngleDeviation(double target){
        return target - GetAngle();
    }

    /*
     * function that checks to see if the IMU is calibrated
     */
    private boolean IMU_Calibrated() {
        return imu1.isGyroCalibrated();
    }

    /*
     * contains all of the IMU initialization routines
     */
    private void IMUInit() {
        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        imu1 = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters IMU_Parameters;

        IMU_Parameters = new BNO055IMU.Parameters();
        // Set the IMU sensor mode to IMU. This mode uses
        // the IMU gyroscope and accelerometer to
        // calculate the relative orientation of hub and
        // therefore the robot.
        IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU_Parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        // Initialize the IMU using parameters object.
        imu1.initialize(IMU_Parameters);
        imu1.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // Report the initialization to the Driver Station
    }

    /*
     * Resets the cumulative angle tracking to zero.
     */
    public void ResetAngle()
    {
        lastAngles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        globalAngle = 0;
    }

    /*
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. - = left, + = right.
     */
    public double GetAngle()
    {
        /* We  determined the Y axis is the axis we want to use for heading angle.
         * We have to process the angle because the imu works in euler angles so the Y axis is
         * returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
         * 180 degrees. We detect this transition and track the total cumulative angle of rotation.
         */

        double angles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        double deltaAngle = angles - lastAngles;

        //angles += 180;
        if (deltaAngle < -180)
        {
            deltaAngle += 360;
        }
        else if (deltaAngle > 180)
        {
            deltaAngle -= 360;
        }

        heading += deltaAngle;
        if (heading < 0){
            heading += 360;
        }
        else if (heading > 360){
            heading -= 360;
        }

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}
