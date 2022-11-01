package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class IMUController {


    private BNO055IMU imu1;

    double lastAngles;
    double globalAngle;
    float Yaw_Angle;

    /*
     * function that checks to see if the IMU is calibrated
     */
    private boolean IMU_Calibrated() {
        return imu1.isGyroCalibrated();
    }

    /*
     * contains all of the IMU initilization rutines
     */
    private void IMUInit() {
        BNO055IMU.Parameters IMU_Parameters;

        IMU_Parameters = new BNO055IMU.Parameters();
        // Set the IMU sensor mode to IMU. This mode uses
        // the IMU gyroscope and accelerometer to
        // calculate the relative orientation of hub and
        // therefore the robot.
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        // Intialize the IMU using parameters object.
        imu1.initialize(IMU_Parameters);
        // Report the initialization to the Driver Station.
        // Wait one second to ensure the IMU is ready.
        // Loop until IMU has been calibrated.
        // Report calibration complete to Driver Station.
    }

    /*
     * Resets the cumulative angle tracking to zero.
     */
    private void ResetAngle()
    {
        lastAngles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        globalAngle = 0;
    }

    /*
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. - = left, + = right.
     */
    private double GetAngle()
    {
        /* We  determined the Y axis is the axis we want to use for heading angle.
         * We have to process the angle because the imu works in euler angles so the Y axis is
         * returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
         * 180 degrees. We detect this transition and track the total cumulative angle of rotation.
         */

        double angles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        double deltaAngle = angles - lastAngles;

        //angles += 180;
        if (deltaAngle < -180){
            deltaAngle += 360;
        }
        else if (deltaAngle > 180){
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
        //return angles;
    }



}
