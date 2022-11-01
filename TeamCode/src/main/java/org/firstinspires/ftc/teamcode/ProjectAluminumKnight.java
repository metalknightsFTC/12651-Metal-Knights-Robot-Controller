package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Vectors.*;

@TeleOp
public class ProjectAluminumKnight extends LinearOpMode {

    Blinker Expansion_Hub_1;
    Blinker Expansion_Hub_2;
    DcMotor lift;
    Servo grabber;

    Orientation angles;
    Acceleration gravity;

    DriveTrainCode driveTrainCode;

    private int targetRotations = 0;
    float currentSpeed = .6f;
    float slowSpeed = .3f;
    float regularSpeed = .6f;

    private BNO055IMU imu1;

    double lastAngles;
    double globalAngle;
    double heading;

    @Override
    public void runOpMode(){
        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        imu1 = hardwareMap.get(BNO055IMU.class,"imu");
        lift = hardwareMap.get(DcMotor.class,"lifter");
        grabber = hardwareMap.get(Servo.class,"grabber");

        driveTrainCode = new DriveTrainCode(gamepad1,hardwareMap);

        driveTrainCode.InvertMotorDirection(Motor.backLeft);
        driveTrainCode.InvertMotorDirection(Motor.frontLeft);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        IMUInit();
        ResetAngle();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            if(gamepad1.right_bumper){
                currentSpeed = slowSpeed;
            }else{
                currentSpeed = regularSpeed;
            }

            if(gamepad1.dpad_right){
                driveTrainCode.UpdateDriveTrain(new Vector3(-currentSpeed,StrafeCorrection(),0));
            }else if(gamepad1.dpad_left) {
                driveTrainCode.UpdateDriveTrain(new Vector3(currentSpeed,StrafeCorrection(),0));
            }
            else {
                if(driveTrainCode.RSX >= 0.001 || driveTrainCode.RSX <= -0.001 || !(driveTrainCode.LSX > 0.02f || driveTrainCode.LSX < -0.02f))
                {
                    ResetAngle();
                }
                if(driveTrainCode.LSX > 0.02f || driveTrainCode.LSX < 0.02f)
                {
                    driveTrainCode.UpdateDriveTrain(currentSpeed, StrafeCorrection());
                }else
                {
                    driveTrainCode.UpdateDriveTrain(currentSpeed, StrafeCorrection());
                }
            }

            //region lifter buttons
            //538
            if(gamepad1.a){
                targetRotations = (538 * 4);
            }
            if(gamepad1.x){
                targetRotations = (int)(538 * 6.5);
            }
            if(gamepad1.y){
                targetRotations = 4650;
            }
            if(gamepad1.b){
                targetRotations = 0;
            }
            if(gamepad1.dpad_up){
                targetRotations += (538);
            }
            if(gamepad1.dpad_down){
                targetRotations -= (538);
            }
            //endregion

            //region lifter code
            targetRotations += (gamepad1.right_trigger - gamepad1.left_trigger) * 10;
            if(targetRotations < 0){
                targetRotations = 0;
            }
            if(targetRotations > 4650){
                targetRotations = 4650;
            }
            lift.setTargetPosition(targetRotations);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);

            if(gamepad1.left_bumper){
                grabber.setPosition(.22f);
            }else{
                grabber.setPosition(.33f);
            }

            angles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu1.getGravity();
            GetAngle();
            telemetry.addData("Angle : ", heading);
            telemetry.addData("Lift Target: ", ((double)targetRotations));
            telemetry.addData("Current Lift Position: ", lift.getCurrentPosition());
            telemetry.update();
            //endregion
        }

    }

    public  void LockToHeading()
    {

    }

    public  float StrafeCorrection()
    {
        float turnMod = (float) AngleDeviation(0)/20;
        return -turnMod;
    }

    private double AngleDeviation(double target)
    {
        return target - GetAngle();
    }

    /*
     * function that checks to see if the IMU is calibrated
     */
    private boolean IMU_Calibrated()
    {
        telemetry.addData("IMU Calibration Status", imu1.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", imu1.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", imu1.getSystemStatus().toString());
        return imu1.isGyroCalibrated();
    }

    /*
     * contains all of the IMU initilization rutines
     */
    private void IMUInit()
    {
        BNO055IMU.Parameters parameters;

        parameters = new BNO055IMU.Parameters();
        // Set the IMU sensor mode to IMU. This mode uses
        // the IMU gyroscope and accelerometer to
        // calculate the relative orientation of hub and
        // therefore the robot.
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        // Intialize the IMU using parameters object.
        imu1.initialize(parameters);
        imu1.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Report the initialization to the Driver Station.
        telemetry.addData("Status", "IMU initialized, calibration started.");
        telemetry.update();
        // Wait one second to ensure the IMU is ready.
        sleep(1000);
        // Loop until IMU has been calibrated.
        while (!IMU_Calibrated() && opModeIsActive()) {
            telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
            telemetry.update();
            // Wait one second before checking calibration
            // status again.
            sleep(1000);
        }
        // Report calibration complete to Driver Station.
        telemetry.addData("Status", "Calibration Complete");
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