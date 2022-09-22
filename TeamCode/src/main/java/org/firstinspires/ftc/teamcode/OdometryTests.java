package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Enums.EndPoint;
import org.firstinspires.ftc.teamcode.Enums.StartPoint;

import java.util.List;

@Autonomous
public class OdometryTests extends LinearOpMode {
    Blinker Expansion_Hub_1;
    Blinker Expansion_Hub_2;
    private DcMotorEx right;
    private DcMotorEx left;
    private DcMotorEx rear;
    OdometryControl odometryControl;
    private EndPoint endPoint;
    private StartPoint startPoint = StartPoint.UnSet;
    private SubSystemControl subSystemControl;
    String lift = "Lift";
    String grabber = "Grabber";

    String[] motors = new String[1];
    String[] servos = new String[1];

    private static final String TFOD_MODEL_ASSET =  "PowerPlay.tflite";//"/sdcard/FIRST/tflitemodels/model.tflite";
    private static final String[] LABELS = {
            "Class 1",
            "Class 2"
    };
    private static final String VUFORIA_KEY =
            "AWdhXNj/////AAABmRSQQCEQY0Z+t33w9GIgzFpsCMHl909n/+kfa54XDdq6fPjSi/8sBVItFQ/J/d5SoF48FrZl4Nz1zeCrwudfhFr4bfWTfh5oiLwKepThOhOYHf8V/GemTPe0+igXEu4VhznKcr3Bm5DiLe2b6zBVzvWFDWEHI/mkhLxRkU+llmwvitwodynP2arFgZ43thde9GJPCBFne/q6tPXeeN8/PoTUOtycTrnTkL6fBuHelMMnvN2RjqnMJ9SBUcaVX8DsWukq1fDr29O8bguAJU5JKxt9E3+XXiexpE/EJ9jxJc7YoMtpxfMro/e0sm9gRNckw4uPtZHnaoDjFhaK9t2D7kQQc3rwgK1OEZlY7FGQyy8g";;
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {

        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        //Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub");

        right = hardwareMap.get(DcMotorEx.class, "left");
        left = hardwareMap.get(DcMotorEx.class, "right");
        rear = hardwareMap.get(DcMotorEx.class, "rear");

        motors[0] = lift;
        servos[0] = grabber;

        subSystemControl = new SubSystemControl(hardwareMap, motors, servos);

        //SetStartPoint();
        //OperateTensorFlow();
        InitializeOdometry();
        telemetry.addData("Path Set", ", End: ", endPoint, ", Start: ", startPoint);
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            odometryControl.SetStickPower(0,1,0);
            double[] deltas = odometryControl.CalculateRobotPosition();

            odometryControl.robotPosition.currentHeading += deltas[2];
            if (deltas[2] <= .2f || deltas[2] >= -.2f)
            {
                odometryControl.robotPosition.x += deltas[0];
            }
            odometryControl.robotPosition.z += deltas[1];
            telemetry.addData("X: ",odometryControl.robotPosition.x);
            telemetry.addData("Z: ",odometryControl.robotPosition.z);
            telemetry.addData("Heading: ",odometryControl.robotPosition.currentHeading);
            telemetry.update();
        }
    }

    void  SetStartPoint(){
        while(startPoint == StartPoint.UnSet)
        {
            telemetry.addData("A: ","Red Side Left");
            telemetry.addData("X: ","Red Side Right");
            telemetry.addData("Y: ","Blue Side Left");
            telemetry.addData("B: ","Blue Side Right");
            telemetry.update();
            if (gamepad1.a){
                startPoint = StartPoint.redLeft;
                return;
            }
            if (gamepad1.x){
                startPoint = StartPoint.redRight;
                return;
            }
            if (gamepad1.y){
                startPoint = StartPoint.blueLeft;
                return;
            }
            if (gamepad1.b){
                startPoint = StartPoint.blueRight;
                return;
            }
        }
    }

    void InitializeOdometry(){
        odometryControl = new OdometryControl(right, left, rear, hardwareMap, gamepad1);
        telemetry.addData("Odometry Status: ", "Ready");
    }

}