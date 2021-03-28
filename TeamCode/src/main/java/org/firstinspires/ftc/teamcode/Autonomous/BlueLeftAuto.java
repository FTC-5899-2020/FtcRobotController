package org.firstinspires.ftc.teamcode.Autonomous;
//-----imports-----
//main imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//opencv imports
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutoSupplies;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
//webcam imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="Blue Left Auto", group="CompetitionAuto")
//@Disabled
public class BlueLeftAuto extends AutoSupplies{
    @Override
    public void runOpMode() {

        //  Establish all hardware

        initForAutonomous();



        //  Wait until start
        waitForStart();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();
        sleep(300);
        int ringCnt = pipeline.getAnalysis(); // 4 rings: >150 --- 1 ring: >130 & <150 --- 0 rings: <130
        sleep(300);
        if(ringCnt <= 130){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if(ringCnt > 130 && ringCnt < 150){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else{
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        encoderMove(500, 0, .7);
        encoderMove(420, -.7, 0);
        turnToS(0,.6,2);
        encoderMove(3000, 0, 1);
        turnToS(0, .6, 2);
        while((distanceFwdLeft.getDistance(DistanceUnit.MM)+distanceFwdRight.getDistance(DistanceUnit.MM))/2 > 700){
            setPower(0, .4);
        }
        setPower(0,0);
        //  Turn all motors off and sleep
        setPower(0, 0);
        sleep(1000);
    }
}
