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
        //basketServoUp(); -- tried moving this down lower. see how it works before deleting this
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();
        sleep(300);
        int ringCnt = pipeline.getAnalysis(); // 4 rings: >150 --- 1 ring: >130 & <150 --- 0 rings: <130
        sleep(300);
        if (ringCnt <= 130) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (ringCnt > 130 && ringCnt < 150) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        basketServoUp();
        ringPullPivotServoOut();
        encoderMove(500, 0, .7);
        while (opModeIsActive()) {//>300
            double dist = distanceLeftTop.getDistance(DistanceUnit.MM);
            double dist2 = distanceLeftBottom.getDistance(DistanceUnit.MM);
            //time out check
            if(dist == 65535 || distanceLeftTop.didTimeoutOccur()) {
                dist = 65535;
            }
            if(dist2 == 65535 || distanceLeftBottom.didTimeoutOccur()){
                dist2 = 65535;
            }
            //telemetry
            telemetry.addData("distance val 1", dist);
            telemetry.addData("distance val 2", dist2);
            //power set or loop break if distance traveled is met
            if(dist != 65535 && dist2 != 65535 && (dist + dist2)/2 > 300){
                setPower(-.5, 0);
            }
            else if(dist != 65535 && dist2 == 65535 && dist > 300){
                setPower(-.5, 0);
            }
            else if(dist == 65535 && dist2 != 65535 && dist2 > 300){
                setPower(-.5, 0);
            }
            else{
                break;
            }
            telemetry.update();
        }
        turnToS(0, .6, 2);
        wobbleArmDown();
        //encoderMove(2500, 0, 1);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
        turnToS(0, .6, 2);
        while (opModeIsActive() && (distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM)) / 2 > 850) {
            setPower(0, 1);
            telemetry.addData("distance val", distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM) / 2);
            telemetry.update();
        }
        if (ringCnt >= 150) {
            while (opModeIsActive() && (distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM)) / 2 > 500) {
                setPower(0, .4);
                telemetry.addData("distance val", distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM) / 2);
                telemetry.update();
            }
        } else {
            while (opModeIsActive() && (distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM)) / 2 > 700) {
                setPower(0, .4);
                telemetry.addData("distance val", distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM) / 2);
                telemetry.update();
            }
        }

        setPower(0, 0);
        if (ringCnt > 130 && ringCnt < 150) {
            turnToS(90, .7, 2);
        } else if (ringCnt >= 150) {
            turnToS(210, .7, 2);
        }
        wobbleGrabberOpen();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
        sleep(400);
        wobbleGrabberEject();
        sleep(200);
        wobbleArmUp();
        sleep(500);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
        /*if (ringCnt > 130 && ringCnt < 150) {
            while (opModeIsActive() && (distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM)) / 2 > 200) {
                setPower(0, .4);
            }
        }*/
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON);
        if(ringCnt > 150){
            turnToS(360, .7, 2);
            resetAngle();
        }
        else {
            turnToS(0, .7, 2);
        }
        basketServoUp();
        if (ringCnt > 150) {
            while (opModeIsActive() && distanceLeftTop.getDistance(DistanceUnit.MM) < 700) {
                setPower(.5, 0);
                telemetry.addData("distance val", distanceLeftTop.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            turnToS(0, .6, 2);
            while (opModeIsActive() && (distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM)) / 2 > 200) {
                setPower(0, .5);
                telemetry.addData("distance val", distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM) / 2);
                telemetry.update();
            }
            turnToS(0, .6, 2);
        } else {
            while (opModeIsActive() && (distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM)) / 2 > 200) {
                setPower(0, .5);
                telemetry.addData("distance val", distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM) / 2);
                telemetry.update();
            }
            turnToS(0, .6, 2);
            while (opModeIsActive() && distanceLeftTop.getDistance(DistanceUnit.MM) < 700) {
                setPower(.5, 0);
                telemetry.addData("distance val", distanceLeftTop.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            turnToS(0, .6, 2);
        }

        shooterLeft.setPower(.4);
        shooterRight.setPower(-.4);
        setPower(0,.3);
        sleep(800);//was not updated to the bot yet... was at 500
        setPower(0,0);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
        for(int i = 0; i < 3; i++){
            basketServoUp();
            sleep(500);
            unloadServoPush();
            sleep(500);
            unloadServoBack();
            sleep(100);
            basketServoDown();
            sleep(100);
        }
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        wobbleArmDown();
        while (opModeIsActive() && (distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM)) / 2 < 200) {
            setPower(0, -.3);
            telemetry.addData("distance val", distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM) / 2);
            telemetry.update();
        }
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        while (opModeIsActive() && distanceLeftTop.getDistance(DistanceUnit.MM) < 1400) {
            setPower(.6, 0);
            telemetry.addData("distance val", distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM) / 2);
            telemetry.update();
        }
        turnToS(0, .7, 2);

        while (opModeIsActive() && (distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM)) / 2 < 950) {
            setPower(0, -1);
            telemetry.addData("distance val", distanceFwdLeft.getDistance(DistanceUnit.MM) + distanceFwdRight.getDistance(DistanceUnit.MM) / 2);
            telemetry.update();
        }

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
        //  Turn all motors off and sleep
        setPower(0, 0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        sleep(1000);
    }
}
//Possible link to add voltage sensor into our code.
//https://www.reddit.com/r/FTC/comments/5cnilm/help_how_to_get_robot_battery_levelvoltage/