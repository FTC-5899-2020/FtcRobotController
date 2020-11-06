package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Cheese.AutoSuppliesCheese;

import java.io.File;
import java.util.ArrayList;
import java.util.List;


@TeleOp(name="Vuforia Line Alignment", group ="Test Group")
//@Disabled
public class VuforiaLineAlignment extends AutoSuppliesCheese {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    OpenGLMatrix robotLocationTransform = null;

    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    VuforiaLocalizer vuforia;

    WebcamName webcamName;



    @Override
    public void runOpMode() {

        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ac8qVVb/////AAABmW0ZY5qaKUVegMYq2LOSDO1OzcyAP6IoQTVXJ5E6V+Xier9dD5quzzS0toHeXCyiWZn6Wsw2WdgS9GLwIjNfmuozNwBTuU9DBkABBpyBwAXiiZmzTgLLkNR1dw9+Vwl/S76TuqcaNHTl8vvQOTssFkIvXC0f5acepwlTL8xjEsvb3Y6Fys/mMQprOuhg/9f44K5DsQwutOaTrsVjGyJ1fWyT6cDM+BPqLcBs+/oisbHud/8Q8Iz3I/9+xXJW1ZChn659VoZ0a2Sdoa5FdLl72OpVEzA+d+lYaGcZXmE8NszlxxdOivvNkcFfF45zRyqisSfGowjpyFglNBSWTsNiD1shkpP0uyoeK9lRVxIE4Qug";

        parameters.cameraName = webcamName;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        vuforia.enableConvertFrameToBitmap();

        /** @see #captureFrameToFile() */
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);


        VuforiaTrackables pictures = vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable Target1 = pictures.get(0);
        VuforiaTrackable Target2 = pictures.get(1);
        VuforiaTrackable Target3 = pictures.get(2);
        VuforiaTrackable Target4 = pictures.get(3);
        VuforiaTrackable Target5 = pictures.get(4);
        Target1.setName("BlueTowerGoal");  // Stones
        Target2.setName("RedTowerGoal");
        Target3.setName("RedAlliance");
        Target4.setName("BlueAlliance");
        Target5.setName("FrontWall");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(pictures);

        /** Wait for the game to begin */
        initForAutonomous();
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        pictures.activate();

        boolean buttonPressed = false;
        while (opModeIsActive()) {

            buttonPressed = gamepad1.a;

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getFtcCameraFromTarget();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;

                }
                if (lastLocation != null) {
                    //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                    telemetry.addData("Pos", format(lastLocation));
                    VectorF trans = lastLocation.getTranslation();
                    Orientation rot = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                    telemetry.addData("X", tX);
                    telemetry.addData("Y", tY);
                    telemetry.addData("Z", tZ);
                    telemetry.addData("X'", rX);
                    telemetry.addData("Y'", rY);
                    telemetry.addData("Z'", rZ);
                    if(rY > 0 && rY < 175 && robotLocationTransform != null){
                        resetAngle();
                        turnToS( (180 - (int)rY),0.4,2);
                    }
                    else if (rY < 0 && rY > -175 && robotLocationTransform != null){
                        resetAngle();
                        turnToS( -(180 + (int)rY),0.4,2);
                    }
                    else{
                        setPower(0, 0);
                        telemetry.addData("GOOD?",0);
                        telemetry.update();
                    }
                } else {
                    telemetry.addData("Pos", "Unknown");
                    setPower(0, 0);
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */


            }
            telemetry.update();

        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}