package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.List;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private Mat camMat;
    private Mat distCoef;

    final String TAG = "CARTOGRAPHER";
    final String SIM = "Simulator";
    final String IRL = "Orbit"; // IRL -> 'In Real Life'

    /**
     * Wrapper function for api.moveTo(point, quaternion, boolean) to make a fail-safe
     * in case initial movement fails, and log movement details.
     * @param pos_x x pos to move to
     * @param pos_y y pos to move to
     * @param pos_z z pos to move to
     * @param qua_x quaternion x to move to
     * @param qua_y quaternion y to move to
     * @param qua_z quaternion z to move to
     * @param qua_w quaternion w to move to
     */
    private void moveTo(double pos_x, double pos_y, double pos_z,
                                  double qua_x, double qua_y, double qua_z,
                                  double qua_w) {
        final int LOOP_MAX = 10;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);


        Log.i(TAG, "[0] Calling moveTo function ");
        Log.i(TAG, pos_x + " " + pos_y + " " + pos_z);
        long start = System.currentTimeMillis();

        Result result = api.moveTo(point, quaternion, true);

        long end = System.currentTimeMillis();
        long elapsedTime = end - start;
        Log.i(TAG, "[0] moveTo finished in : " + elapsedTime/1000 + " seconds");
        Log.i(TAG, "[0] hasSucceeded : " + result.hasSucceeded());
        Log.i(TAG, "[0] getStatus : " + result.getStatus().toString());
        Log.i(TAG, "[0] getMessage : " + result.getMessage());

        int loopCounter = 1;
        while (!result.hasSucceeded() && loopCounter <= LOOP_MAX) {

            Log.i(TAG, "[" + loopCounter + "] " + "Calling moveTo function");
            start = System.currentTimeMillis();

            result = api.moveTo(point, quaternion, true);

            end = System.currentTimeMillis();
            elapsedTime = end - start;
            Log.i(TAG, "[" + loopCounter + "] " + "moveTo finished in : " + elapsedTime / 1000 +
                    " seconds");
            Log.i(TAG, "[" + loopCounter + "] " + "hasSucceeded : " + result.hasSucceeded());
            Log.i(TAG, "[" + loopCounter + "] " + "getStatus : " + result.getStatus().toString());
            Log.i(TAG, "[" + loopCounter + "] " + "getMessage : " + result.getMessage());

            ++loopCounter;

        }
    }


    //TODO the method below is for image processing which comes with laser detection
    /**
     * Pre-Load the Camera Matrix and Distortion Coefficients to save time.
     * @param mode 'SIM' or 'IRL' -> Simulator or Real Coefficients, Respectively
     */
    private void initCam(String mode){
        camMat = new Mat(3, 3, CvType.CV_32F);
        distCoef = new Mat(1, 5, CvType.CV_32F);

        if(mode.equals(SIM)){
            // all numbers via programming manual
            float[] camArr = {
                    661.783002f, 0.000000f, 595.212041f,
                    0.000000f, 671.508662f, 489.094196f,
                    0.000000f, 0.000000f, 1.000000f
            };
            float[] distortionCoefficients = {
                    -0.215168f, 0.044354f, 0.003615f, 0.005093f, 0.000000f
            };

            camMat.put(0, 0, camArr);
            distCoef.put(0,0, distortionCoefficients);
        }
        else if(mode.equals(IRL)){
            float[] camArr = {
                    753.51021f, 0.0f, 631.11512f,
                    0.0f, 751.3611f, 508.69621f,
                    0.0f, 0.0f, 1.0f
            };
            float[] distortionCoefficients = {
                    -0.411405f, 0.177240f, -0.017145f, 0.006421f, 0.000000f
            };

            camMat.put(0, 0, camArr);
            distCoef.put(0,0, distortionCoefficients);
        }
        Log.i(TAG, "Initialized Camera Matrices in Mode: " + mode);
    }

    @Override
    protected void runPlan1(){
        // the mission starts
        api.startMission();
        initCam(SIM);
        Log.d(TAG, camMat.dump());
        Log.d(TAG, distCoef.dump());

        int loops = 0;
        while (true){
            // get the list of active target id
            List<Integer> targets = api.getActiveTargets();
            Log.d(TAG, targets.toString().substring(1, targets.toString().length() - 1));

            // move to a point
            moveTo(10.4d, -10d, 4.5d, 0f, 0f, 0f, 1f);

            // get a camera image
            Mat image = api.getMatNavCam();

            // irradiate the laser
            api.laserControl(true);

            // take active target snapshots
            int target_id = 1;
            api.takeTargetSnapshot(target_id);

            /* ************************************************ */
            /* write your own code and repair the ammonia leak! */
            /* ************************************************ */


            // check the remaining milliseconds of mission time
            if (api.getTimeRemaining().get(1) < 60000)
                break;

            ++loops;
            if (loops == 2)
                break;
        }
        // turn on the front flash light
        api.flashlightControlFront(0.05f);
        
        // get QR code content
        String mQrContent = getQRContent();

        // turn off the front flash light
        api.flashlightControlFront(0.00f);

        // notify that astrobee is heading to the goal
        api.notifyGoingToGoal();

        /* ********************************************************** */
        /* write your own code to move Astrobee to the goal positiion */
        /* ********************************************************** */
        // TODO hardcode movement functions

        // send mission completion
        api.reportMissionCompletion(mQrContent);
    }

    @Override
    protected void runPlan2(){
       // write your plan 2 here
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

    // You can add your method
    private String getQRContent(){
        return "null_temp";
    }
}
