package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.util.Log;

import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.aruco.Aruco;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private Mat
            camMat,
            distCoef,
            map1,
            map2;

    final String
            TAG = "CARTOGRAPHER",
            SIM = "Simulator",
            IRL = "Orbit"; // IRL -> 'In Real Life'

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

    /**
     * Wrapper function for api.moveTo(point, quaternion, boolean) to make a fail-safe
     * in case initial movement fails, and log movement details.
     * @param point the Point to move to
     * @param quaternion the Quaternion to orient angles to
     */
    private void moveTo(Point point, Quaternion quaternion) {
        final int LOOP_MAX = 10;

        Log.i(TAG, "[0] Calling moveTo function ");
        Log.i(TAG, point.getX() + ", " + point.getY() + ", " + point.getZ());
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
        Imgproc.initUndistortRectifyMap(camMat, distCoef, new Mat(), camMat, new Size(1280, 960), CvType.CV_16SC2, map1, map2);
        Log.i(TAG, "Created Un-distortion Map Filters");
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
            //moveTo(10.4d, -10d, 4.5d, 0f, 0f, 0f, 1f);
            moveTo(new Point(10.515, -9.806, 4.593), new Quaternion(1f, 0f, 0f, 0f));
            moveTo(new Point(10.412, -9.071, 4.48), new Quaternion(1f, 0f, 0f, 0f));

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
        String mQrContent = "No QR Code Content was Found";
        try {
            mQrContent = getQRContentBuffer();

            switch (mQrContent) {
                case "JEM":
                    mQrContent = "STAY_AT_JEM";
                    break;
                case "COLUMBUS":
                    mQrContent = "GO_TO_COLUMBUS";
                    break;
                case "RACK1":
                    mQrContent = "CHECK_RACK_1";
                    break;
                case "ASTROBEE":
                    mQrContent = "I_AM_HERE";
                    break;
                case "INTBALL":
                    mQrContent = "LOOKING_FORWARD_TO_SEE_YOU";
                    break;
                case "BLANK":
                    mQrContent = "NO_PROBLEM";
                    break;
                default:
                    /* do nothing */
                    break;
            }
        } catch(Exception e) {
            Log.i(TAG, "QR Code Content was Never Found!\nUsing an error String instead.");
        }

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

    /**
     * Processes NavCam Matrix and Scans for AprilTags within NavCam
     * @return the ID of the Target found in the NavCam, and 0 if none found.
     */
    private int getTagInfo(){
        Log.i(TAG, "Calling getTagInfo() function");
        long start = System.currentTimeMillis();

        Mat undistorted = new Mat(),
                ids = new Mat();

        api.flashlightControlFront(0.05f); // enable flashlight for tag read clarity
        Mat distorted = api.getMatNavCam();
        api.flashlightControlFront(0.0f);

        // TODO test with `Imgproc.INTER_NEAREST` for speed's sake?
        Imgproc.remap(distorted, undistorted, map1, map2, Imgproc.INTER_LINEAR);
        Log.i(TAG, "Re-mapped Camera with Distortion Filter");

        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters detParams = DetectorParameters.create();

        List<Mat> corners = new ArrayList<>();
        Aruco.detectMarkers(undistorted, dict, corners, ids, detParams);

        List<Integer> markerIds = new ArrayList<>();
        for(int i = 0; i < ids.rows(); i++){
            for(int j = 0; j < ids.cols(); j++){
                double[] idData = ids.get(i, j);
                int id = (int) idData[0];
                markerIds.add(id); }}

        if (markerIds.containsAll(Arrays.asList(1, 2, 3, 4))){return 1;}
        else if (markerIds.containsAll(Arrays.asList(5, 6, 7, 8))){return 2;}
        else if (markerIds.containsAll(Arrays.asList(9, 10, 11, 12))) {return 3;}
        else if (markerIds.containsAll(Arrays.asList(13, 14, 15, 16))) {return 4;}
        else if (markerIds.containsAll(Arrays.asList(17, 18, 19, 20))) {return 5;}
        else if (markerIds.containsAll(Arrays.asList(21, 22, 23, 24))) {return 6;}

        long delta = (System.currentTimeMillis() - start)/1000;
        Log.i(TAG, "Read AprilTags in " + delta + " seconds");

        return 0;
    }

    /**
     * Scans NavCam Mat for a QR Code
     * @param QR the NavCam Mat with a QR Code in the image.
     * @return QR Code Content as a String
     */
    private String scanQRCode(Mat QR){
        QRCodeDetector detector = new QRCodeDetector();
        String qrData = detector.detectAndDecode(QR);

        if(!qrData.isEmpty()){
            Log.i(TAG, "QR Code Data: " + qrData);
            return qrData;
        }
        else{
            Log.i(TAG, "QR Code Detection Failed.");
        }

        return null;
    }

    private int getTagContentBuffer(){
        //TODO finish this with getTagInfo();
        return 0;
    }

    private String getQRContentBuffer(){
        int LOOP_MAX = 10;
        api.flashlightControlFront(0.05f);
        Mat
                distorted = api.getMatNavCam(),
                QR = new Mat();
        api.flashlightControlFront(0.00f);

        Imgproc.remap(distorted, QR, map1, map2, Imgproc.INTER_LINEAR);
        String data = scanQRCode(QR);

        int loopcounter = 0;
        while(data == null && loopcounter <= LOOP_MAX){
            data = scanQRCode(QR);

            if(data != null){
                return data;
            }

            loopcounter++;
        }
        return null;
    }
}
