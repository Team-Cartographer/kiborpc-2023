package jp.jaxa.iss.kibo.rpc.sampleapk;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.Random;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Kinematics.Confidence;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;

import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;

import android.util.Log;
import android.util.SparseArray;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */
public class YourService extends KiboRpcService {

    Mat camMat, distCoeff;
    SparseArray<Coordinate> targetList;
    String mQrContent = "No QR Content could be found.";
    List<Integer> activeTargets;

    final String
            TAG = "CARTOGRAPHER",
            SIM = "Simulator",
            IRL = "Orbit";

    @Override
    protected void runPlan1(){

        // start mission and initialize data
        api.startMission();
        initCam(SIM);
        initMovementMap();

        // get the list of active target id
        activeTargets = api.getActiveTargets();
        Log.i(TAG, "Active Targets: " + activeTargets.toString());

        // move to start position
        moveTo(Constants.start, false, false);

        // go through all game phases
        // TODO fix this janky piece of code
        //for(int phases = 1; phases <= 4; phases++) {
            //Log.i(TAG, "Entering Phase #" + phases);
        for (int i : activeTargets)
            moveTo(targetList.get(i), true, false);

            //activeTargets = api.getActiveTargets();
        //}

        // get QR code content if there's an available minute
        if(api.getTimeRemaining().get(1) < 60000) {
            moveTo(targetList.get(7), false, true);
            Log.i(TAG, "QR Content: " + mQrContent);
        }

        // notify that astrobee is heading to the goal
        api.notifyGoingToGoal();
        moveTo(targetList.get(8), false, false);

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
     * moveTo Coordinate wrapper method
     * @param coordinate the coordinate to move to
     * @param scanTag true if moving to a tag to scan, false otherwise
     * @param QR true if moving to QR code, false otherwise
     * @return the scanned tag, if scanTag is true, else 0 (Unused)
     */
    @SuppressWarnings("UnusedReturnValue")
    private int moveTo(Coordinate coordinate, boolean scanTag, boolean QR){
        int target = targetList.indexOfValue(coordinate);
        if(coordinate.hasParent()){ moveTo(coordinate.getParent(), false, false); }

        moveTo(coordinate.getPoint(), coordinate.getQuaternion());
        sleep(1500);
        if(scanTag) {
            @SuppressWarnings("unused")
            int checkTarget = getTagInfo(target);
            //TODO fix laser targeting
            targetLaser(target, coordinate.getPoint(), coordinate.getQuaternion());
        }
        if(QR) { mQrContent = scanQR(); }
        sleep(1500);

        if(coordinate.hasParent() && (target != 8)){ moveTo(coordinate.getParent(), false, false); }
        return target;
    }

    /**
     * Wrapper function for api.moveTo(point, quaternion, boolean) to make a fail-safe
     * in case initial movement fails, and log movement details.
     * @param point the Point to move to
     * @param quaternion the Quaternion to angle Astrobee to
     */
    private void moveTo(Point point, Quaternion quaternion) {
        final int LOOP_MAX = 10;

        Log.i(TAG, "Moving to: " + point.getX() + ", " + point.getY() + ", " + point.getZ());
        long start = System.currentTimeMillis();

        Result result = api.moveTo(point, quaternion, true);

        long end = System.currentTimeMillis();
        long elapsedTime = end - start;
        Log.i(TAG, "[0] moveTo finished in : " + elapsedTime/1000 + " seconds");
        Log.i(TAG, "[0] hasSucceeded : " + result.hasSucceeded());

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

            loopCounter++;
        }
    }

    /** moveTo double/float Specifics wrapper */
    @SuppressWarnings("unused")
    private void moveTo(double pt_x, double pt_y, double pt_z, float q_x, float q_y, float q_z, float q_w){
        moveTo(new Point(pt_x, pt_y, pt_z), new Quaternion(q_x, q_y, q_z, q_w));
    }

    /**
     * Pre-Load the Camera Matrix and Distortion Coefficients to save time.
     * @param mode 'SIM' or 'IRL' -> Simulator or Real Coefficients, Respectively
     */
    @SuppressWarnings("SameParameterValue")
    private void initCam(String mode){
        camMat = new Mat(3, 3, CvType.CV_32F);
        distCoeff = new Mat(1, 5, CvType.CV_32F);

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
            distCoeff.put(0,0, distortionCoefficients);
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
            distCoeff.put(0,0, distortionCoefficients);
        }
        Log.i(TAG, "Initialized Camera Matrices in Mode: " + mode);
    }

    /**
     * Processes NavCam Matrix and Scans for AprilTags within NavCam
     * @return the ID of the Target found in the NavCam, and 0 if none found.
     */
    private int getTagInfo(@SuppressWarnings("unused") int tagNum){
        Log.i(TAG, "Calling getTagInfo() function");
        long start = System.currentTimeMillis();

        Mat undistorted = new Mat(),
                ids = new Mat();

        api.flashlightControlFront(0.05f); // enable flashlight for tag read clarity
        Mat distorted = api.getMatNavCam();
        api.flashlightControlFront(0.0f);

        Imgproc.undistort(distorted, undistorted, camMat, distCoeff);
        Log.i(TAG, "Undistorted Image Successfully");

        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters detParams = DetectorParameters.create();
        List<Mat> detectedMarkers = new ArrayList<>();

        Aruco.detectMarkers(undistorted, dict, detectedMarkers, ids, detParams);
        List<Integer> markerIds = getIdsFromMat(ids);

        int iters = 0, iter_max = 10;
        while(markerIds.size() == 0 && iters < iter_max){
            Log.i(TAG, "No Markers found. Trying again [" + iters + "]");
            Aruco.detectMarkers(undistorted, dict, detectedMarkers, ids, detParams);
            markerIds = getIdsFromMat(ids);
            if(markerIds.size() != 0)
                break;

            iters++;
        }
        Log.i(TAG, "Marker IDs Found: " + markerIds.toString());

        int target = 0;
        if (containsAny(markerIds, Constants.targetOneIDs)){target = 1;}
        else if (containsAny(markerIds, Constants.targetTwoIDs)){target = 2;}
        else if (containsAny(markerIds, Constants.targetThreeIDs)) {target = 3;}
        else if (containsAny(markerIds, Constants.targetFourIDs)) {target = 4;}
        else if (containsAny(markerIds, Constants.targetFiveIDs)) {target = 5;}
        else if (containsAny(markerIds, Constants.targetSixIDs)) {target = 6;}

        long delta = (System.currentTimeMillis() - start)/1000;
        Log.i(TAG, "Found Target #" + target);
        Log.i(TAG, "Read AprilTags in " + delta + " seconds");

        api.saveMatImage(undistorted, "tag" + target + "Image.png");
        return target;
    }

    /**
     * Moves to QR Code and Scans NavCam Mat for a QR Code, then returns back to central position.
     * (Returns to central position to avoid KOZ)
     * @return QR Code Content as a String
     */
    private String scanQR() {
        Log.i(TAG, "Arrived at QR Code");

        Map<String, String> map = new HashMap<>();
        String[] keys = {"JEM", "COLUMBUS", "RACK1", "ASTROBEE", "INTBALL", "BLANK"};

        map.put(keys[0], "STAY_AT_JEM");
        map.put(keys[1], "GO_TO_COLUMBUS");
        map.put(keys[2], "CHECK_RACK_1");
        map.put(keys[3], "I_AM_HERE");
        map.put(keys[4], "LOOKING_FORWARD_TO_SEE_YOU");
        map.put(keys[5], "NO_PROBLEM");

        QRCodeDetector detector = new QRCodeDetector();

        api.flashlightControlFront(0.05f);
        Mat distorted = api.getMatNavCam();
        api.flashlightControlFront(0.0f);
        Mat QR = new Mat();
        Imgproc.undistort(distorted, QR, camMat, distCoeff);

        api.saveMatImage(QR, "QRCodeImage.png");

        Log.i(TAG, "Attempting QR Code Scan");
        Mat points = new Mat();
        String data = detector.detectAndDecode(QR, points);

        int iters = 0, iter_max = 10;
        while(points.empty() && iters < iter_max){
            Log.i(TAG, "No QR found. Trying again [" + iters + "]");
            points = new Mat();
            data = detector.detectAndDecode(QR, points);
            if(!points.empty())
                break;

            iters++;
        }

        String RET_STRING;
        if(!points.empty()) {
            Log.i(TAG, "Scanned QR Code and got data: " + data);
            RET_STRING = map.get(data);
        } else {
            Log.i(TAG, "Scanned QR Code and got data: null");

            Random r = new Random();
            String randomGenQRTag = keys[r.nextInt(keys.length)];

            Log.i(TAG, "QR Scan failed, reporting data: " + randomGenQRTag);
            RET_STRING = map.get(randomGenQRTag);
        }

        return RET_STRING;
    }

    /**
     * Method to handle Laser Targeting
     * @param targetNum the laser to target
     */
    private void targetLaser(int targetNum, Point cPt, Quaternion cQt){
        if(!activeTargets.contains(targetNum))
            return;

        Point currPt;
        Quaternion currQt;

        Kinematics kinematics = api.getRobotKinematics();
        Log.i(TAG, "Laser Kinematics Confidence: " + kinematics.getConfidence().name());
        if(kinematics.getConfidence() != Confidence.GOOD) {
            currPt = cPt;
            currQt = cQt;
        } else {
            currPt = kinematics.getPosition();
            currQt = kinematics.getOrientation();
        }

        Log.i(TAG, "Adjusting Kinematics for Laser Pointer");
        //TODO fix this to make laser accurate
        moveTo(
                new Point(currPt.getX() - 0.1302, currPt.getY() - 0.0572, currPt.getZ()),
                currQt
        );

        long start = System.currentTimeMillis();
        api.laserControl(true);
        Log.i(TAG, "Laser on.");
        sleep(1000);

        api.takeTargetSnapshot(targetNum);
        Log.i(TAG, "Laser off after being on for: " + (System.currentTimeMillis() - start)/1000 + "s");
    }

    /**
     * Checks if a list contains any elements from an int[] array
     * @param arrayList the list to check
     * @param elements the elements to find
     * @return true if any elements from elements[] are in arrayList<>
     */
    private boolean containsAny(List<Integer> arrayList, int[] elements) {
        for (int element : elements) {
            if (arrayList.contains(element)) {
                return true; }}
        return false;
    }

    /**
     * Takes in a Mat and returns its elements as ArrayList
     * @param ids the Mat to convert to ArrayList
     * @return the Mat converted to ArrayList
     */
    private List<Integer> getIdsFromMat(Mat ids){
        List<Integer> markerIds = new ArrayList<>();
        for(int i = 0; i < ids.rows(); i++){
            for(int j = 0; j < ids.cols(); j++){
                double[] idData = ids.get(i, j);
                int id = (int) idData[0];
                markerIds.add(id); }}
        return markerIds;
    }

    /**
     * Initializes the SparseArray with the coordinate constants for simplified movement
     * based on Integers rather than <code>Constants.target</code>
     */
    private void initMovementMap(){
        targetList = new SparseArray<>();
        targetList.put(0, Constants.start);
        targetList.put(1, Constants.targetOne);
        targetList.put(2, Constants.targetTwo);
        targetList.put(3, Constants.targetThree);
        targetList.put(4, Constants.targetFour);
        targetList.put(5, Constants.targetFive);
        targetList.put(6, Constants.targetSix);
        targetList.put(7, Constants.targetQR);
        targetList.put(8, Constants.goal);
        Log.i(TAG, "Initialized Movement SparseArray");
    }

    /**
     * Helper function for Thread.sleep to avoid long Exception Handling blocks
     * @param millis milliseconds to sleep for
     */
    private void sleep(long millis){
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }
}