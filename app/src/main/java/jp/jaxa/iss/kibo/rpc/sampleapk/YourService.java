package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.List;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import java.util.Map;
import java.util.HashMap;

import org.opencv.core.Mat;

import org.opencv.core.MatOfPoint;
import org.opencv.objdetect.QRCodeDetector;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private static final String TAG = "KiboFiendFyre";
    private float down = 5.25f;
    private Quaternion q;
    private Point p;
    private String QRmes;
    private Point[] P = {
            new Point(10.4f    , -10       , 4.4f),       // start(0)
            new Point(11.2746f , -9.92284f , 5.2988f),    //v
            new Point(10.612f  , -9.0709f  , 4.48f),      //v
            new Point(10.71f   , -7.7f     , 4.48f),      //v
            new Point(10.51f   , -6.7185f  , 5.1804f),    //v
            new Point(11.114f  , -7.9756f  , 5.3393f),    //v
            new Point(11.355f  , -8.9929f  , 4.7818f),    //v
            new Point(11.369f  , -8.5518   , 4.7818f),    // QR code(7)
            new Point(11.143f  , -6.7607f  , 4.48f),      // goal(8)
            new Point(10.4f    , -10       , down),         // startz(9 = 0 + 9)
            new Point(11.2746f , -9.92284f , down),
            new Point(10.612f  , -9.0709f  , down),
            new Point(10.71f   , -7.7f     , down),
            new Point(10.51f   , -6.7185f  , down),
            new Point(11.114f  , -7.9756f  , down),
            new Point(11.355f  , -8.9929f  , down),
            new Point(11.369f  , -8.5518   , down),         // QR codez(16 = 7 + 9)
            new Point(11.143f  , -6.7607f  , down),         // goalz(17 = 8 + 9)

    };
    private Quaternion[] quaternion = {
            new Quaternion(0, 0, 0, 0),                   // start
            new Quaternion(0, 0, -0.707f, 0.707f),
            new Quaternion(0.5f, 0.5f, -0.5f, 0.5f),
            new Quaternion(0, 0.707f, 0, 0.707f),
            new Quaternion(0, 0, -1, 0),
            new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f),
            new Quaternion(0, 0, 0, 1),
            new Quaternion(0.707f,0,-0.707f,0)            // QR code
    };

    int flag_obstacle = 0;//keeping track of each obstacle crossed

    //Keep In Zone Co - ordinates

    private double[][][] KIZ = {
            {
                    {10.3, 11.55}, {-10.2, -6}, {4.32, 5.57}
            },
            {
                    {9.5, 10.5}, {-10.5, -9.6}, {4.02, 4.8}
            }
    };

    //Keep Out Zone Co-ordinates
    private double[][][] KOZ = {
            {
                    // 000    001        010       011
                    {10.783, 11.071}, {-9.8899, -9.6929}, {4.8385, 5.0665}
            },
            {
                    // 100    101         110       111
                    {10.8652, 10.9628}, {-9.0734, -8.7314}, {4.3861, 4.6401}
            },
            {
                    {10.185, 11.665}, {-8.3826, -8.2826}, {4.1475, 4.6725}
            },
            {
                    {10.7955, 11.3525}, {-8.0635, -7.7305}, {5.1055, 5.1305}
            },
            {
                    {10.563, 10.709}, {-7.1449, -6.8099}, {4.6544, 4.8164}
            }

    };

    private String photo_name(int x) {
        String res = "";

        res += "photo";
        res += Integer.toString(x);
        res += ".png";

        return res;
    }


    private String scanQRcode() {
        Map<String, String> map = new HashMap<>();
        map.put("JEM", "STAY_AT_JEM");
        map.put("COLUMBUS", "GO_TO_COLUMBUS");
        map.put("RACK1", "CHECK_RACK_1");
        map.put("ASTROBEE", "I_AM_HERE");
        map.put("INTBALL", "LOOKING_FORWARD_TO_SEE_YOU");
        map.put("BLANK", "NO_PROBLEM");

        MatOfPoint point = new MatOfPoint();
        QRCodeDetector detector = new QRCodeDetector();

        String data = detector.detectAndDecode(api.getMatNavCam(), point);
        return map.get(data);
    }

    private void gotoStart(){
        p = P[9];
        q = quaternion[0];
        api.moveTo(p, q, true);
        Log.i(TAG, "arrive start z");
    }

    private void gotoTarget(int x){ // T1 ~ T6
        q = quaternion[x];

        if(x <= 2){
            p = P[11];
            api.moveTo(p, q, true);
            Log.i(TAG, "arrivez " + Integer.toString(x));


            p = P[x];
            api.moveTo(p, q, true);
            api.saveMatImage(api.getMatNavCam(), photo_name(x));
            Log.i(TAG, "arrive " + Integer.toString(x));


            p = P[11];
            api.moveTo(p, q, true);
            Log.i(TAG, "arrivez2 " + Integer.toString(x));
        }
        else if(x == 4){
            p = P[13];
            api.moveTo(p, q, true);
            Log.i(TAG, "arrivez " + Integer.toString(x));


            p = P[x];
            api.moveTo(p, q, true);
            api.saveMatImage(api.getMatNavCam(), photo_name(x));
            Log.i(TAG, "arrive " + Integer.toString(x));


            p = P[13];
            api.moveTo(p, q, true);
            Log.i(TAG, "arrivez2 " + Integer.toString(x));
        }
        else{
            p = P[16];
            api.moveTo(p, q, true);
            Log.i(TAG, "arrivez " + Integer.toString(x));

            p = P[x];
            api.moveTo(p, q, true);
            api.saveMatImage(api.getMatNavCam(), photo_name(x));
            Log.i(TAG, "arrive " + Integer.toString(x));


            p = P[16];
            api.moveTo(p, q, true);
            Log.i(TAG, "arrivez2 " + Integer.toString(x));
        }
    }

    private void gotoQR(){
        p = P[16];
        q = quaternion[7];


        api.moveTo(p, q, true);
        Log.i(TAG, "arrive QR z");

        p = P[7];
        api.moveTo(p, q, true);
        Log.i(TAG, "arrive QR");
        api.flashlightControlFront(0.05f);
        api.saveMatImage(api.getMatNavCam(), photo_name(103));
        QRmes = scanQRcode();


        p = P[16];
        api.moveTo(p, q, true);
        Log.i(TAG, "back to QR z");
    }

    private void gotoGoal(){
        api.notifyGoingToGoal();
        p = P[13];
        api.moveTo(p, q, true);
        Log.i(TAG, "arrive goal z ");


        p = P[8];
        api.moveTo(p, q, true);
        Log.i(TAG, "arrive goal");
        api.reportMissionCompletion(QRmes);

    }


    @Override
    protected void runPlan1(){
        api.startMission();
        //Astrobee 1 ft cube  = 0.3048 meter per side, half approx = 0.16 m, diagonally half length = 0.22 m
        //the below 7 arrays constitute the position of P1-1 to P2-3

        gotoStart();
        gotoTarget(1);
        gotoQR();
        gotoGoal();

        Log.i(TAG, "mission complete");
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }



}