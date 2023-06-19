package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Constants {

    public static final Coordinate start = new Coordinate(
            new Point(9.815, -9.806, 4.293), new Quaternion(1, 0, 0, 0)
    );

    public static final Coordinate goal = new Coordinate(
            new Point(11.143, -6.7607, 4.9654), new Quaternion(0, 0, -0.707f, 0.707f)
    );

    public static final Coordinate pointOne = new Coordinate(
            new Point(11.2746, -9.92284, 5.2988), new Quaternion(0f, 0f, -0.707f, 0.707f)
    );

    public static final Coordinate pointTwo = new Coordinate(
            new Point(10.612, -9.0709, 4.48), new Quaternion(0.5f, 0.5f, -0.5f, -0.5f)
    );

    public static final Coordinate pointThree = new Coordinate(
            new Point(10.71, -7.7, 4.48), new Quaternion(0, 0.707f, 0, 0.707f)
    );

    public static final Coordinate pointFour = new Coordinate(
            new Point(10.51, -6.7185, 5.1804), new Quaternion(0, 0, -1, 0)
    );

    public static final Coordinate pointFive = new Coordinate(
            new Point(11.114, -7.9756, 5.3393), new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f)
    );

    public static final Coordinate pointSix = new Coordinate(
            new Point(11.355, -8.9929, 4.7818), new Quaternion(0, 0, 0, 1)
    );

    public static final Coordinate pointSeven = new Coordinate(
            new Point(11.369, -8.5518, 4.48), new Quaternion(0f, 0.707f, 0f, 0.707f)
    );

    public static final Coordinate targetOne = new Coordinate(
            new Point(11.2625, -10.58, 5.3625), new Quaternion(0.707f, 0f, 0f, 0.707f)
    );

    public static final Coordinate targetTwo = new Coordinate(
            new Point(10.513384, -9.085172, 3.76203), new Quaternion(0f, 0f, 0f, 1f)
    );

    public static final Coordinate targetThree = new Coordinate(
            new Point(10.6031, -7.71007, 3.76093), new Quaternion(0.707f, 0f, 0f, 0.707f)
    );

    public static final Coordinate targetFour = new Coordinate(
            new Point(9.866984, -6.673972, 5.09531), new Quaternion(-0.5f, 0.5f, -0.5f, 0.5f)
    );

    public static final Coordinate targetFive = new Coordinate(
            new Point(11.102, -8.0304, 5.9076), new Quaternion(1f, 0f, 0f, 0f)
    );

    public static final Coordinate targetSix = new Coordinate(
            new Point(12.023, -8.989, 4.8305), new Quaternion(0.5f, 0.5f, -0.5f, -0.5f)
    );

    public static final Coordinate QRCode = new Coordinate(
            new Point(11.381944, -8.566172, 3.76203), new Quaternion(0f, 0f, 0f, 1f)
    );
}

class Coordinate {
    private Point point;
    private Quaternion quaternion;

    public Coordinate(Point pt, Quaternion qt) {
        point = pt;
        quaternion = qt;
    }

    public Point getPoint() {return point;}
    public Quaternion getQuaternion() {return quaternion;}

    public void setPoint(Point pt) {point = pt;}
    public void setQuaternion(Quaternion qt) {quaternion = qt;}
}
