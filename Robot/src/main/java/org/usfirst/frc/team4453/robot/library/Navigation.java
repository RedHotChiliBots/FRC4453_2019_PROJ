package org.usfirst.frc.team4453.robot.library;

public class Navigation {

    public static class Coordinate {
        public double X;
        public double Y;
        public double A;

        public Coordinate(double x_in, double y_in, double a_in) {
            X = x_in;
            Y = y_in;
            A = a_in;
        }
    }

    public static double calculateDist(double x1, double y1, double x2, double y2){
    double x = x1 - x2;
    double y = y1 - y2;
    double dist = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    return dist;
    }

    public static double calculateCoordDist(Coordinate coord1, Coordinate coord2) {
        double x = coord1.X - coord2.X;
        double y = coord1.Y - coord2.Y;
        double coordDist = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        return coordDist;
    }
    
    public static double calculateAngle (double x1, double y1, double x2, double y2) {
        double x = x2 - x1;
        double y = y2 - y1;
        double dangle = 90 - Math.toDegrees(Math.atan2(y, x));
        return dangle;
    }

    public static double calculateCoordAngle(Coordinate coord1, Coordinate coord2) {
        double x = coord2.X - coord1.X;
        double y = coord2.Y - coord1.Y;
        double dangle = 90 - Math.toDegrees(Math.atan2(y, x));
        return dangle;
    }

    public static Coordinate getCurrentCoordinate() {
        // TODO
        return new Coordinate(0, 0, 0);
    }
}