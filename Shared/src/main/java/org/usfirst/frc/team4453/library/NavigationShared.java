package org.usfirst.frc.team4453.library;

import com.google.gson.Gson;

public class NavigationShared {
    public static class Coordinate {
        public double X;
        public double Y;
        public double A;

        public Coordinate(double x_in, double y_in, double a_in) {
            X = x_in;
            Y = y_in;
            A = a_in;
        }

        public String serialize() {
            Gson gson = new Gson();

            return gson.toJson(this);
        }

        public static Coordinate deserialize(String dat) {
            Gson gson = new Gson();
            return gson.fromJson(dat, Coordinate.class);
        }
    }
}