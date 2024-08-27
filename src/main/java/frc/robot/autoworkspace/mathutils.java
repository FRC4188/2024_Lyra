package frc.robot.autoworkspace;

public class mathutils {
    static int clamp(int min, int val, int max) {
        if (val < min) return min;
        if (val > max) return max;

        return val;
    }

    static short clamp(short min, short val, short max) {
        if (val < min) return min;
        if (val > max) return max;

        return val;
    }

    static double clamp(double min, double val, double max) {
        if (val < min) return min;
        if (val > max) return max;

        return val;
    }

    //am i stupid
    static double distancePoints(double x0, double y0, double x1, double y1) {
        return Math.hypot(x0 - x1, y0 - y1);
    }

    //point = connection point for obstacle
    //closest distance between a point and a line
    static double pointFromLine(double x0, double y0, double x1, double y1, double xp, double yp) {
        return Math.abs((x1 - x0) * yp - (y1 - y0) * xp - x1 * y0 + x0 * y1) / distancePoints(x0, y0, x1, y1);
    }

    //checks if point is on a line
    static boolean pointOnLine(double x0, double y0, double x1, double y1, double xp, double yp) {
        return (x1 - x0) * yp - (y1 - y0) * xp - x1 * y0 + x0 * y1 == 0;
    }

    //checks if two lines intersect
    static boolean linesIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        double uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
    
        return uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1;
    }
    
    //checks if line intersects through rectangle
    static boolean lineIntersectRect(double x0, double y0, double x1, double y1, double xr, double yr, double wr, double hr) {
        return linesIntersect(x0, y0, x1, y1, xr - wr * 0.5, yr - hr * 0.5, xr + wr * 0.5, yr - hr * 0.5) ||
            linesIntersect(x0, y0, x1, y1, xr + wr * 0.5, yr - hr * 0.5, xr + wr * 0.5, yr + hr * 0.5) ||
            linesIntersect(x0, y0, x1, y1, xr + wr * 0.5, yr + hr * 0.5, xr - wr * 0.5, yr + hr * 0.5) ||
            linesIntersect(x0, y0, x1, y1, xr - wr * 0.5, yr + hr * 0.5, xr - wr * 0.5, yr - hr * 0.5);
    }
    
    //checks if line intersects through circle
    static boolean lineIntersectCircle(double x0, double y0, double x1, double y1, double x2, double y2, double r2) {
        double b = 2.f * ((x0 - x2) * (x1 - x0) + (y0 - y2) * (y1 - y0));
        double c = ((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2) - r2 * r2);
        double a = ((y1 - y0) * (y1 - y0) + (x1 - x0) * (x1 - x0));
    
        double discriminate = b * b - 4.f * a * c;
    
        if (discriminate < 0) { return false; }
        double val1 = (-b + Math.sqrt(discriminate)) / (2.f * a);
        double val2 = (-b - Math.sqrt(discriminate)) / (2.f * a);
        return ((0 <= val1 && val1 <= 1) || (0 <= val2 && val2 <= 1));
    }

    //cubic interpolation, 4 points and a t value needed
    static double cubicInterpolation(double val0, double val1, double val2, double val3, double t) {
        //t modulated to 0 and <1
        t -= (int) t;

        val0 = clamp(val1 - 100, val0, val1 + 100);
        val3 = clamp(val2 - 100, val3, val2 + 100);

        double tt = t * t;
        double ttt = tt * t;

        double q1 = -ttt + 2.f * tt - t;
        double q2 = 3.f * ttt - 5.f * tt + 2.f;
        double q3 = -3.f * ttt + 4.f * tt + t;
        double q4 = ttt - tt;

        return (0.5 * (val0 * q1 + val1 * q2 + val2 * q3 + val3 * q4));
    }
    





}
