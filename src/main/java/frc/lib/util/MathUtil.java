package frc.lib.util;

import java.util.ArrayList;
import java.util.List;

public class MathUtil {
    public static List<Double> quadratic_equation(double a, double b, double c) {
        double discriminant = (b * b) - (4 * a * c);
        List<Double> roots = new ArrayList<>();
        if (discriminant > 0) {
            // Two distinct real roots
            double root1 = (-b + Math.sqrt(discriminant)) / (2 * a);
            double root2 = (-b - Math.sqrt(discriminant)) / (2 * a);
            roots.add(root1);
            roots.add(root2);
            return roots;
        } else {
            return roots;
        }
    }
}
