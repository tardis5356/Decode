package org.firstinspires.ftc.teamcode.Zenith;



/** Math utility functions. */
public final class MathUtil {
    private MathUtil() {
        throw new AssertionError("utility class");
    }



    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }




    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue The value to end at.
     * @param t How far between the two values to interpolate. This is clamped to [0, 1].
     * @return The interpolated value.
     */
    public static double interpolate(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * MathUtil.clamp(t, 0, 1);
    }

    /**
     * Return where within interpolation range [0, 1] q is between startValue and endValue.
     *
     * @param startValue Lower part of interpolation range.
     * @param endValue Upper part of interpolation range.
     * @param q Query.
     * @return Interpolant in range [0, 1].
     */
    public static double inverseInterpolate(double startValue, double endValue, double q) {
        double totalRange = endValue - startValue;
        if (totalRange <= 0) {
            return 0.0;
        }
        double queryToStart = q - startValue;
        if (queryToStart <= 0) {
            return 0.0;
        }
        return queryToStart / totalRange;
    }




    }
