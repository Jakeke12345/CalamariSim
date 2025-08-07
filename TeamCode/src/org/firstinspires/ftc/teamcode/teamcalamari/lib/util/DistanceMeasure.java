package org.firstinspires.ftc.teamcode.teamcalamari.lib.util;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Objects;

public class DistanceMeasure {

    private final double value;
    private final DistanceUnit unit;

    public static final DistanceMeasure ZERO = inches(0);

    public static DistanceMeasure inches(double value){
        return new DistanceMeasure(value, DistanceUnit.INCH);
    }
    public static DistanceMeasure mms(double value){
        return new DistanceMeasure(value, DistanceUnit.MM);
    }

    public DistanceMeasure(double value, DistanceUnit unit){
        this.value = value;
        this.unit = unit;
    }

    public double getValue(DistanceUnit unit){
        return unit.fromUnit(this.unit, value);
    }

    public double inches(){
        return getValue(DistanceUnit.INCH);
    }
    public double mms(){
        return getValue(DistanceUnit.MM);
    }

    @Override
    public String toString() {
        return String.format("%.2f %s", value, unit);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof DistanceMeasure other)) return false;
        return Double.compare(value, other.value) == 0 &&
                unit.equals(other.unit);
    }

    @Override
    public int hashCode() {
        return Objects.hash(value, unit);
    }
}
