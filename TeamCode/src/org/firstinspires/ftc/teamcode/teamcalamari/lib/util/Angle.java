package org.firstinspires.ftc.teamcode.teamcalamari.lib.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Objects;

public class Angle {

    private final double value;
    private final AngleUnit unit;

    public static final Angle ZERO = degs(0);

    public static Angle degs(double value){
        return new Angle(value, AngleUnit.DEGREES);
    }
    public static Angle rads(double value){
        return new Angle(value, AngleUnit.RADIANS);
    }

    public Angle(double value, AngleUnit unit){
        this.value = value;
        this.unit = unit;
    }

    public double getValue(AngleUnit unit){
        return unit.fromUnit(this.unit, value);
    }

    public double degs(){
        return getValue(AngleUnit.DEGREES);
    }
    public double rads(){
        return getValue(AngleUnit.RADIANS);
    }

    public Angle plus(Angle angle){
        return new Angle(value+angle.getValue(unit), unit);
    }
    public Angle minus(Angle angle){
        return new Angle(value-angle.getValue(unit), unit);
    }

    @Override
    public String toString() {
        return String.format("%.2f %s", value, unit);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Angle other)) return false;
        return Double.compare(value, other.value) == 0 &&
                unit.equals(other.unit);
    }

    @Override
    public int hashCode() {
        return Objects.hash(value, unit);
    }
}
