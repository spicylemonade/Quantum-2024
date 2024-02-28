from pint import UnitRegistry
import math
ureg = UnitRegistry()
class Constants:
    class DriveTrainConstants:
        mechGearRatio = 12.6
        mechWheelRadiusInches = 6 * ureg.inch
        mechLinearDistanceConversionFactor = (1/(mechGearRatio*2*math.pi*mechWheelRadiusInches.to("meter")))
        #public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10)); 