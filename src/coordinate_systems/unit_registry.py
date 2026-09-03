import pint

ureg = pint.get_application_registry()
if ureg is None or ureg._registry is None:
    ureg = pint.UnitRegistry()
    pint.set_application_registry(ureg)

LENGTH_DIMENSIONALITY = ureg.meter.dimensionality
ANGLE_DIMENSIONALITY = ureg.rad.dimensionality
SPEED_DIMENSIONALITY = ureg.mps.dimensionality
ANGULAR_SPEED_DIMENSIONALITY = ureg.rpm.dimensionality
