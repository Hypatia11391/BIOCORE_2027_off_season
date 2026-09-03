from enum import Enum
from typing import override
import pint
from wpimath.kinematics import ChassisSpeeds

from src.coordinate_systems import unit_registry  # sets application registry
from src.coordiante_systems.unit_registry import LENGTH_DIMENSIONALITY, ANGLE_DIMENSIONALITY, SPEED_DIMENSIONALITY, ANGULAR_SPEED_DIMENSIONALITY

class LinearCoordinateConvention2d(Enum):
    # CONVENTION_NAME = (to_default_convention_fn, from_default_convention_fn)
    X_FORWARD_Y_LEFT = (lambda x,y:x,y, lambda x,y:x,y)
    X_LEFT_Y_FORWARD = (lambda x,y:y,x, lambda x,y:y,x)

class AngularCoordinateConvention1d(Enum):
    CW_POSITIVE = (lambda t:t, lambda t:t)
    CCW_POSITIVE = (lambda t:-t, lambda t:-t)


class SpatialQuantities2d:
    def __init__(self, x: pint.Quantity, y: pint.Quantity, linear_coordinate_convention: LinearCoordinateConvention2d, theta: pint.Quantity, angular_coordinate_convention: AngularCoordinateConvention1d):
        self.ureg = pint.get_application_registry()
        for q in (x,y,theta): assert isinstance(q, pint.Quantity)

        to_default_linear_convention, _ = linear_coordinate_convention.value
        self.x, self.y = to_default_linear_convention(x,y)
        to_default_angular_convention, _ = angular_coordinate_convention.value
        self.theta = to_default_angular_convention(theta)
    
    def linear(unit, coordinate_convention):
        _, from_default_convention = coordinate_convention.value
        return from_default_convention(self.x.to(unit), self.y.to(unit))

    def angular(unit, coordinate_convention):
        _, from_default_convention = coordinate_convention.value
        return from_default_convention(self.theta.to(unit))

class SE2d(SpatialQuantities2d):
    @override
    def __init__(self, x: pint.Quantity, y: pint.Quantity, linear_coordinate_convention: LinearCoordinateConvention2d, theta: pint.Quantity, angular_coordinate_convention: AngularCoordinateConvention1d):
        super().__init__(x, y, linear_coordinate_convention, theta, angular_coordinate_convention)
        
        assert x.dimensionality == y.dimensionality == LENGTH_DIMENSIONALITY
        assert omega.dimensionality == ANGLE_DIMENSIONALITY

class Velocities2d(SpatialQuantities2d):
    @override
    def __init__(self, vx: pint.Quantity, vy: pint.Quantity, linear_coordinate_convention: LinearCoordinateConvention2d, omega: pint.Quantity, angular_coordinate_convention: AngularCoordinateConvention1d):
        super().__init__(vx, vy, linear_coordinate_convention, omega, angular_coordinate_convention)
        
        assert vx.dimensionality == vy.dimensionality == SPEED_DIMENSIONALITY
        assert omega.dimensionality == ANGULAR_SPEED_DIMENSIONALITY

    def from_chassis_speeds(speeds: ChassisSpeeds):
        ureg = pint.get_application_registry()
        return Velocities2d(speeds.vx*ureg.mps, speeds.vy*ureg.mps, LinearCoordinateConvention2d.X_FORWARD, speeds.omega*(ureg.rad/ureg.s), AngularCoordinateConvention1d.CCW_POSITIVE)

    def to_chassis_speeds(self):
        return ChassisSpeeds(*self.linear('m/s', LinearCoordinateConvention2d.X_FORWARD_Y_LEFT), self.angular('rad/s', AngularCoordinateConvention1d.CCW_POSITIVE))
