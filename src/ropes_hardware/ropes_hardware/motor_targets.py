"""Select a reachable motor pulse setpoint, including at calibrated boundaries."""

from itertools import product
import math


def quantized_motor_target(model, settings, target, measured, home, motor_steps):
    """Return the nearest feasible combination of the adjacent pulse coordinates."""
    motor_target = settings.to_motor_angles(target, home)
    choices = []
    for angle, step in zip(motor_target, motor_steps):
        pulses = angle / step
        low, high = math.floor(pulses), math.ceil(pulses)
        neighbors = (low,) if low == high else (low, high)
        choices.append(tuple(pulse * step for pulse in neighbors))

    def distance(candidate):
        return sum(((motor - desired) / axis.gear_ratio) ** 2
                   for motor, desired, axis in zip(candidate, motor_target, settings.axes))

    for motors in sorted(product(*choices), key=distance):
        drums = settings.to_drum_angles(motors, home)
        if any(abs(target_angle - actual_angle) > settings.max_command_lead_rad
               for target_angle, actual_angle in zip(drums, measured)):
            continue
        try:
            settings.check_drums(drums)
            model.forward(drums)
        except ValueError:
            continue
        return motors
    raise ValueError('No adjacent motor pulse target is inside the calibrated workspace and lead')
