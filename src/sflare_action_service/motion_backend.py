"""Six game actions default to move_j; unrelated actions retain their backend."""
import os
from emi_coordinator import EmiCoordinator
from movej_coordinator import MoveJCoordinator, GAME_ACTIONS


def coordinator_class(action, scope, emi_class=EmiCoordinator):
    mode = os.environ.get('PALLAS_GAME_MOTION_MODE', 'movej').strip().lower()
    if mode not in {'movej', 'emi'}:
        raise ValueError('PALLAS_GAME_MOTION_MODE必须为movej或emi')
    if mode == 'movej' and scope == 'right' and action in GAME_ACTIONS | {'gozero'}:
        return MoveJCoordinator
    return emi_class
