"""Delivery/return lifecycle. Events are requests, never physical acknowledgements.

One robot-side owner must serialize calls and dispatch STOP_REQUEST to the motor
supervisor. PC messages must never stand in for camera or motor confirmations.
"""
from copy import deepcopy
from grid_map import BASE


class MissionController:
    def __init__(self, navigation):
        self.navigation = navigation
        self.phase = 'IDLE'
        self.events = []
        self.blocked_edges = ()

    def _emit(self, event, **data):
        result = {'event': event, 'phase': self.phase, **data}
        self.events.append(result)
        return result

    def _observe(self, event):
        if event['event'] == 'STOP_REQUEST':
            self.phase = 'STOPPED'
            return self._emit('STOP_REQUEST', reason=event['reason'], requires_hardware_stop=True)
        if self.navigation.phase == 'COMPLETED':
            self.phase = ('DELIVERY_STOP_PENDING' if self.phase == 'OUTBOUND'
                          else 'BASE_STOP_PENDING')
            return self._emit('STOP_REQUEST', reason='Destination reached; confirm physical stop',
                              requires_hardware_stop=True)
        return event

    def start(self, target, blocked_edges=None):
        if self.phase not in ('IDLE', 'COMPLETED'):
            raise RuntimeError('A mission is already active or requires recovery')
        loc = self.navigation.localizer
        if loc.from_node is not None or loc.current_node != BASE:
            raise RuntimeError('A delivery must start at confirmed START')
        blocked = tuple(tuple(edge) for edge in (blocked_edges or ()))
        event = self.navigation.load_mission(deepcopy(target), blocked)
        self.blocked_edges = blocked
        self.phase = 'OUTBOUND'
        return self._observe(event)

    def _navigate(self, method, *args, **kwargs):
        if self.phase not in ('OUTBOUND', 'RETURNING'):
            raise RuntimeError('Navigation is not active')
        return self._observe(getattr(self.navigation, method)(*args, **kwargs))

    def next_command(self):
        return self._navigate('next_command')

    def maneuver_result(self, success, line_reacquired):
        return self._navigate('maneuver_result', success, line_reacquired)

    def progress_update(self, fraction):
        return self._navigate('progress_update', fraction)

    def confirmed_intersection(self, node):
        """Robot-side validated arrival, including the terminal START marker."""
        return self._navigate('confirmed_intersection', node)

    def reached_partial_target(self, stopped):
        return self._navigate('reached_partial_target', stopped)

    def confirm_stop(self, *, stopped):
        if self.phase not in ('DELIVERY_STOP_PENDING', 'BASE_STOP_PENDING'):
            raise RuntimeError('No destination stop pending')
        if not stopped:
            return self.emergency_stop('Physical stop was not confirmed')
        if self.phase == 'DELIVERY_STOP_PENDING':
            self.phase = 'WAITING_DELIVERY'
            return self._emit('DELIVERY_POSITION_REACHED', message_fr='Point de livraison atteint.')
        self.phase = 'COMPLETED'
        return self._emit('RETURN_COMPLETED', message_fr='Retour au point de départ terminé.')

    def confirm_delivery(self, *, delivered, stopped):
        """Call from the delivery procedure; arrival alone is not delivery."""
        if self.phase != 'WAITING_DELIVERY':
            raise RuntimeError('Not waiting for delivery confirmation')
        if not stopped:
            return self.emergency_stop('Robot moved during delivery')
        if not delivered:
            raise ValueError('Delivery has not been confirmed')
        self._emit('DELIVERY_COMPLETED', message_fr='Livraison effectuée.')
        try:
            event = self.navigation.load_mission({'type': 'node', 'node': BASE},
                                                  self.blocked_edges, stopped=True)
        except (ValueError, RuntimeError) as exc:
            return self.emergency_stop('Cannot plan return: ' + str(exc))
        self.phase = 'RETURNING'
        return self._observe(event)

    def emergency_stop(self, reason):
        self.navigation.emergency_stop(reason)
        self.phase = 'STOPPED'
        return self._emit('STOP_REQUEST', reason=reason, requires_hardware_stop=True)
