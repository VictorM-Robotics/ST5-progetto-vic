"""T.3 integration adapter: high-level manoeuvre commands, WITHOUT motor control.

The caller must validate camera events, clearance and successful line reacquisition.
State is advanced only by acknowledgements; no timers or automatic motor actuation.
Supports departure from a confirmed node or a confirmed stationary partial target.
The caller must verify physical stop before loading a route from mid-segment.
"""
from dataclasses import dataclass, field
from math import isfinite
from typing import Any, Dict, List, Optional
from grid_map import GridMap
from localization import RobotLocalizer
from path_planning import plan_mission_from_state


@dataclass
class NavigationController:
    grid: GridMap
    localizer: RobotLocalizer
    phase: str = 'IDLE'
    plan: Optional[Dict[str, Any]] = None
    step_index: int = 0
    events: List[Dict[str, Any]] = field(default_factory=list)

    def _emit(self, typ: str, **kwargs) -> Dict[str, Any]:
        event = {'event': typ, **kwargs}
        self.events.append(event)
        return event

    def load_mission(self, target: Dict[str, Any], blocked_edges=None, *, stopped=False) -> Dict[str, Any]:
        if self.phase not in ('IDLE', 'COMPLETED', 'STOPPED'):
            raise RuntimeError('Stop/finish current mission before replacing it')
        if self.localizer.from_node is not None and not stopped:
            raise RuntimeError('Mid-segment replanning requires confirmed physical stop')
        plan = plan_mission_from_state(self.grid, self.localizer.snapshot(), target, blocked_edges)
        if not plan['feasible']:
            raise ValueError(plan['reason'])
        self.plan, self.step_index = plan, 0
        self.phase = 'READY' if plan['detailed_steps'] else 'COMPLETED'
        return self._emit('MISSION_LOADED', path=plan['path'], phase=self.phase)

    def next_command(self) -> Dict[str, Any]:
        if self.phase != 'READY':
            raise RuntimeError('No manoeuvre ready')
        step = self.plan['detailed_steps'][self.step_index]
        a, b = step['segment']
        loc = self.localizer
        if loc.from_node is None:
            valid = loc.current_node == a and step['start_progress'] == 0.0
        else:
            same = (loc.from_node, loc.to_node) == (a, b)
            reverse = (loc.from_node, loc.to_node) == (b, a)
            progress = loc.progress if same else 1.0 - loc.progress
            valid = (same or reverse) and abs(progress - step['start_progress']) <= 1e-8
        if not valid:
            return self.emergency_stop('Unexpected position: reconcile localization')
        self.phase = 'MANEUVERING'
        return self._emit('MANEUVER_REQUEST', action=step['action'],
                          at_node=a if loc.from_node is None else None, next_node=b,
                          segment=list(step['segment']), start_progress=step['start_progress'],
                          note=('Specific curve/heading alignment required' if
                                step['action'] == 'ALIGN_REQUIRED' else None))

    def maneuver_result(self, success: bool, line_reacquired: bool) -> Dict[str, Any]:
        if self.phase != 'MANEUVERING':
            raise RuntimeError('No manoeuvre pending')
        if not success or not line_reacquired:
            return self.emergency_stop('Manoeuvre failed or line not reacquired')
        step = self.plan['detailed_steps'][self.step_index]
        if self.localizer.from_node is None:
            self.localizer.begin_segment(step['segment'][1])
        elif (self.localizer.from_node, self.localizer.to_node) != step['segment']:
            # Reference changes only AFTER the physical reversal was confirmed.
            self.localizer.reverse_direction()
        self.phase = 'FOLLOWING'
        return self._emit('FOLLOW_LINE_REQUEST', segment=list(step['segment']),
                          start_progress=step['start_progress'],
                          stop_progress=step['end_progress'])

    def progress_update(self, fraction: float) -> Dict[str, Any]:
        """Odometry measurement in [0,1]; it does NOT itself confirm reaching target."""
        if self.phase != 'FOLLOWING':
            raise RuntimeError('Robot is not in a traversing state')
        step = self.plan['detailed_steps'][self.step_index]
        if (not isinstance(fraction, (int, float)) or isinstance(fraction, bool)
                or not isfinite(fraction) or not 0 <= fraction <= 1):
            return self.emergency_stop('Invalid measured progress')
        if fraction < step['start_progress'] - 1e-8:
            return self.emergency_stop('Progress before departure position')
        if fraction > step['end_progress'] + 1e-8:
            return self.emergency_stop('Passed target progress without confirmed stop')
        self.localizer.update_progress(fraction)
        return self._emit('PROGRESS', fraction=fraction)

    def reached_partial_target(self, stopped: bool) -> Dict[str, Any]:
        if self.phase != 'FOLLOWING':
            raise RuntimeError('Not following line')
        step = self.plan['detailed_steps'][self.step_index]
        if step['end_progress'] >= 1.0 - 1e-8 or self.step_index != len(self.plan['detailed_steps']) - 1:
            raise RuntimeError('Current step does not end at a partial target')
        if not stopped or abs(self.localizer.progress - step['end_progress']) > 1e-3:
            return self.emergency_stop('Partial target stop not physically verified')
        self.phase = 'COMPLETED'
        return self._emit('MISSION_COMPLETED', position=self.localizer.snapshot())

    def confirmed_intersection(self, node: str) -> Dict[str, Any]:
        """Call ONLY on validated, debounced physical arrival, not first visual sighting."""
        if self.phase != 'FOLLOWING':
            raise RuntimeError('Unexpected intersection event outside FOLLOWING')
        step = self.plan['detailed_steps'][self.step_index]
        expected = step['segment'][1]
        if step['end_progress'] < 1.0 - 1e-8 or node != expected:
            return self.emergency_stop('Unexpected intersection or partial target not finished')
        self.localizer.confirm_arrival(node)
        self.step_index += 1
        self.phase = 'COMPLETED' if self.step_index == len(self.plan['detailed_steps']) else 'READY'
        return self._emit('INTERSECTION_CONFIRMED', node=node, phase=self.phase)

    def emergency_stop(self, reason: str) -> Dict[str, Any]:
        """Safety output ONLY. Hardware supervisor must independently cut motor power."""
        self.phase = 'STOPPED'
        return self._emit('STOP_REQUEST', reason=reason, requires_hardware_stop=True)

    def snapshot(self) -> Dict[str, Any]:
        return {'phase': self.phase, 'step_index': self.step_index,
                'localization': self.localizer.snapshot()}
