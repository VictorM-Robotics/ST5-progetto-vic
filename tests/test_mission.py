import math
import unittest
from grid_map import GridMap, BASE, NODES
from localization import RobotLocalizer
from navigation_controller import NavigationController
from mission_controller import MissionController
from encoder_odometry import EncoderCalibration
from odometry_adapter import NavigationOdometryAdapter


def navigation(grid=None, localizer=None):
    grid = grid or GridMap()
    return NavigationController(grid, localizer or RobotLocalizer(grid))


def travel(mission):
    for _ in range(100):
        if mission.phase not in ('OUTBOUND', 'RETURNING'):
            return
        request = mission.next_command()
        assert request['event'] == 'MANEUVER_REQUEST'
        mission.maneuver_result(True, True)
        nav = mission.navigation
        step = nav.plan['detailed_steps'][nav.step_index]
        mission.progress_update(step['end_progress'])
        if step['end_progress'] < 1:
            mission.reached_partial_target(True)
        else:
            mission.confirmed_intersection(step['segment'][1])
    raise AssertionError('Mission did not terminate')


class MissionTests(unittest.TestCase):
    def round_trip(self, target):
        mission = MissionController(navigation())
        mission.start(target)
        travel(mission)
        self.assertEqual(mission.phase, 'DELIVERY_STOP_PENDING')
        self.assertNotIn('DELIVERY_COMPLETED', [e['event'] for e in mission.events])
        with self.assertRaises(RuntimeError):
            mission.confirm_delivery(delivered=True, stopped=True)
        mission.confirm_stop(stopped=True)
        self.assertEqual(mission.phase, 'WAITING_DELIVERY')
        mission.confirm_delivery(delivered=True, stopped=True)
        self.assertEqual(mission.phase, 'RETURNING')
        travel(mission)
        self.assertEqual(mission.phase, 'BASE_STOP_PENDING')
        self.assertNotIn('RETURN_COMPLETED', [e['event'] for e in mission.events])
        mission.confirm_stop(stopped=True)
        self.assertEqual(mission.phase, 'COMPLETED')
        self.assertEqual(mission.navigation.localizer.current_node, BASE)
        self.assertIsNone(mission.navigation.localizer.from_node)
        self.assertEqual(sum(e['event'] == 'DELIVERY_COMPLETED' for e in mission.events), 1)
        return mission

    def test_every_grid_node(self):
        for node in NODES:
            with self.subTest(node=node):
                self.round_trip({'type': 'node', 'node': node})

    def test_partial_targets_both_references(self):
        grid = GridMap()
        for a in NODES:
            for b in sorted(grid.adjacency[a]):
                if b == BASE:
                    continue
                for fraction in (.25, .5, .75):
                    with self.subTest(edge=(a, b), fraction=fraction):
                        self.round_trip({'type': 'edge', 'edge': (a, b), 'progress': fraction})

    def test_mid_segment_reversal_requires_confirmation(self):
        grid = GridMap()
        loc = RobotLocalizer(grid, current_node='B', orientation='E')
        loc.begin_segment('C')
        loc.update_progress(.75)
        nav = navigation(grid, loc)
        with self.assertRaises(RuntimeError):
            nav.load_mission({'type': 'node', 'node': 'B'})
        nav.load_mission({'type': 'node', 'node': 'B'}, stopped=True)
        request = nav.next_command()
        self.assertEqual(request['action'], 'UTURN')
        self.assertIsNone(request['at_node'])
        self.assertEqual(loc.from_node, 'B')
        nav.maneuver_result(True, True)
        self.assertEqual((loc.from_node, loc.to_node, loc.progress), ('C', 'B', .25))
        nav.progress_update(1)
        nav.confirmed_intersection('B')
        self.assertEqual(nav.phase, 'COMPLETED')

    def test_failed_reversal_does_not_change_reference(self):
        grid = GridMap()
        loc = RobotLocalizer(grid, current_node='B')
        loc.begin_segment('C'); loc.update_progress(.75)
        nav = navigation(grid, loc)
        nav.load_mission({'type': 'node', 'node': 'B'}, stopped=True)
        nav.next_command()
        self.assertEqual(nav.maneuver_result(True, False)['event'], 'STOP_REQUEST')
        self.assertEqual((loc.from_node, loc.to_node, loc.progress), ('B', 'C', .75))

    def test_odometry_keeps_partial_origin(self):
        grid = GridMap(); grid.set_length('B', 'C', 1.)
        loc = RobotLocalizer(grid, current_node='B')
        loc.begin_segment('C'); loc.update_progress(.75)
        nav = navigation(grid, loc)
        nav.load_mission({'type': 'node', 'node': 'B'}, stopped=True)
        nav.next_command(); nav.maneuver_result(True, True)
        # Fictional geometry: 1000 ticks per metre.
        cal = EncoderCalibration(1000, 1 / math.pi, (1, 1, 1, 1), .3)
        adapter = NavigationOdometryAdapter(nav, cal)
        adapter.begin_following((0, 0, 0, 0))
        self.assertAlmostEqual(adapter.update((100,)*4)['estimated_progress'], .35)
        self.assertAlmostEqual(adapter.update((750,)*4)['estimated_progress'], 1.)

    def test_continuing_mid_segment(self):
        grid = GridMap(); loc = RobotLocalizer(grid, current_node='B')
        loc.begin_segment('C'); loc.update_progress(.4)
        nav = navigation(grid, loc)
        nav.load_mission({'type': 'edge', 'edge': ('B', 'C'), 'progress': .75}, stopped=True)
        self.assertEqual(nav.next_command()['action'], 'STRAIGHT')
        nav.maneuver_result(True, True)
        self.assertEqual(loc.progress, .4)
        nav.progress_update(.75)
        nav.reached_partial_target(True)
        self.assertEqual(nav.phase, 'COMPLETED')

    def test_wrong_intersection_stops_entire_mission(self):
        mission = MissionController(navigation())
        mission.start({'type': 'node', 'node': 'B'})
        mission.next_command(); mission.maneuver_result(True, True)
        self.assertEqual(mission.confirmed_intersection('A')['event'], 'STOP_REQUEST')
        self.assertEqual(mission.phase, 'STOPPED')
        with self.assertRaises(RuntimeError): mission.next_command()

    def test_unconfirmed_stop_never_completes(self):
        mission = MissionController(navigation())
        mission.start({'type': 'node', 'node': 'Y'}); travel(mission)
        mission.confirm_stop(stopped=False)
        self.assertEqual(mission.phase, 'STOPPED')

    def test_overshoot_stops(self):
        mission = MissionController(navigation())
        mission.start({'type': 'edge', 'edge': (BASE, 'Y'), 'progress': .5})
        mission.next_command(); mission.maneuver_result(True, True)
        self.assertEqual(mission.progress_update(.6)['event'], 'STOP_REQUEST')
        self.assertEqual(mission.phase, 'STOPPED')

    def test_invalid_odometry_requests_stop(self):
        for fraction in (float('nan'), float('inf'), -.1, 'bad', True):
            with self.subTest(fraction=fraction):
                mission = MissionController(navigation())
                mission.start({'type': 'node', 'node': 'Y'})
                mission.next_command(); mission.maneuver_result(True, True)
                self.assertEqual(mission.progress_update(fraction)['event'], 'STOP_REQUEST')
                self.assertEqual(mission.phase, 'STOPPED')

    def test_no_new_departure_before_delivery_or_after_stop(self):
        mission = MissionController(navigation())
        mission.start({'type': 'node', 'node': 'Y'}); travel(mission)
        mission.confirm_stop(stopped=True)
        with self.assertRaises(RuntimeError): mission.next_command()
        with self.assertRaises(ValueError):
            mission.confirm_delivery(delivered=False, stopped=True)
        self.assertEqual(mission.phase, 'WAITING_DELIVERY')
        mission.confirm_delivery(delivered=True, stopped=False)
        self.assertEqual(mission.phase, 'STOPPED')

    def test_return_unreachable_stops_after_confirmed_delivery(self):
        mission = MissionController(navigation())
        mission.start({'type': 'node', 'node': 'Y'}); travel(mission)
        mission.confirm_stop(stopped=True)
        mission.blocked_edges = ((BASE, 'Y'),)
        event = mission.confirm_delivery(delivered=True, stopped=True)
        self.assertEqual(event['event'], 'STOP_REQUEST')
        self.assertEqual(mission.phase, 'STOPPED')
        self.assertIn('DELIVERY_COMPLETED', [e['event'] for e in mission.events])

    def test_base_target_does_not_trigger_motion(self):
        mission = MissionController(navigation())
        self.assertEqual(mission.start({'type': 'node', 'node': BASE})['event'], 'STOP_REQUEST')
        mission.confirm_stop(stopped=True)
        mission.confirm_delivery(delivered=True, stopped=True)
        self.assertEqual(mission.phase, 'BASE_STOP_PENDING')
        mission.confirm_stop(stopped=True)
        self.assertEqual(mission.phase, 'COMPLETED')


if __name__ == '__main__':
    unittest.main()
