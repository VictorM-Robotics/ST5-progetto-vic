"""Full mission with fabricated sensor events. No camera, network or motors."""
from grid_map import GridMap
from localization import RobotLocalizer
from navigation_controller import NavigationController
from mission_controller import MissionController


def run_demo():
    grid = GridMap()
    mission = MissionController(NavigationController(grid, RobotLocalizer(grid)))
    print(mission.start({'type': 'edge', 'edge': ('B', 'C'), 'progress': 0.75}))
    while mission.phase != 'COMPLETED':
        if mission.phase in ('OUTBOUND', 'RETURNING'):
            print(mission.next_command())
            print(mission.maneuver_result(True, True))  # simulated confirmations
            nav = mission.navigation
            step = nav.plan['detailed_steps'][nav.step_index]
            mission.progress_update(step['end_progress'])
            if step['end_progress'] < 1:
                print(mission.reached_partial_target(stopped=True))
            else:
                print(mission.confirmed_intersection(step['segment'][1]))
        elif mission.phase in ('DELIVERY_STOP_PENDING', 'BASE_STOP_PENDING'):
            print(mission.confirm_stop(stopped=True))
        elif mission.phase == 'WAITING_DELIVERY':
            first = len(mission.events)
            result = mission.confirm_delivery(delivered=True, stopped=True)
            for event in mission.events[first:]:
                print(event)
            print(result)
        else:
            raise RuntimeError('Unexpected phase: ' + mission.phase)
    return mission


if __name__ == '__main__':
    run_demo()
