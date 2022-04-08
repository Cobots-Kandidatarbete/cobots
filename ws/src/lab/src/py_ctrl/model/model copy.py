from dataclasses import dataclass
import json
from typing import List, Optional, Dict
from model.operation import Operation, Transition
from predicates.state import State
import predicates.guards
import predicates.actions
from predicates.guards import AlwaysTrue, Guard, And
from predicates.guards import AlwaysFalse


@dataclass
class Model(object):
    initial_state: State
    operations: Dict[str, Operation]
    transitions: List[Transition]

    def __post_init__(self):
        ops = {o: "i" for o in self.operations}
        self.initial_state = self.initial_state.next(**ops)


g = predicates.guards.from_str
a = predicates.actions.from_str


def the_model() -> Model:

    initial_state = State(
        # control variables
        # trigger action when true. Change to false and then to true to trigger again
        robot_run=False,
        robot_command='move_j',   # move_j, move_l, pick, place
        robot_velocity=0.5,
        robot_acceleration=0.5,
        robot_goal_frame='unknown',   # where to go with the tool tcp
        robot_tcp_frame='suction_cup_1',  # the tool tcp to use
        gesture='unknown',

        goal_as_string="cyl_at_pose_2",
        replan=False,
        lock_run=False,

        aruco_run=False,
        lock_done=False,

        # measured variables
        robot_state="initial",  # "exec", "done", "failed"
        robot_pose='unknown',

        replanned=False,

        aruco_done=False,

        # estimated
        suction_cup_1_occ=False,  # If a suction cup is occupied or not
        suction_cup_2_occ=False,

        arucos_locked=False,
        trigger_goal_pos1=False,
        trigger_goal_pos2=False,

        # This can be either 1 or 2 (representing box 1 or 2) Initial.
        pos_object_1=1,
        pos_object_2=1,
        pos_object_3=1,
        pos_object_4=1,

        object_at_pose_1_box_1=False,
        object_at_pose_2_box_1=False,
        object_at_pose_3_box_1=False,
        object_at_pose_4_box_1=False,
        object_at_pose_1_box_2=False,
        object_at_pose_2_box_2=False,
        object_at_pose_3_box_2=False,
        object_at_pose_4_box_2=False

    )

    ops = {}

    for i in range(1, 5):
        for j in range(1, 3):
            ops[f"move_to_object_{i}_at_box{j}"] = Operation(
                name=f"move_to_object_{i}",
                precondition=Transition("pre",
                                        g(f"!robot_run && robot_state == initial && arucos_locked && pos_object_{i} == {j} && robot_pose == above_box_{j}"),
                                        a(f"robot_command = move_j, robot_run, robot_goal_frame = object_{i}")),
                postcondition=Transition("post",
                                         g(f"robot_state == done"),
                                         a(f"!robot_run, robot_pose <- object_{i}")),
                effects=(),
                to_run=Transition.default()
            )

    # move to pose_1
    ops[f"move_to_pose_1"] = Operation(
        name=f"move_to_pose_1",
        precondition=Transition("pre",
                                g(f"!robot_run && robot_state == initial && arucos_locked && robot_pose == above_pose_1"),
                                a(f"robot_command = move_j, robot_run, robot_goal_frame = pose_1")),
        postcondition=Transition("post",
                                 g(f"robot_state == done"),
                                 a(f"!robot_run, robot_pose <- pose_1")),
        effects=(),
        to_run=Transition.default()
    )

    # move to pose_2
    ops[f"move_to_pose_2"] = Operation(
        name=f"move_to_pose_2",
        precondition=Transition("pre",
                                g(f"!robot_run && robot_state == initial && robot_pose == above_box_2"),
                                a(f"robot_command = move_j, robot_run, robot_goal_frame = pose_2")),
        postcondition=Transition("post",
                                 g(f"robot_state == done"),
                                 a(f"!robot_run, robot_pose <- pose_2")),
        effects=(),
        to_run=Transition.default()
    )

    # move to camera
    ops[f"move_to_camera"] = Operation(
        name=f"move_to_camera",
        precondition=Transition("pre",
                                g(f"!robot_run && robot_state == initial && robot_pose == init_pose"),
                                a(f"robot_command = move_j, robot_run, robot_goal_frame = camera")),
        postcondition=Transition("post",
                                 g(f"robot_state == done"),
                                 a(f"!robot_run, robot_pose <- camera")),
        effects=(),
        to_run=Transition.default()
    )

    # move to init_pose
    ops[f"move_to_init_pose"] = Operation(
        name=f"move_to_init_pose",
        precondition=Transition("pre",
                                g(f"!robot_run && robot_state == initial && robot_pose != init_pose"),
                                a(f"robot_command = move_j, robot_run, robot_goal_frame = init_pose")),
        postcondition=Transition("post",
                                 g(f"robot_state == done"),
                                 a(f"!robot_run, robot_pose <- init_pose")),
        effects=(),
        to_run=Transition.default()
    )

    # move to above_box_1
    ops[f"move_to_above_box_1"] = Operation(
        name=f"move_to_above_box_1",
        precondition=Transition("pre",
                                g(f"!robot_run && robot_state == initial && robot_pose == init_pose && arucos_locked"),
                                a(f"robot_command = move_j, robot_run, robot_goal_frame = above_box_1")),
        postcondition=Transition("post",
                                 g(f"robot_state == done"),
                                 a(f"!robot_run, robot_pose <- above_box_1, !arucos_locked")),
        effects=(),
        to_run=Transition.default()
    )
    # move to above_box_2
    ops[f"move_to_above_box_2"] = Operation(
        name=f"move_to_above_box_2",
        precondition=Transition("pre",
                                g(f"!robot_run && robot_state == initial"),
                                a(f"robot_command = move_j, robot_run, robot_goal_frame = above_box_2")),
        postcondition=Transition("post",
                                 g(f"robot_state == done"),
                                 a(f"!robot_run, robot_pose <- above_box_2")),
        effects=(),
        to_run=Transition.default()
    )

    # Pick and place operations for all objects and suction cups
    for i in range(1, 5):
        for j in range(1, 5):
            for k in range(1, 3):
                ops[f"pick_object_{i}"] = Operation(
                    name=f"pick_object_{i}",
                    precondition=Transition("pre",
                                            g(f"(robot_pose == object_{i}) && !suction_cup_1_occ && object_at_pose_{j}_box_{k}"),
                                            a(f"robot_command = pick, robot_tcp_frame = suction_cup_1, robot_run")),
                    postcondition=Transition("post",
                                             g(f"robot_state == done"),
                                             a(f"!robot_run, suction_cup_1_occ, !object_at_pose_{j}_box_{k}")),
                    effects=(),
                    to_run=Transition.default()
                )

                ops[f"place_object_{i}_at_pose{j}_at_box_{k}"] = Operation(
                    name=f"place_object_{i}",
                    precondition=Transition("pre",
                                            g(f"!robot_run && robot_state == initial && robot_pose == pose_{j}_box_{k} && suction_cup_1_occ && !object_at_pose_{j}_box_{k}"),
                                            a(f"robot_command = place, robot_tcp_frame = suction_cup_1, robot_run")),
                    postcondition=Transition("post",
                                             g(f"robot_state == done"),
                                             a(f"!robot_run, !suction_cup_1_occ, pos_object_{i} = {k}, object_at_pose_{j}_box_{k}")),
                    effects=(),
                    to_run=Transition.default()
                )

    ops[f"lock_arucos"] = Operation(
        name=f"lock_arucos",
        precondition=Transition(
            "pre", g(f"!arucos_locked && robot_pose == camera"), a("lock_run")),
        postcondition=Transition(
            "post", g(f"lock_done"), a("!lock_run, arucos_locked")),
        effects=(),
        to_run=Transition.default()
    )

    # Locks the arucos above the two boxes (hopefully)
    for i in range(1, 3):
        ops[f"lock_arucos_above_box_{i}"] = Operation(
            name=f"lock_arucos_above_box_{i}",
            precondition=Transition(
                "pre", g(f"!arucos_locked && robot_pose == above_box_{i}"), a("lock_run")),
            postcondition=Transition(
                "post", g(f"lock_done"), a("!lock_run, arucos_locked")),
            effects=(),
            to_run=Transition.default()
        )

    # To be used to run "free" transitions.
    # Example of setting a goal
    transitions: List[Transition] = [
        Transition("trigger_goal_pos2_pre", g(
            "trigger_goal_pos2"), a("!replan")),
        Transition("trigger_goal_pos2_post", g("trigger_goal_pos2 && !replanned"), a(
            "!trigger_goal_pos2, replan, goal_as_string <= cyl_at_pose_2 == True")),

        Transition("trigger_goal_pos1_pre", g(
            "trigger_goal_pos1"), a("!replan")),
        Transition("trigger_goal_pos1_post", g("trigger_goal_pos1 && !replanned"), a(
            "!trigger_goal_pos1, replan, goal_as_string <= cyl_at_pose_1 == True")),
    ]

    return Model(
        initial_state,
        ops,
        transitions
    )


def from_goal_to_goal(state: State) -> Guard:
    """
    Create a goal predicate 
    """
    goal: str = state.get("goal_as_string")
    if goal != "":
        return g(goal)

    return AlwaysFalse()
