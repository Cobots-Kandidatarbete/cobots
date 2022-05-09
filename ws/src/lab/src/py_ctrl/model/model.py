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
        r3_robot_run=False,
        r3_robot_command='move_j',   # move_j, move_l, pick, place
        r3_robot_velocity=0.5,
        r3_robot_acceleration=0.5,
        r3_robot_goal_frame='unknown',   # where to go with the tool tcp
        r3_robot_tcp_frame='r3_suction_cup_1',  # the tool tcp to use

        r4_robot_run=False,
        r4_robot_command='move_j',   # move_j, move_l, pick, place
        r4_robot_velocity=0.5,
        r4_robot_acceleration=0.5,
        r4_robot_goal_frame='unknown',   # where to go with the tool tcp
        r4_robot_tcp_frame='r4_suction_cup_1',  # the tool tcp to use

        gesture='unknown',


        bool_to_plc_1=False,
        bool_to_plc_2=False,
        bool_to_plc_3=False,
        bool_to_plc_4=False,
        bool_to_plc_5=False,
        int_to_plc_1=0,
        int_to_plc_2=0,
        int_to_plc_3=0,
        int_to_plc_4=0,
        int_to_plc_5=0,

        goal_as_string="cyl_at_pose_2",
        replan=False,
        lock_run=False,

        aruco_run=False,
        lock_done=False,

        # measured variables
        r3_robot_state="initial",  # "exec", "done", "failed"
        r4_robot_state="initial",
        r3_robot_pose='unknown',
        r4_robot_pose='unknown',

        replanned=False,

        aruco_done=False,

        # estimated
        suction_cup_1_occ=False,  # If a suction cup is occupied or not
        suction_cup_2_occ=False,

        arucos_locked=False,
        arucos_locked_boxes=False,
        lidar_locked=False,
        trigger_goal_pos1=False,
        trigger_goal_pos2=False,

        # This can be either 1 or 2 (representing box 1 or 2) Initial.
        pos_object_1=1,
        pos_object_2=1,
        # pos_object_3=1,
        # pos_object_4=1,

        object_at_pose_1_box_1=True,
        object_at_pose_2_box_1=True,
        # object_at_pose_3_box_1=False,
        # object_at_pose_4_box_1=False,
        object_at_pose_1_box_2=False,
        object_at_pose_2_box_2=False,
        # object_at_pose_3_box_2=False,
        # object_at_pose_4_box_2=False

        holding=0,
    )

    nr_of_boxes = 2  # 2 boxes
    nr_of_poses = 2  # 2 poses per box
    nr_of_objects = 2  # 2 objects

    ops = {}

    

    # move to camera
    ops[f"r4_move_to_camera"] = Operation(
        name=f"r4_move_to_camera",
        precondition=Transition("pre",
                                g(f"!r4_robot_run && r4_robot_state == initial && r3_robot_pose == init_pose && r4_robot_pose == init_pose"),
                                a(f"r4_robot_command = move_j, r4_robot_run, r4_robot_goal_frame = camera")),
        postcondition=Transition("post",
                                 g(f"r4_robot_state == done"),
                                 a(f"!r4_robot_run, r4_robot_pose <- camera")),
        effects=(),
        to_run=Transition.default()
    )

    
    ops[f"r3_move_to_init_pose"] = Operation(
        name=f"r3_move_to_init_pose",
        precondition=Transition("pre",
                                g(f"!r3_robot_run && r3_robot_state == initial && r4_robot_pose != r3_init_pose"),
                                a(f"r3_robot_command = move_j, r3_robot_run, r3_robot_goal_frame = r3_init_pose")),
        postcondition=Transition("post",
                                g(f"r3_robot_state == done"),
                                a(f"!r3_robot_run, r3_robot_pose <- r3_init_pose")),
        effects=(),
        to_run=Transition.default()
    )

    ops[f"r4_move_to_init_pose"] = Operation(
        name=f"r4_move_to_init_pose",
        precondition=Transition("pre",
                                g(f"!r4_robot_run && r4_robot_state == initial && r3_robot_pose != r4_init_pose"),
                                a(f"r4_robot_command = move_j, r4_robot_run, r4_robot_goal_frame = r4_init_pose")),
        postcondition=Transition("post",
                                g(f"r4_robot_state == done"),
                                a(f"!r4_robot_run, r4_robot_pose <- r4_init_pose")),
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
