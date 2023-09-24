import py_trees_ros
import py_trees
import rclpy
import py_trees_ros_interfaces

def create_root()->py_trees.behaviour.Behaviour:
    root=py_trees.composites.Parallel(
        name="Tutorial_one",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topcis2bb=py_trees.composites.Sequence("Topics2BB")
    battery2bb=py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched,
        threshold=30.0
    )

    tasks=py_trees.composites.Selector("Tasks")
    idle=py_trees.behaviours.Running(name="idle")
    flipper=py_trees.behaviours.Periodic(name="flip_eggs",n=2)

    root.add_child(topcis2bb)
    topcis2bb.add_child(battery2bb)
    root.add_child(tasks)
    tasks.add_child(flipper)
    tasks.add_child(idle)

    return root
    