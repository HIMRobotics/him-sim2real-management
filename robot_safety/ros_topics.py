"""ROS topic names shared by the dry-run node adapters."""

ROBOT_FEEDBACK = "/safety/robot_feedback"
POLICY_OBSERVATION = "/safety/policy_observation"
PRECHECK_COMMAND = "/safety/precheck"
POSTCHECK_COMMAND = "/safety/postcheck"
ABORT = "/safety/abort"
DRY_RUN_OUTPUT = "/safety/dry_run_output"
CHECKER_ARM = "/safety/checker/arm"
GATE_ARM = "/safety/motor_gate/arm"
CONTROLLER_READY = "/safety/controller_stop/ready"
ABORT_REPEAT_SECS = 0.1
