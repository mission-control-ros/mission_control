
class Constants:

    MAX_CBS = 4
    """int: how many times get_var functions can call itself"""

    VAR_RECHECK_DELAY = 0.5
    """float: how long function sleeps, before it goes into recursion"""

    VAR_GET_TOPIC = "/mission_control/variable/get"
    """string: topic name for getting variables"""

    VAR_SET_TOPIC = "/mission_control/variable/set"
    """string: topic name for setting variables"""

    QUEUE_SIZE = 10
    """int: determines queue size for rospy publishers"""
