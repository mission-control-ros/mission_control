import sys
import rospy
import rospkg
import coverage

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

files = [
  'mission_control_utils_cache',
  'mission_control_utils',
  'watchdog',
  'behaviour_fail_safe',
  'behaviour',
  'mission_control_utils_constants'
]

mission_control_src = "%s/src" % mission_control_path
cov = coverage.coverage(source=[mission_control_src])

def cov_start():
    global cov, files

    cov.load()
    cov.start()

def cov_stop(report=True):
    global cov, files

    cov.stop()
    cov.save()

    mods = []

    for module in files:
        if module in sys.modules:
            mods.append(sys.modules[module])

    if report:
        cov.report(include=['*mission_control/src*'], omit=['*mission_control/src/*_node.py'], show_missing=1)
