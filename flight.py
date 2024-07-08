import math
import rospy
from clover import srv
from std_srvs.srv import Trigger

# rospy.init_node('flight')  # Убедитесь, что инициализация узла происходит только один раз, если необходимо

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land_srv = rospy.ServiceProxy('land', Trigger)  # Изменено имя переменной для вызова службы посадки

def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)

def takeoff():
    navigate_wait(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    
def navigate_to(x, y, z):
    navigate_wait(x=x, y=y, z=z, frame_id='body', auto_arm=True)
    
def land():
    land_srv()  # Вызов службы посадки

