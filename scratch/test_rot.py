import numpy as np
from scipy.spatial.transform import Rotation as R

# Initial Mahony attitude from logs:
current_attitude = [-0.74070301,  0.01988527, -0.1211625,   0.66051743]
# Let's see what happens to NED vectors:
a_cmd_ned = np.array([-1.0, 0.0, 0.0]) # South

rot = R.from_quat(current_attitude)
print("rot.apply(a_cmd_ned)       (NED -> Body if rot=NED->Body):")
print(np.round(rot.apply(a_cmd_ned), 3))

print("rot.inv().apply(a_cmd_ned) (Body -> NED if rot=NED->Body):")
print(np.round(rot.inv().apply(a_cmd_ned), 3))

print("Check up_vector (0, 0, -1):")
print("rot.apply(up):", np.round(rot.apply([0, 0, -1]), 3))
print("rot.inv().apply(up):", np.round(rot.inv().apply([0, 0, -1]), 3))
