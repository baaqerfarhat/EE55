from scipy.spatial.transform import Rotation

# x, y, z, w
quaternion = [-0.20233, -0.65964, 0.65964, -0.29804]



R = Rotation.from_quat(quaternion)
rotation_matrix = R.as_matrix()

print(rotation_matrix)
