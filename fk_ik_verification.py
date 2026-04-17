import numpy as np

# Define Denavit-Hartenberg Parameters for the 4 DOF Robot Arm
# DH parameters [theta, d, a, alpha]
DH_PARAMS = np.array([
    [0, 144.15, 0, 90],    # Joint 1 (Base rotation)
    [0, 243.81, 0, 0],     # Joint 2 (Arm 1)
    [0, 124.75, 0, 0],     # Joint 3 (Arm 2)
    [0, 84.75, 0, -90]     # Joint 4 (Wrist)
])

def dh_transform(theta, d, a, alpha):
    """Compute transformation matrix using DH parameters"""
    theta, alpha = np.radians(theta), np.radians(alpha)
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,             np.sin(alpha),                  np.cos(alpha),                 d],
        [0,             0,                              0,                             1]
    ])

def forward_kinematics(joint_angles):
    """Compute FK to get the end-effector position"""
    T = np.eye(4)  # Identity matrix
    for i in range(len(joint_angles)):
        T_i = dh_transform(joint_angles[i], DH_PARAMS[i,1], DH_PARAMS[i,2], DH_PARAMS[i,3])
        T = np.dot(T, T_i)  # Multiply transformation matrices
    return T[:3, 3]  # Extract (X, Y, Z)

def inverse_kinematics(x, y, z):
    """Compute IK - Returns joint angles given (X, Y, Z)"""
    theta1 = np.arctan2(y, x)  # Base rotation
    d = np.sqrt(x**2 + y**2)   # Distance in XY plane
    theta2 = np.arctan2(z - DH_PARAMS[0, 1], d)  # Shoulder joint
    theta3 = np.arctan2(z - (DH_PARAMS[0, 1] + DH_PARAMS[1, 1]), d - DH_PARAMS[1, 1])  # Elbow joint
    theta4 = 0  # Assume end-effector is fixed

    return np.degrees([theta1, theta2, theta3, theta4])

def verify_fk_ik(joint_angles):
    """Verify FK and IK consistency"""
    print("\n--- Forward Kinematics ---")
    fk_position = forward_kinematics(joint_angles)
    print(f"FK End-Effector Position: {fk_position}")

    print("\n--- Inverse Kinematics ---")
    ik_angles = inverse_kinematics(*fk_position)
    print(f"Computed Joint Angles (IK): {ik_angles}")

    print("\n--- Recomputed Forward Kinematics from IK ---")
    recomputed_fk_position = forward_kinematics(ik_angles)
    print(f"Recomputed FK Position: {recomputed_fk_position}")

    error = np.linalg.norm(fk_position - recomputed_fk_position)
    print(f"\nVerification Error: {error:.6f}")
    print("✅ FK and IK Verification Successful!" if error < 1.0 else "❌ Verification Failed (Check Kinematics Model)")

if __name__ == "__main__":
    # Example joint angles [theta1, theta2, theta3, theta4]
    joint_angles = [30, 45, 30, 0]  # Modify for your robot
    verify_fk_ik(joint_angles)
