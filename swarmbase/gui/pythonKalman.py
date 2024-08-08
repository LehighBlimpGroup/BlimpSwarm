import numpy as np
import math
import pygame
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


class KalmanHandler:
    def __init__(self, x, win, list_of_objects):
        self.win = win
        self.list_of_objects = list_of_objects  # Store the list of objects
        B = np.array([[0,0,0,1], [0,0,1,0]]).T
        Q = np.eye(4) * 8  # Rate of "decay" in confidence
        P = np.eye(4) * 20000
        self.kalman = KalmanFilter(B, Q, P, x)



    def update(self, walls, state, u, dt):
        angle = state["angle"]
        position = (int(self.kalman.x[0, 0]), int(self.kalman.x[1, 0]))
        shapes_in_range = state["shapes_in_range"]
        distance_to_wall = state["distance_to_wall"]
        wall_id = state["wall_id"]

        A = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])
        self.kalman.predict(A, u)

        
        corrected_compass_measurement = angle #+ bias  # Correct the compass measurement
        
        # Use the corrected compass measurement in the Kalman update
        if wall_id != -1:
            for wall in walls:
                if wall["index"] == wall_id:
                    z, H, R, theta = wall_measurement(wall, corrected_compass_measurement, distance_to_wall)
                    rotation_2x2 = np.array([[np.cos(theta), -np.sin(theta)],
                                             [np.sin(theta), np.cos(theta)]])
                    rotation_matrix = np.block([
                        [rotation_2x2, np.zeros((2, 2))],
                        [np.zeros((2, 2)), rotation_2x2]])
                    self.kalman.update(z, H, R, rotation_matrix)
                    break
        
        for shape_info in shapes_in_range:
            object_data = find_and_measure_object_angle(
                self.list_of_objects,
                kalman_estimated_location=position,
                compass_measurement=corrected_compass_measurement,
                angle_measurement=shape_info[2],
                shape = None#shape_info[0]
            )

            if object_data is not None:
                z, H, R, theta = object_data
                rotation_2x2 = np.array([[np.cos(theta), -np.sin(theta)],
                                         [np.sin(theta), np.cos(theta)]])
                rotation_matrix = np.block([
                    [rotation_2x2, np.zeros((2, 2))],
                    [np.zeros((2, 2)), rotation_2x2]])
                self.kalman.update(z, H, R, rotation_matrix)
        
        return self.kalman.x, self.kalman.P





class KalmanFilter:
    def __init__(self, B, Q, P, x):
        
        self.B = B  # Control input matrix
        self.Q = Q  # Process noise covariance
        self.P = P  # Estimate error covariance
        self.x = x  # State estimate

    def predict(self, A, u=0):
        self.x = np.dot(A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(A, self.P), A.T) + self.Q

    def update(self, z, H, R, rotation_matrix=None):
        if rotation_matrix is not None:
            # Apply rotation to the state and covariance matrix
            self.x = np.dot(rotation_matrix.T, self.x)
            self.P = np.dot(np.dot(rotation_matrix.T, self.P), rotation_matrix)

        # Perform the update step
        y = z - np.dot(H, self.x)
        S = np.dot(np.dot(H, self.P), H.T) + R
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot((I - np.dot(K, H)), self.P)

        if rotation_matrix is not None:
            # Rotate the state and covariance matrix back to the original frame
            self.x = np.dot(rotation_matrix, self.x)
            self.P = np.dot(np.dot(rotation_matrix, self.P), rotation_matrix.T)





def calculate_distance_to_origin(wall):
    """
    Calculate the minimal distance from the wall to the origin.

    Parameters:
    - wall: Dictionary with the properties of the wall (x, y, length, angle).

    Returns:
    - distance_to_origin: Minimal distance from the wall to the origin.
    """
    wall_x, wall_y, wall_length, wall_angle = wall["x"], wall["y"], wall["length"], wall["angle"]

    # Convert angle to radians
    wall_angle_rad = np.radians(wall_angle)

    # Calculate the endpoints of the wall
    wall_end_x = wall_x + wall_length * np.cos(wall_angle_rad)
    wall_end_y = wall_y + wall_length * np.sin(wall_angle_rad)

    # Use point-to-line distance formula
    x1, y1 = wall_x, wall_y
    x2, y2 = wall_end_x, wall_end_y
    distance_to_origin = abs((y2 - y1) * 0 - (x2 - x1) * 0 + x2 * y1 - y2 * x1) / np.sqrt((y2 - y1)**2 + (x2 - x1)**2)

    return distance_to_origin

def wall_measurement(wall, compass_measurement, distance_measurement):
    """
    Calculate the estimated distance to the wall and the difference between the distance to the wall and the origin.

    Parameters:
    - wall: Dictionary with the properties of the wall (x, y, length, angle).
    - compass_measurement: The angle of the robot relative to the origin.
    - distance_measurement: The measured distance to the wall.

    Returns:
    - perpendicular_distance: Perpendicular distance to the wall.
    - distance_diff: Difference between the perpendicular distance and the distance from the wall to the origin.
    - H: Observation matrix.
    - R: Measurement noise covariance.
    - wall_angle: Angle of the wall in environment relative to the origin axes.
    """
    wall_x, wall_y, wall_length, wall_angle = wall["x"], wall["y"], wall["length"], wall["angle"]

    # Calculate the distance to origin
    distance_to_origin = calculate_distance_to_origin(wall)
    # print(distance_to_origin)

    # Convert angles to radians
    wall_angle_rad = np.radians(wall_angle)
    compass_angle_rad = np.radians(compass_measurement)

    # Calculate the perpendicular distance to the wall using trigonometry
    perpendicular_distance = distance_measurement * np.sin(wall_angle_rad -compass_angle_rad)

    # Calculate the difference
    distance_diff = distance_to_origin - perpendicular_distance

    # Observation matrix for the position
    H = np.array([[0, 1, 0, 0]])

    # Measurement noise covariance
    R = np.array([[10000]])
    # print(perpendicular_distance)
    print(distance_diff,wall_angle )
    return -distance_diff, H, R, wall_angle_rad

# # Example usage
# width, height = 800, 600
# # Wall parameters (distance_to_origin will be calculated)
# walls = [
#     {"x": width // 2, "y": 10, "length": width, "width": 10, "angle": 0},  # Top wall
#     {"x": width // 2, "y": height - 10, "length": width, "width": 10, "angle": 180},  # Bottom wall
#     {"x": 10, "y": height // 2, "length": height, "width": 10, "angle": 90},  # Left wall
#     {"x": width - 10, "y": height // 2, "length": height, "width": 10, "angle": 270},  # Right wall
# ]

# # Calculate distance to origin for each wall
# for wall in walls:
#     wall["distance_to_origin"] = calculate_distance_to_origin(wall)
# print(walls)
# compass_measurement = 180  # Example compass measurement (degrees)
# distance_measurement = 100  # Example distance measurement (units)

# distance_diff, H, R, wall_angle = wall_measurement(walls[1], compass_measurement, distance_measurement)

# # print("Perpendicular Distance:", perpendicular_distance)
# print("Distance Difference:", distance_diff)
# print("H:", H)
# print("R:", R)
# print("Wall Angle:", wall_angle)

def calculate_angle_to_object(robot_x, robot_y, obj_x, obj_y):
    return math.atan2(obj_y - robot_y, obj_x - robot_x)



def object_angle_function(object, compass_measurement, angle_measurement):
    """
    Calculate the estimated distance to the origin relative to the closest point on the line
    created by the point of the object and the (robot's angle + angle measurement).

    Parameters:
    - object: Dictionary with the properties of the object (x, y, size, shape).
    - compass_measurement: The angle of the robot relative to the origin.
    - angle_measurement: The angle of the object relative to the robot.

    Returns:
    - distance_to_origin: Estimated distance to the origin relative to the closest point on the line.
    - H: Observation matrix.
    - R: Measurement noise covariance.
    - line_angle: Angle of the line between robot's angle and object.
    """
    object_x, object_y = object["x"], object["y"]
    robot_angle = compass_measurement
    total_angle = robot_angle + angle_measurement

    # Convert angles to radians
    total_angle_rad = np.radians(total_angle)

    # Calculate the unit vector in the direction of the total angle
    line_dx = np.cos(total_angle_rad)
    line_dy = np.sin(total_angle_rad)

    # Use point-to-line distance formula to find the distance from the origin to the line
    x1, y1 = 0, 0  # Origin
    x2, y2 = object_x, object_y  # Object position
    # distance_to_origin = abs(line_dy * x1 - line_dx * y1 + line_dx * y2 - line_dy * x2) / np.sqrt(line_dy**2 + line_dx**2)
    # Point-to-line distance calculation
    distance_to_origin = (line_dx * y2- line_dy * x2) / np.sqrt(line_dy**2 + line_dx**2)

    # Observation matrix for the position
    H = np.array([[0,1, 0, 0]])

    # Measurement noise covariance
    R = np.array([[10000]])
    # print(distance_to_origin, total_angle)
    return distance_to_origin, H, R, total_angle_rad 


def find_and_measure_object_angle(list_of_objects, kalman_estimated_location, compass_measurement, angle_measurement, shape=None):
    closest_object = None
    closest_distance = float('inf')

    robot_x, robot_y = kalman_estimated_location

    # Convert angles to radians
    compass_angle_rad = math.radians(compass_measurement)
    total_angle_rad = math.radians(compass_measurement + angle_measurement)

    # Calculate the direction vector from the robot's compass and angle measurements
    line_dx = math.cos(total_angle_rad)
    line_dy = math.sin(total_angle_rad)

    for obj in list_of_objects:
        if shape is None or obj['shape'] == shape:
            obj_x = obj['x']
            obj_y = obj['y']

            # Calculate the angle to the object from the robot's current position
            angle_to_object = math.atan2(obj_y - robot_y, obj_x - robot_x)

            # Calculate the absolute difference between the robot's direction and the object angle
            angle_diff = abs(angle_to_object - total_angle_rad) % (2 * math.pi)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff

            # Check if this object is the closest one in the specified direction
            if angle_diff < closest_distance:
                closest_distance = angle_diff
                closest_object = obj

    if closest_object:
        # Call the existing object_angle_function with the identified object
        return object_angle_function(closest_object, compass_measurement, angle_measurement)
    else:
        return None  # No object matched the criteria







# # Example usage
# width, height = 800, 600
# # Object parameters
# objects = [
#     {"shape": "square", "x": width // 2 + 100, "y": height * 3 // 4, "size": 20},
#     {"shape": "circle", "x": width // 2 + 100, "y": height // 4, "size": 20},
#     {"shape": "triangle", "x": width // 2 + 50, "y": height // 2, "size": 20},
#     {"shape": "square", "x": width // 2 - 100, "y": height * 3 // 4, "size": 20},
#     {"shape": "circle", "x": width // 2 - 100, "y": height // 4, "size": 20},
#     {"shape": "triangle", "x": width // 2 - 50, "y": height // 2, "size": 20},
# ]

# compass_measurement = 0  # Example compass measurement (degrees)
# angle_measurement = 0  # Example angle measurement (degrees)

# distance_to_origin, H, R, line_angle = object_angle_function(objects[5], compass_measurement, angle_measurement)

# print("Distance to Origin:", distance_to_origin)
# print("H:", H)
# print("R:", R)
# print("Line Angle:", line_angle)


# def plot_state_and_covariance(kf, measurements, rotation_matrix):
#     # Initialize lists to store positions and covariance ellipses
#     positions = []
#     covariances = []

#     A = np.array([[1, 0, dt, 0],
#               [0, 1, 0, dt],
#               [0, 0, 1, 0],
#               [0, 0, 0, 1]])
#     for z, H, R in measurements:
        
#         kf.predict(A)
#         kf.update(z, H, R, rotation_matrix)
        
#         positions.append(kf.x[:2])
#         covariances.append(kf.P[:2, :2])
    
#     fig, ax = plt.subplots()
#     ax.set_aspect('equal')

#     for pos, cov in zip(positions, covariances):
#         plot_covariance_ellipse(pos, cov, ax)

#     positions = np.array(positions)
#     ax.plot(positions[:, 0], positions[:, 1], 'ro-', label='Position Estimate')
    
#     plt.xlabel('Position X')
#     plt.ylabel('Position Y')
#     plt.legend()
#     plt.title('Kalman Filter State Estimates and Covariance Ellipses')
#     plt.grid(True)
#     plt.show()


# def plot_covariance_ellipse(mean, cov, ax, n_std=1.0, facecolor='none', **kwargs):
#     """
#     Plots an ellipse representing the covariance matrix on the given axis.
#     """
#     mean = mean.flatten()  # Ensure mean is a 1D array
#     eigvals, eigvecs = np.linalg.eigh(cov)
#     order = eigvals.argsort()[::-1]
#     eigvals, eigvecs = eigvals[order], eigvecs[:, order]

#     vx, vy = eigvecs[:, 0][0], eigvecs[:, 0][1]
#     theta = np.degrees(np.arctan2(vy, vx))

#     width, height = 2 * n_std * np.sqrt(eigvals)
#     ellipse = Ellipse(xy=(mean[0], mean[1]), width=width, height=height, angle=theta, facecolor=facecolor, edgecolor='blue', **kwargs)

#     ax.add_patch(ellipse)



# if __name__ == "__main__":
#     # Parameters
#     dt = 1.0
#     B = np.array([[0], [0], [0], [0]])
#     Q = np.eye(4)
#     P = np.eye(4)
#     x = np.array([1, 1, 0, 0])  # Initial state (position and velocity)

#     # Create Kalman filter instance
#     kf = KalmanFilter(B, Q, P, x)

#     # Define a 45-degree rotation matrix for position and velocity parts
#     theta = np.radians(45)
#     rotation_2x2 = np.array([[np.cos(theta), -np.sin(theta)],
#                             [np.sin(theta),  np.cos(theta)]])
#     rotation_matrix = np.block([
#         [rotation_2x2, np.zeros((2, 2))],
#         [np.zeros((2, 2)), rotation_2x2]
#     ])

#     # Example measurements with different H and R matrices
#     # [measurement, H, R]
#     measurements = [
#         (np.array([1]).reshape(1, 1), np.array([[1, 0, 0, 0]]), np.array([[10]])),
#         (np.array([2]).reshape(1, 1), np.array([[0, 1, 0, 0]]), np.array([[10]])),
#         (np.array([3, 3]).reshape(2, 1), np.array([[1, 0, 0, 0], [0, 1, 0, 0]]), np.eye(2) * 10),
#     ]


#     # Plot the state estimates and covariance ellipses
#     plot_state_and_covariance(kf, measurements, rotation_matrix)
