import pygame
import math
import numpy as np
from gui.pythonKalman import *


class NonHolonomicRobotSimulator:
    def __init__(self, noise=False, scale=1.0, width=30.00, height=12.50):
        # Initialize Pygame
        pygame.init()
        self.noise = noise
        self.scale = scale

        # Set up display
        self.width = int(width * scale)
        self.height = int(height * scale)
        self.win = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Non-Holonomic Robot Simulator")

        # Define colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.light_green = (144, 238, 144)
        self.yellow = (255, 255, 0)

        # Robot parameters
        self.robot_x = self.width // 2
        self.robot_y = self.height // 2
        self.robot_angle = 0
        self.robot_speed = 0
        self.rotation_speed = 5
        self.acceleration = 0.5
        self.friction = 0.1
        self.robot_size = int(.20 * scale)  # Half base of the isosceles triangle
        self.line_length = int(5.00 * scale)  # Length of the forward-pointing line (doubled)
        self.search_radius = self.line_length * 2  # Radius of the forward-facing search quarter circle

        # Wall parameters
        self.walls = [
            {"id": "top", "index": 1, "x": self.width // 2, "y": int(.10* scale), "length": self.width, "width": int(.10 * scale), "angle": 0},  # Top wall
            {"id": "bottom", "index": 2, "x": self.width // 2, "y": self.height - int(.10* scale), "length": self.width, "width": int(.10 * scale), "angle": 180},  # Bottom wall
            {"id": "left", "index": 3, "x": int(.10* scale), "y": self.height // 2, "length": self.height, "width": int(.10 * scale), "angle": 270},  # Left wall
            {"id": "right", "index": 4, "x": self.width - int(.10* scale), "y": self.height // 2, "length": self.height, "width": int(.10 * scale), "angle": 90},  # Right wall
        ]

        # Object parameters
        base_length = int(self.width  * .185/.300)
        top_y = self.height // 4
        middle_y = self.height // 2
        bottom_y = self.height * 3 / 4

        left_x_east = self.width // 2 + base_length // 2
        right_x_east = left_x_east - int(3.50 * scale)
        top_x_east = left_x_east

        left_x_west = self.width // 2 - base_length // 2
        right_x_west = left_x_west + int(3.50 * scale)
        top_x_west = left_x_west

        # Define displacement factor
        displacement_factor = int(0.50 * scale)  # Adjust the factor as needed

        # Initialize objects with duplicates
        self.objects = [
            {"shape": "circle", "x": left_x_east, "y": bottom_y - displacement_factor, "size": int(.20 * scale), "index": 0},  # Up duplicate of original index 0
            {"shape": "circle", "x": left_x_east, "y": bottom_y + displacement_factor, "size": int(.20 * scale), "index": 1},  # Down duplicate of original index 0

            {"shape": "triangle", "x": top_x_east, "y": top_y - displacement_factor, "size": int(.20 * scale), "index": 2},    # Up duplicate of original index 1
            {"shape": "triangle", "x": top_x_east, "y": top_y + displacement_factor, "size": int(.20 * scale), "index": 3},    # Down duplicate of original index 1

            {"shape": "square", "x": right_x_east, "y": middle_y - displacement_factor, "size": int(.20 * scale), "index": 4}, # Up duplicate of original index 2
            {"shape": "square", "x": right_x_east, "y": middle_y + displacement_factor, "size": int(.20 * scale), "index": 5}, # Down duplicate of original index 2

            {"shape": "circle", "x": left_x_west, "y": bottom_y - displacement_factor, "size": int(.20 * scale), "index": 6},  # Up duplicate of original index 3
            {"shape": "circle", "x": left_x_west, "y": bottom_y + displacement_factor, "size": int(.20 * scale), "index": 7},  # Down duplicate of original index 3

            {"shape": "triangle", "x": top_x_west, "y": top_y - displacement_factor, "size": int(.20 * scale), "index": 8},    # Up duplicate of original index 4
            {"shape": "triangle", "x": top_x_west, "y": top_y + displacement_factor, "size": int(.20 * scale), "index": 9},    # Down duplicate of original index 4

            {"shape": "square", "x": right_x_west, "y": middle_y - displacement_factor, "size": int(.20 * scale), "index": 10},# Up duplicate of original index 5
            {"shape": "square", "x": right_x_west, "y": middle_y + displacement_factor, "size": int(.20 * scale), "index": 11},# Down duplicate of original index 5
        ]
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 36)

    def draw_robot(self):
        x, y, angle = self.robot_x, self.robot_y, self.robot_angle
        point1 = (x + self.robot_size * math.sin(math.radians(angle)), 
                  y - self.robot_size * math.cos(math.radians(angle)))
        point2 = (x - self.robot_size * math.sin(math.radians(angle + 120)), 
                  y + self.robot_size * math.cos(math.radians(angle + 120)))
        point3 = (x - self.robot_size * math.sin(math.radians(angle - 120)), 
                  y + self.robot_size * math.cos(math.radians(angle - 120)))

        pygame.draw.polygon(self.win, self.red, [point1, point2, point3])
        
        line_end_x = x + (self.robot_size + self.line_length) * math.sin(math.radians(angle))
        line_end_y = y - (self.robot_size + self.line_length) * math.cos(math.radians(angle))
        pygame.draw.line(self.win, self.green, point1, (line_end_x, line_end_y), 2)

        arc_rect = pygame.Rect(point1[0] - self.search_radius, point1[1] - self.search_radius, 2 * self.search_radius, 2 * self.search_radius)
        start_angle = math.radians(angle - 45 - 90)
        end_angle = start_angle + math.radians(90)
        pygame.draw.arc(self.win, self.light_green, arc_rect, -end_angle, -start_angle, 2)

        arc_start_x = point1[0] + self.search_radius * math.cos(start_angle)
        arc_start_y = point1[1] + self.search_radius * math.sin(start_angle)
        arc_end_x = point1[0] + self.search_radius * math.cos(end_angle)
        arc_end_y = point1[1] + self.search_radius * math.sin(end_angle)
        pygame.draw.line(self.win, self.light_green, point1, (arc_start_x, arc_start_y), 2)
        pygame.draw.line(self.win, self.light_green, point1, (arc_end_x, arc_end_y), 2)

        return (point1, (line_end_x, line_end_y))

    def draw_wall(self, wall):
        length = wall["length"]
        width = wall["width"]
        angle = wall["angle"]
        x = wall["x"]
        y = wall["y"]
        
        half_length = length / 2
        half_width = width / 2
        
        points = [
            (x + half_length * math.cos(math.radians(angle)) - half_width * math.sin(math.radians(angle)),
             y + half_length * math.sin(math.radians(angle)) + half_width * math.cos(math.radians(angle))),
            (x - half_length * math.cos(math.radians(angle)) - half_width * math.sin(math.radians(angle)),
             y - half_length * math.sin(math.radians(angle)) + half_width * math.cos(math.radians(angle))),
            (x - half_length * math.cos(math.radians(angle)) + half_width * math.sin(math.radians(angle)),
             y - half_length * math.sin(math.radians(angle)) - half_width * math.cos(math.radians(angle))),
            (x + half_length * math.cos(math.radians(angle)) + half_width * math.sin(math.radians(angle)),
             y + half_length * math.sin(math.radians(angle)) - half_width * math.cos(math.radians(angle))),
        ]

        pygame.draw.polygon(self.win, self.blue, points)

    def draw_object(self, obj, color=None):
        if color is None:
            color = self.black
        shape = obj["shape"]
        x = obj["x"]
        y = obj["y"]
        size = obj["size"]

        if shape == "square":
            pygame.draw.rect(self.win, color, (x - size // 2, y - size // 2, size, size))
        elif shape == "circle":
            pygame.draw.circle(self.win, color, (x, y), size // 2)
        elif shape == "triangle":
            half_base = size // 2
            point1 = (x, y - size // 2)
            point2 = (x - half_base, y + half_base)
            point3 = (x + half_base, y + half_base)
            pygame.draw.polygon(self.win, color, [point1, point2, point3])

    def draw(self, shapes_in_range):
        self.win.fill(self.white)
        
        for wall in self.walls:
            self.draw_wall(wall)
        
        for obj in self.objects:
            color = self.red if obj in [shape[0] for shape in shapes_in_range] else self.black
            self.draw_object(obj, color)
        
        line_start, line_end = self.draw_robot()

        return line_start, line_end

    def line_intersects_line(self, p1, p2, p3, p4):
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

    def get_line_intersection(self, p1, p2, p3, p4):
        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        xdiff = (p1[0] - p2[0], p3[0] - p4[0])
        ydiff = (p1[1] - p2[1], p3[1] - p4[1])

        div = det(xdiff, ydiff)
        if div == 0:
            return None

        d = (det(p1, p2), det(p3, p4))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y

    def control(self, speed, rotation):
        self.robot_speed = speed 
        self.robot_angle += rotation * self.rotation_speed
        self.robot_angle = self.robot_angle % 360

    def draw_covariance_ellipse(self, x, P):
        eigenvalues, eigenvectors = np.linalg.eigh(P[:2, :2])
        order = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[order]
        eigenvectors = eigenvectors[:, order]

        # print(eigenvalues)

        angle = 180 - math.degrees(math.atan2(*eigenvectors[:, 0][::-1]))
        width, height = 2 * np.sqrt(eigenvalues) * 2

        ellipse_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        pygame.draw.ellipse(ellipse_surface, (255, 255, 0, 100), (0, 0, width, height))
        rotated_ellipse = pygame.transform.rotate(ellipse_surface, angle)
        
        ellipse_center = (int(x[0, 0]), int(x[1, 0]))
        self.win.blit(rotated_ellipse, (ellipse_center[0] - rotated_ellipse.get_width() / 2, ellipse_center[1] - rotated_ellipse.get_height() / 2))
        
        # Calculate the end points of the line
        radians_angle = math.radians(180 - angle)
        dx = self.width * math.cos(radians_angle)
        dy = self.width * math.sin(radians_angle)

        # Line endpoints
        start_point = (ellipse_center[0] - dx, ellipse_center[1] - dy)
        end_point = (ellipse_center[0] + dx, ellipse_center[1] + dy)

        # Draw the line
        pygame.draw.line(self.win, (50, 255, 0), start_point, end_point, 2)

    def draw_premade_ellipse(self, x, ell):
        (eig1, eig2, angle) = ell

        eig1 = min(eig1, self.width * self.scale)
        eig2 = min(eig2, self.width * self.scale)

        eigenvalues = np.array([eig1, eig2])
        angle = 180 - angle
        #angle = 180 - math.degrees(math.atan2(*eigenvectors[:, 0][::-1]))
        width, height = 2 * np.sqrt(eigenvalues) * 2
        
        ellipse_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        pygame.draw.ellipse(ellipse_surface, (255, 255, 0, 100), (0, 0, width, height))
        rotated_ellipse = pygame.transform.rotate(ellipse_surface, angle)
        
        ellipse_center = (int(x[0, 0]), int(x[1, 0]))
        self.win.blit(rotated_ellipse, (ellipse_center[0] - rotated_ellipse.get_width() / 2, ellipse_center[1] - rotated_ellipse.get_height() / 2))
        
        # Calculate the end points of the line
        radians_angle = math.radians(180 - angle)
        dx = self.width * math.cos(radians_angle)
        dy = self.width * math.sin(radians_angle)

        # Line endpoints
        start_point = (ellipse_center[0] - dx, ellipse_center[1] - dy)
        end_point = (ellipse_center[0] + dx, ellipse_center[1] + dy)

        # Draw the line
        pygame.draw.line(self.win, (50, 255, 0), start_point, end_point, 2)

    def draw_estimated_state(self, x):
        pygame.draw.circle(self.win, self.black, (int(x[0, 0]), int(x[1, 0])), 5)

    def simulate(self, x = None, P = None, ell = None):
        self.robot_speed *= (1 - self.friction)
        
        new_robot_x = self.robot_x + self.robot_speed * math.sin(math.radians(self.robot_angle))
        new_robot_y = self.robot_y - self.robot_speed * math.cos(math.radians(self.robot_angle))
        
        collision, wall = self.check_collision(new_robot_x, new_robot_y)
        if collision:
            new_robot_x, new_robot_y = self.robot_x, self.robot_y
        
        self.robot_x = new_robot_x
        self.robot_y = new_robot_y

        state = self.get_state()

        self.draw(state["shapes_in_range"])
         
        if x is not None:
            self.draw_estimated_state(x)
            if P is not None:
                self.draw_covariance_ellipse(x, P)
            if ell is not None:
                self.draw_premade_ellipse(x, ell)

        if state["distance_to_wall"] < self.line_length:
            text = self.font.render(f"Distance: {int(state['distance_to_wall'])}", True, self.black)
            self.win.blit(text, (10, 10))

        shapes_text = self.font.render(f"Shapes in range: {len(state['shapes_in_range'])}", True, self.black)
        self.win.blit(shapes_text, (10, 50))

        for i, (shape, distance, relative_angle) in enumerate(state["shapes_in_range"]):
            info_text = self.font.render(f"Shape {i + 1}: Distance: {int(distance)}, Angle: {int(relative_angle)}", True, self.black)
            self.win.blit(info_text, (10, 80 + i * 30))

        for shape in state["shapes_in_range"]:
            z, H, R, theta = object_angle_function(shape[0], (state["angle"] - 90), shape[2])
            
            x2, y2 = shape[0]["x"], shape[0]["y"]
            line_length = int(2000 * self.scale)
            end_x = line_length * math.cos(theta)
            end_y = line_length * math.sin(theta)
            
            pygame.draw.circle(self.win, self.blue, (x2, y2), abs(z), 2)
            pygame.draw.line(self.win, self.red, (-end_x, -end_y), (end_x, end_y), 2)

        pygame.display.flip()

    def check_collision(self, x, y):
        for wall in self.walls:
            length = wall["length"]
            width = wall["width"]
            angle = wall["angle"]
            wall_x = wall["x"]
            wall_y = wall["y"]
            
            half_length = length / 2
            half_width = width / 2
            
            points = [
                (wall_x + half_length * math.cos(math.radians(angle)) - half_width * math.sin(math.radians(angle)),
                 wall_y + half_length * math.sin(math.radians(angle)) + half_width * math.cos(math.radians(angle))),
                (wall_x - half_length * math.cos(math.radians(angle)) - half_width * math.sin(math.radians(angle)),
                 wall_y - half_length * math.sin(math.radians(angle)) + half_width * math.cos(math.radians(angle))),
                (wall_x - half_length * math.cos(math.radians(angle)) + half_width * math.sin(math.radians(angle)),
                 wall_y - half_length * math.sin(math.radians(angle)) - half_width * math.cos(math.radians(angle))),
                (wall_x + half_length * math.cos(math.radians(angle)) + half_width * math.sin(math.radians(angle)),
                 wall_y + half_length * math.sin(math.radians(angle)) - half_width * math.cos(math.radians(angle))),
            ]
            
            if x > min(p[0] for p in points) and x < max(p[0] for p in points) and \
               y > min(p[1] for p in points) and y < max(p[1] for p in points):
                return True, wall
        return False, None

    def closest_distance_to_wall_center(self):
        line_start = (self.robot_x, self.robot_y)
        line_end = (
            self.robot_x + self.line_length * math.sin(math.radians(self.robot_angle)),
            self.robot_y - self.line_length * math.cos(math.radians(self.robot_angle))
        )

        min_distance = 1000000 * self.scale
        closest_intersection = None
        closest_wall_id = 0

        if self.noise:
            if np.random.random() < (1 / 500):
                line_end = (
                    self.robot_x + self.line_length * 100 * math.sin(math.radians(self.robot_angle)),
                    self.robot_y - self.line_length * 100 * math.cos(math.radians(self.robot_angle))
                )
                for wall in self.walls:
                    length = wall["length"]
                    angle = wall["angle"]
                    wall_x = wall["x"]
                    wall_y = wall["y"]

                    half_length = length / 2

                    center_line_start = (
                        wall_x - half_length * math.cos(math.radians(angle)),
                        wall_y - half_length * math.sin(math.radians(angle))
                    )
                    center_line_end = (
                        wall_x + half_length * math.cos(math.radians(angle)),
                        wall_y + half_length * math.sin(math.radians(angle))
                    )

                    if self.line_intersects_line(line_start, line_end, center_line_start, center_line_end):
                        intersection = self.get_line_intersection(line_start, line_end, center_line_start, center_line_end)
                        if intersection:
                            distance = np.random.random() * self.line_length
                            if distance < min_distance:
                                min_distance = distance
                                closest_intersection = intersection
                                closest_wall_id = wall["index"]

        for wall in self.walls:
            length = wall["length"]
            angle = wall["angle"]
            wall_x = wall["x"]
            wall_y = wall["y"]

            half_length = length / 2

            center_line_start = (
                wall_x - half_length * math.cos(math.radians(angle)),
                wall_y - half_length * math.sin(math.radians(angle))
            )
            center_line_end = (
                wall_x + half_length * math.cos(math.radians(angle)),
                wall_y + half_length * math.sin(math.radians(angle))
            )

            if self.line_intersects_line(line_start, line_end, center_line_start, center_line_end):
                intersection = self.get_line_intersection(line_start, line_end, center_line_start, center_line_end)
                if intersection:
                    distance = math.hypot(intersection[0] - self.robot_x, intersection[1] - self.robot_y)
                    if distance < min_distance:
                        min_distance = distance
                        closest_intersection = intersection
                        closest_wall_id = wall["index"]

        return min_distance if closest_intersection else 1000000* self.scale  , closest_wall_id

    def get_shapes_within_quarter_circle(self):
        within_quarter_circle = []

        start_angle = math.radians(self.robot_angle - 135)
        end_angle = start_angle + math.radians(90)

        if self.noise:
            while np.random.random() < (1 / 100):
                shape = self.objects[np.random.randint(0, len(self.objects))]
                distance = np.random.random() * self.line_length * 2
                relative_angle = 45 - np.random.random() * 90
                within_quarter_circle.append((shape, distance, relative_angle))

        for shape in self.objects:
            shape_x = shape['x']
            shape_y = shape['y']

            distance = math.hypot(shape_x - self.robot_x, shape_y - self.robot_y)
            
            if distance <= self.search_radius:
                angle_to_shape = math.atan2(shape_y - self.robot_y, shape_x - self.robot_x)
                
                angle_to_shape = angle_to_shape % (2 * math.pi)
                start_angle_normalized = start_angle % (2 * math.pi)
                end_angle_normalized = end_angle % (2 * math.pi)
                
                if start_angle_normalized < end_angle_normalized:
                    if start_angle_normalized <= angle_to_shape <= end_angle_normalized:
                        relative_angle = angle_to_shape - math.radians(self.robot_angle)
                        relative_angle = math.degrees(relative_angle) % 360 - 270
                        within_quarter_circle.append((shape, distance, relative_angle))
                else:
                    if angle_to_shape >= start_angle_normalized or angle_to_shape <= end_angle_normalized:
                        relative_angle = angle_to_shape - math.radians(self.robot_angle)
                        relative_angle = math.degrees(relative_angle) % 360 - 270
                        within_quarter_circle.append((shape, distance, relative_angle))
        
        return within_quarter_circle

    def get_state(self):
        shapes_within_quarter_circle = self.get_shapes_within_quarter_circle()
        distance_to_wall, wall_id = self.closest_distance_to_wall_center()
        
        if self.noise:
            state = {
                "position": (self.robot_x, self.robot_y),
                "angle": self.robot_angle - 90 + np.random.random() * 0 ,
                "velocity": self.robot_speed,
                "distance_to_wall": distance_to_wall,
                "shapes_in_range": shapes_within_quarter_circle,
                "wall_id": wall_id
            }
        else:
            state = {
                "position": (self.robot_x, self.robot_y),
                "angle": self.robot_angle - 90,
                "velocity": self.robot_speed,
                "distance_to_wall": distance_to_wall,
                "shapes_in_range": shapes_within_quarter_circle,
                "wall_id": wall_id
            }

        return state

# Main simulation logic
if __name__ == "__main__":
    noise = False
    scale = 50  # Adjust the scale factor as needed
    simulator = NonHolonomicRobotSimulator(noise, scale)
    state = simulator.get_state()
    x = np.array(([state["position"][0]], [state["position"][1]], [0], [0]))
    kalman = KalmanHandler(x, simulator.win, simulator.objects)
    run_simulation = True
    dt = 0.1
    while run_simulation:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run_simulation = False
        
        keys = pygame.key.get_pressed()
        speed = (keys[pygame.K_UP] - keys[pygame.K_DOWN]) * 4
        rotation = (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT])
        simulator.control(speed, rotation)
        state = simulator.get_state()
        u = np.array(([speed / dt * math.sin(math.radians(state["angle"]))], [speed / dt * math.cos(math.radians(state["angle"]))]))
        x2, P = kalman.update(simulator.walls, state, u, dt)
        simulator.simulate(x2, P)
        simulator.clock.tick(60)

    pygame.quit()
