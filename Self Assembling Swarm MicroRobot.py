import pygame
import numpy as np
import random
from enum import Enum
from scipy.interpolate import splprep, splev
from sklearn.neighbors import NearestNeighbors


SCREEN_WIDTH = 1024
SCREEN_HEIGHT = 600
NUM_BOIDS = 100
MAX_SPEED = 5
MAX_FORCE = 0.05
NEIGHBOR_RADIUS = 50
SEPARATION_RADIUS = NEIGHBOR_RADIUS / 2
GROUP_RADIUS = 100
FPS = 60
FORMATION_THRESHOLD = 5
BOUNDARY_POINT_SPACING = 10  


WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
GRAY = (128, 128, 128)

class DrawingState(Enum):
    IDLE = "idle"
    DRAWING = "drawing"
    FOLLOWING = "following"

class Boid:
    def __init__(self):
        self.position = np.array([random.uniform(0, SCREEN_WIDTH), random.uniform(0, SCREEN_HEIGHT)], dtype=float)
        self.velocity = np.array([random.uniform(-MAX_SPEED, MAX_SPEED), random.uniform(-MAX_SPEED, MAX_SPEED)], dtype=float)
        self.acceleration = np.array([0.0, 0.0], dtype=float)
        self.target_position = None
        self.group_id = None
        self.stopped = False
        self.assigned_boundary_point = None

    def apply_force(self, force):
        if not self.stopped:
            self.acceleration += force

    def limit(self, vector, max_val):
        mag = np.linalg.norm(vector)
        if mag > max_val:
            return vector / mag * max_val
        return vector

    def align(self, boids):
        if self.stopped:
            return np.zeros(2)
            
        steering = np.array([0.0, 0.0], dtype=float)
        total = 0
        avg_vector = np.array([0.0, 0.0], dtype=float)
        
        for other in boids:
            if not other.stopped and np.linalg.norm(other.position - self.position) < NEIGHBOR_RADIUS:
                avg_vector += other.velocity
                total += 1
                
        if total > 0:
            avg_vector /= total
            avg_vector = self.limit(avg_vector, MAX_SPEED)
            steering = avg_vector - self.velocity
            steering = self.limit(steering, MAX_FORCE)
        return steering

    def cohesion(self, boids):
        if self.stopped:
            return np.zeros(2)
            
        steering = np.array([0.0, 0.0], dtype=float)
        total = 0
        center_of_mass = np.array([0.0, 0.0], dtype=float)
        
        for other in boids:
            if not other.stopped and np.linalg.norm(other.position - self.position) < NEIGHBOR_RADIUS:
                center_of_mass += other.position
                total += 1
                
        if total > 0:
            center_of_mass /= total
            desired = center_of_mass - self.position
            desired = self.limit(desired, MAX_SPEED)
            steering = desired - self.velocity
            steering = self.limit(steering, MAX_FORCE)
        return steering

    def separation(self, boids):
        if self.stopped:
            steering = np.array([0.0, 0.0], dtype=float)
            total = 0
            
            for other in boids:
                distance = np.linalg.norm(other.position - self.position)
                if distance < SEPARATION_RADIUS / 2 and distance > 0:
                    diff = self.position - other.position
                    diff /= distance
                    steering += diff
                    total += 1
                    
            if total > 0:
                steering /= total
                steering = self.limit(steering, MAX_FORCE / 2)
            return steering
        
        steering = np.array([0.0, 0.0], dtype=float)
        total = 0
        
        for other in boids:
            distance = np.linalg.norm(other.position - self.position)
            if distance < SEPARATION_RADIUS and distance > 0:
                diff = self.position - other.position
                diff /= distance
                steering += diff
                total += 1
                
        if total > 0:
            steering /= total
            steering = self.limit(steering, MAX_SPEED)
        steering = self.limit(steering, MAX_FORCE)
        return steering

    def flock(self, boids):
        if not self.stopped:
            alignment = self.align(boids)
            cohesion = self.cohesion(boids)
            separation = self.separation(boids)

            self.apply_force(alignment * 1.0)
            self.apply_force(cohesion * 1.0)
            self.apply_force(separation * 1.5)
        else:
            separation = self.separation(boids)
            self.apply_force(separation * 0.5)

    def move_to_boundary_point(self, target_pos, boundary_weight=1.0):
        if target_pos is None:
            return
            
        desired = target_pos - self.position
        distance = np.linalg.norm(desired)
        
        if distance < FORMATION_THRESHOLD:
            self.stopped = True
            self.velocity *= 0
            self.position = target_pos
            return
            
        if distance < 10:
            speed = np.interp(distance, [0, 10], [0, MAX_SPEED])
        else:
            speed = MAX_SPEED
            
        if distance > 0:
            desired = (desired / distance) * speed
            
        steering = desired - self.velocity
        steering = self.limit(steering, MAX_FORCE)
        self.apply_force(steering * boundary_weight)

    def update(self):
        if self.stopped:
            return
            
        if np.linalg.norm(self.velocity) < 0.1:
            self.velocity = np.array([random.uniform(-MAX_SPEED, MAX_SPEED), 
                                    random.uniform(-MAX_SPEED, MAX_SPEED)], dtype=float)

        self.velocity += self.acceleration
        self.velocity = self.limit(self.velocity, MAX_SPEED)
        self.position += self.velocity
        self.acceleration *= 0

        
        if self.position[0] <= 0 or self.position[0] >= SCREEN_WIDTH:
            self.velocity[0] = -self.velocity[0]
        if self.position[1] <= 0 or self.position[1] >= SCREEN_HEIGHT:
            self.velocity[1] = -self.velocity[1]

class BoundaryController:
    def __init__(self):
        self.drawing_points = []
        self.boundary_points = []
        self.state = DrawingState.IDLE
        
    def add_point(self, point):
        self.drawing_points.append(point)
        
    def clear_points(self):
        self.drawing_points = []
        self.boundary_points = []
        
    def generate_boundary_points(self):
        if len(self.drawing_points) < 2:
            return []
            
        
        points = np.array(self.drawing_points)
        
       
        if not np.array_equal(points[0], points[-1]):
            points = np.vstack([points, points[0]])
            
       
        tck, u = splprep([points[:, 0], points[:, 1]], s=0, per=True)
        
        
        u_new = np.linspace(0, 1, num=len(points) * 5)
        smooth_points = np.column_stack(splev(u_new, tck))
        
       
        boundary_points = []
        current_point = smooth_points[0]
        boundary_points.append(current_point)
        
        for point in smooth_points[1:]:
            distance = np.linalg.norm(point - current_point)
            if distance >= BOUNDARY_POINT_SPACING:
                boundary_points.append(point)
                current_point = point
                
        self.boundary_points = boundary_points
        return boundary_points
        
    def assign_boundary_points(self, boids):
        if not self.boundary_points:
            return
            
        
        boid_positions = np.array([boid.position for boid in boids])
        boundary_points = np.array(self.boundary_points)
        
        
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(boundary_points)
        distances, indices = nbrs.kneighbors(boid_positions)
        
        
        assigned_points = set()
        for i, boid in enumerate(boids):
            if not boid.stopped:
                point_idx = indices[i][0]
                if point_idx not in assigned_points:
                    boid.assigned_boundary_point = boundary_points[point_idx]
                    assigned_points.add(point_idx)
                else:
                    
                    _, all_indices = nbrs.kneighbors([boid.position])
                    for idx in all_indices[0]:
                        if idx not in assigned_points:
                            boid.assigned_boundary_point = boundary_points[idx]
                            assigned_points.add(idx)
                            break

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Boid Shape Filling Simulation")
    
    boids = [Boid() for _ in range(NUM_BOIDS)]
    boundary_controller = BoundaryController()
    
    clock = pygame.time.Clock()
    running = True
    
    while running:
        screen.fill(WHITE)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: 
                    boundary_controller.state = DrawingState.DRAWING
                    boundary_controller.clear_points()
                    
                    for boid in boids:
                        boid.stopped = False
                        boid.assigned_boundary_point = None
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1: 
                    boundary_controller.state = DrawingState.FOLLOWING
                    boundary_controller.generate_boundary_points()
            elif event.type == pygame.MOUSEMOTION:
                if boundary_controller.state == DrawingState.DRAWING:
                    boundary_controller.add_point(np.array(event.pos))
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c: 
                    boundary_controller.state = DrawingState.IDLE
                    boundary_controller.clear_points()
                    for boid in boids:
                        boid.stopped = False
                        boid.assigned_boundary_point = None
        
        
        if len(boundary_controller.drawing_points) > 1:
            pygame.draw.lines(screen, BLACK, False, boundary_controller.drawing_points, 2)
            
        
        for point in boundary_controller.boundary_points:
            pygame.draw.circle(screen, GRAY, point.astype(int), 2)
        
        
        if boundary_controller.state == DrawingState.FOLLOWING:
            boundary_controller.assign_boundary_points(boids)
            
        
        for boid in boids:
            if boundary_controller.state == DrawingState.FOLLOWING and boid.assigned_boundary_point is not None:
                boid.flock(boids)
                boid.move_to_boundary_point(boid.assigned_boundary_point, boundary_weight=1.5)
            else:
                boid.stopped = False
                boid.flock(boids)
            
            boid.update()
            
            
            color = YELLOW if boid.stopped else BLUE
            pygame.draw.circle(screen, color, boid.position.astype(int), 5)
            
            
            if boid.assigned_boundary_point is not None and not boid.stopped:
                pygame.draw.line(screen, GREEN, boid.position.astype(int), 
                               boid.assigned_boundary_point.astype(int), 1)
        
        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()

if __name__ == "__main__":
    main()