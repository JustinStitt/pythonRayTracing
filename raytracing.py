# to-do: interpret color
import numpy as np
import matplotlib.pyplot as plt

def sphere_intersect(center, radius, ray_origin, ray_direction):
    b = 2 * np.dot(ray_direction, ray_origin - center)
    c = np.linalg.norm(ray_origin - center) ** 2 - radius ** 2
    delta = b ** 2 - 4 * c
    if delta > 0:
        t1 = (-b + np.sqrt(delta)) / 2
        t2 = (-b - np.sqrt(delta)) / 2
        if t1 > 0 and t2 > 0:
            return min(t1, t2)
    return None

def nearest_intersected_object(objects, ray_origin, ray_direction):
    distances = [sphere_intersect(obj['center'], obj['radius'],
                    ray_origin, ray_direction) for obj in objects]

    neartest_object = None
    min_distance = np.inf

    for index, distance in enumerate(distances):
        if distance and distance < min_distance:
            min_distance = distance
            nearest_object = objects[index]

    return (nearest_object, min_distance)

def normalize(vector):
    return vector / np.linalg.norm(vector)

# screen parameter constants
WIDTH, HEIGHT = 300, 200

camera = np.array([0, 0, 1])
light = np.array([5, 5, 5])
ratio = float(WIDTH) / HEIGHT # computer aspect ratio

# set up screen with aspect ratio (3:2 in our case)
screen = [-1, 1/ratio, 1, -1/ratio] # left, top, right, bottom

objects = [
        {'center': np.array([-0.2, 0, -1]), 'radius': 0.7},
        {'center': np.array([0.1, -0.3, 0]), 'radius': 0.7},
        {'center': np.array([-0.3, 0, 0]), 'radius': 0.15},
]

# 200x300x3 matrix of zeros
image = np.zeros((HEIGHT, WIDTH, 3))

# enumerate returns index, value pairs
# np.linspace returns values evenly distributed between a, b with 
# a sample size of c
for i, y in enumerate(np.linspace(screen[1], screen[3], HEIGHT)):
    for j, x in enumerate(np.linspace(screen[0], screen[2], WIDTH)):
        pixel = np.array([x, y, 0])
        origin = camera
        direction = normalize(pixel - origin)
        # direction vector used in paramterized linear equation
        # Ray(t) = Origin + direction_vector * t
        # the higher the t value, the further away from the Origin (camera)
        nearest_object, min_distance = nearest_intersected_object(objects,
                                                                  origin,
                                                                  direction,)
        if nearest_object is None:
            continue

        intersection = origin + direction * min_distance

        normal_to_surface = normalize(intersection - nearest_object['center'])
        shifted_point = intersection + 1e-5 * normal_to_surface
        intersection_to_light = normalize(light['position'] - shifted_point)
        _, min_distance = nearest_intersected_object(objects, shifted_point,
                                                intersection_to_light,)
        intersection_to_light_distance = np.linalg.norm(light['position'] - \
                                                        intersection)
        is_shadowed = min_distance < intersection_to_light_distance
        
        if is_shadowed:
            continue # leave black
        # image[i, j] = ...i

    print(f'progress: {i + 1}, {HEIGHT}')

plt.imsave('image.png', image)
