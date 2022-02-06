import numpy as np
import matplotlib.pyplot as plt
from math import ceil, cos, sin

def normalize(vector):
    return vector / np.linalg.norm(vector)

def reflected(vector, axis):
    return vector - 2 * np.dot(vector, axis) * axis

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
    distances = [sphere_intersect(obj['center'], obj['radius'], ray_origin, ray_direction) for obj in objects]
    nearest_object = None
    min_distance = np.inf
    for index, distance in enumerate(distances):
        if distance and distance < min_distance:
            min_distance = distance
            nearest_object = objects[index]
    return nearest_object, min_distance

width = 250

height = 125

max_depth = 3

camera = np.array([0., 0., 1.])
ratio = float(width) / height
screen = (-1, 1 / ratio, 1, -1 / ratio) # left, top, right, bottom

light = { 'position': np.array([5., 5., 5.]), 'ambient': np.array([1, 1, 1]), 'diffuse': np.array([1, 1, 1]), 'specular': np.array([1, 1, 1]) }

def make_circle(center = np.array([-0.2, -0.2, -0.8]), radius = .8, ambient = np.array([0.1, 0, 0]),
                        diffuse = np.array([0.7, 0, 0]), specular = np.array([1,1,1]), shininess = 100, reflection = 0.5):
    print(f'made circle at: {center=}')
    return {'center': center, 'radius': radius, 'ambient': ambient, 'diffuse': diffuse, 
                                'specular': specular, 'shininess': shininess, 'reflection': reflection}

#objects = [
#    { 'center': np.array([-0.2, 0, -1]), 'radius': 0.7, 'ambient': np.array([0.1, 0, 0]), 'diffuse': np.array([0.7, 0, 0]), 'specular': np.array([1, 1, 1]), 'shininess': 100, 'reflection': 0.5 },
#    { 'center': np.array([0.1, -0.3, 0]), 'radius': 0.1, 'ambient': np.array([0.1, 0, 0.1]), 'diffuse': np.array([0.7, 0, 0.7]), 'specular': np.array([1, 1, 1]), 'shininess': 100, 'reflection': 0.5 },
#    { 'center': np.array([-0.3, 0, 0]), 'radius': 0.15, 'ambient': np.array([0, 0.1, 0]), 'diffuse': np.array([0, 0.6, 0]), 'specular': np.array([1, 1, 1]), 'shininess': 100, 'reflection': 0.5 },
#    { 'center': np.array([0, -9000, 0]), 'radius': 9000 - 0.7, 'ambient': np.array([0.1, 0.1, 0.1]), 'diffuse': np.array([0.6, 0.6, 0.6]), 'specular': np.array([1, 1, 1]), 'shininess': 100, 'reflection': 0.5 }
#]

objects = [
        make_circle(center=np.array([0,-9000, 0]), radius=9000-0.8, diffuse=np.array([0.3,0.3,0.3]), 
                        ambient=(np.array([0.1,0.1,0.1]))),
        #make_circle(center=[-1,.6,-.5], radius=.2, diffuse=[1.0,0,0]),
        #make_circle(center=[-.5,.6,-.5], radius=.2,diffuse=[.8, .2, 0]),
        make_circle(center=[.0,.6,-.5], radius=.2, diffuse=[.1, .4, .8]),
        #make_circle(center=[.5,.6,-.5], radius=.2, diffuse=[.05,.8,.05]),
        #make_circle(center=[1.0,.6,-.5], radius=.2,diffuse=[.7,.05,.4]),

        make_circle(center=[-1,.1,-.1], radius=.2, diffuse=[1.0,0,0]),
        #make_circle(center=[-1,.1,-.3], radius=.2,diffuse=[.8, .2, 0]),
        #make_circle(center=[-1,.1,-.5], radius=.2, diffuse=[.1, .4, .8]),
        #make_circle(center=[-1,.1,-.7], radius=.2, diffuse=[.05,.8,.05]),
        make_circle(center=[-1,.1,-.9], radius=.2,diffuse=[.7,.05,.4]),
    
        make_circle(center=[-0.12,0.,-0.2], radius=.4, diffuse=[.7,.1,.6]),  

        make_circle(center=[1.5,-.4,-1.], radius=.2, diffuse=[1.0,0,0]),
        #make_circle(center=[1.5,-.1,-1.], radius=.2,diffuse=[.8, .2, 0]),
        #make_circle(center=[1.5,.2,-1.], radius=.2, diffuse=[.1, .4, .8]),
        make_circle(center=[1.5,.5,-1.], radius=.2, diffuse=[.05,.8,.05]),
        #make_circle(center=[1.5,.8,-1.], radius=.2,diffuse=[.7,.05,.4]),
 ]


def render_image():
    image = np.zeros((height, width, 3))
    for i, y in enumerate(np.linspace(screen[1], screen[3], height)):
        for j, x in enumerate(np.linspace(screen[0], screen[2], width)):
            # screen is on origin
            pixel = np.array([x, y, 0])
            origin = camera
            direction = normalize(pixel - origin)
            #print(f'{direction=}')    
            color = np.zeros((3))
            reflection = 1
    
            for k in range(max_depth):
                # check for intersections
                nearest_object, min_distance = nearest_intersected_object(objects, origin, direction)
                if nearest_object is None:
                    break
    
                intersection = origin + min_distance * direction
                normal_to_surface = normalize(intersection - nearest_object['center'])
                shifted_point = intersection + 1e-5 * normal_to_surface
                intersection_to_light = normalize(light['position'] - shifted_point)
    
                _, min_distance = nearest_intersected_object(objects, shifted_point, intersection_to_light)
                intersection_to_light_distance = np.linalg.norm(light['position'] - intersection)
                is_shadowed = min_distance < intersection_to_light_distance
    
                if is_shadowed:
                    break
    
                illumination = np.zeros((3))
    
                # ambiant
                illumination += nearest_object['ambient'] * light['ambient']
    
                # diffuse
                illumination += nearest_object['diffuse'] * light['diffuse'] * np.dot(intersection_to_light, normal_to_surface)
    
                # specular
                intersection_to_camera = normalize(camera - intersection)
                H = normalize(intersection_to_light + intersection_to_camera)
                illumination += nearest_object['specular'] * light['specular'] * np.dot(normal_to_surface, H) ** (nearest_object['shininess'] / 4)
    
                # reflection
                color += reflection * illumination
                reflection *= nearest_object['reflection']
    
                origin = shifted_point
                direction = reflected(direction, normal_to_surface)
        

            image[i, j] = np.clip(color, 0, 1)

        #print("%d/%d" % (i + 1, height), flush=True)
    return image

def rotate_camera(theta): 
    global camera
    x, z = (camera[0], camera[2])
    x1 = x*cos(theta) - z*sin(theta)
    z1 = x*sin(theta) + z*cos(theta)
    camera[0] = x1
    camera[2] = z1

def rotate_light(theta):
    global light
    pos = light['position']
    x, z = (pos[0], pos[2])
    x1 = x*cos(theta) - z*sin(theta)
    z1 = x*sin(theta) + z*cos(theta)
    light['position'][0] = x1
    light['position'][2] = z1

def rotate_objects(theta):
    global objects
    for obj in objects:
        pos = obj['center']
        x, z = (pos[0], pos[2])
        x1 = x*cos(theta) - z*sin(theta)
        z1 = x*sin(theta) + z*cos(theta)
        obj['center'][0] = x1
        obj['center'][2] = z1


def main():
    from math import pi
    import imageio
    print('here')
    
   
    theta = pi/45
    n = 90
    images = []
    for i in range(n+1):
        print(f'{i/n*100}%')
        image = render_image()
        images.append(image)
        #plt.imsave(f'image{i}.png', image)
        #rotate_camera(theta)
        #rotate_light(theta)
        rotate_objects(theta)
    # write gif
    with imageio.get_writer('out.gif', mode='I') as writer:
        for im in images:
            writer.append_data(im) 

main()

