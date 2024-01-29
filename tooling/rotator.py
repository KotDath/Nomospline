import math

def rotate_z(vector, x):
    """Rotate a 3D vector around the z-axis by an angle x (in radians)."""
    cos_x = math.cos(x)
    sin_x = math.sin(x)
    x_new = vector[0] * cos_x - vector[1] * sin_x
    y_new = vector[0] * sin_x + vector[1] * cos_x
    z_new = vector[2]
    return [round(x_new, 5), round(y_new, 5), z_new, vector[3]]

def rotate_y(vector, x):
    """Rotate a 3D vector around the y-axis by an angle x (in radians)."""
    cos_x = math.cos(x)
    sin_x = math.sin(x)
    x_new = vector[0] * cos_x + vector[2] * sin_x
    y_new = vector[1]
    z_new = -vector[0] * sin_x + vector[2] * cos_x
    return [x_new, y_new, z_new, vector[3]]

def rotator(input, output, angle, type):
    with open(input, 'r') as f:
        lines = f.readlines()

    with open(output, 'w') as f:
        for line in lines:
            if line.startswith('v '):
                values = line.strip().split()
                if len(values) >= 2:
                    if type == 'z':
                        vec4 = [float(values[1]), float(values[2]), float(values[3]), float(values[4])]
                        vec4 = rotate_z(vec4, angle)
                    if type == 'y':
                        vec4 = [float(values[1]), float(values[2]), float(values[3]), float(values[4])]
                        vec4 = rotate_y(vec4, angle)
                f.write(f'{values[0]} ' + ' '.join([str(i) for i in vec4]) + '\n')
            else:
                f.write(line)

rotator('result.obj', 'result_2.obj', math.pi / 2, 'y')