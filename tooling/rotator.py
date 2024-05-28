import math

def scale(vector, vec):
    vector[0] *= vec[0]
    vector[1] *= vec[1]
    vector[2] *= vec[2]
    return [round(vector[0], 5), round(vector[1], 5), round(vector[2], 5), vector[3]]

def translate(vector, offset):
    vector[0] += offset[0]
    vector[1] += offset[1]
    vector[2] += offset[2]
    return [round(vector[0], 5), round(vector[1], 5), round(vector[2], 5), vector[3]]

def rotate_z(vector, x):
    """Rotate a 3D vector around the z-axis by an angle x (in radians)."""
    cos_x = math.cos(x)
    sin_x = math.sin(x)
    x_new = vector[0] * cos_x - vector[1] * sin_x
    y_new = vector[0] * sin_x + vector[1] * cos_x
    z_new = vector[2]
    return [round(x_new, 5), round(y_new, 5), round(z_new, 5), vector[3]]

def rotate_y(vector, x):
    """Rotate a 3D vector around the y-axis by an angle x (in radians)."""
    cos_x = math.cos(x)
    sin_x = math.sin(x)
    x_new = vector[0] * cos_x + vector[2] * sin_x
    y_new = vector[1]
    z_new = -vector[0] * sin_x + vector[2] * cos_x
    return [round(x_new, 5), round(y_new, 5), round(z_new, 5), vector[3]]

def rotate_x(vector, x):
    """Rotate a 3D vector around the y-axis by an angle x (in radians)."""
    cos_x = math.cos(x)
    sin_x = math.sin(x)
    x_new = vector[0]
    y_new = vector[1] * cos_x + vector[2] * sin_x
    z_new = -vector[1] * sin_x + vector[2] * cos_x
    return [round(x_new, 5), round(y_new, 5), round(z_new, 5), vector[3]]


def transformator(input, output, offset):
    with open(input, 'r') as f:
        lines = f.readlines()

    with open(output, 'w') as f:
        for line in lines:
            if line.startswith('v '):
                values = line.strip().split()
                if len(values) >= 2:
                    vec4 = [float(values[1]), float(values[2]), float(values[3]), float(values[4])]
                    vec4 = translate(vec4, offset)
                f.write(f'{values[0]} ' + ' '.join([str(i) for i in vec4]) + '\n')
            else:
                f.write(line)

def scalor(input, output, vec):
    with open(input, 'r') as f:
        lines = f.readlines()

    with open(output, 'w') as f:
        for line in lines:
            if line.startswith('v '):
                values = line.strip().split()
                if len(values) >= 2:
                    vec4 = [float(values[1]), float(values[2]), float(values[3]), float(values[4])]
                    vec4 = scale(vec4, vec)
                f.write(f'{values[0]} ' + ' '.join([str(i) for i in vec4]) + '\n')
            else:
                f.write(line)

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
                    if type == 'x':
                        vec4 = [float(values[1]), float(values[2]), float(values[3]), float(values[4])]
                        vec4 = rotate_x(vec4, angle)
                f.write(f'{values[0]} ' + ' '.join([str(i) for i in vec4]) + '\n')
            else:
                f.write(line)


scalor('face_1.obj', 'face_1.obj', [4, 4, 4, 1])
scalor('face_2.obj', 'face_2.obj', [4, 4, 4, 1])
scalor('face_3.obj', 'face_3.obj', [4, 4, 4, 1])
scalor('face_4.obj', 'face_4.obj', [4, 4, 4, 1])
scalor('face_5.obj', 'face_5.obj', [4, 4, 4, 1])
scalor('face_6.obj', 'face_6.obj', [4, 4, 4, 1])