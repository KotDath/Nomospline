#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Examples for the NURBS-Python Package
    Released under MIT License
    Developed by Onur Rauf Bingol (c) 2016-2017

    This example is contributed by John-Eric Dufour (@jedufour)
"""

import os
from geomdl import NURBS
from geomdl import construct
from geomdl import exchange
from geomdl.visualization import VisVTK as vis
import operator
import os

def output_figure(surf):
    with open('result.obj', 'w') as f:
        for i in range(len(surf.weights)):
            coordinate = ' '.join([str(i) for i in surf.ctrlpts[i]]) + ' ' + str(surf.weights[i])
            f.write(f'v {coordinate}\n')

        f.write('\ncstype rat bspline\n')
        f.write(f'deg {surf.degree_u} {surf.degree_v}\n')
        result = 'surf 0 1 0 1 '
        for id_v in range(9):
            for id_u in range(2):
                result += f'{id_v + id_u * 9 + 1} '
        
        f.write(f'{result[:-1]}\n')
        u = ' '.join([str(i) for i in surf.knotvector_u])
        v = ' '.join([str(i) for i in surf.knotvector_v])
        f.write(f'parm u {u}\n')
        f.write(f'parm v {v}\n')

def translate(surf, vec):
    for point in surf.ctrlpts:
        point = map(operator.add, point, vec)
        print(list(point))

# Fix file path
os.chdir(os.path.dirname(os.path.realpath(__file__)))

# Create a NURBS surface instance
surf = NURBS.Surface()

# Set degrees
surf.degree_u = 1
surf.degree_v = 2


# Set control points
surf.set_ctrlpts(*exchange.import_txt("ex_cylinder.cptw", two_dimensional=True))


translate(surf, [0, 0, 10])

# Set knot vectors
surf.knotvector_u = [0, 0, 1, 1]
surf.knotvector_v = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1]

# Set evaluation delta
surf.delta = 0.01


surf_curves = construct.extract_curves(surf)
plot_extras = [
    dict(
        points=surf_curves['u'][0].evalpts,
        name="u",
        color="cyan",
        size=15
    ),
    dict(
        points=surf_curves['v'][0].evalpts,
        name="v",
        color="magenta",
        size=5
    )
]

output_figure(surf)

# Plot the control point grid and the evaluated surface
surf.vis = vis.VisSurface()
surf.render(extras=plot_extras)

# Good to have something here to put a breakpoint
pass