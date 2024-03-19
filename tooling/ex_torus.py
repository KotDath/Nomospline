#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Examples for the NURBS-Python Package
    Released under MIT License
    Developed by Onur Rauf Bingol (c) 2018

    Toroidal surface is contributed by Harshil Shah (@harshilsofeshah)
"""

import os
from geomdl import NURBS
from geomdl import exchange
from geomdl.visualization import VisVTK as vis

def output_figure(surf, name):
    with open(name, 'w') as f:
        for i in range(len(surf.weights)):
            coordinate = ' '.join([str(i) for i in surf.ctrlpts[i]]) + ' ' + str(surf.weights[i])
            f.write(f'v {coordinate}\n')

        f.write('\ncstype rat bspline\n')
        f.write(f'deg {surf.degree_u} {surf.degree_v}\n')
        result = 'surf 0 1 0 1 '
        for id_v in range(len(surf.ctrlpts2d)):
            for id_u in range(len(surf.ctrlpts2d[0])):
                result += f'{id_u + id_v * 9 + 1} '
        
        f.write(f'{result[:-1]}\n')
        u = ' '.join([str(i) for i in surf.knotvector_u])
        v = ' '.join([str(i) for i in surf.knotvector_v])
        f.write(f'parm u {u}\n')
        f.write(f'parm v {v}\n')

# Fix file path
os.chdir(os.path.dirname(os.path.realpath(__file__)))

# Create a NURBS surface instance
surf = NURBS.Surface()


# Set degress
surf.degree_u = 2
surf.degree_v = 2

# Set control points
surf.set_ctrlpts(*exchange.import_txt("ex_torus.cptw", two_dimensional=True))

# Set knot vectors
surf.knotvector_u = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1]
surf.knotvector_v = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1]

# Set sample size and evaluate surface
surf.sample_size = 50
surf.evaluate()

output_figure(surf, 'result2.obj')

from matplotlib import cm

vis_comp = vis.VisSurface()
surf.vis = vis_comp
surf.render(colormap=cm.coolwarm)

# Good to have something here to put a breakpoint
pass
