#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 mvp_matrix;
uniform vec3 point_color;
uniform float point_size;

attribute vec3 a_Position;
varying vec4 fragColor;

void main()
{
    fragColor = vec4(point_color, 1.0);
    gl_PointSize = point_size;
    gl_Position = mvp_matrix * vec4(a_Position, 1.0);
}