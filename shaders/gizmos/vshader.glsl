#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 mvp_matrix;

attribute vec3 a_Position;
attribute vec3 a_Color;
varying vec4 fragColor;

void main()
{
    fragColor = vec4(a_Color, 1.0);
    gl_PointSize = 2.0;
    gl_Position = mvp_matrix * vec4(a_Position, 1.0) + vec4(0.0, 0.0, 0.0, 0.0);
}