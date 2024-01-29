#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 mvp_matrix;

attribute vec3 a_Position;
varying vec4 fragColor;

void main()
{
    fragColor = vec4(1, 0, 0, 1);
    gl_PointSize = 8;
    gl_Position = mvp_matrix * vec4(a_Position, 1.0);
}