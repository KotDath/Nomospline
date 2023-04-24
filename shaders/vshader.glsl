#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 mvp_matrix;

attribute vec3 a_position;

void main()
{

    gl_Position = mvp_matrix * vec4(a_position, 1.0);
}