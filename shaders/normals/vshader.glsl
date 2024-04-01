#version 150
in vec4 a_Position;
in vec4 a_Norm;

out Vertex
{
    vec4 normal;
    vec4 color;
} vertex;

void main()
{
    gl_Position = a_Position;
    vertex.normal = a_Norm;
    vertex.color =  vec4(1.0, 1.0, 0.0, 1.0);
}