attribute vec3 a_Position;

uniform mat4 projection, modelview;

void main(){
  vec4 vertPos4 = modelview * vec4(a_Position, 1.0);

  gl_Position = projection * vertPos4;
}
