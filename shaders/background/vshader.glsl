attribute vec2 a_Position;

varying vec2 position;
void main(){
    gl_Position = vec4(a_Position, 0.0, 1.0);
    position = a_Position;
}
