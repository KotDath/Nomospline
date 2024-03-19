varying vec2 position;

uniform vec3 color1;
uniform vec3 color2;

void main() {
    vec3 color = mix(color1, color2, position.y / 2.0 + 0.6);
    gl_FragColor = vec4(color, 1);
}