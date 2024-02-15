//precision mediump float;
varying vec3 normalInterp;  // Surface normal
varying vec3 vertPos;       // Vertex position
uniform float Ka;   // Ambient reflection coefficient
uniform float Kd;   // Diffuse reflection coefficient
uniform float Ks;   // Specular reflection coefficient
uniform float shininessVal; // Shininess
// Material color

uniform vec3 ambientColor;
uniform vec3 diffuseColor;
uniform vec3 specularColor;
uniform vec3 lightPos; // Light position

vec3 phongModel (vec3 position, vec3 norm) {
  vec3 L = normalize(lightPos - position);
  float lambertian = max(dot(norm, L), 0.0);
  float specular = 0.0;
  if(lambertian > 0.0) {
    vec3 R = reflect(-L, norm);      // Reflected light vector
    vec3 V = normalize(-position); // Vector to viewer
    // Compute the specular term
    float specAngle = max(dot(R, V), 0.0);
    specular = pow(specAngle, shininessVal);
  }

  return Ka * ambientColor +
  Kd * lambertian * diffuseColor +
  Ks * specular * specularColor;
}

void main() {
  vec3 N = normalize(normalInterp);
  if(gl_FrontFacing) {
    gl_FragColor = vec4(phongModel(vertPos, N), 1.0);
  } else {
    gl_FragColor = vec4(phongModel(vertPos, -N), 1.0);
  }



}
