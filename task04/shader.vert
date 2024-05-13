#version 120

// see the GLSL 1.2 specification:
// https://www.khronos.org/registry/OpenGL/specs/gl/GLSLangSpec.1.20.pdf

uniform bool is_reflection; // variable of the program
varying vec3 normal; // normal vector pass to the rasterizer and fragment shader

void main()
{
    normal = vec3(gl_Normal);// set normal and pass it to fragment shader

    // "gl_Vertex" is the *input* vertex coordinate of triangle.
    // "gl_Vertex" has type of "vec4", which is homogeneious coordinate
    float x0 = gl_Vertex.x/gl_Vertex.w;// x-coord
    float y0 = gl_Vertex.y/gl_Vertex.w;// y-coord
    float z0 = gl_Vertex.z/gl_Vertex.w;// z-coord
    if (is_reflection) {
        vec3 nrm = normalize(vec3(0.4, 0.0, 1.0)); // normal of the mirror
        vec3 org = vec3(-0.3, 0.0, -0.5); // point on the mirror
        // wite code to change the input position (x0,y0,z0).
        // the transformed position (x0, y0, z0) should be drawn as the mirror reflection.
        //
        // make sure the occlusion is correctly computed.
        // the mirror is behind the armadillo, so the reflected image should be behind the armadillo.
        // furthermore, make sure the occlusion is correctly computed for the reflected image.
        //x0 = ???
        //y0 = ???
        //z0 = ???
        
        // reflect the vertex
        vec3 v = vec3(x0, y0, z0) - org;
        vec3 r = v - 2.0 * dot(v, nrm) * nrm;
        v = r + org;
        
        // mirror plane equation
        vec4 p = vec4(nrm, -dot(nrm, org)) / nrm.z;
        vec4 q = vec4(v.x, v.y, 0.0, 1.0);
        
        // calculate the z of intersection point
        float t = -dot(p, q) * p.z;
        
        // This is inspired from something often needed in machine learning that maps the value to a range,
        // a sigmoid-like function, I'm not sure if it's an appropriate solution
        x0 = v.x;
        y0 = v.y;
        z0 = t - (2.0 / (1.0 + exp(-(t - v.z))) - 1.0) * (t - -1.0);
    }
    
//    // Some debug code
//    z0 = -z0;
//    float angle = radians(-89);
//    y0 = y0 * cos(angle) - z0 * sin(angle);
//    z0 = y0 * sin(angle) + z0 * cos(angle);
//    z0 = -z0;
//
//    x0 = x0 * 0.5;
//    y0 = y0 * 0.5;
//    z0 = z0 * 0.5;
    
    // do not edit below

    // "gl_Position" is the *output* vertex coordinate in the
    // "canonical view volume (i.e.. [-1,+1]^3)" pass to the rasterizer.
    gl_Position = vec4(x0, y0, -z0, 1);// opengl actually draw a pixel with *maximum* depth. so invert z
}
