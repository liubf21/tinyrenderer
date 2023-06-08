#include <limits>
#include "model.h"
#include "our_gl.h"

constexpr int width  = 800; // output image size
constexpr int height = 800;
constexpr vec3 light_dir{1,1,1}; // light source
constexpr vec3       eye{0,0,3}; // camera position
constexpr vec3    center{0,0,0}; // camera direction
constexpr vec3        up{0,1,0}; // camera up vector

extern mat<4,4> ModelView; // "OpenGL" state matrices
extern mat<4,4> Projection;

float BoxX[2] = {-100, 100};
float BoxY[2] = {-100, 200};
float BoxZ[2] = {-100, 50};

vec3 BoxColor[2][2][2] = {
    {{{0,0,0},{255,0,0}},
    {{0,255,0},{255,255,0}}},
    {{{0,0,255},{255,0,255}},
    {{0,255,255},{255,255,255}}}
};


vec3 cloudColor(vec3 p)
{
    // std::cout << p << std::endl;
    // return vec3{255, 255, 55};
    if (p.x<BoxX[0]||p.x>BoxX[1]||p.y<BoxY[0]||p.y>BoxY[1]||p.z<BoxZ[0]||p.z>BoxZ[1])return vec3{};//vec3{255,255,255};
    float fx = (p.x - BoxX[0])/(BoxX[1]-BoxX[0]);
    float fy = (p.y - BoxY[0])/(BoxY[1]-BoxY[0]);
    float fz = (p.z - BoxZ[0])/(BoxZ[1]-BoxZ[0]);
    vec3 c1[2][2];
    vec3 c2[2];
    for(int i : {0,1})
    {
        for(int j : {0,1})
        {
            c1[i][j]=fx * BoxColor[0][i][j] + (1-fx) * BoxColor[1][i][j];
        }
    }
    for(int i : {0,1})
    {
        c2[i] = fy * c1[0][i] + (1-fy) * c1[1][i];
    }
    return fz * c2[0] + (1-fz) * c2[1];
    // return vec3{255, 255, 255};
    // return vec3{255, 255, 255} * (100*100-p*p>0 ? (100*100-p*p)/10000 : 0);
}

struct Shader : IShader {
    const Model &model;
    vec3 uniform_l;       // light direction in view coordinates
    mat<2,3> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
    mat<3,3> varying_nrm; // normal per vertex to be interpolated by FS
    mat<3,3> view_tri;    // triangle in view coordinates

    Shader(const Model &m) : model(m) {
        uniform_l = proj<3>((ModelView*embed<4>(light_dir, 0.))).normalized(); // transform the light vector to view coordinates
    }

    virtual void vertex(const int iface, const int nthvert, vec4& gl_Position) {
        varying_uv.set_col(nthvert, model.uv(iface, nthvert));
        varying_nrm.set_col(nthvert, proj<3>((ModelView).invert_transpose()*embed<4>(model.normal(iface, nthvert), 0.)));
        gl_Position= ModelView*embed<4>(model.vert(iface, nthvert));
        view_tri.set_col(nthvert, proj<3>(gl_Position));
        gl_Position = Projection*gl_Position;
        // std::cout << gl_Position << " ";
    }

    virtual bool fragment(const vec2 fragCoord, const vec3 bar, TGAColor &gl_FragColor) {
        vec3 bn = (varying_nrm*bar).normalized(); // per-vertex normal interpolation
        vec2 uv = varying_uv*bar; // tex coord interpolation

        // for the math refer to the tangent space normal mapping lecture
        // https://github.com/ssloy/tinyrenderer/wiki/Lesson-6bis-tangent-space-normal-mapping
        mat<3,3> AI = mat<3,3>{ {view_tri.col(1) - view_tri.col(0), view_tri.col(2) - view_tri.col(0), bn} }.invert();
        vec3 i = AI * vec3{varying_uv[0][1] - varying_uv[0][0], varying_uv[0][2] - varying_uv[0][0], 0};
        vec3 j = AI * vec3{varying_uv[1][1] - varying_uv[1][0], varying_uv[1][2] - varying_uv[1][0], 0};
        mat<3,3> B = mat<3,3>{ {i.normalized(), j.normalized(), bn} }.transpose();

        vec3 n = (B * model.normal(uv)).normalized(); // transform the normal from the texture to the tangent space
        double diff = std::max(0., n*uniform_l); // diffuse light intensity
        vec3 r = (n*(n*uniform_l)*2 - uniform_l).normalized(); // reflected light direction, specular mapping is described here: https://github.com/ssloy/tinyrenderer/wiki/Lesson-6-Shaders-for-the-software-renderer
        double spec = std::pow(std::max(-r.z, 0.), 5+sample2D(model.specular(), uv)[0]); // specular intensity, note that the camera lies on the z-axis (in view), therefore simple -r.z

        TGAColor c = sample2D(model.diffuse(), uv);
        for (int i : {0,1,2})
            gl_FragColor[i] = std::min<int>(10 + c[i]*(diff + spec), 255); // (a bit of ambient light, diff + spec), clamp the result
            
 
        return false; // the pixel is not discarded
    }

    static void mainImage(const vec2 fragCoord, TGAColor &gl_FragColor)
    {
        for (int i : {0,1,2})
        //     std::cout << gl_FragColor[i] << " ";
            gl_FragColor[i] = 100;

        // if (fragCoord*fragCoord<100*100)
        // for (int i : {0,1,2})
        //     gl_FragColor[i] = 200;
        // return;
        const int MAX_STEPS = 1000;
        const float MAX_DIST = 10.0;
        const float alpha = 0.99;
        vec3 rayDirection{fragCoord.x, fragCoord.y, 0.0};
        // std::cout << rayDirection << ";";
        vec3 p{0, 0, 200};
        rayDirection = rayDirection - p;
        rayDirection = rayDirection.normalized();
        for (int i=0; i<MAX_STEPS; i++)
        {
            p = p + rayDirection;
            if (p.z<-200)break;
            vec3 color = cloudColor(p); 
            // if(color.x>0)
            //     std::cout << color << ";";
            // for (int i : {0, 1, 2})
            //     std::cout << gl_FragColor[i] << " ";
            // if(100*100-p*p>0)//std::cout<<"!";
            for (int i : {0, 1, 2})
                // gl_FragColor[i] = gl_FragColor[i] * alpha + color[i] * (1-alpha) ;
                gl_FragColor[i] = gl_FragColor[i] * (gl_FragColor[0]/255.0) + color[i] * (1-gl_FragColor[0]/255.0);
                // gl_FragColor[i] = color[i];
                // gl_FragColor[i] = 255.0 * (100*100-p*p)/10000;
                // if(color.x>0)break;
        }

            // for (int i : {0})
            //     std::cout << gl_FragColor[i] << " ";
                // gl_FragColor[i] = 0.5;
        // int k;
        // std::cin >> k;
    }
};

int main(int argc, char** argv) {
    if (2>argc) {
        std::cerr << "Usage: " << argv[0] << " obj/model.obj" << std::endl;
        return 1;
    }
    TGAImage framebuffer(width, height, TGAImage::RGB); // the output image
    lookat(eye, center, up);                            // build the ModelView matrix
    viewport(width/8, height/8, width*3/4, height*3/4); // build the Viewport matrix
    projection((eye-center).norm());                    // build the Projection matrix
    std::vector<double> zbuffer(width*height, std::numeric_limits<double>::max());

    // for (int m=1; m<argc; m++) { // iterate through all input objects
    //     Model model(argv[m]);
    //     Shader shader(model);
    //     for (int i=0; i<model.nfaces(); i++) { // for every triangle
    //         vec4 clip_vert[3]; // triangle coordinates (clip coordinates), written by VS, read by FS
    //         for (int j : {0,1,2})
    //             shader.vertex(i, j, clip_vert[j]); // call the vertex shader for each triangle vertex
    //         triangle(clip_vert, shader, framebuffer, zbuffer); // actual rasterization routine call
    //     }
    // }
    for (int x=0; x<=framebuffer.width()-1; x++) {
        for (int y=0; y<=framebuffer.height()-1; y++) {
            TGAColor color;
            Shader::mainImage(vec2{(double)(x-framebuffer.width()/2), (double)(y-framebuffer.height()/2)}, color);
            framebuffer.set(x, y, color);
        }
    }

    framebuffer.write_tga_file("framebuffer.tga");
    std::cout << "framebuffer.tga" << std::endl;
    return 0;
}

