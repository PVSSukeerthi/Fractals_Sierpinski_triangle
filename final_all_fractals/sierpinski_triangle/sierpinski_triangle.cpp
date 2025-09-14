


#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <complex>
#include <universal/number/posit/posit.hpp>
#include <universal/number/cfloat/cfloat.hpp>
#include <universal/utility/convert_to.hpp>  // for double()

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace std;

const int WIDTH = 3200;
const int HEIGHT = 3200;

using Pixel = std::array<unsigned char, 3>;


inline void setPixel(vector<Pixel>& img, int x, int y,
                     unsigned char r, unsigned char g, unsigned char b) {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT)
        img[y * WIDTH + x] = {r, g, b};
}

inline void drawLine(vector<Pixel>& img, int x0, int y0, int x1, int y1,
                     unsigned char r, unsigned char g, unsigned char b) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
        setPixel(img, x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Recursive Sierpinski with generic arithmetic type T
template<typename T>
void divideTriangle(vector<Pixel>& img,T a[2], T b[2], T c[2], int depth)
{
    
    if (depth == 0) {
        int ax = int( (a[0]) * WIDTH);
        int ay = int( (a[1]) * HEIGHT);
        int bx = int( (b[0]) * WIDTH);
        int by = int( (b[1]) * HEIGHT);
        int cx = int( (c[0]) * WIDTH);
        int cy = int( (c[1]) * HEIGHT);

        drawLine(img, ax, ay, bx, by, 255, 255, 255);
        drawLine(img, bx, by, cx, cy, 255, 255, 255);
        drawLine(img, cx, cy, ax, ay, 255, 255, 255);
        return;
    }

    T ab[2] = {(a[0] + b[0]) / T(2), (a[1] + b[1]) / T(2)};
    T ac[2] = {(a[0] + c[0]) / T(2), (a[1] + c[1]) / T(2)};
    T bc[2] = {(b[0] + c[0]) / T(2), (b[1] + c[1]) / T(2)};

    divideTriangle(img, a, ab, ac, depth - 1);
    divideTriangle(img, b, ab, bc, depth - 1);
    divideTriangle(img, c, ac, bc, depth - 1);
}

void savePNG(const std::string& filename, vector<Pixel>& img) {
    stbi_write_png(filename.c_str(), WIDTH, HEIGHT, 3, img.data(), WIDTH * 3);
    std::cout << "Saved: " << filename << std::endl;
}

template<typename T>
void runSierpinski(const std::string& filename, int depth) {
    vector<Pixel> img(WIDTH * HEIGHT, {0, 0, 0});

    T A[2] = { T(0.1), T(0.1) };
    T B[2] = { T(0.9), T(0.1) };
    T C[2] = { T(0.5), T(0.9) };

    divideTriangle(img, A, B, C, depth);
    savePNG(filename, img);
}

int main() {
    int depth = 8;

    // float
    runSierpinski<float>("sierpinski_float.png", depth);

    // double
    runSierpinski<double>("sierpinski_double.png", depth);

    // posit16<1>
    runSierpinski< sw::universal::posit<16,2> >("sierpinski_posit16.png", depth);

    // posit32<2>
    runSierpinski< sw::universal::posit<32,2> >("sierpinski_posit32.png", depth);

    // cfloat<16,5> (example: 16-bit float with 5 exponent bits)
    runSierpinski< sw::universal::cfloat<16,5> >("sierpinski_cfloat16.png", depth);

    return 0;
}







