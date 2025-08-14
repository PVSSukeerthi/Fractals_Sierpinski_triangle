
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <universal/number/posit/posit.hpp>
#include <universal/number/cfloat/cfloat.hpp>
#include <universal/utility/convert_to.hpp>  // for double()

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

const int WIDTH = 512;
const int HEIGHT = 512;

using Posit16 = sw::universal::posit<16, 2>;
using Posit32 = sw::universal::posit<32, 2>;
using cfloat16 = sw::universal::cfloat<16, 5>;
using Pixel = std::array<unsigned char, 3>;


inline void setPixel(std::vector<Pixel>& img, int x, int y, unsigned char r, unsigned char g, unsigned char b) {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT)
        img[y * WIDTH + x] = {r, g, b};
}


template<typename T>
void drawSierpinski(T x, T y, T size, std::vector<Pixel>& img) {
    if (size < T(1.0)) return;

    int x1 = (double)(x);
    int y1 = (double)(y);
    int x2 = (double)(x + size);
    int y2 = y1;
    int x3 = (double)(x + size / T(2.0));
    int y3 = (double)(y + size * T(0.866));

    


    setPixel(img, x1, y1, 255, 255, 255);
    setPixel(img, x2, y2, 255, 255, 255);
    setPixel(img, x3, y3, 255, 255, 255);

    drawSierpinski(x, y, size / T(2.0), img);
    drawSierpinski(x + size / T(2.0), y, size / T(2.0), img);
    drawSierpinski(x + size / T(4.0), y + size * T(0.433), size / T(2.0), img);
}



void savePNG(const std::string& filename, const std::vector<Pixel>& img) {
    stbi_write_png(filename.c_str(), WIDTH, HEIGHT, 3, img.data(), WIDTH * 3);
    std::cout << "Saved: " << filename << std::endl;
}

int main() {
    // Posit<16,2>
    std::vector<Pixel> imgP16(WIDTH * HEIGHT, {0, 0, 0});
    drawSierpinski(Posit16(0), Posit16(0), Posit16(WIDTH), imgP16);
    savePNG("sierpinski_posit16.png", imgP16);

    // Posit<32,2>
    std::vector<Pixel> imgP32(WIDTH * HEIGHT, {0, 0, 0});
    drawSierpinski(Posit32(0), Posit32(0), Posit32(WIDTH), imgP32);
    savePNG("sierpinski_posit32.png", imgP32);

    // Float
    std::vector<Pixel> imgFloat(WIDTH * HEIGHT, {0, 0, 0});
    drawSierpinski(0.0f, 0.0f, (float)WIDTH, imgFloat);
    savePNG("sierpinski_float.png", imgFloat);

    // Double
    std::vector<Pixel> imgDouble(WIDTH * HEIGHT, {0, 0, 0});
    drawSierpinski<double>(0.0, 0.0, (double)WIDTH, imgDouble);
    savePNG("sierpinski_double.png", imgDouble);

    // cfloat
    std::vector<Pixel> imgHalf(WIDTH * HEIGHT, {0, 0, 0});
    drawSierpinski(cfloat16(0.0f), cfloat16(0.0f), cfloat16(WIDTH), imgHalf);
    savePNG("sierpinski_cfloat16.png", imgHalf);

    return 0;
}
