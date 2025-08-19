
// #include <iostream>
// #include <vector>
// #include <array>
// #include <cmath>
// #include <string>
// #include <universal/number/posit/posit.hpp>
// #include <universal/number/cfloat/cfloat.hpp>
// #include <universal/utility/convert_to.hpp>  // for double()

// #define STB_IMAGE_WRITE_IMPLEMENTATION
// #include "stb_image_write.h"


// const int WIDTH = 512;
// const int HEIGHT = 512;

// using Posit16 = sw::universal::posit<8, 2>;
// using Posit32 = sw::universal::posit<32, 2>;
// using cfloat16 = sw::universal::cfloat<8, 3>;
// using Pixel = std::array<unsigned char, 3>;


// inline void setPixel(std::vector<Pixel>& img, int x, int y, unsigned char r, unsigned char g, unsigned char b) {
//     if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT)
//         img[y * WIDTH + x] = {r, g, b};
// }

// inline void drawLine(std::vector<Pixel>& img, int x0, int y0, int x1, int y1,
//                      unsigned char r, unsigned char g, unsigned char b) {
//     int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
//     int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
//     int err = dx + dy, e2;

//     while (true) {
//         setPixel(img, x0, y0, r, g, b);
//         if (x0 == x1 && y0 == y1) break;
//         e2 = 2 * err;
//         if (e2 >= dy) { err += dy; x0 += sx; }
//         if (e2 <= dx) { err += dx; y0 += sy; }
//     }
// }



// template<typename T>
// void drawSierpinski(T x, T y, T size, std::vector<Pixel>& img) {
//     T scale = T(WIDTH);
//     if (size < (T(1.0))) return;
    
//     int x1 = (double)(x);
//     int y1 = (double)(y);
//     int x2 = (double)((x + size));
//     int y2 = y1;
//     int x3 = (double)((x + size / T(2.0)));
//     int y3 = (double)((y + size * T(0.866)));

    


//     // setPixel(img, x1, y1, 255, 255, 255);
//     // setPixel(img, x2, y2, 255, 255, 255);
//     // setPixel(img, x3, y3, 255, 255, 255);

//     drawLine(img, x1, y1, x2, y2, 255, 255, 255);
//     drawLine(img, x2, y2, x3, y3, 255, 255, 255);
//     drawLine(img, x3, y3, x1, y1, 255, 255, 255);


//     drawSierpinski(x, y, size / T(2.0), img);
//     drawSierpinski(x + size / T(2.0), y, size / T(2.0), img);
//     drawSierpinski(x + size / T(4.0), y + size * T(0.433), size / T(2.0), img);
// }



// void savePNG(const std::string& filename, const std::vector<Pixel>& img) {
//     stbi_write_png(filename.c_str(), WIDTH, HEIGHT, 3, img.data(), WIDTH * 3);
//     std::cout << "Saved: " << filename << std::endl;
// }

// int main() {
//     // Posit<16,2>
//     std::vector<Pixel> imgP16(WIDTH * HEIGHT, {0, 0, 0});
//     drawSierpinski(Posit16(0), Posit16(0), Posit16(WIDTH), imgP16);
//     savePNG("sierpinski_posit16.png", imgP16);

//     // Posit<32,2>
//     std::vector<Pixel> imgP32(WIDTH * HEIGHT, {0, 0, 0});
//     drawSierpinski(Posit32(0), Posit32(0), Posit32(WIDTH), imgP32);
//     savePNG("sierpinski_posit32.png", imgP32);

//     // Float
//     std::vector<Pixel> imgFloat(WIDTH * HEIGHT, {0, 0, 0});
//     drawSierpinski(0.0f, 0.0f,float(WIDTH), imgFloat);
//     savePNG("sierpinski_float.png", imgFloat);

//     // Double
//     std::vector<Pixel> imgDouble(WIDTH * HEIGHT, {0, 0, 0});
//     drawSierpinski<double>(0.0, 0.0, float(WIDTH), imgDouble);
//     savePNG("sierpinski_double.png", imgDouble);

//     // cfloat
//     std::vector<Pixel> imgHalf(WIDTH * HEIGHT, {0, 0, 0});
//     drawSierpinski(cfloat16(0.0f), cfloat16(0.0f), cfloat16(WIDTH), imgHalf);
//     savePNG("sierpinski_cfloat16.png", imgHalf);

 

//     return 0;
// }



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

const int WIDTH = 800;
const int HEIGHT = 800;

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
    int depth = 6;

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















// #include <iostream>
// #include <vector>
// #include <array>
// #include <cmath>
// #include <string>
// #include <fstream>
// #include <universal/number/posit/posit.hpp>
// #include <universal/number/cfloat/cfloat.hpp>

// #define STB_IMAGE_WRITE_IMPLEMENTATION
// #include "stb_image_write.h"

// const int WIDTH = 512;
// const int HEIGHT = 512;

// using Posit16 = sw::universal::posit<16, 2>;
// using Posit32 = sw::universal::posit<32, 2>;
// using cfloat16 = sw::universal::cfloat<16, 5>;
// using Pixel = std::array<unsigned char, 3>;

// inline void setPixel(std::vector<Pixel>& img, int x, int y, unsigned char r, unsigned char g, unsigned char b) {
//     if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT)
//         img[y * WIDTH + x] = {r, g, b};
// }

// // Structure to hold all number formats for one recursion call
// struct CoordSet {
//     double p16[6];
//     double p32[6];
//     double f32[6];
//     double f64[6];
//     double cf16[6];
// };

// template<typename T>
// void computeCoords(T x, T y, T size, double out[6]) {
//     out[0] = (double)x;
//     out[1] = (double)y;
//     out[2] = (double)(x + size);
//     out[3] = (double)y;
//     out[4] = (double)(x + size / T(2.0));
//     out[5] = (double)(y + size * ((sqrt(T(3)) / T(2))));
// }

// void drawAllFormats(Posit16 x_p16, Posit16 y_p16, Posit16 size_p16,
//                     Posit32 x_p32, Posit32 y_p32, Posit32 size_p32,
//                     float   x_f32, float   y_f32, float   size_f32,
//                     double  x_f64, double  y_f64, double  size_f64,
//                     cfloat16 x_cf16, cfloat16 y_cf16, cfloat16 size_cf16,
//                     std::vector<Pixel>& imgP16, std::vector<Pixel>& imgP32,
//                     std::vector<Pixel>& imgF32, std::vector<Pixel>& imgF64,
//                     std::vector<Pixel>& imgCF16,
//                     std::ofstream &log) 
// {
//     if (size_f64 < 1.0) return; // double as termination reference

//     CoordSet row;
//     computeCoords(x_p16, y_p16, size_p16, row.p16);
//     computeCoords(x_p32, y_p32, size_p32, row.p32);
//     computeCoords(x_f32, y_f32, size_f32, row.f32);
//     computeCoords(x_f64, y_f64, size_f64, row.f64);
//     computeCoords(x_cf16, y_cf16, size_cf16, row.cf16);

//     // Write CSV row
//     log << row.p16[0] << "," << row.p16[1] << "," << row.p16[2] << "," << row.p16[3] << "," << row.p16[4] << "," << row.p16[5] << ",";
//     log << row.p32[0] << "," << row.p32[1] << "," << row.p32[2] << "," << row.p32[3] << "," << row.p32[4] << "," << row.p32[5] << ",";
//     log << row.f32[0] << "," << row.f32[1] << "," << row.f32[2] << "," << row.f32[3] << "," << row.f32[4] << "," << row.f32[5] << ",";
//     log << row.f64[0] << "," << row.f64[1] << "," << row.f64[2] << "," << row.f64[3] << "," << row.f64[4] << "," << row.f64[5] << ",";
//     log << row.cf16[0] << "," << row.cf16[1] << "," << row.cf16[2] << "," << row.cf16[3] << "," << row.cf16[4] << "," << row.cf16[5] << "\n";

//     // Draw pixels
//     setPixel(imgP16,  (int)row.p16[0],  (int)row.p16[1], 255, 255, 255);
//     setPixel(imgP16,  (int)row.p16[2],  (int)row.p16[3], 255, 255, 255);
//     setPixel(imgP16,  (int)row.p16[4],  (int)row.p16[5], 255, 255, 255);

//     setPixel(imgP32,  (int)row.p32[0],  (int)row.p32[1], 255, 255, 255);
//     setPixel(imgP32,  (int)row.p32[2],  (int)row.p32[3], 255, 255, 255);
//     setPixel(imgP32,  (int)row.p32[4],  (int)row.p32[5], 255, 255, 255);

//     setPixel(imgF32,  (int)row.f32[0],  (int)row.f32[1], 255, 255, 255);
//     setPixel(imgF32,  (int)row.f32[2],  (int)row.f32[3], 255, 255, 255);
//     setPixel(imgF32,  (int)row.f32[4],  (int)row.f32[5], 255, 255, 255);

//     setPixel(imgF64,  (int)row.f64[0],  (int)row.f64[1], 255, 255, 255);
//     setPixel(imgF64,  (int)row.f64[2],  (int)row.f64[3], 255, 255, 255);
//     setPixel(imgF64,  (int)row.f64[4],  (int)row.f64[5], 255, 255, 255);

//     setPixel(imgCF16, (int)row.cf16[0], (int)row.cf16[1], 255, 255, 255);
//     setPixel(imgCF16, (int)row.cf16[2], (int)row.cf16[3], 255, 255, 255);
//     setPixel(imgCF16, (int)row.cf16[4], (int)row.cf16[5], 255, 255, 255);

//     // Recursive calls
//     drawAllFormats(x_p16, y_p16, size_p16 / Posit16(2.0),
//                    x_p32, y_p32, size_p32 / Posit32(2.0),
//                    x_f32, y_f32, size_f32 / 2.0f,
//                    x_f64, y_f64, size_f64 / 2.0,
//                    x_cf16, y_cf16, size_cf16 / cfloat16(2.0),
//                    imgP16, imgP32, imgF32, imgF64, imgCF16, log);

//     drawAllFormats(x_p16 + size_p16 / Posit16(2.0), y_p16, size_p16 / Posit16(2.0),
//                    x_p32 + size_p32 / Posit32(2.0), y_p32, size_p32 / Posit32(2.0),
//                    x_f32 + size_f32 / 2.0f, y_f32, size_f32 / 2.0f,
//                    x_f64 + size_f64 / 2.0, y_f64, size_f64 / 2.0,
//                    x_cf16 + size_cf16 / cfloat16(2.0), y_cf16, size_cf16 / cfloat16(2.0),
//                    imgP16, imgP32, imgF32, imgF64, imgCF16, log);

//     drawAllFormats(x_p16 + size_p16 / Posit16(4.0), y_p16 + size_p16 * (sqrt(Posit16(3)) / Posit16(4)), size_p16 / Posit16(2.0),
//                    x_p32 + size_p32 / Posit32(4.0), y_p32 + size_p32 *  (sqrt(Posit32(3)) / Posit32(4)), size_p32 / Posit32(2.0),
//                    x_f32 + size_f32 / 4.0f, y_f32 + size_f32 * 0.433f, size_f32 / 2.0f,
//                    x_f64 + size_f64 / 4.0, y_f64 + size_f64 * 0.433, size_f64 / 2.0,
//                    x_cf16 + size_cf16 / cfloat16(4.0), y_cf16 + size_cf16 * (sqrt(cfloat16(3)) / cfloat16(4)), size_cf16 / cfloat16(2.0),
//                    imgP16, imgP32, imgF32, imgF64, imgCF16, log);
// }

// void savePNG(const std::string& filename, const std::vector<Pixel>& img) {
//     stbi_write_png(filename.c_str(), WIDTH, HEIGHT, 3, img.data(), WIDTH * 3);
//     std::cout << "Saved: " << filename << std::endl;
// }

// int main() {
//     std::ofstream log("sierpinski_side_by_side_double_new.csv");
//     log << "P16_x1,P16_y1,P16_x2,P16_y2,P16_x3,P16_y3,"
//            "P32_x1,P32_y1,P32_x2,P32_y2,P32_x3,P32_y3,"
//            "F32_x1,F32_y1,F32_x2,F32_y2,F32_x3,F32_y3,"
//            "F64_x1,F64_y1,F64_x2,F64_y2,F64_x3,F64_y3,"
//            "CF16_x1,CF16_y1,CF16_x2,CF16_y2,CF16_x3,CF16_y3\n";

//     std::vector<Pixel> imgP16(WIDTH * HEIGHT, {0, 0, 0});
//     std::vector<Pixel> imgP32(WIDTH * HEIGHT, {0, 0, 0});
//     std::vector<Pixel> imgF32(WIDTH * HEIGHT, {0, 0, 0});
//     std::vector<Pixel> imgF64(WIDTH * HEIGHT, {0, 0, 0});
//     std::vector<Pixel> imgCF16(WIDTH * HEIGHT, {0, 0, 0});

//     drawAllFormats(Posit16(0), Posit16(0), Posit16(WIDTH),
//                    Posit32(0), Posit32(0), Posit32(WIDTH),
//                    0.0f, 0.0f, (float)WIDTH,
//                    0.0, 0.0, (double)WIDTH,
//                    cfloat16(0.0f), cfloat16(0.0f), cfloat16(WIDTH),
//                    imgP16, imgP32, imgF32, imgF64, imgCF16, log);

//     savePNG("sierpinski_posit16.png", imgP16);
//     savePNG("sierpinski_posit32.png", imgP32);
//     savePNG("sierpinski_float.png", imgF32);
//     savePNG("sierpinski_double.png", imgF64);
//     savePNG("sierpinski_cfloat16.png", imgCF16);

//     log.close();
//     std::cout << "CSV saved: sierpinski_side_by_side_ints.csv\n";
//     return 0;
// }
