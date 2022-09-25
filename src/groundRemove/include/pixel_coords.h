#ifndef SRC_PIXEL_COORDS_H_
#define SRC_PIXEL_COORDS_H_

#include <cstdint>
/**
 * @brief      Pixel coordinates structure
 */
struct PixelCoord 
{
    PixelCoord() : row(0), col(0) {}
    PixelCoord(int16_t row_, int16_t col_) : row(row_), col(col_) {}
    PixelCoord operator+(const PixelCoord& other) const 
    {
        return PixelCoord(row + other.row, col + other.col);
    }

    int16_t row;
    int16_t col;
};
#endif  // SRC_IMAGE_LABELERS_PIXEL_COORDS_H_