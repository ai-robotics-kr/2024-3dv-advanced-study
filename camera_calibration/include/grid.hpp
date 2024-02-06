#pragma once

#include <vector>
#include <opencv2/core/core.hpp>






template <typename T>
class Grid2d
{
public:
    using iterator = typename std::vector<T>::iterator;
    using const_iterator = typename std::vector<T>::const_iterator;
    
    using reverse_iterator = typename std::vector<T>::reverse_iterator;
    using const_reverse_iterator = typename std::vector<T>::const_reverse_iterator;


    Grid2d() = default;
    Grid2d(int rows, int cols, const T& val = {})
        : grid_size{cols, rows}, data(rows * cols, val) {}
    explicit Grid2d(const cv::Size& grid_size, const T& val = {})
        : Grid2d(grid_size.height, grid_size.width, val) {}

    T& at(int r, int c) { return data.at(rc2ind(r, c)); }
    const T& at(int r, int c) const { return data.at(rc2ind(r, c)); }

    void reset(const T& val = {}) { data.assign(data.size(), val); }
    void resize(const cv::Size& grid_size_, const T& val = {})
    {
        grid_size = grid_size_;
        data.resize(grid_size.area(), val);
    }

    T operator[](int i) const { return data[i]; }
    T& operator[](int i) { return data[i]; }

    int size() const noexcept { return data.size(); }
    int rows() const noexcept { return grid_size.height; }
    int cols() const noexcept { return grid_size.width; }

    iterator begin() noexcept { return data.begin(); }
    iterator end() noexcept { return data.end(); }

    const_iterator begin() const noexcept { return data.begin(); }
    const_iterator end() const noexcept { return data.end(); }

    const_iterator cbegin() const noexcept { return data.cbegin(); }
    const_iterator cend() const noexcept { return data.cend(); }

    // Util in struct
    int rc2ind(int r, int c) const noexcept { return r * grid_size.width + c; }

    cv::Size grid_size{};
    std::vector<T> data{};
};