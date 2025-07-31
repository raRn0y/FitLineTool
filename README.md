# 1D Edge Detection and Line Fitting Tool

A computer vision tool for 1D edge detection and line fitting in industrial measurement applications.

## Features

- **Multi-point Edge Detection**  
  Simultaneously detects edges at multiple measurement points along a line

- **Advanced Filtering**  
  Combines Gaussian blur and gradient thresholding for noise-resistant edge detection

- **Smart Edge Selection**  
  Supports multiple edge selection strategies:
  - First/Last edge
  - Strongest/Weakest gradient edge
  - Positive/Negative transition edges

- **Robust Line Fitting**  
  Implements three-stage fitting pipeline:
  1. RANSAC outlier rejection
  2. OpenCV's DIST_HUBER fitting
  3. Intersection-based endpoint calculation

- **Interactive GUI**  
  Real-time parameter adjustment via trackbars and mouse controls

## Installation

### Prerequisites
- OpenCV 4.x
- C++17 compatible compiler

### Build Instructions
```bash
mkdir build && cd build
cmake ..
make
```

## Usage

```cpp
// Create caliper tool
CCaliperGraphics caliper;
caliper.CreateCaliper(image, startPoint, endPoint, 
                    measureLength, measureHeight, 
                    sigma, threshold, translation, measureCount);

// Find and fit line
Point2d lineStart, lineEnd;
double angle;
caliper.FindLine(lineStart, lineEnd, angle);
```

## Parameters

| Parameter       | Description                          | Typical Value |
|-----------------|--------------------------------------|---------------|
| Measure Length  | Sampling line length in pixels       | 30-100 px     |
| Measure Height  | Profile averaging height             | 5-20 px       |
| Sigma           | Gaussian blur strength               | 1-3 px        |
| Threshold       | Minimum gradient magnitude           | 10-50         |
| Measure Points  | Number of sampling points            | 5-20          |

## Non commercial, Not original
please email eziozhang7956@gmail.com if there is any infringement
