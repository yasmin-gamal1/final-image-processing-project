
---

# MATLAB Image Processing Project

This MATLAB project contains various image filters implemented using MATLAB functions.

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Usage](#usage)
- [Image Filters](#image-filters)
- [Screenshots](#screenshots)
- [Contributing](#contributing)
- [License](#license)

## Overview

This repository contains MATLAB scripts that demonstrate different image processing techniques and filters. These scripts can be used to manipulate images using various filters such as Gaussian blur, edge detection, and grayscale conversion.

## Installation

To run these scripts, ensure you have MATLAB installed on your system. Clone this repository using the following command:

```
git clone https://github.com/your-username/your-repository.git
```

## Usage

1. Navigate to the directory where you cloned the repository.
2. Open MATLAB and run the scripts directly from the MATLAB command window or editor.
3. Follow the prompts or edit the scripts to process your own images.

## Image Filters
The following image filters are implemented in this project:

Gaussian Blur: Smooths the image using a Gaussian kernel.
Median Filter: Reduces noise in the image by replacing each pixel's value with the median value of the neighboring pixels.
Bilateral Filter: Preserves edges while reducing noise by considering both spatial and intensity differences.
Sobel Edge Detection: Detects edges in the image using the Sobel operator.
Canny Edge Detection: Detects edges using the Canny edge detection algorithm.
Prewitt Edge Detection: Detects edges using the Prewitt operator.
Laplacian Filter: Detects edges by calculating the second derivative of the image.
Unsharp Masking: Enhances edges by subtracting a blurred version of the image from the original image.
Histogram Equalization: Improves contrast in the image by spreading out the intensity values.
Adaptive Histogram Equalization (CLAHE): Improves contrast using a method that adapts to small regions in the image.
Fourier Transform: Converts the image from the spatial domain to the frequency domain.
Inverse Fourier Transform: Converts the image from the frequency domain back to the spatial domain.
Grayscale Conversion: Converts a color image to grayscale.
Thresholding: Converts the image to a binary image based on a threshold value.
Morphological Operations: Performs operations such as dilation, erosion, opening, and closing on binary images.
Gaussian Noise Addition: Adds Gaussian noise to the image.
Salt and Pepper Noise Addition: Adds salt and pepper noise to the image.
Speckle Noise Addition: Adds speckle noise to the image.
Each filter is implemented in its respective MATLAB script with detailed explanations and usage instructions provided within each script file.
## Screenshots

Include screenshots of the input images and the output images after applying each filter. These screenshots can help demonstrate the effectiveness of each filter. Insert them below:
![WhatsApp Image 2024-06-23 at 09 49 33_037046c5](https://github.com/yasmin-gamal1/final-image-processing-project/assets/148600481/1b49883e-bc80-4bc0-ba21-a0ce4e279386)
![WhatsApp Image 2024-06-23 at 09 49 32_3139b9c5](https://github.com/yasmin-gamal1/final-image-processing-project/assets/148600481/33a1092b-c65b-43f7-9bb4-9d344e9a7648)
![WhatsApp Image 2024-06-23 at 09 49 28_8e43c15e](https://github.com/yasmin-gamal1/final-image-processing-project/assets/148600481/5bb7ade3-d357-4d43-b294-a8004e70e4fa)


## Contributing

Contributions to improve existing filters or add new filters are welcome. Follow these steps:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature/add-new-filter`).
3. Make your changes.
4. Commit your changes (`git commit -am 'Add new filter: <filter-name>'`).
5. Push to the branch (`git push origin feature/add-new-filter`).
6. Create a new Pull Request.

## License

This project is licensed under the MIT License - see the `LICENSE` file for details.

---
