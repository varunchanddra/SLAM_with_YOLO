#include <stdio.h>
#include "darknet.h"
// #include "stdio.h"
// #include "stdlib.h"
// #include "opencv2/opencv.hpp"
// #include "image.h"

// image ipl_to_image(IplImage* src)
// {
//     int h = src->height;
//     int w = src->width;
//     int c = src->nChannels;
//     image im = make_image(w, h, c);
//     unsigned char *data = (unsigned char *)src->imageData;
//     int step = src->widthStep;
//     int i, j, k;

//     for(i = 0; i < h; ++i){
//         for(k= 0; k < c; ++k){
//             for(j = 0; j < w; ++j){
//                 im.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
//             }
//         }
//     }
//     return im;
// }




// image mat_to_image(cv::Mat m)
// {
//     IplImage ipl = m;
//     image im = ipl_to_image(&ipl);
//     rgbgr_image(im);
//     return im;
// }

// image mat_to_image(cv::Mat m);


