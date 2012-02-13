/**
 * \file  ImageRef.h
 * \brief Provides fast, efficient and convenient access to an IplImage object
 *
 * Code for fast element access in opencv IplImage object. Taken from  
 * http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/opencv-intro.html 
 *
 * $ Id: 08/16/2011 02:04:00 PM piyushk $
 */

#ifndef IMAGEREF_KV1N7KNH
#define IMAGEREF_KV1N7KNH

#include <opencv/cv.h>

template <class T> 
class ImageRef {

  private:
    IplImage* imgp;

  public:
    ImageRef(IplImage* img = 0) { 
      imgp = img; 
    }
    ~ImageRef() { 
      imgp = 0; 
    }
    void operator = (IplImage* img) { 
      imgp = img; 
    }
    inline T* operator[](const int rowIndx) {
      return ((T *)(imgp->imageData + rowIndx*imgp->widthStep));
    }
};

typedef struct {
  unsigned char b, g, r;
} RgbPixel;

typedef struct {
  float b, g, r;
} RgbPixelFloat;

typedef ImageRef<RgbPixel>       RgbImage;
typedef ImageRef<RgbPixelFloat>  RgbImageFloat;
typedef ImageRef<unsigned char>  BwImage;
typedef ImageRef<float>          BwImageFloat;
typedef ImageRef<int>            BwImageInt;

#endif /* end of include guard: IMAGEREF_KV1N7KNH */
