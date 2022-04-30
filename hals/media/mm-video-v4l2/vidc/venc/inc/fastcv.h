#ifndef FASTCV_H
#define FASTCV_H

//==============================================================================
// Defines
//==============================================================================

#ifdef __GNUC__

   #define FASTCV_ALIGN32( VAR ) (VAR)  __attribute__ ((aligned(4)))

   #define FASTCV_ALIGN64( VAR )  (VAR) __attribute__ ((aligned(8)))

   #define FASTCV_ALIGN128( VAR ) (VAR) __attribute__ ((aligned(16)))
   #ifdef BUILDING_SO

   #define FASTCV_API __attribute__ ((visibility ("default")))
   #else

   #define FASTCV_API
   #endif
#else

   #define FASTCV_ALIGN32( VAR ) __declspec(align(4)) (VAR)

   #define FASTCV_ALIGN64( VAR ) __declspec(align(8)) (VAR)

   #define FASTCV_ALIGN128( VAR ) __declspec(align(16)) (VAR)
   #ifdef BUILDING_DLL

   #define FASTCV_API __declspec(dllexport)
   #else

   #define FASTCV_API
   #endif
#endif

//==============================================================================
// Included modules
//==============================================================================

#include <stddef.h>
#include <stdint.h>
typedef float  float32_t;
typedef double float64_t;

//==============================================================================
// Declarations
//==============================================================================


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef enum
{
   FASTCV_OP_LOW_POWER       = 0,

   FASTCV_OP_PERFORMANCE     = 1,

   FASTCV_OP_CPU_OFFLOAD     = 2,

   FASTCV_OP_CPU_PERFORMANCE = 3,

   FASTCV_OP_RESERVED        = 0x80000000

} fcvOperationMode;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef enum
{
   FASTCV_FLIP_HORIZ    = 1,

   FASTCV_FLIP_VERT     = 2,

   FASTCV_FLIP_BOTH     = 3

} fcvFlipDir;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef enum
{
   FASTCV_ROTATE_90     = 1,

   FASTCV_ROTATE_180    = 2,

   FASTCV_ROTATE_270    = 3

} fcvRotateDegree;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

typedef enum
{
   FASTCV_INTERPOLATION_TYPE_NEAREST_NEIGHBOR = 0,

   FASTCV_INTERPOLATION_TYPE_BILINEAR,

   FASTCV_INTERPOLATION_TYPE_AREA

} fcvInterpolationType;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

typedef enum
{
   FASTCV_CONVERT_POLICY_WRAP = 0,

   FASTCV_CONVERT_POLICY_SATURATE

} fcvConvertPolicy;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

typedef enum
{
   FASTCV_BORDER_UNDEFINED = 0,

   FASTCV_BORDER_CONSTANT,

   FASTCV_BORDER_REPLICATE

} fcvBorderType;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

typedef enum
{
   FASTCV_NORM_L1,

   FASTCV_NORM_L2

} fcvNormType;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

typedef enum
{
    FASTCV_CHANNEL_0,

    FASTCV_CHANNEL_1,

    FASTCV_CHANNEL_2,

    FASTCV_CHANNEL_3,

    FASTCV_CHANNEL_R,

    FASTCV_CHANNEL_G,

    FASTCV_CHANNEL_B,

    FASTCV_CHANNEL_A,

    FASTCV_CHANNEL_Y,

    FASTCV_CHANNEL_U,

    FASTCV_CHANNEL_V

}  fcvChannelType;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

typedef enum
{
    FASTCV_RGB,

    FASTCV_RGBX,

    FASTCV_NV12,

    FASTCV_NV21,

    FASTCV_UYVY,

    FASTCV_YUYV,

    FASTCV_IYUV,

    FASTCV_YUV4

} fcvImageFormat;


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef enum
{
   FASTCV_SUCCESS = 0,

   FASTCV_EFAIL,

   FASTCV_EUNALIGNPARAM,

   FASTCV_EBADPARAM,

   FASTCV_EINVALSTATE,

   FASTCV_ENORES,

   FASTCV_EUNSUPPORTED,

   FASTCV_EHWQDSP,

   FASTCV_EHWGPU

} fcvStatus;


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef enum
{
   FASTCV_SVM_LINEAR,
   FASTCV_SVM_POLY,
   FASTCV_SVM_RBF,
   FASTCV_SVM_SIGMOID
} fcvSVMKernelType;


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef enum
{
    FASTCV_PYRAMID_SCALE_HALF,
    FASTCV_PYRAMID_SCALE_ORB
} fcvPyramidScale;


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef enum
{
    FASTCV_TERM_CRITERIA_ITERATIONS,

    FASTCV_TERM_CRITERIA_EPSILON,

    FASTCV_TERM_CRITERIA_BOTH,
} fcvTerminationCriteria;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef enum
{
   FASTCV_HOG_NORM_REGULAR = 0,

   FASTCV_HOG_NORM_RENORMALIZATION = 1,

   FASTCV_HOG_NORM_FHOG = 2,

} fcvHOGNormMethod;


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef enum
{
   FASTCV_UNBIASED_VARIANCE_ESTIMATOR = 0,

   FASTCV_BIASED_VARIANCE_ESTIMATOR = 1,

} fcvVarianceEstimator;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef struct
{
   const float32_t*               from;
   /*~ FIELD fcvCorrespondences.from
       VARRAY LENGTH ( fcvCorrespondences.numCorrespondences * \
       (fcvCorrespondences.fromStride ? fcvCorrespondences.fromStride : 3) ) */

   const float32_t*               to;
   /*~ FIELD fcvCorrespondences.to
       VARRAY LENGTH ( fcvCorrespondences.numCorrespondences * \
       (fcvCorrespondences.toStride ? fcvCorrespondences.toStride : 2) ) */

   uint32_t                       fromStride;

   uint32_t                       toStride;

   uint32_t                       numCorrespondences;

   const uint16_t*                indices;
   /*~ FIELD fcvCorrespondences.indices VARRAY LENGTH (fcvCorrespondences.numIndices) */

   uint32_t                       numIndices;
} fcvCorrespondences;


// -----------------------------------------------------------------------------
//------------------------------------------------------------------------------

typedef struct
{
   const void* ptr;
   unsigned int width;
   unsigned int height;
} fcvPyramidLevel ;

// -----------------------------------------------------------------------------
//------------------------------------------------------------------------------

typedef struct
{
   const void* ptr;
   unsigned int width;
   unsigned int height;
   unsigned int stride;
} fcvPyramidLevel_v2 ;

// -----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
typedef struct fcvKDTreeNodef32
{
   float32_t divVal;

   int32_t divFeat;

   int32_t childLeft;

   int32_t childRight;

} fcvKDTreeNodef32;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
typedef struct fcvKDTreeBranchf32
{
   float32_t minDistSq;

   int32_t topNode;

} fcvKDTreeBranchf32;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
typedef struct fcvKDTreeDatas8f32
{
   // info about the dataset for which KDTrees are constructed
   const int8_t *dataset;

   const float32_t* invLen;

   int32_t numVectors;

   // info about trees
   int32_t* trees;

   fcvKDTreeNodef32* nodes;

   int32_t numNodes;

   int32_t maxNumNodes;

   // info used during lookups
   fcvKDTreeBranchf32* heap;

   int32_t numBranches;

   int32_t maxNumBranches;

   int32_t* vind;

   int32_t checkID;

   int32_t numNNs;

} fcvKDTreeDatas8f32;


// -----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
typedef struct fcvKDTreeNodes32
{
   int32_t divVal;

   int32_t divFeat;

   int32_t childLeft;

   int32_t childRight;

} fcvKDTreeNodes32;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
typedef struct fcvKDTreeBranchs32
{
   int32_t minDistSq;

   int32_t topNode;

} fcvKDTreeBranchs32;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
typedef struct fcvKDTreeDatas8s32
{
   // info about the dataset for which KDTrees are constructed
   const int8_t *dataset;

   const int32_t* invLen;

   int32_t numVectors;

   // info about trees
   int32_t* trees;

   int32_t numTrees;

   fcvKDTreeNodes32* nodes;

   int32_t numNodes;

   int32_t maxNumNodes;

   // info used during lookups
   fcvKDTreeBranchs32* heap;

   int32_t numBranches;

   int32_t maxNumBranches;

   int32_t* vind;

   int32_t checkID;

   int32_t numNNs;

} fcvKDTreeDatas8s32;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef struct
{
    int32_t x;
    int32_t y;
    uint32_t width;
    uint32_t height;
} fcvRectangleInt;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef struct
{
    int32_t      max_iter;
    float32_t    epsilon;
}fcvTermCriteria;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef struct
{
    // Center of the box
    int32_t x;
    int32_t y;
    // The box size
    int32_t    columns;
    int32_t    rows;
    // The orientation of the principal axis
    int32_t orientation;
}fcvBox2D;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef struct {
    // spatial moments
    float32_t m00, m10, m01, m20, m11, m02, m30, m21, m12, m03;
    // central moments
    float32_t mu20, mu11, mu02, mu30, mu21, mu12, mu03;
    // m00 != 0 ? 1/sqrt(m00) : 0
    float32_t inv_sqrt_m00;
} fcvMoments;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef struct fcvBGCodeWord
{
    struct fcvBGCodeWord* next;

    int32_t tLastUpdate;

    int32_t stale;
    uint8_t min0, min1, min2;

    uint8_t max0, max1, max2;

    uint8_t learnLow0, learnLow1, learnLow2;

    uint8_t learnHigh0, learnHigh1, learnHigh2;
} fcvBGCodeWord;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef struct fcvCircle
{
    int32_t x;
    int32_t y;
    int32_t radius;
} fcvCircle;


typedef struct fcvPoint2D
{
    float x;
    float y;
} fcvPoint2D;

typedef struct fcvLine
{
    fcvPoint2D start;
    fcvPoint2D end;
} fcvLine;


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef struct fcvLineSegment {
    fcvPoint2D start, end;          
    float32_t  normal[2];           
    uint32_t   nPoints;             
    int32_t   pointsStartIndex;     
    uint32_t   sumMag;              
} fcvLineSegment;


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef struct fcvDepthFusionBlockConfig {
    uint32_t  flags;                    
    float32_t ramp;                     
    float32_t p0[3],                    
              dX[3],                    
              dY[3],                    
              dZ[3];                    
    uint32_t  volumeIndex;              
} fcvDepthFusionBlockConfig;

//==============================================================================
// UTILITY FUNCTIONS
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvGetVersion( char*        version,
               unsigned int versionLength );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int
fcvSetOperationMode( fcvOperationMode mode );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvCleanUp( void );


// -----------------------------------------------------------------------------
//----------------------------------------------------------------------------

FASTCV_API int
fcvPyramidAllocate( fcvPyramidLevel* pyr,
                    unsigned int     baseWidth,
                    unsigned int     baseHeight,
                    unsigned int     bytesPerPixel,
                    unsigned int     numLevels,
                    int              allocateBase );


// -----------------------------------------------------------------------------
//----------------------------------------------------------------------------

FASTCV_API int
fcvPyramidAllocate_v2(  fcvPyramidLevel_v2* pyr,
                        uint32_t            baseWidth,
                        uint32_t            baseHeight,
                        uint32_t            baseStride,
                        uint32_t            bytesPerPixel,
                        uint32_t            numLevels,
                        int32_t             allocateBase );


// -----------------------------------------------------------------------------
//----------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvPyramidAllocate_v3(fcvPyramidLevel_v2* __restrict  pyr,
                      uint32_t            baseWidth,
                      uint32_t            baseHeight,
                      uint32_t            baseStride,
                      uint32_t            bytesPerPixel,
                      uint32_t            alignment,
                      uint32_t            numLevels,
                      fcvPyramidScale     scale,
                      int32_t             allocateBase);


// -----------------------------------------------------------------------------
//----------------------------------------------------------------------------

FASTCV_API void
fcvPyramidDelete( fcvPyramidLevel* pyr,
                  unsigned int     numLevels,
                  unsigned int     startLevel );

// -----------------------------------------------------------------------------
//----------------------------------------------------------------------------

FASTCV_API void
fcvPyramidDelete_v2( fcvPyramidLevel_v2* pyr,
                     uint32_t            numLevels,
                     uint32_t            startLevel );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void*
fcvMemAlloc( unsigned int nBytes,
             unsigned int byteAlignment );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvMemFree( void* ptr );


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FASTCV_API void
fcvMemInit(void);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FASTCV_API void
fcvMemInitPreAlloc( uint32_t preAllocBytes );

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FASTCV_API void
fcvMemDeInit(void);

//End - Utility functions


//==============================================================================
// FUNCTIONS
//==============================================================================


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterMedian3x3u8( const uint8_t* __restrict srcImg,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      uint8_t* __restrict       dstImg );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterMedian3x3u8_v2( const uint8_t* __restrict srcImg,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         uint8_t* __restrict       dstImg,
                         unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian3x3u8( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        uint8_t* __restrict       dst,
                        int                       blurBorder );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian3x3u8_v2( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           uint8_t* __restrict       dst,
                           unsigned int              dstStride,
                           int                       blurBorder );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5u8( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        uint8_t* __restrict       dst,
                        int                       blurBorder );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5u8_v2( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           uint8_t* __restrict       dst,
                           unsigned int              dstStride,
                           int                       blurBorder );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian11x11u8( const uint8_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          uint8_t* __restrict       dst,
                          int                       blurBorder );



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian11x11u8_v2( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             uint8_t* __restrict       dst,
                             unsigned int              dstStride,
                             int                       blurBorder );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYUV420toRGB8888u8( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           uint32_t* __restrict      dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCrCb420PseudoPlanarToRGB8888u8( const uint8_t* __restrict src,
                                         unsigned int              srcWidth,
                                         unsigned int              srcHeight,
                                         unsigned int              srcYStride,
                                         unsigned int              srcCStride,
                                         uint32_t* __restrict      dst,
                                         unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYUV420toRGB565u8( const uint8_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          uint32_t*  __restrict     dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCrCbH1V1toRGB888u8( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCrCbH2V2toRGB888u8( const uint8_t* __restrict ysrc,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             uint8_t* __restrict       dst );



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCrCbH2V1toRGB888u8( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCrCbH1V2toRGB888u8( const uint8_t* __restrict ysrc,

                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             uint8_t* __restrict       dst );



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888toYCrCbu8( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888toYCrCbu8_v2( const uint8_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            unsigned int              srcStride,
                            uint8_t* __restrict       dst,
                            unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvDescriptor17x17u8To36s8( const uint8_t* __restrict patch,
                            int8_t* __restrict        descriptorChar,
                            int32_t* __restrict       descriptorNormSq );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int32_t
fcvDotProducts8( const int8_t* __restrict a,
                 const int8_t* __restrict b,
                 unsigned int             abSize );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvDotProductu8( const uint8_t* __restrict  a,
                 const uint8_t* __restrict  b,
                 unsigned int               abSize );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int32_t
fcvDotProduct36x1s8( const int8_t* __restrict a,
                     const int8_t* __restrict b );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct36x4s8( const int8_t* __restrict a,
                     const int8_t* __restrict b,
                     const int8_t* __restrict c,
                     const int8_t* __restrict d,
                     const int8_t* __restrict e,
                     int32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm36x4s8( const int8_t* __restrict a,
                         float                    invLengthA,
                         const int8_t* __restrict b0,
                         const int8_t* __restrict b1,
                         const int8_t* __restrict b2,
                         const int8_t* __restrict b3,
                         float* __restrict        invLengthsB,
                         float* __restrict        dotProducts  );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvDotProduct36x1u8( const uint8_t* __restrict a,
                     const uint8_t* __restrict b );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct36x4u8( const uint8_t* __restrict a,
                     const uint8_t* __restrict b,
                     const uint8_t* __restrict c,
                     const uint8_t* __restrict d,
                     const uint8_t* __restrict e,
                     uint32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm36x4u8( const uint8_t* __restrict  a,
                         float                      invLengthA,
                         const uint8_t* __restrict  b0,
                         const uint8_t* __restrict  b1,
                         const uint8_t* __restrict  b2,
                         const uint8_t* __restrict  b3,
                         float* __restrict          invLengthsB,
                         float* __restrict          dotProducts );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int32_t
fcvDotProduct64x1s8( const int8_t* __restrict a,
                     const int8_t* __restrict b );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct64x4s8( const int8_t* __restrict a,
                     const int8_t* __restrict b,
                     const int8_t* __restrict c,
                     const int8_t* __restrict d,
                     const int8_t* __restrict e,
                     int32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm64x4s8( const int8_t* __restrict a,
                         float                    invLengthA,
                         const int8_t* __restrict b0,
                         const int8_t* __restrict b1,
                         const int8_t* __restrict b2,
                         const int8_t* __restrict b3,
                         float* __restrict        invLengthsB,
                         float* __restrict        dotProducts  );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvDotProduct64x1u8( const uint8_t* __restrict a,
                     const uint8_t* __restrict b );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct64x4u8( const uint8_t* __restrict a,
                     const uint8_t* __restrict b,
                     const uint8_t* __restrict c,
                     const uint8_t* __restrict d,
                     const uint8_t* __restrict e,
                     uint32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm64x4u8( const uint8_t* __restrict  a,
                         float                      invLengthA,
                         const uint8_t* __restrict  b0,
                         const uint8_t* __restrict  b1,
                         const uint8_t* __restrict  b2,
                         const uint8_t* __restrict  b3,
                         float* __restrict          invLengthsB,
                         float* __restrict          dotProducts );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int32_t
fcvDotProduct128x1s8( const int8_t* __restrict a,
                      const int8_t* __restrict b );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct128x4s8( const int8_t* __restrict a,
                      const int8_t* __restrict b,
                      const int8_t* __restrict c,
                      const int8_t* __restrict d,
                      const int8_t* __restrict e,
                      int32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm128x4s8( const int8_t* __restrict a,
                          float                    invLengthA,
                          const int8_t* __restrict b0,
                          const int8_t* __restrict b1,
                          const int8_t* __restrict b2,
                          const int8_t* __restrict b3,
                          float* __restrict        invLengthsB,
                          float* __restrict        dotProducts  );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvDotProduct128x1u8( const uint8_t* __restrict a,
                      const uint8_t* __restrict b );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct128x4u8( const uint8_t* __restrict a,
                      const uint8_t* __restrict b,
                      const uint8_t* __restrict c,
                      const uint8_t* __restrict d,
                      const uint8_t* __restrict e,
                      uint32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm128x4u8( const uint8_t* __restrict  a,
                          float                      invLengthA,
                          const uint8_t* __restrict  b0,
                          const uint8_t* __restrict  b1,
                          const uint8_t* __restrict  b2,
                          const uint8_t* __restrict  b3,
                          float* __restrict          invLengthsB,
                          float* __restrict          dotProducts );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct8x8u8( const uint8_t* __restrict patchPixels,
                    const uint8_t* __restrict imagePixels,
                    unsigned short            imgW,
                    unsigned short            imgH,
                    int                       nX,
                    int                       nY,
                    unsigned int              nNum,
                    int32_t* __restrict       dotProducts );


//------------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct11x12u8( const uint8_t* __restrict patchPixels,
                      const uint8_t* __restrict imagePixels,
                      unsigned short            imgW,
                      unsigned short            imgH,
                      int                       iX,
                      int                       iY,
                      int32_t* __restrict       dotProducts );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterSobel3x3u8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterSobel3x3u8_v2( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        uint8_t* __restrict       dst,
                        unsigned int              dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCanny3x3u8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     uint8_t* __restrict       dst,
                     int                       lowThresh,
                     int                       highThresh );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCanny3x3u8_v2( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        uint8_t* __restrict       dst,
                        unsigned int              dstStride,
                        int                       lowThresh,
                        int                       highThresh );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvFilterCanny3x3u8_v3( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        uint8_t* __restrict       dst,
                        unsigned int              dstStride,
                        int16_t* __restrict       gx,
                        int16_t* __restrict       gy,
                        unsigned int              gradStride,
                        int                       lowThresh,
                        int                       highThresh );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageDiffu8(   const uint8_t* __restrict src1,
                  const uint8_t* __restrict src2,
                   unsigned int             srcWidth,
                   unsigned int             srcHeight,
                        uint8_t* __restrict dst );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageDiffu8_v2( const uint8_t* __restrict src1,
                   const uint8_t* __restrict src2,
                   unsigned int              srcWidth,
                   unsigned int              srcHeight,
                   unsigned int              srcStride,
                   uint8_t* __restrict       dst,
                   unsigned int              dstStride );


//--------------------------------------------------------------------------
FASTCV_API void
fcvImageDiffs16( const int16_t* __restrict src1,
                 const int16_t* __restrict src2,
                       unsigned int             srcWidth,
                       unsigned int             srcHeight,
                       unsigned int             srcStride,
                            int16_t* __restrict dst,
                       unsigned int             dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageDifff32( const float* __restrict src1,
                 const float* __restrict src2,
                unsigned int             srcWidth,
                unsigned int             srcHeight,
                unsigned int             srcStride,
                       float* __restrict dst,
                unsigned int             dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageDiffu8f32( const uint8_t* __restrict src1,
                   const uint8_t* __restrict src2,
                    unsigned int             srcWidth,
                    unsigned int             srcHeight,
                    unsigned int             srcStride,
                           float* __restrict dst,
                    unsigned int             dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageDiffu8s8( const uint8_t* __restrict src1,
                  const uint8_t* __restrict src2,
                   unsigned int             srcWidth,
                   unsigned int             srcHeight,
                   unsigned int             srcStride,
                         int8_t* __restrict dst,
                    unsigned int             dstStride );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientInterleaveds16( const uint8_t* __restrict src,
                                unsigned int              srcWidth,
                                unsigned int              srcHeight,
                                unsigned int              srcStride,
                                int16_t* __restrict       gradients
                              );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientInterleaveds16_v2( const uint8_t* __restrict src,
                                   unsigned int              srcWidth,
                                   unsigned int              srcHeight,
                                   unsigned int              srcStride,
                                   int16_t* __restrict       gradients,
                                   unsigned int              gradStride );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API int
fcvMserInit(const unsigned int width,
            const unsigned int height,
            unsigned int       delta,
            unsigned int       minArea,
            unsigned int       maxArea,
            float              maxVariation,
            float              minDiversity,
            void            ** mserHandle );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvMserRelease(void *mserHandle);

//------------------------------------------------------------------------------
FASTCV_API void
fcvMseru8( void                     *mserHandle,
           const uint8_t* __restrict srcPtr,
           unsigned int              srcWidth,
           unsigned int              srcHeight,
           unsigned int              srcStride,
           unsigned int              maxContours,
           unsigned int * __restrict numContours,
           unsigned int * __restrict numPointsInContour,
           unsigned int              pointsArraySize,
           unsigned int * __restrict pointsArray);

//------------------------------------------------------------------------------
FASTCV_API void
fcvMserExtu8( void                     *mserHandle,
              const uint8_t* __restrict srcPtr,
              unsigned int              srcWidth,
              unsigned int              srcHeight,
              unsigned int              srcStride,
              unsigned int              maxContours,
              unsigned int * __restrict numContours,
              unsigned int * __restrict numPointsInContour,
              unsigned int * __restrict pointsArray,
              unsigned int              pointsArraySize,
              unsigned int * __restrict contourVariation,
              int * __restrict          contourPolarity,
              unsigned int * __restrict contourNodeId,
              unsigned int * __restrict contourNodeCounter);

//------------------------------------------------------------------------------
FASTCV_API int
fcvMseru8_v2( void                     *mserHandle,
              const uint8_t* __restrict srcPtr,
              uint32_t                  srcWidth,
              uint32_t                  srcHeight,
              uint32_t                  srcStride,
              uint32_t                  maxContours,
              uint32_t* __restrict      numContours,
              uint16_t* __restrict      recArray,
              uint32_t* __restrict      numPointsInContour,
              uint32_t                  pointsArraySize,
              uint16_t* __restrict      pointsArray);

//------------------------------------------------------------------------------
FASTCV_API int
fcvMserExtu8_v2( void                     *mserHandle,
                 const uint8_t* __restrict srcPtr,
                 uint32_t                  srcWidth,
                 uint32_t                  srcHeight,
                 uint32_t                  srcStride,
                 uint32_t                  maxContours,
                 uint32_t* __restrict      numContours,
                 uint16_t* __restrict      recArray,
                 uint32_t* __restrict      numPointsInContour,
                 uint32_t                  pointsArraySize,
                 uint16_t* __restrict      pointsArray,
                 uint32_t* __restrict      contourVariation,
                 int8_t* __restrict        contourPolarity,
                 uint32_t* __restrict      contourNodeId,
                 uint32_t* __restrict      contourNodeCounter);

//------------------------------------------------------------------------------
FASTCV_API int
fcvMserExtu8_v3( void                     *mserHandle,
                 const uint8_t* __restrict srcPtr,
                 uint32_t                  srcWidth,
                 uint32_t                  srcHeight,
                 uint32_t                  srcStride,
                 uint32_t                  maxContours,
                 uint32_t* __restrict      numContours,
                 uint16_t* __restrict      recArray,
                 uint32_t* __restrict      staPointsInPath,
                 uint32_t* __restrict      numPointsInContour,
                 uint32_t                  pathArraySize,
                 uint16_t* __restrict      pathArray,
                 uint32_t* __restrict      contourVariation,
                 int8_t* __restrict        contourPolarity,
                 uint32_t* __restrict      contourNodeId,
                 uint32_t* __restrict      contourNodeCounter);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API int
fcvMserNN8Init(const uint32_t width,
               const uint32_t height,
               uint32_t       delta,
               uint32_t       minArea ,
               uint32_t       maxArea ,
               float32_t      maxVariation ,
               float32_t      minDiversity ,
               void         **mserHandle );

//------------------------------------------------------------------------------
FASTCV_API int
fcvMserNN8u8 ( void                      *mserHandle,
               const uint8_t* __restrict  srcPtr,
               uint32_t                   srcWidth,
               uint32_t                   srcHeight,
               uint32_t                   srcStride,
               uint32_t                   maxContours,
               uint32_t* __restrict       numContours,
               uint16_t* __restrict       recArray,
               uint32_t* __restrict       numPointsInContour,
               uint32_t                   pointsArraySize,
               uint16_t* __restrict       pointsArray);

//------------------------------------------------------------------------------
FASTCV_API int
fcvMserExtNN8u8(void                     *mserHandle,
                const uint8_t* __restrict srcPtr,
                uint32_t                  srcWidth,
                uint32_t                  srcHeight,
                uint32_t                  srcStride,
                uint32_t                  maxContours,
                uint32_t* __restrict      numContours,
                uint16_t* __restrict      recArray,
                uint32_t* __restrict      numPointsInContour,
                uint32_t                  pointsArraySize,
                uint16_t* __restrict      pointsArray,
                uint32_t* __restrict      contourVariation,
                int8_t* __restrict        contourPolarity,
                uint32_t* __restrict      contourNodeId,
                uint32_t* __restrict      contourNodeCounter);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientInterleavedf32( const uint8_t* __restrict src,
                                unsigned int              srcWidth,
                                unsigned int              srcHeight,
                                unsigned int              srcStride,
                                float* __restrict         gradients );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientInterleavedf32_v2( const uint8_t* __restrict src,
                                   unsigned int              srcWidth,
                                   unsigned int              srcHeight,
                                   unsigned int              srcStride,
                                   float* __restrict         gradients,
                                   unsigned int              gradStride );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientPlanars16( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           int16_t* __restrict       dx,
                           int16_t* __restrict       dy );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientPlanars16_v2( const uint8_t* __restrict src,
                              unsigned int              srcWidth,
                              unsigned int              srcHeight,
                              unsigned int              srcStride,
                              int16_t* __restrict       dx,
                              int16_t* __restrict       dy,
                              unsigned int              dxyStride );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientPlanarf32( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           float* __restrict         dx,
                           float* __restrict         dy );



//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientPlanarf32_v2( const uint8_t* __restrict src,
                              unsigned int              srcWidth,
                              unsigned int              srcHeight,
                              unsigned int              srcStride,
                              float* __restrict         dx,
                              float* __restrict         dy,
                              unsigned int              dxyStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast9u8( const uint8_t* __restrict src,
                  unsigned int              srcWidth,
                  unsigned int              srcHeight,
                  unsigned int              srcStride,
                  int                       barrier,
                  unsigned int              border,
                  uint32_t* __restrict      xy,
                  unsigned int              nCornersMax,
                  uint32_t* __restrict      nCorners );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast9InMasku8( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        int                       barrier,
                        unsigned int              border,
                        uint32_t* __restrict      xy,
                        unsigned int              nCornersMax,
                        uint32_t* __restrict      nCorners,
                        const uint8_t* __restrict mask,
                        unsigned int              maskWidth,
                        unsigned int              maskHeight );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast10u8( const uint8_t* __restrict src,
                   uint32_t                  srcWidth,
                   uint32_t                  srcHeight,
                   uint32_t                  srcStride,
                   int32_t                   barrier,
                   uint32_t                  border,
                   uint32_t* __restrict      xy,
                   uint32_t                  nCornersMax,
                   uint32_t* __restrict      nCorners);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast10InMasku8( const uint8_t* __restrict src,
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         int32_t                   barrier,
                         uint32_t                  border,
                         uint32_t* __restrict      xy,
                         uint32_t                  nCornersMax,
                         uint32_t* __restrict      nCorners,
                         const uint8_t* __restrict mask,
                         uint32_t                  maskWidth,
                         uint32_t                  maskHeight );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerHarrisu8( const uint8_t* __restrict src,
                   unsigned int              srcWidth,
                   unsigned int              srcHeight,
                   unsigned int              srcStride,
                   unsigned int              border,
                   uint32_t* __restrict      xy,
                   unsigned int              nCornersMax,
                   uint32_t* __restrict      nCorners,
                   int                       threshold );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API unsigned int
fcvLocalHarrisMaxu8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     unsigned int              srcStride,
                     unsigned int              posX,
                     unsigned int              posY,
                     unsigned int             *maxX,
                     unsigned int             *maxY,
                     int                      *maxScore);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerHarrisInMasku8( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         unsigned int              border,
                         uint32_t* __restrict      xy,
                         unsigned int              nCornersMax,
                         uint32_t* __restrict      nCorners,
                         int                       threshold,
                         const uint8_t* __restrict mask,
                         unsigned int              maskWidth,
                         unsigned int              maskHeight );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerHarrisAdaptiveu8( const uint8_t* __restrict src,
                           uint32_t                  srcWidth,
                           uint32_t                  srcHeight,
                           uint32_t                  srcStride,
                           uint32_t                  border,
                           float32_t* __restrict     xy,
                           uint32_t                  nCornersMax,
                           uint32_t* __restrict      nCorners,
                           int32_t                   threshold);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvCornerHarrisScoreu8(const uint8_t* __restrict src,
                       uint32_t srcWidth,
                       uint32_t srcHeight,
                       uint32_t srcStride,
                       float32_t*  __restrict harrisResp,
                       uint32_t respStride,
                       uint32_t* __restrict xy,
                       uint32_t nCornersMax,
                       uint32_t*  __restrict nCorners,
                       float32_t threshold,
                       float32_t sensitivity,
                       uint32_t kernelSize,
                       uint32_t blockSize,
                       uint32_t nmsEnabled,
                       float32_t minDistance,
                       uint32_t normalizeResponse);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvGeomAffineFitf32( const fcvCorrespondences* __restrict corrs,
                     float* __restrict                    affine );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomAffineEvaluatef32( const fcvCorrespondences* __restrict corrs,
                          float* __restrict                    affine,
                          float                                maxsqerr,
                          uint16_t* __restrict                 inliers,
                          int32_t*                             numinliers );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvGeomHomographyFitf32( const fcvCorrespondences* __restrict corrs,
                         float* __restrict                    homography );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomHomographyEvaluatef32( const fcvCorrespondences* __restrict corrs,
                              float* __restrict                    homography,
                              float                                maxsqerr,
                              uint16_t* __restrict                 inliers,
                              int32_t*                             numinliers );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomHomographyEvaluatef32_v2( const fcvCorrespondences* __restrict corrs,
                                 float32_t* __restrict                homography,
                                 float32_t                            maxsqerr,
                                 uint16_t* __restrict                 inliers,
                                 float32_t*__restrict                 errinliers,
                                 int32_t*                             numinliers );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API float
fcvGeomPoseRefineGNf32( const fcvCorrespondences* __restrict corrs,
                        short                                minIterations,
                        short                                maxIterations,
                        float                                stopCriteria,
                        float*                               initpose,
                        float*                               refinedpose );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomPoseUpdatef32(
   const float* __restrict projected,
   const float* __restrict reprojErr,
   const float* __restrict invz,
   const float* __restrict reprojVariance,
   unsigned int                numpts,
   float*       __restrict pose );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomPoseOptimizeGNf32( const float* __restrict projected,
                          const float* __restrict reprojErr,
                          const float* __restrict invz,
                          const float* __restrict reprojVariance,
                          unsigned int            numpts,
                          float*       __restrict pose );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API float
fcvGeomPoseEvaluateErrorf32( const fcvCorrespondences* __restrict corrs,
                             const float*              __restrict pose,
                             float*                    __restrict projected,
                             float*                    __restrict reprojErr,
                             float*                    __restrict invz,
                             float*                    __restrict reprojVariance );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomPoseEvaluatef32( const fcvCorrespondences* __restrict corrs,
                        const float*                         pose,
                        float                                maxSquErr,
                        uint16_t* __restrict                 inliers,
                        uint32_t*                            numInliers );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvGeom3PointPoseEstimatef32( const fcvCorrespondences* __restrict corrs,
                                                 float*            pose,
                                               int32_t*            numPoses );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorr3x3s8( const int8_t* __restrict  kernel,
                    const uint8_t* __restrict src,
                    unsigned int              srcWidth,
                    unsigned int              srcHeight,
                    uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorr3x3s8_v2( const int8_t* __restrict  kernel,
                       const uint8_t* __restrict src,
                       unsigned int              srcWidth,
                       unsigned int              srcHeight,
                       unsigned int              srcStride,
                       uint8_t* __restrict       dst,
                       unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterCorrNxNu8f32( const float32_t* __restrict kernel,
                       uint32_t                    N,
                       const uint8_t* __restrict   src,
                       uint32_t                    srcWidth,
                       uint32_t                    srcHeight,
                       uint32_t                    srcStride,
                       float32_t* __restrict       dst,
                       uint32_t                    dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterCorrNxNu8s16( const int8_t* __restrict   kernel,
                       uint32_t                   N,
                       int8_t                     shift,
                       const uint8_t* __restrict  src,
                       uint32_t                   srcWidth,
                       uint32_t                   srcHeight,
                       uint32_t                   srcStride,
                       int16_t* __restrict        dst,
                       uint32_t                   dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterCorrNxNu8( const int8_t* __restrict kernel,
                      uint32_t                    N,
                      int8_t                      shift,
                      const uint8_t* __restrict   src,
                      uint32_t                    srcWidth,
                      uint32_t                    srcHeight,
                      uint32_t                    srcStride,
                      uint8_t* __restrict         dst,
                      uint32_t                    dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSep9x9s16( const int16_t* __restrict kernel,
                        const int16_t*            src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        int16_t* __restrict       tmp,
                        int16_t*                  dst );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvFilterCorrSep9x9s16_v2( const int16_t* __restrict kernel,
                           const int16_t*            srcImg,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           int16_t* __restrict       tmpImg,
                           int16_t*                  dstImg,
                           unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSep11x11s16( const int16_t* __restrict kernel,
                          const int16_t*            src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int16_t* __restrict       tmpImg,
                          int16_t*                  dst );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvFilterCorrSep11x11s16_v2( const int16_t* __restrict kernel,
                             const int16_t*            srcImg,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             int16_t* __restrict       tmpImg,
                             int16_t*                  dstImg,
                             unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSep13x13s16( const int16_t* __restrict kernel,
                          const int16_t*            src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int16_t* __restrict       tmpImg,
                          int16_t*                  dst );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvFilterCorrSep13x13s16_v2( const int16_t* __restrict kernel,
                             const int16_t*            srcImg,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             int16_t* __restrict       tmpImg,
                             int16_t*                  dstImg,
                             unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSep15x15s16( const int16_t* __restrict kernel,
                          const int16_t*            src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int16_t* __restrict       tmpImg,
                          int16_t*                  dst );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvFilterCorrSep15x15s16_v2( const int16_t* __restrict kernel,
                             const int16_t*            srcImg,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             int16_t* __restrict       tmpImg,
                             int16_t*                  dstImg,
                             unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSep17x17s16( const int16_t* __restrict kernel,
                          const int16_t*            src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int16_t* __restrict       tmpImg,
                          int16_t*                  dst );



//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvFilterCorrSep17x17s16_v2( const int16_t* __restrict kernel,
                             const int16_t*            srcImg,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             int16_t* __restrict       tmpImg,
                             int16_t*                  dstImg,
                             unsigned int              dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSepNxNs16( const int16_t* __restrict kernel,
                        int                       knlSize,
                        const int16_t*            srcImg,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        int16_t* __restrict       tmpImg,
                        int16_t*                  dstImg,
                        unsigned int              dstStride );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterCorrSepMxNu8( const int8_t*               kernelX,
                       uint32_t                    knlSizeX,
                       const int8_t*               kernelY,
                       uint32_t                    knlSizeY,
                       int8_t                      shift,
                       const uint8_t*              srcImg,
                       uint32_t                    srcWidth,
                       uint32_t                    srcHeight,
                       uint32_t                    srcStride,
                       uint8_t*                    dstImg,
                       uint32_t                    dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageIntensityStats( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        int                       xBegin,
                        int                       yBegin,
                        unsigned int              recWidth,
                        unsigned int              recHeight,
                        float*                    mean,
                        float*                    variance );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvImageIntensityStats_v2( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           int                       xBegin,
                           int                       yBegin,
                           uint32_t                  recWidth,
                           uint32_t                  recHeight,
                           float32_t*                mean,
                           float32_t*                variance,
                           fcvVarianceEstimator      varianceEstimator);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageIntensityHistogram( const uint8_t* __restrict src,
                            unsigned int              srcWidth,
                            int                       xBegin,
                            int                       yBegin,
                            unsigned int              recWidth,
                            unsigned int              recHeight,
                            int32_t*                  histogram  );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegrateImageu8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     uint32_t* __restrict      dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegrateImageu8_v2( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        uint32_t* __restrict      dst,
                        unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatchu8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     int                       patchX,
                     int                       patchY,
                     unsigned int              patchW,
                     unsigned int              patchH,
                     uint32_t* __restrict      intgrlImgOut,
                     uint32_t* __restrict      intgrlSqrdImgOut );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatchu8_v2( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        int                       patchX,
                        int                       patchY,
                        unsigned int              patchW,
                        unsigned int              patchH,
                        uint32_t* __restrict      intgrlImgOut,
                        uint32_t* __restrict      intgrlSqrdImgOut );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvIntegratePatchu8_v3(const uint8_t* __restrict src,
                       uint32_t    srcWidth,
                       uint32_t    srcHeight,
                       uint32_t    srcStride,
                       uint32_t    patchX,
                       uint32_t    patchY,
                       uint32_t    patchW,
                       uint32_t    patchH,
                       uint32_t* __restrict      intgrlImgOut,
                       uint32_t    intgrlStride,
                       uint32_t* __restrict      intgrlSqrdImgOut,
                       uint32_t    intgrlSqrdStride);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatch12x12u8( const uint8_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int                       patchX,
                          int                       patchY,
                          uint32_t* __restrict      intgrlImgOut,
                          uint32_t* __restrict      intgrlSqrdImgOut );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatch12x12u8_v2( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             int                       patchX,
                             int                       patchY,
                             uint32_t* __restrict      intgrlImgOut,
                             uint32_t* __restrict      intgrlSqrdImgOut );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatch18x18u8( const uint8_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int                       patchX,
                          int                       patchY,
                          uint32_t* __restrict      intgrlImgOut,
                          uint32_t* __restrict      intgrlSqrdImgOut );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatch18x18u8_v2( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             int                       patchX,
                             int                       patchY,
                             uint32_t* __restrict      intgrlImgOut,
                             uint32_t* __restrict      intgrlSqrdImgOut );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvIntegrateImageLineu8( const uint8_t* __restrict src,
                         uint16_t                  srcWidth,
                         uint32_t*                 intgrl,
                         uint32_t*                 intgrlSqrd );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegrateImageLine64u8( const uint8_t* __restrict src,
                           uint16_t*                 intgrl,
                           uint32_t*                 intgrlSqrd );


//------------------------------------------------------------------------------
// -----------------------------------------------------------------------------

FASTCV_API int
fcvDescriptorSampledMeanAndVar36f32( const float* __restrict src,
                                     int                     first,
                                     int                     last,
                                     int32_t*                vind,
                                     float* __restrict       means,
                                     float* __restrict       vars,
                                     float* __restrict       temp );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvNCCPatchOnCircle8x8u8( const uint8_t* __restrict patch,
                          const uint8_t* __restrict src,
                          unsigned short            srcWidth,
                          unsigned short            srcHeight,
                          unsigned short            search_center_x,
                          unsigned short            search_center_y,
                          unsigned short            search_radius,
                          uint16_t*                 best_x,
                          uint16_t*                 best_y,
                          uint32_t*                 bestNCC,
                          int                       findSubPixel,
                          float*                    subX,
                          float*                    subY );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvNCCPatchOnCircle8x8u8_v2( const uint8_t* __restrict patch,
                             const uint8_t* __restrict src,
                             unsigned short            srcWidth,
                             unsigned short            srcHeight,
                             unsigned short            search_center_x,
                             unsigned short            search_center_y,
                             unsigned short            search_radius,
                             int                       filterLowVariance,
                             uint16_t*                 best_x,
                             uint16_t*                 best_y,
                             uint32_t*                 bestNCC,
                             int                       findSubPixel,
                             float*                    subX,
                             float*                    subY );




//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvNCCPatchOnSquare8x8u8( const uint8_t* __restrict patch,
                          const uint8_t* __restrict src,
                          unsigned short            srcWidth,
                          unsigned short            srcHeight,
                          unsigned short            search_center_x,
                          unsigned short            search_center_y,
                          unsigned short            search_w,
                          uint16_t*                 best_x,
                          uint16_t*                 best_y,
                          uint32_t*                 bestNCC,
                          int                       doSubPixel,
                          float*                    subX,
                          float*                    subY );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvNCCPatchOnSquare8x8u8_v2( const uint8_t* __restrict patch,
                             const uint8_t* __restrict src,
                             unsigned short            srcWidth,
                             unsigned short            srcHeight,
                             unsigned short            search_center_x,
                             unsigned short            search_center_y,
                             unsigned short            search_w,
                             int                       filterLowVariance,
                             uint16_t*                 best_x,
                             uint16_t*                 best_y,
                             uint32_t*                 bestNCC,
                             int                       doSubPixel,
                             float*                    subX,
                             float*                    subY );



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfAbsoluteDiffs8x8u8( const uint8_t* __restrict patch,
                            const uint8_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            unsigned int              srcStride,
                            uint16_t* __restrict      dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfAbsoluteDiffs8x8u8_v2( const uint8_t* __restrict patch,
                               unsigned int              patchStride,
                               const uint8_t* __restrict src,
                               unsigned int              srcWidth,
                               unsigned int              srcHeight,
                               unsigned int              srcStride,
                               uint16_t* __restrict      dst,
                               unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDownBy2u8( const uint8_t* __restrict src,
                   unsigned int              srcWidth,
                   unsigned int              srcHeight,
                   uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDownBy2u8_v2( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      unsigned int              srcStride,
                      uint8_t* __restrict       dst,
                      unsigned int              dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownBy2Gaussian5x5u8( const uint8_t* __restrict src,
                              unsigned int              srcWidth,
                              unsigned int              srcHeight,
                              uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownBy2Gaussian5x5u8_v2( const uint8_t* __restrict src,
                                 unsigned int              srcWidth,
                                 unsigned int              srcHeight,
                                 unsigned int              srcStride,
                                 uint8_t* __restrict       dst,
                                 unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDownBy4u8( const uint8_t* __restrict src,
                    unsigned int             srcWidth,
                    unsigned int             srcHeight,
                    uint8_t* __restrict      dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDownBy4u8_v2( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      unsigned int              srcStride,
                      uint8_t* __restrict       dst,
                      unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDown3To2u8( const uint8_t* __restrict src,
                    unsigned                  srcWidth,
                    unsigned                  srcHeight,
                    unsigned int              srcStride,
                    uint8_t* __restrict       dst,
                    unsigned int              dstStride);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDownNNu8( const uint8_t* __restrict src,
                  unsigned int              srcWidth,
                  unsigned int              srcHeight,
                  unsigned int              srcStride,
                  uint8_t* __restrict       dst,
                  unsigned int              dstWidth,
                  unsigned int              dstHeight,
                  unsigned int              dstStride );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownu8( const uint8_t* __restrict src,
                unsigned int              srcWidth,
                unsigned int              srcHeight,
                uint8_t* __restrict       dst,
                unsigned int              dstWidth,
                unsigned int              dstHeight );


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownu8_v2( const uint8_t* __restrict src,
                   unsigned int              srcWidth,
                   unsigned int              srcHeight,
                   unsigned int              srcStride,
                   uint8_t* __restrict       dst,
                   unsigned int              dstWidth,
                   unsigned int              dstHeight,
                   unsigned int              dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleUpBy2Gaussian5x5u8( const uint8_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleUpBy2Gaussian5x5u8_v2( const uint8_t* __restrict src,
                               unsigned int              srcWidth,
                               unsigned int              srcHeight,
                               unsigned int              srcStride,
                               uint8_t* __restrict       dst,
                               unsigned int              dstStride );


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

FASTCV_API int
fcvVecNormalize36s8f32( const int8_t* __restrict src,
                        unsigned int             srcStride,
                        const float*  __restrict invLen,
                        unsigned int             numVecs,
                        float                    reqNorm,
                        float*        __restrict dst,
                        int32_t*                 stopBuild );


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfSquaredDiffs36x4s8( const int8_t* __restrict a,
                            float                    invLenA,
                            const int8_t* __restrict b0,
                            const int8_t* __restrict b1,
                            const int8_t* __restrict b2,
                            const int8_t* __restrict b3,
                            const float* __restrict  invLenB,
                            float* __restrict        distances );


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfSquaredDiffs36xNs8( const int8_t* __restrict         a,
                            float                            invLenA,
                            const int8_t* const * __restrict b,
                            const float* __restrict          invLenB,
                            unsigned int                     numB,
                            float* __restrict                distances );


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvSort8Scoresf32( float* __restrict inScores, float* __restrict outScores );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvFilterThresholdu8( const uint8_t*            src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      uint8_t*                  dst,
                      unsigned int              threshold );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterThresholdu8_v2( const uint8_t*            src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         uint8_t*                  dst,
                         unsigned int              dstStride,
                         unsigned int              threshold );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterThresholdu8_v3( const uint8_t*            src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        uint8_t*                  dst,
                        unsigned int              dstStride,
                        unsigned int              threshold,
                        uint8_t                   trueValue,
                        uint8_t                   falseValue);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterThresholdRangeu8( const uint8_t* src,
                           uint32_t srcWidth,
                           uint32_t srcHeight,
                           uint32_t srcStride,
                           uint8_t* dst,
                           uint32_t dstStride,
                           uint8_t lowThresh,
                           uint8_t highThresh );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterThresholdRangeu8_v2( const uint8_t*    src,
                                uint32_t        srcWidth,
                                uint32_t        srcHeight,
                                uint32_t        srcStride,
                                uint8_t*        dst,
                                uint32_t        dstStride,
                                uint8_t         lowThresh,
                                uint8_t         highThresh,
                                uint8_t         trueValue,
                                uint8_t         falseValue);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterDilate3x3u8( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterDilate3x3u8_v2( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         uint8_t* __restrict       dst,
                         unsigned int              dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterErode3x3u8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     uint8_t* __restrict       dst );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterErode3x3u8_v2( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        uint8_t* __restrict       dst,
                        unsigned int              dstStride );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvTransformAffine8x8u8( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         const int32_t* __restrict nPos,
                         const int32_t* __restrict nAffine,
                         uint8_t* __restrict       nPatch );


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API int
fcvTransformAffine8x8u8_v2( const uint8_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            unsigned int              srcStride,
                            const int32_t* __restrict nPos,
                            const int32_t* __restrict nAffine,
                            uint8_t* __restrict       patch,
                            unsigned int              patchStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvWarpPerspectiveu8( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      uint8_t* __restrict       dst,
                      unsigned int              dstWidth,
                      unsigned int              dstHeight,
                      float* __restrict         projectionMatrix );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvWarpPerspectiveu8_v2( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         uint8_t* __restrict       dst,
                         unsigned int              dstWidth,
                         unsigned int              dstHeight,
                         unsigned int              dstStride,
                         float* __restrict         projectionMatrix );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcv3ChannelWarpPerspectiveu8( const uint8_t* __restrict src,
                              unsigned int              srcWidth,
                              unsigned int              srcHeight,
                              uint8_t* __restrict       dst,
                              unsigned int              dstWidth,
                              unsigned int              dstHeight,
                              float* __restrict         projectionMatrix );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcv3ChannelWarpPerspectiveu8_v2( const uint8_t* __restrict src,
                                 unsigned int              srcWidth,
                                 unsigned int              srcHeight,
                                 unsigned int              srcStride,
                                 uint8_t* __restrict       dst,
                                 unsigned int              dstWidth,
                                 unsigned int              dstHeight,
                                 unsigned int              dstStride,
                                 float* __restrict         projectionMatrix );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int
fcvClusterEuclideanf32( const float* __restrict  points,
                        int                      numPoints,  // actually not used but helpful
                        int                      dim,
                        int                      pointStride,
                        const size_t* __restrict indices,
                        int                      numIndices,
                        int                      numClusters,
                        float* __restrict        clusterCenters,
                        int                      clusterCenterStride,
                        float* __restrict        newClusterCenters,
                        size_t* __restrict       clusterMemberCounts,
                        size_t* __restrict       clusterBindings,
                        float*                   sumOfClusterDistances );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int
fcvClusterEuclideanNormedf32( const float* __restrict  points,
                              int                      numPoints,
                              int                      dim,
                              int                      pointStride,
                              const size_t* __restrict indices,
                              int                      numIndices,
                              int                      numClusters,
                              float* __restrict        clusterCenters,
                              int                      clusterCenterStride,
                              float* __restrict        newClusterCenters,
                              size_t* __restrict       clusterMemberCounts,
                              size_t* __restrict       clusterBindings,
                              float*                   sumOfClusterDistances );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int
fcvClusterEuclideanNormed36f32( const float* __restrict  points,
                                int                      numPoints,
                                int                      pointStride,
                                const size_t* __restrict indices,
                                int                      numIndices,
                                int                      numClusters,
                                float* __restrict        clusterCenters,
                                int                      clusterCenterStride,
                                float* __restrict        newClusterCenters,
                                size_t* __restrict       clusterMemberCounts,
                                size_t* __restrict       clusterBindings,
                                float*                   sumOfClusterDistances );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5s16( const int16_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         int16_t* __restrict       dst,
                         int                       blurBorder );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5s16_v2( const int16_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            unsigned int              srcStride,
                            int16_t* __restrict       dst,
                            unsigned int              dstStride,
                            int                       blurBorder );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5s32( const int32_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         int32_t* __restrict       dst,
                         int                       blurBorder );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5s32_v2( const int32_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            unsigned int              srcStride,
                            int32_t* __restrict       dst,
                            unsigned int              dstStride,
                            int                       blurBorder );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API uint32_t
fcvImageSegmentationRegionGrow( const uint8_t* __restrict src,
                                uint32_t srcWidth,
                                uint32_t srcHeight,
                                uint32_t srcStride,
                                uint32_t numChannel,
                                uint32_t thresholdSplit,
                                uint32_t thresholdMerge,
                                uint32_t* __restrict segLabel,
                                uint32_t segLabelStride,
                                uint8_t* __restrict data );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int
fcvTransformAffineu8( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      const float* __restrict   position,
                      const float* __restrict   affine,
                      uint8_t* __restrict       patch,
                      unsigned int              patchWidth,
                      unsigned int              patchHeight );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int
fcvTransformAffineu8_v2( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         const float* __restrict   position,
                         const float* __restrict   affine,
                         uint8_t* __restrict       patch,
                         unsigned int              patchWidth,
                         unsigned int              patchHeight,
                         unsigned int              patchStride );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvCopyRotated17x17u8( const uint8_t* __restrict src,
                       uint8_t* __restrict       dst,
                       int                       orientation );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvBitCountu8( const uint8_t* __restrict src,
               unsigned int              srcLength );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvBitCount32x1u8( const uint8_t* __restrict src );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvBitCount32x4u8( const uint8_t* __restrict a,
                   const uint8_t* __restrict b,
                   const uint8_t* __restrict c,
                   const uint8_t* __restrict d,
                   uint32_t* __restrict      bitCount );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvBitCount64x1u8( const uint8_t* __restrict src );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvBitCount64x4u8( const uint8_t* __restrict a,
                   const uint8_t* __restrict b,
                   const uint8_t* __restrict c,
                   const uint8_t* __restrict d,
                   uint32_t* __restrict      bitCount );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvBitCountu32( const uint32_t* __restrict src,
                unsigned int               srcLength );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvHammingDistanceu8( const uint8_t* __restrict a,
                      const uint8_t* __restrict b,
                      unsigned int              abLength );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvHammingDistance32x1u8a4( const uint8_t* __restrict a,
                            const uint8_t* __restrict b );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvHammingDistance64x1u8a4( const uint8_t* __restrict a,
                            const uint8_t* __restrict b );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvHammingDistance32x1u8( const uint8_t* __restrict a,
                          const uint8_t* __restrict b );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvHammingDistance64x1u8( const uint8_t* __restrict a,
                          const uint8_t* __restrict b );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvHammingDistance32x4u8a4( const uint8_t* __restrict a,
                            const uint8_t* __restrict b,
                            const uint8_t* __restrict c,
                            const uint8_t* __restrict d,
                            const uint8_t* __restrict e,
                            uint32_t* __restrict      hammingDistances );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvHammingDistance64x4u8a4( const uint8_t* __restrict a,
                            const uint8_t* __restrict b,
                            const uint8_t* __restrict c,
                            const uint8_t* __restrict d,
                            const uint8_t* __restrict e,
                            uint32_t* __restrict      hammingDistances );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvHammingDistance64x4u8( const uint8_t* __restrict a,
                          const uint8_t* __restrict b,
                          const uint8_t* __restrict c,
                          const uint8_t* __restrict d,
                          const uint8_t* __restrict e,
                          uint32_t* __restrict      hammingDistances );


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast9Scoreu8( const uint8_t* __restrict src,
                       unsigned int              srcWidth,
                       unsigned int              srcHeight,
                       unsigned int              srcStride,
                       int                       barrier,
                       unsigned int              border,
                       uint32_t* __restrict      xy,
                       uint32_t* __restrict      scores,
                       unsigned int              nCornersMax,
                       uint32_t* __restrict      nCorners );


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast9InMaskScoreu8( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             int                       barrier,
                             unsigned int              border,
                             uint32_t* __restrict      xy,
                             uint32_t* __restrict      scores,
                             unsigned int              nCornersMax,
                             uint32_t* __restrict      nCorners,
                             const uint8_t* __restrict mask,
                             unsigned int              maskWidth,
                             unsigned int              maskHeight );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast9Scoreu8_v2( const uint8_t* __restrict src,
                           unsigned int             srcWidth,
                           unsigned int             srcHeight,
                           unsigned int             srcStride,
                                    int             barrier,
                           unsigned int             border,
                               uint32_t* __restrict xy,
                               uint32_t* __restrict scores,
                           unsigned int             nCornersMax,
                               uint32_t* __restrict nCorners,
                               uint32_t             nmsEnabled,
                                   void* __restrict tempBuf);


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast9InMaskScoreu8_v2( const uint8_t* __restrict src,
                                 unsigned int             srcWidth,
                                 unsigned int             srcHeight,
                                 unsigned int             srcStride,
                                          int             barrier,
                                 unsigned int             border,
                                     uint32_t* __restrict xy,
                                     uint32_t* __restrict scores,
                                 unsigned int             nCornersMax,
                                     uint32_t* __restrict nCorners,
                                const uint8_t* __restrict mask,
                                 unsigned int             maskWidth,
                                 unsigned int             maskHeight,
                                     uint32_t             nmsEnabled,
                                         void* __restrict tempBuf);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast10Scoreu8( const uint8_t* __restrict src,
                             uint32_t             srcWidth,
                             uint32_t             srcHeight,
                             uint32_t             srcStride,
                              int32_t             barrier,
                             uint32_t             border,
                             uint32_t* __restrict xy,
                             uint32_t* __restrict scores,
                             uint32_t             nCornersMax,
                             uint32_t* __restrict nCorners,
                             uint32_t             nmsEnabled,
                                 void* __restrict tempBuf);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast10InMaskScoreu8( const uint8_t* __restrict src,
                                   uint32_t             srcWidth,
                                   uint32_t             srcHeight,
                                   uint32_t             srcStride,
                                    int32_t             barrier,
                                   uint32_t             border,
                                   uint32_t* __restrict xy,
                                   uint32_t* __restrict scores,
                                   uint32_t             nCornersMax,
                                   uint32_t* __restrict nCorners,
                              const uint8_t* __restrict mask,
                                   uint32_t             maskWidth,
                                   uint32_t             maskHeight,
                                   uint32_t             nmsEnabled,
                                       void* __restrict tempBuf);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

FASTCV_API void
fcvTrackLKOpticalFlowu8( const uint8_t* __restrict src1,
                         const uint8_t* __restrict src2,
                         int                       srcWidth,
                         int                       srcHeight,
                         const fcvPyramidLevel*    src1Pyr,
                         const fcvPyramidLevel*    src2Pyr,
                         const fcvPyramidLevel*    dx1Pyr,
                         const fcvPyramidLevel*    dy1Pyr,
                         const float*              featureXY,
                         float*                    featureXY_out,
                         int32_t*                  featureStatus,
                         int                       featureLen,
                         int                       windowWidth,
                         int                       windowHeight,
                         int                       maxIterations,
                         int                       nPyramidLevels,
                         float                     maxResidue,
                         float                     minDisplacement,
                         float                     minEigenvalue,
                         int                       lightingNormalized );


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

FASTCV_API void
fcvTrackLKOpticalFlowu8_v2(  const uint8_t* __restrict   src1,
                             const uint8_t* __restrict   src2,
                             uint32_t                    width,
                             uint32_t                    height,
                             uint32_t                    stride,
                             const fcvPyramidLevel_v2    *src1Pyr,
                             const fcvPyramidLevel_v2    *src2Pyr,
                             const float32_t*            featureXY,
                             float32_t*                  featureXY_out,
                             int32_t*                    featureStatus,
                             int32_t                     featureLen,
                             int32_t                     windowWidth,
                             int32_t                     windowHeight,
                             int32_t                     maxIterations,
                             int32_t                     nPyramidLevels);


// -----------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvTrackLKOpticalFlowf32( const uint8_t* __restrict src1,
                          const uint8_t* __restrict src2,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          const fcvPyramidLevel*    src1Pyr,
                          const fcvPyramidLevel*    src2Pyr,
                          const fcvPyramidLevel*    dx1Pyr,
                          const fcvPyramidLevel*    dy1Pyr,
                          const float*              featureXY,
                          float*                    featureXY_out,
                          int32_t*                  featureStatus,
                          int                       featureLen,
                          int                       windowWidth,
                          int                       windowHeight,
                          int                       maxIterations,
                          int                       nPyramidLevels,
                          float                     maxResidue,
                          float                     minDisplacement,
                          float                     minEigenvalue,
                          int                       lightingNormalized );


// -----------------------------------------------------------------------------
//-------------------------------------------------------------------------------

FASTCV_API int
fcvPyramidCreatef32( const float* __restrict src,
                     unsigned int            srcWidth,
                     unsigned int            srcHeight,
                     unsigned int            numLevels,
                     fcvPyramidLevel*        pyramid );

// -----------------------------------------------------------------------------
//-------------------------------------------------------------------------------

FASTCV_API int
fcvPyramidCreatef32_v2( const float32_t* __restrict src,
                        uint32_t                    srcWidth,
                        uint32_t                    srcHeight,
                        uint32_t                    srcStride,
                        uint32_t                    numLevels,
                        fcvPyramidLevel_v2*         pyramid );

// -----------------------------------------------------------------------------
//-------------------------------------------------------------------------------

FASTCV_API int
fcvPyramidCreateu8( const uint8_t* __restrict src,
                    unsigned int              srcWidth,
                    unsigned int              srcHeight,
                    unsigned int              numLevels,
                    fcvPyramidLevel*          pyramid );

// -----------------------------------------------------------------------------
//-------------------------------------------------------------------------------

FASTCV_API int
fcvPyramidCreateu8_v2( const uint8_t * __restrict src,
                       uint32_t                   srcWidth,
                       uint32_t                   srcHeight,
                       uint32_t                   srcStride,
                       uint32_t                   numLevels,
                       fcvPyramidLevel_v2*        pyramid );


// -----------------------------------------------------------------------------
//-------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvPyramidCreateu8_v3(const uint8_t * __restrict src,
                      uint32_t           srcWidth,
                      uint32_t           srcHeight,
                      uint32_t           srcStride,
                      uint32_t           numLevels,
                      fcvPyramidScale  scale,
                      fcvPyramidLevel_v2*  __restrict      pyramidGaussian);

// -----------------------------------------------------------------------------
//-------------------------------------------------------------------------------

FASTCV_API int
fcvPyramidSobelGradientCreatei16( const fcvPyramidLevel* imgPyr,
                                  fcvPyramidLevel*       dxPyr,
                                  fcvPyramidLevel*       dyPyr,
                                  unsigned int           numLevels );


// -----------------------------------------------------------------------------
//-------------------------------------------------------------------------------

FASTCV_API int
fcvPyramidSobelGradientCreatef32( const fcvPyramidLevel* imgPyr,
                                  fcvPyramidLevel*       dxPyr,
                                  fcvPyramidLevel*       dyPyr,
                                  unsigned int           numLevels  );


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

FASTCV_API int
fcvPyramidSobelGradientCreatei8( const fcvPyramidLevel* imgPyr,
                                 fcvPyramidLevel*       dxPyr,
                                 fcvPyramidLevel*       dyPyr,
                                 unsigned int           numLevels );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelPlanars16( const uint8_t* __restrict  src,
                                unsigned int               srcWidth,
                                unsigned int               srcHeight,
                                unsigned int               srcStride,
                                int16_t* __restrict        dx,
                                int16_t* __restrict        dy);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelPlanars16_v2( const uint8_t* __restrict  src,
                                   unsigned int               srcWidth,
                                   unsigned int               srcHeight,
                                   unsigned int               srcStride,
                                   int16_t* __restrict        dx,
                                   int16_t* __restrict        dy,
                                   unsigned int               dxyStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageGradientSobelPlanars16_v3( const uint8_t* __restrict  src,
                                    unsigned int              srcWidth,
                                    unsigned int              srcHeight,
                                    unsigned int              srcStride,
                                         int16_t* __restrict  dx,
                                         int16_t* __restrict  dy,
                                    unsigned int              dxyStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelInterleaveds16( const uint8_t* __restrict  src,
                                     unsigned int               srcWidth,
                                     unsigned int               srcHeight,
                                     unsigned int               srcStride,
                                     int16_t* __restrict        gradients );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelInterleaveds16_v2( const uint8_t* __restrict  src,
                                        unsigned int               srcWidth,
                                        unsigned int               srcHeight,
                                        unsigned int               srcStride,
                                        int16_t* __restrict        gradients,
                                        unsigned int               gradStride );

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FASTCV_API void
fcvImageGradientSobelInterleaveds16_v3( const uint8_t* __restrict  src,
                                        unsigned int               srcWidth,
                                        unsigned int               srcHeight,
                                        unsigned int               srcStride,
                                        int16_t*       __restrict  gradients,
                                        unsigned int               gradStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelInterleavedf32( const uint8_t* __restrict src,
                                     unsigned int              srcWidth,
                                     unsigned int              srcHeight,
                                     unsigned int              srcStride,
                                     float* __restrict         gradients);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelInterleavedf32_v2( const uint8_t* __restrict src,
                                        unsigned int              srcWidth,
                                        unsigned int              srcHeight,
                                        unsigned int              srcStride,
                                        float* __restrict         gradients,
                                        unsigned int              gradStride);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelPlanars8( const uint8_t* __restrict src,
                               unsigned int              srcWidth,
                               unsigned int              srcHeight,
                               unsigned int              srcStride,
                               int8_t* __restrict        dx,
                               int8_t* __restrict        dy);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelPlanars8_v2( const uint8_t* __restrict src,
                                  unsigned int              srcWidth,
                                  unsigned int              srcHeight,
                                  unsigned int              srcStride,
                                  int8_t* __restrict        dx,
                                  int8_t* __restrict        dy,
                                  unsigned int              dxyStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageGradientSobelPlanarf32( const uint8_t* __restrict  src,
                                unsigned int               srcWidth,
                                unsigned int               srcHeight,
                                unsigned int               srcStride,
                                float*                     dx,
                                float*                     dy);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageGradientSobelPlanarf32_v2( const uint8_t* __restrict  src,
                                   unsigned int               srcWidth,
                                   unsigned int               srcHeight,
                                   unsigned int               srcStride,
                                   float*                     dx,
                                   float*                     dy,
                                   unsigned int               dxyStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageGradientSobelPlanarf32f32( const float * __restrict  src,
                                   unsigned int              srcWidth,
                                   unsigned int              srcHeight,
                                   unsigned int              srcStride,
                                   float*                    dx,
                                   float*                    dy);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageGradientSobelPlanarf32f32_v2( const float * __restrict  src,
                                      unsigned int              srcWidth,
                                      unsigned int              srcHeight,
                                      unsigned int              srcStride,
                                      float*                    dx,
                                      float*                    dy,
                                      unsigned int              dxyStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API int
fcvTrackBMOpticalFlow16x16u8( const uint8_t* __restrict   src1,
                              const uint8_t* __restrict   src2,
                              uint32_t                    srcWidth,
                              uint32_t                    srcHeight,
                              uint32_t                    srcStride,
                              uint32_t                    roiLeft,
                              uint32_t                    roiTop,
                              uint32_t                    roiRight,
                              uint32_t                    roiBottom,
                              uint32_t                    shiftSize,
                              uint32_t                    searchWidth,
                              uint32_t                    searchHeight,
                              uint32_t                    searchStep,
                              uint32_t                    usePrevious,
                              uint32_t *                  numMv,
                              uint32_t *                  locX,
                              uint32_t *                  locY,
                              uint32_t *                  mvX,
                              uint32_t *                  mvY);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvNCCPatchesOnRectu8 ( const uint8_t* __restrict     patches,
                        uint32_t                      patchWidth,
                        uint32_t                      patchHeight,
                        const uint8_t* __restrict     src,
                        uint32_t                      srcWidth,
                        uint32_t                      srcHeight,
                        uint32_t                      srcStride,
                        const uint32_t * __restrict   searchCenterX,
                        const uint32_t * __restrict   searchCenterY,
                        uint32_t                      searchWidth,
                        uint32_t                      searchHeight,
                        int32_t                       filterLowVariance,
                        uint32_t* __restrict          bestX,
                        uint32_t* __restrict          bestY,
                        uint32_t* __restrict          bestNCC,
                        int32_t                       findSubPixel,
                        float32_t* __restrict         subX,
                        float32_t* __restrict         subY,
                        uint32_t                      numSearches );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvBitwiseOru8(const uint8_t* __restrict src1,
                                const uint8_t* __restrict src2,
                                uint32_t                  srcWidth,
                                uint32_t                  srcHeight,
                                uint32_t                  srcStride,
                                uint8_t * __restrict      dst,
                                uint32_t                  dstStride,
                                uint8_t * __restrict      mask,
                                uint32_t                  maskStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvBitwiseOrs32(const int32_t* __restrict src1,
                                 const int32_t* __restrict src2,
                                 uint32_t                  srcWidth,
                                 uint32_t                  srcHeight,
                                 uint32_t                  srcStride,
                                 int32_t * __restrict      dst,
                                 uint32_t                  dstStride,
                                 uint8_t * __restrict      mask,
                                 uint32_t                  maskStride);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvColorRGB888ToGrayu8( const uint8_t* __restrict src,
                     uint32_t srcWidth,
                     uint32_t srcHeight,
                     uint32_t srcStride,
                     uint8_t* __restrict dst,
                     uint32_t  dstStride);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvTiltedIntegralu8s32( const uint8_t* __restrict src,
                                        uint32_t             srcWidth,
                                        uint32_t            srcHeight,
                                        uint32_t            srcStride,
                                        int32_t* __restrict       dst,
                                        uint32_t            dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvConvValids16( const int16_t* __restrict src1,
                 uint32_t src1Width,
                 uint32_t src1Height,
                 uint32_t src1Stride,
                 const int16_t* __restrict src2,
                 uint32_t src2Width,
                 uint32_t src2Height,
                 uint32_t src2Stride,
                 int32_t* __restrict dst,
                 uint32_t dstStride);




//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvBoundingRectangle(const uint32_t * __restrict xy, uint32_t numPoints,
                                      uint32_t * rectTopLeftX, uint32_t * rectTopLeftY,
                                      uint32_t * rectWidth, uint32_t *rectHeight);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsampleVerticalu8( const uint8_t* __restrict src,
                       uint32_t                  srcWidth,
                       uint32_t                  srcHeight,
                       uint32_t                  srcStride,
                       uint8_t* __restrict       dst,
                       uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsampleHorizontalu8( const uint8_t* __restrict src,
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         uint8_t* __restrict       dst,
                         uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsample2Du8( const uint8_t* __restrict src,
                 uint32_t                  srcWidth,
                 uint32_t                  srcHeight,
                 uint32_t                  srcStride,
                 uint8_t* __restrict       dst,
                 uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsampleVerticalInterleavedu8( const uint8_t* __restrict src,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsampleHorizontalInterleavedu8( const uint8_t* __restrict src,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsample2DInterleavedu8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToYCbCr444Planaru8( const uint8_t* __restrict src,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcStride,
                                  uint8_t* __restrict       dstY,
                                  uint8_t* __restrict       dstCb,
                                  uint8_t* __restrict       dstCr,
                                  uint32_t                  dstYStride,
                                  uint32_t                  dstCbStride,
                                  uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToYCbCr422Planaru8( const uint8_t* __restrict src,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcStride,
                                  uint8_t* __restrict       dstY,
                                  uint8_t* __restrict       dstCb,
                                  uint8_t* __restrict       dstCr,
                                  uint32_t                  dstYStride,
                                  uint32_t                  dstCbStride,
                                  uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToYCbCr420Planaru8( const uint8_t* __restrict src,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcStride,
                                  uint8_t* __restrict       dstY,
                                  uint8_t* __restrict       dstCb,
                                  uint8_t* __restrict       dstCr,
                                  uint32_t                  dstYStride,
                                  uint32_t                  dstCbStride,
                                  uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToYCbCr444Planaru8( const uint8_t* __restrict src,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcStride,
                                  uint8_t* __restrict       dstY,
                                  uint8_t* __restrict       dstCb,
                                  uint8_t* __restrict       dstCr,
                                  uint32_t                  dstYStride,
                                  uint32_t                  dstCbStride,
                                  uint32_t                  dstCrStride );


//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToYCbCr422Planaru8( const uint8_t* __restrict src,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcStride,
                                  uint8_t* __restrict       dstY,
                                  uint8_t* __restrict       dstCb,
                                  uint8_t* __restrict       dstCr,
                                  uint32_t                  dstYStride,
                                  uint32_t                  dstCbStride,
                                  uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToYCbCr420Planaru8( const uint8_t* __restrict src,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcStride,
                                  uint8_t* __restrict       dstY,
                                  uint8_t* __restrict       dstCb,
                                  uint8_t* __restrict       dstCr,
                                  uint32_t                  dstYStride,
                                  uint32_t                  dstCbStride,
                                  uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToYCbCr444Planaru8( const uint8_t* __restrict src,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcStride,
                                    uint8_t* __restrict       dstY,
                                    uint8_t* __restrict       dstCb,
                                    uint8_t* __restrict       dstCr,
                                    uint32_t                  dstYStride,
                                    uint32_t                  dstCbStride,
                                    uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToYCbCr422Planaru8( const uint8_t* __restrict src,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcStride,
                                    uint8_t* __restrict       dstY,
                                    uint8_t* __restrict       dstCb,
                                    uint8_t* __restrict       dstCr,
                                    uint32_t                  dstYStride,
                                    uint32_t                  dstCbStride,
                                    uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToYCbCr420Planaru8( const uint8_t* __restrict src,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcStride,
                                    uint8_t* __restrict       dstY,
                                    uint8_t* __restrict       dstCb,
                                    uint8_t* __restrict       dstCr,
                                    uint32_t                  dstYStride,
                                    uint32_t                  dstCbStride,
                                    uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToYCbCr444PseudoPlanaru8( const uint8_t* __restrict src,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcStride,
                                        uint8_t* __restrict       dstY,
                                        uint8_t* __restrict       dstC,
                                        uint32_t                  dstYStride,
                                        uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToYCbCr422PseudoPlanaru8( const uint8_t* __restrict src,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcStride,
                                        uint8_t* __restrict       dstY,
                                        uint8_t* __restrict       dstC,
                                        uint32_t                  dstYStride,
                                        uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToYCbCr420PseudoPlanaru8( const uint8_t* __restrict src,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcStride,
                                        uint8_t* __restrict       dstY,
                                        uint8_t* __restrict       dstC,
                                        uint32_t                  dstYStride,
                                        uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToYCbCr444PseudoPlanaru8( const uint8_t* __restrict src,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcStride,
                                        uint8_t* __restrict       dstY,
                                        uint8_t* __restrict       dstC,
                                        uint32_t                  dstYStride,
                                        uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToYCbCr422PseudoPlanaru8( const uint8_t* __restrict src,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcStride,
                                        uint8_t* __restrict       dstY,
                                        uint8_t* __restrict       dstC,
                                        uint32_t                  dstYStride,
                                        uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToYCbCr420PseudoPlanaru8( const uint8_t* __restrict src,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcStride,
                                        uint8_t* __restrict       dstY,
                                        uint8_t* __restrict       dstC,
                                        uint32_t                  dstYStride,
                                        uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToYCbCr444PseudoPlanaru8( const uint8_t* __restrict src,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcStride,
                                          uint8_t* __restrict       dstY,
                                          uint8_t* __restrict       dstC,
                                          uint32_t                  dstYStride,
                                          uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToYCbCr422PseudoPlanaru8( const uint8_t* __restrict src,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcStride,
                                          uint8_t* __restrict       dstY,
                                          uint8_t* __restrict       dstC,
                                          uint32_t                  dstYStride,
                                          uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToYCbCr420PseudoPlanaru8( const uint8_t* __restrict src,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcStride,
                                          uint8_t* __restrict       dstY,
                                          uint8_t* __restrict       dstC,
                                          uint32_t                  dstYStride,
                                          uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToRGB888u8( const uint8_t* __restrict src,
                          uint32_t                  srcWidth,
                          uint32_t                  srcHeight,
                          uint32_t                  srcStride,
                          uint8_t* __restrict       dst,
                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToRGBA8888u8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToBGR565u8( const uint8_t* __restrict src,
                          uint32_t                  srcWidth,
                          uint32_t                  srcHeight,
                          uint32_t                  srcStride,
                          uint8_t* __restrict       dst,
                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToBGR888u8( const uint8_t* __restrict src,
                          uint32_t                  srcWidth,
                          uint32_t                  srcHeight,
                          uint32_t                  srcStride,
                          uint8_t* __restrict       dst,
                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB565ToBGRA8888u8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToRGB565u8( const uint8_t* __restrict src,
                          uint32_t                  srcWidth,
                          uint32_t                  srcHeight,
                          uint32_t                  srcStride,
                          uint8_t* __restrict       dst,
                          uint32_t                  dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToRGBA8888u8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToBGR565u8( const uint8_t* __restrict src,
                          uint32_t                  srcWidth,
                          uint32_t                  srcHeight,
                          uint32_t                  srcStride,
                          uint8_t* __restrict       dst,
                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToBGR888u8( const uint8_t* __restrict src,
                          uint32_t                  srcWidth,
                          uint32_t                  srcHeight,
                          uint32_t                  srcStride,
                          uint8_t* __restrict       dst,
                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888ToBGRA8888u8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToRGB565u8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToRGB888u8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToBGR565u8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToBGR888u8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToBGRA8888u8( const uint8_t* __restrict src,
                              uint32_t                  srcWidth,
                              uint32_t                  srcHeight,
                              uint32_t                  srcStride,
                              uint8_t* __restrict       dst,
                              uint32_t                  dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGBA8888ToLABu8( const uint8_t* __restrict src,
                         uint32_t            srcWidth,
                         uint32_t            srcHeight,
                         uint32_t            srcStride,
                         uint8_t* __restrict dst,
                         uint32_t            dstStride );

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToYCbCr422Planaru8( const uint8_t*            srcY,
                                          const uint8_t* __restrict srcCb,
                                          const uint8_t* __restrict srcCr,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCbStride,
                                          uint32_t                  srcCrStride,
                                          uint8_t*                  dstY,
                                          uint8_t* __restrict       dstCb,
                                          uint8_t* __restrict       dstCr,
                                          uint32_t                  dstYStride,
                                          uint32_t                  dstCbStride,
                                          uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToYCbCr420Planaru8( const uint8_t*            srcY,
                                          const uint8_t* __restrict srcCb,
                                          const uint8_t* __restrict srcCr,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCbStride,
                                          uint32_t                  srcCrStride,
                                          uint8_t*                  dstY,
                                          uint8_t* __restrict       dstCb,
                                          uint8_t* __restrict       dstCr,
                                          uint32_t                  dstYStride,
                                          uint32_t                  dstCbStride,
                                          uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToYCbCr444PseudoPlanaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcCb,
                                                const uint8_t* __restrict srcCr,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCbStride,
                                                uint32_t                  srcCrStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstC,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToYCbCr422PseudoPlanaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcCb,
                                                const uint8_t* __restrict srcCr,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCbStride,
                                                uint32_t                  srcCrStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstC,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToYCbCr420PseudoPlanaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcCb,
                                                const uint8_t* __restrict srcCr,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCbStride,
                                                uint32_t                  srcCrStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstC,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCStride );


//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToYCbCr444Planaru8( const uint8_t*            srcY,
                                          const uint8_t* __restrict srcCb,
                                          const uint8_t* __restrict srcCr,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCbStride,
                                          uint32_t                  srcCrStride,
                                          uint8_t*                  dstY,
                                          uint8_t* __restrict       dstCb,
                                          uint8_t* __restrict       dstCr,
                                          uint32_t                  dstYStride,
                                          uint32_t                  dstCbStride,
                                          uint32_t                  dstCrStride );


//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToYCbCr420Planaru8( const uint8_t*            srcY,
                                          const uint8_t* __restrict srcCb,
                                          const uint8_t* __restrict srcCr,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCbStride,
                                          uint32_t                  srcCrStride,
                                          uint8_t*                  dstY,
                                          uint8_t* __restrict       dstCb,
                                          uint8_t* __restrict       dstCr,
                                          uint32_t                  dstYStride,
                                          uint32_t                  dstCbStride,
                                          uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToYCbCr444PseudoPlanaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcCb,
                                                const uint8_t* __restrict srcCr,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCbStride,
                                                uint32_t                  srcCrStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstC,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToYCbCr422PseudoPlanaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcCb,
                                                const uint8_t* __restrict srcCr,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCbStride,
                                                uint32_t                  srcCrStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstC,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToYCbCr420PseudoPlanaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcCb,
                                                const uint8_t* __restrict srcCr,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCbStride,
                                                uint32_t                  srcCrStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstC,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToYCbCr444Planaru8( const uint8_t*            srcY,
                                          const uint8_t* __restrict srcCb,
                                          const uint8_t* __restrict srcCr,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCbStride,
                                          uint32_t                  srcCrStride,
                                          uint8_t*                  dstY,
                                          uint8_t* __restrict       dstCb,
                                          uint8_t* __restrict       dstCr,
                                          uint32_t                  dstYStride,
                                          uint32_t                  dstCbStride,
                                          uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToYCbCr422Planaru8( const uint8_t*            srcY,
                                          const uint8_t* __restrict srcCb,
                                          const uint8_t* __restrict srcCr,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCbStride,
                                          uint32_t                  srcCrStride,
                                          uint8_t*                  dstY,
                                          uint8_t* __restrict       dstCb,
                                          uint8_t* __restrict       dstCr,
                                          uint32_t                  dstYStride,
                                          uint32_t                  dstCbStride,
                                          uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToYCbCr444PseudoPlanaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcCb,
                                                const uint8_t* __restrict srcCr,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCbStride,
                                                uint32_t                  srcCrStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstC,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToYCbCr422PseudoPlanaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcCb,
                                                const uint8_t* __restrict srcCr,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCbStride,
                                                uint32_t                  srcCrStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstC,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToYCbCr420PseudoPlanaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcCb,
                                                const uint8_t* __restrict srcCr,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCbStride,
                                                uint32_t                  srcCrStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstC,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToYCbCr422PseudoPlanaru8( const uint8_t*            srcY,
                                                      const uint8_t* __restrict srcC,
                                                      uint32_t                  srcWidth,
                                                      uint32_t                  srcHeight,
                                                      uint32_t                  srcYStride,
                                                      uint32_t                  srcCStride,
                                                      uint8_t*                  dstY,
                                                      uint8_t* __restrict       dstC,
                                                      uint32_t                  dstYStride,
                                                      uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToYCbCr420PseudoPlanaru8( const uint8_t*            srcY,
                                                      const uint8_t* __restrict srcC,
                                                      uint32_t                  srcWidth,
                                                      uint32_t                  srcHeight,
                                                      uint32_t                  srcYStride,
                                                      uint32_t                  srcCStride,
                                                      uint8_t*                  dstY,
                                                      uint8_t* __restrict       dstC,
                                                      uint32_t                  dstYStride,
                                                      uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToYCbCr444Planaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcC,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstCb,
                                                uint8_t* __restrict       dstCr,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCbStride,
                                                uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToYCbCr422Planaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcC,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstCb,
                                                uint8_t* __restrict       dstCr,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCbStride,
                                                uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToYCbCr420Planaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcC,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstCb,
                                                uint8_t* __restrict       dstCr,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCbStride,
                                                uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToYCbCr444PseudoPlanaru8( const uint8_t*            srcY,
                                                      const uint8_t* __restrict srcC,
                                                      uint32_t                  srcWidth,
                                                      uint32_t                  srcHeight,
                                                      uint32_t                  srcYStride,
                                                      uint32_t                  srcCStride,
                                                      uint8_t*                  dstY,
                                                      uint8_t* __restrict       dstC,
                                                      uint32_t                  dstYStride,
                                                      uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToYCbCr420PseudoPlanaru8( const uint8_t*            srcY,
                                                      const uint8_t* __restrict srcC,
                                                      uint32_t                  srcWidth,
                                                      uint32_t                  srcHeight,
                                                      uint32_t                  srcYStride,
                                                      uint32_t                  srcCStride,
                                                      uint8_t*                  dstY,
                                                      uint8_t* __restrict       dstC,
                                                      uint32_t                  dstYStride,
                                                      uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToYCbCr444Planaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcC,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstCb,
                                                uint8_t* __restrict       dstCr,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCbStride,
                                                uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToYCbCr422Planaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcC,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstCb,
                                                uint8_t* __restrict       dstCr,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCbStride,
                                                uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToYCbCr420Planaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcC,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstCb,
                                                uint8_t* __restrict       dstCr,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCbStride,
                                                uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToYCbCr444PseudoPlanaru8( const uint8_t*            srcY,
                                                      const uint8_t* __restrict srcC,
                                                      uint32_t                  srcWidth,
                                                      uint32_t                  srcHeight,
                                                      uint32_t                  srcYStride,
                                                      uint32_t                  srcCStride,
                                                      uint8_t*                  dstY,
                                                      uint8_t* __restrict       dstC,
                                                      uint32_t                  dstYStride,
                                                      uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToYCbCr422PseudoPlanaru8( const uint8_t*            srcY,
                                                      const uint8_t* __restrict srcC,
                                                      uint32_t                  srcWidth,
                                                      uint32_t                  srcHeight,
                                                      uint32_t                  srcYStride,
                                                      uint32_t                  srcCStride,
                                                      uint8_t*                  dstY,
                                                      uint8_t* __restrict       dstC,
                                                      uint32_t                  dstYStride,
                                                      uint32_t                  dstCStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToYCbCr444Planaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcC,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstCb,
                                                uint8_t* __restrict       dstCr,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCbStride,
                                                uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToYCbCr422Planaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcC,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstCb,
                                                uint8_t* __restrict       dstCr,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCbStride,
                                                uint32_t                  dstCrStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToYCbCr420Planaru8( const uint8_t*            srcY,
                                                const uint8_t* __restrict srcC,
                                                uint32_t                  srcWidth,
                                                uint32_t                  srcHeight,
                                                uint32_t                  srcYStride,
                                                uint32_t                  srcCStride,
                                                uint8_t*                  dstY,
                                                uint8_t* __restrict       dstCb,
                                                uint8_t* __restrict       dstCr,
                                                uint32_t                  dstYStride,
                                                uint32_t                  dstCbStride,
                                                uint32_t                  dstCrStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToRGB565u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToRGB888u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                    const uint8_t* __restrict srcCb,
                                    const uint8_t* __restrict srcCr,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcYStride,
                                    uint32_t                  srcCbStride,
                                    uint32_t                  srcCrStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToRGB565u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToRGB888u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                    const uint8_t* __restrict srcCb,
                                    const uint8_t* __restrict srcCr,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcYStride,
                                    uint32_t                  srcCbStride,
                                    uint32_t                  srcCrStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToRGB565u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToRGB888u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                    const uint8_t* __restrict srcCb,
                                    const uint8_t* __restrict srcCr,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcYStride,
                                    uint32_t                  srcCbStride,
                                    uint32_t                  srcCrStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToRGB565u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToRGB888u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                          const uint8_t* __restrict srcC,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCStride,
                                          uint8_t* __restrict       dst,
                                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToRGB565u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToRGB888u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                          const uint8_t* __restrict srcC,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCStride,
                                          uint8_t* __restrict       dst,
                                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToRGB565u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToRGB888u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                          const uint8_t* __restrict srcC,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCStride,
                                          uint8_t* __restrict       dst,
                                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvEdgeWeightings16( int16_t* __restrict edgeMap,
                     const uint32_t      edgeMapWidth,
                     const uint32_t      edgeMapHeight,
                     const uint32_t      edgeMapStride,
                     const uint32_t      weight,
                     const uint32_t      edge_limit,
                     const uint32_t      hl_threshold,
                     const uint32_t      hh_threshold,
                     const uint32_t      edge_denoise_factor );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDeinterleaveu8( const uint8_t* __restrict src,
                   uint32_t                  srcWidth,
                   uint32_t                  srcHeight,
                   uint32_t                  srcStride,
                   uint8_t* __restrict       dst0,
                   uint32_t                  dst0Stride,
                   uint8_t* __restrict       dst1,
                   uint32_t                  dst1Stride );


//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

FASTCV_API void
fcvInterleaveu8( const uint8_t* __restrict src0,
                 const uint8_t* __restrict src1,
                 uint32_t                  imageWidth,
                 uint32_t                  imageHeight,
                 uint32_t                  src0Stride,
                 uint32_t                  src1Stride,
                 uint8_t* __restrict       dst,
                 uint32_t                  dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDWTHarrTransposeu8( const uint8_t* __restrict src,
                       uint32_t                  srcWidth,
                       uint32_t                  srcHeight,
                       uint32_t                  srcStride,
                       int16_t* __restrict       dst,
                       uint32_t                  dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDWTHaarTransposeu8( const uint8_t* __restrict src,
                       uint32_t                  srcWidth,
                       uint32_t                  srcHeight,
                       uint32_t                  srcStride,
                       int16_t* __restrict       dst,
                       uint32_t                  dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDWT53TabTransposes16( const int16_t* __restrict src,
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         int16_t* __restrict       dst,
                         uint32_t                  dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIDWT53TabTransposes16( const int16_t*   __restrict src,
                          uint32_t                    srcWidth,
                          uint32_t                    srcHeight,
                          uint32_t                    srcStride,
                          int16_t* __restrict         dst,
                          uint32_t                    dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIDWTHarrTransposes16( const int16_t* __restrict src,
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         uint8_t* __restrict       dst,
                         uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIDWTHaarTransposes16( const int16_t* __restrict src,
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         uint8_t* __restrict       dst,
                         uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDWTHaaru8( const uint8_t* __restrict src,
              uint32_t                  srcWidth,
              uint32_t                  srcHeight,
              uint32_t                  srcStride,
              int16_t* __restrict       dst,
              uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDWT53Tabs16( const int16_t* __restrict src,
                uint32_t                  srcWidth,
                uint32_t                  srcHeight,
                uint32_t                  srcStride,
                int16_t* __restrict       dst,
                uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIDWT53Tabs16( const int16_t*   __restrict src,
                 uint32_t                    srcWidth,
                 uint32_t                    srcHeight,
                 uint32_t                    srcStride,
                 int16_t* __restrict         dst,
                 uint32_t                    dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIDWTHaars16( const int16_t* __restrict src,
                uint32_t                  srcWidth,
                uint32_t                  srcHeight,
                uint32_t                  srcStride,
                uint8_t* __restrict       dst,
                uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvDCTu8( const uint8_t* __restrict src,
          uint32_t                  srcWidth,
          uint32_t                  srcHeight,
          uint32_t                  srcStride,
          int16_t* __restrict       dst,
          uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIDCTs16( const int16_t* __restrict src,
            uint32_t                  srcWidth,
            uint32_t                  srcHeight,
            uint32_t                  srcStride,
            uint8_t* __restrict       dst,
            uint32_t                  dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleUpPolyu8( const uint8_t* __restrict src,
                  uint32_t                  srcWidth,
                  uint32_t                  srcHeight,
                  uint32_t                  srcStride,
                  uint8_t* __restrict       dst,
                  uint32_t                  dstWidth,
                  uint32_t                  dstHeight,
                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleUpPolyInterleaveu8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstWidth,
                            uint32_t                  dstHeight,
                            uint32_t                  dstStride );


//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownMNu8( const uint8_t* __restrict src,
                  uint32_t                  srcWidth,
                  uint32_t                  srcHeight,
                  uint32_t                  srcStride,
                  uint8_t* __restrict       dst,
                  uint32_t                  dstWidth,
                  uint32_t                  dstHeight,
                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownMNInterleaveu8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstWidth,
                            uint32_t                  dstHeight,
                            uint32_t                  dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API uint32_t
fcvKMeansTreeSearch36x10s8( const   int8_t* __restrict  nodeChildrenCenter,
                            const uint32_t* __restrict  nodeChildrenInvLenQ32,
                            const uint32_t* __restrict  nodeChildrenIndex,
                            const  uint8_t* __restrict  nodeNumChildren,
                                  uint32_t              numNodes,
                            const  int8_t * __restrict  key );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int
fcvLinearSearchPrepare8x36s8(  uint32_t * __restrict   dbLUT,
                               uint32_t                numDBLUT,
                               int8_t   * __restrict   descDB,
                               uint32_t * __restrict   descDBInvLenQ38,
                               uint16_t * __restrict   descDBTargetId,
                               uint32_t * __restrict   descDBOldIdx,
                               uint32_t                numDescDB );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvLinearSearch8x36s8(
   const uint32_t * __restrict dbLUT,
   uint32_t                    numDBLUT,
   const int8_t   * __restrict descDB,
   const uint32_t * __restrict descDBInvLenQ38,
   const uint16_t * __restrict descDBTargetId,
   uint32_t                    numDescDB,
   const int8_t   * __restrict srcDesc,
   const uint32_t * __restrict srcDescInvLenQ38,
   const uint32_t * __restrict srcDescIdx,
   uint32_t                    numSrcDesc,
   const uint16_t * __restrict targetsToIgnore,
   uint32_t                    numTargetsToIgnore,
   uint32_t                    maxDistanceQ31,
   uint32_t       * __restrict correspondenceDBIdx,
   uint32_t       * __restrict correspondenceSrcDescIdx,
   uint32_t       * __restrict correspondenceDistanceQ31,
   uint32_t                    maxNumCorrespondences,
   uint32_t       * __restrict numCorrespondences );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvFindContoursExternalu8( uint8_t* __restrict   src,
                           uint32_t              srcWidth,
                           uint32_t              srcHeight,
                           uint32_t              srcStride,
                           uint32_t              maxNumContours,
                           uint32_t* __restrict  numContours,
                           uint32_t* __restrict  numContourPoints,
                           uint32_t** __restrict contourStartPoints,
                           uint32_t* __restrict  pointBuffer,
                           uint32_t              pointBufferSize,
                           int32_t               hierarchy[][4],
                           void*                 contourHandle );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvFindContoursListu8( uint8_t* __restrict   src,
                       uint32_t              srcWidth,
                       uint32_t              srcHeight,
                       uint32_t              srcStride,
                       uint32_t              maxNumContours,
                       uint32_t* __restrict  numContours,
                       uint32_t* __restrict  numContourPoints,
                       uint32_t** __restrict contourStartPoints,
                       uint32_t* __restrict  pointBuffer,
                       uint32_t              pointBufferSize,
                       void*                 contourHandle );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvFindContoursCcompu8( uint8_t* __restrict   src,
                        uint32_t              srcWidth,
                        uint32_t              srcHeight,
                        uint32_t              srcStride,
                        uint32_t              maxNumContours,
                        uint32_t* __restrict   numContours,
                        uint32_t* __restrict  holeFlag,
                        uint32_t* __restrict  numContourPoints,
                        uint32_t** __restrict contourStartPoints,
                        uint32_t* __restrict  pointBuffer,
                        uint32_t              pointBufferSize,
                        int32_t               hierarchy[][4],
                        void*                 contourHandle );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvFindContoursTreeu8( uint8_t* __restrict   src,
                       uint32_t              srcWidth,
                       uint32_t              srcHeight,
                       uint32_t              srcStride,
                       uint32_t              maxNumContours,
                       uint32_t* __restrict  numContours,
                       uint32_t* __restrict  holeFlag,
                       uint32_t* __restrict  numContourPoints,
                       uint32_t** __restrict contourStartPoints,
                       uint32_t* __restrict  pointBuffer,
                       uint32_t              pointBufferSize,
                       int32_t               hierarchy[][4],
                       void*                 contourHandle );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void*
fcvFindContoursAllocate( uint32_t srcStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvFindContoursDelete( void* contourHandle );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSolvef32(const float32_t * __restrict A,
            int32_t numCols,
            int32_t numRows,
            const float32_t * __restrict b,
            float32_t * __restrict x);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvGetPerspectiveTransformf32( const float32_t src1[8],
                               const float32_t src2[8],
                               float32_t  transformCoefficient[9] );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvSetElementsu8(        uint8_t * __restrict dst,
                         uint32_t             dstWidth,
                         uint32_t             dstHeight,
                         uint32_t             dstStride,
                         uint8_t              value,
                   const uint8_t * __restrict mask,
                         uint32_t             maskStride
                 );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementss32(          int32_t * __restrict dst,
                             uint32_t             dstWidth,
                             uint32_t             dstHeight,
                             uint32_t             dstStride,
                             int32_t              value,
                       const uint8_t * __restrict mask ,
                             uint32_t             maskStride
                     );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsf32(        float32_t * __restrict dst,
                          uint32_t               dstWidth,
                          uint32_t               dstHeight,
                          uint32_t               dstStride,
                          float32_t              value,
                    const uint8_t   * __restrict mask,
                          uint32_t               maskStride
                   );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc4u8(         uint8_t * __restrict dst,
                            uint32_t             dstWidth,
                            uint32_t             dstHeight,
                            uint32_t             dstStride,
                            uint8_t              value1,
                            uint8_t              value2,
                            uint8_t              value3,
                            uint8_t              value4,
                      const uint8_t * __restrict mask,
                            uint32_t             maskStride
                    );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc4s32(         int32_t * __restrict dst,
                             uint32_t             dstWidth,
                             uint32_t             dstHeight,
                             uint32_t             dstStride,
                             int32_t              value1,
                             int32_t              value2,
                             int32_t              value3,
                             int32_t              value4,
                       const uint8_t * __restrict mask,
                             uint32_t             maskStride
                     );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc4f32(         float32_t * __restrict dst,
                             uint32_t               dstWidth,
                             uint32_t               dstHeight,
                             uint32_t               dstStride,
                             float32_t              value1,
                             float32_t              value2,
                             float32_t              value3,
                             float32_t              value4,
                       const uint8_t   * __restrict mask,
                             uint32_t               maskStride
                     );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc3u8(         uint8_t * __restrict dst,
                            uint32_t             dstWidth,
                            uint32_t             dstHeight,
                            uint32_t             dstStride,
                            uint8_t              value1,
                            uint8_t              value2,
                            uint8_t              value3,
                      const uint8_t * __restrict mask,
                            uint32_t             maskStride
                    );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc3s32(         int32_t * __restrict dst,
                             uint32_t             dstWidth,
                             uint32_t             dstHeight,
                             uint32_t             dstStride,
                             int32_t              value1,
                             int32_t              value2,
                             int32_t              value3,
                       const uint8_t * __restrict mask,
                             uint32_t             maskStride
                     );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc3f32(         float32_t * __restrict dst,
                             uint32_t               dstWidth,
                             uint32_t               dstHeight,
                             uint32_t               dstStride,
                             float32_t              value1,
                             float32_t              value2,
                             float32_t              value3,
                       const uint8_t   * __restrict mask,
                             uint32_t               maskStride
                     );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

typedef enum {
    FCV_THRESH_BINARY      = 0,   // value = value > threshold ? max_value : 0
    FCV_THRESH_BINARY_INV       // value = value > threshold ? 0 : max_value
} fcvThreshType;


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvAdaptiveThresholdGaussian3x3u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvAdaptiveThresholdGaussian5x5u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvAdaptiveThresholdGaussian11x11u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvAdaptiveThresholdMean3x3u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvAdaptiveThresholdMean5x5u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvAdaptiveThresholdMean11x11u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvBoxFilter3x3u8( const uint8_t* __restrict src,
                         uint32_t            srcWidth,
                         uint32_t            srcHeight,
                         uint32_t            srcStride,
                         uint8_t* __restrict dst,
                         uint32_t            dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvBoxFilter5x5u8( const uint8_t* __restrict src,
                         uint32_t            srcWidth,
                         uint32_t            srcHeight,
                         uint32_t            srcStride,
                         uint8_t* __restrict dst,
                         uint32_t            dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvBoxFilter11x11u8(const uint8_t* __restrict src,
                          uint32_t            srcWidth,
                          uint32_t            srcHeight,
                          uint32_t            srcStride,
                          uint8_t* __restrict dst,
                          uint32_t            dstStride);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvBoxFilterNxNf32(const float32_t*  src,
                         uint32_t    srcWidth,
                         uint32_t    srcHeight,
                         uint32_t    srcStride,
                         uint32_t    N,
                         float32_t*  dst,
                         uint32_t    dstStride);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvBilateralFilter5x5u8(const uint8_t* __restrict src,
                              uint32_t            srcWidth,
                              uint32_t            srcHeight,
                              uint32_t            srcStride,
                              uint8_t* __restrict dst,
                              uint32_t            dstStride);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvBilateralFilter5x5u8_v2(const uint8_t* __restrict src,
                                 uint32_t            srcWidth,
                                 uint32_t            srcHeight,
                                 uint32_t            srcStride,
                                 uint8_t* __restrict dst,
                                 uint32_t            dstStride,
                                 float32_t           sigmaColor,
                                 float32_t           sigmaSpace);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvBilateralFilter7x7u8(const uint8_t* __restrict src,
                              uint32_t            srcWidth,
                              uint32_t            srcHeight,
                              uint32_t            srcStride,
                              uint8_t* __restrict dst,
                              uint32_t            dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvBilateralFilter7x7u8_v2(const uint8_t* __restrict src,
                                 uint32_t            srcWidth,
                                 uint32_t            srcHeight,
                                 uint32_t            srcStride,
                                 uint8_t* __restrict dst,
                                 uint32_t            dstStride,
                                 float32_t           sigmaColor,
                                 float32_t           sigmaSpace);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvBilateralFilter9x9u8(const uint8_t* __restrict src,
                               uint32_t            srcWidth,
                               uint32_t            srcHeight,
                               uint32_t            srcStride,
                               uint8_t* __restrict dst,
                               uint32_t            dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvBilateralFilter9x9u8_v2(const uint8_t* __restrict src,
                                 uint32_t            srcWidth,
                                 uint32_t            srcHeight,
                                 uint32_t            srcStride,
                                 uint8_t* __restrict dst,
                                 uint32_t            dstStride,
                                 float32_t           sigmaColor,
                                 float32_t           sigmaSpace);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSegmentFGMasku8(uint8_t* __restrict    src,
                   uint32_t               srcWidth,
                   uint32_t               srcHeight,
                   uint32_t               srcStride,
                   uint8_t                Polygonal,
                   uint32_t               perimScale);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvAbsDiffu8(const uint8_t * __restrict src1,
             const uint8_t * __restrict src2,
                   uint32_t             srcWidth,
                   uint32_t             srcHeight,
                   uint32_t             srcStride,
                   uint8_t * __restrict dst,
                   uint32_t             dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffs32(const int32_t * __restrict  src1,
              const int32_t * __restrict  src2,
                    uint32_t              srcWidth,
                    uint32_t              srcHeight,
                    uint32_t              srcStride,
                    int32_t * __restrict  dst,
                    uint32_t              dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDifff32(const float32_t * __restrict  src1,
              const float32_t * __restrict  src2,
                    uint32_t                srcWidth,
                    uint32_t                srcHeight,
                    uint32_t                srcStride,
                    float32_t * __restrict  dst,
                    uint32_t                dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVu8(const uint8_t * __restrict src,
                    uint8_t              value,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVs32(const int32_t * __restrict src,
                     int32_t              value,
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVf32(const float32_t * __restrict src,
                     float32_t              value,
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc4u8(const uint8_t * __restrict src,
                    uint8_t              value1,
                    uint8_t              value2,
                    uint8_t              value3,
                    uint8_t              value4,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc4s32(const int32_t * __restrict src,
                     int32_t              value1,
                     int32_t              value2,
                     int32_t              value3,
                     int32_t              value4,
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc4f32(const float32_t * __restrict src,
                     float32_t              value1,
                     float32_t              value2,
                     float32_t              value3,
                     float32_t              value4,
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc3u8(const uint8_t * __restrict src,
                    uint8_t              value1,
                    uint8_t              value2,
                    uint8_t              value3,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc3s32(const int32_t * __restrict src,
                     int32_t              value1,
                     int32_t              value2,
                     int32_t              value3,
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc3f32(const float32_t * __restrict src,
                     float32_t              value1,
                     float32_t              value2,
                     float32_t              value3,
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FASTCV_API int
fcvKDTreeCreate36s8f32( const        int8_t*  __restrict vectors,
                            const     float32_t*  __restrict invLengths,
                                            int              numVectors,
                             fcvKDTreeDatas8f32**            kdtrees );

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FASTCV_API int
fcvKDTreeDestroy36s8f32( fcvKDTreeDatas8f32* kdtrees );

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FASTCV_API int
fcvKDTreeQuery36s8f32( fcvKDTreeDatas8f32*       kdtrees,
                           const  int8_t* __restrict query,
                               float32_t             queryInvLen,
                                     int             maxNNs,
                               float32_t             maxDist,
                                     int             maxChecks,
                           const uint8_t* __restrict mask,
                                 int32_t*            numNNsFound,
                                 int32_t* __restrict NNInds,
                               float32_t* __restrict NNDists );

typedef struct fcvConnectedComponent
{
    uint32_t area;    //area of the cc
    uint32_t avgValue; //average value of the cc
    uint32_t rectTopLeftX; // the x of the topleft corner of the bounding box of the cc.
    uint32_t rectTopLeftY; // the y of the topleft corner of the bounding box of the cc.
    uint32_t rectWidth;    // the width of the bounding box of the cc.
    uint32_t rectHeight;   // the height of the bounding box of the cc.
}fcvConnectedComponent;

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvFloodfillSimpleu8( const uint8_t* __restrict src,
                           uint32_t             srcWidth,
                           uint32_t             srcHeight,
                           uint32_t             srcStride,
                            uint8_t* __restrict dst,
                           uint32_t             dstStride,
                           uint32_t             xBegin,
                           uint32_t             yBegin,
                            uint8_t             newVal, //new Val can't be zero. zero is background.
              fcvConnectedComponent*            cc,
                            uint8_t             connectivity,
                               void*            lineBuffer);


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFloodfillMergedu8( const uint8_t* __restrict             src,
                       uint32_t                             srcWidth,
                       uint32_t                             srcHeight,
                       uint32_t                             srcStride,
                       uint8_t* __restrict                  dst,
                       uint32_t                             dstStride,
                       uint32_t                             xBegin,
                       uint32_t                             yBegin,
                       uint8_t                              newVal,
                       fcvConnectedComponent* __restrict    cc,
                       uint8_t                              connectivity );

//---------------------------------------------------------------------------
//  maxhistory, it resets the value to zero.
//------------------------------------------------------------------------------
FASTCV_API void
fcvUpdateMotionHistoryu8s32( const uint8_t* __restrict src,
                                  uint32_t             srcWidth,
                                  uint32_t             srcHeight,
                                  uint32_t             srcStride,
                                   int32_t* __restrict dst,
                                  uint32_t             dstStride,
                                   int32_t             timeStamp,
                                   int32_t             maxHistory);


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvIntegrateImageYCbCr420PseudoPlanaru8(
                        const uint8_t* __restrict srcY,
                        const uint8_t* __restrict srcC,
                             uint32_t             srcWidth,
                             uint32_t             srcHeight,
                             uint32_t             srcYStride,
                             uint32_t             srcCStride,
                             uint32_t* __restrict integralY,
                             uint32_t* __restrict integralCb,
                             uint32_t* __restrict integralCr,
                             uint32_t             integralYStride,
                             uint32_t             integralCbStride,
                             uint32_t             integralCrStride);


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvFindForegroundIntegrateImageYCbCr420u32(
    const uint32_t * __restrict bgIntegralY,
    const uint32_t * __restrict bgIntegralCb,
    const uint32_t * __restrict bgIntegralCr,
    const uint32_t * __restrict fgIntegralY,
    const uint32_t * __restrict fgIntegralCb,
    const uint32_t * __restrict fgIntegralCr,
          uint32_t              srcWidth,
          uint32_t              srcHeight,
          uint32_t              srcYStride,
          uint32_t              srcCbStride,
          uint32_t              srcCrStride,
           uint8_t * __restrict outputMask,
          uint32_t              outputWidth,
          uint32_t              outputHeight,
          uint32_t              outputMaskStride,
         float32_t              threshold );


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvAverages32(
              const int32_t* __restrict src,
                   uint32_t             srcWidth,
                   uint32_t             srcHeight,
                   uint32_t             srcStride,
                  float32_t* __restrict avgValue);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvAverageu8(
             const uint8_t* __restrict src,
                  uint32_t             srcWidth,
                  uint32_t             srcHeight,
                  uint32_t             srcStride,
                 float32_t* __restrict avgValue);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API uint32_t
fcvMeanShiftu8(const uint8_t* __restrict src,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
             fcvRectangleInt*            window,
             fcvTermCriteria             criteria);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API uint32_t
fcvMeanShifts32(const int32_t* __restrict src,
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
              fcvRectangleInt*            window,
              fcvTermCriteria             criteria);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API uint32_t
fcvMeanShiftf32(const float32_t* __restrict src,
                       uint32_t             srcWidth,
                       uint32_t             srcHeight,
                       uint32_t             srcStride,
                fcvRectangleInt*            window,
                fcvTermCriteria             criteria);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API uint32_t
fcvConAdaTracku8(const uint8_t* __restrict src,
                   uint32_t             srcWidth,
                   uint32_t             srcHeight,
                   uint32_t             srcStride,
            fcvRectangleInt*            window,
            fcvTermCriteria             criteria,
                   fcvBox2D*            circuBox);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API uint32_t
fcvConAdaTracks32(const int32_t* __restrict src,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
             fcvRectangleInt*            window,
             fcvTermCriteria             criteria,
                    fcvBox2D*            circuBox);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API uint32_t
fcvConAdaTrackf32(const float32_t* __restrict src,
                      uint32_t             srcWidth,
                      uint32_t             srcHeight,
                      uint32_t             srcStride,
               fcvRectangleInt*            window,
               fcvTermCriteria             criteria,
                      fcvBox2D*            circuBox);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvSVDf32(const float32_t * __restrict A,
                 uint32_t              m,
                 uint32_t              n,
                float32_t * __restrict w,
                float32_t * __restrict U,
                float32_t * __restrict Vt,
                float32_t *            tmpU,
                float32_t *            tmpV);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvFillConvexPolyu8( uint32_t             nPts,
               const uint32_t* __restrict polygon,
                 uint32_t             nChannel,
                const uint8_t* __restrict color,
                  uint8_t* __restrict dst,
                 uint32_t             dstWidth,
                 uint32_t             dstHeight,
                 uint32_t             dstStride);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvPointPolygonTest(uint32_t             nPts,
              const uint32_t* __restrict polygonContour,
                    uint32_t             px,
                    uint32_t             py,
                   float32_t*            distance,
                     int16_t*            resultFlag);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFindConvexHull( uint32_t* __restrict polygonContour,
                                   uint32_t             nPtsContour,
                                   uint32_t* __restrict convexHull,
                                   uint32_t*            nPtsHull,
                                   uint32_t* __restrict tmpBuff);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API int32_t
fcvSolveCholeskyf32( float32_t* __restrict A,
               const float32_t* __restrict b,
                     float32_t* __restrict diag,
                      uint32_t             N,
                     float32_t* __restrict x);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvGeomDistortPoint2x1f32(const float32_t* __restrict cameraCalibration,
                          const float32_t* __restrict xyCamera,
                                float32_t* __restrict xyDevice);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvGeomDistortPoint2xNf32(const float32_t* __restrict cameraCalibration,
                          const float32_t* __restrict xyCamera,
                                 uint32_t             srcStride,
                                 uint32_t             xySize,
                                float32_t* __restrict xyDevice,
                                 uint32_t             dstStride);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API void
fcvGeomUndistortPoint2x1f32(const float32_t* __restrict cameraCalibration,
                            const float32_t* __restrict xyDevice,
                                  float32_t* __restrict xyCamera);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvGeomUndistortPoint2xNf32(const float32_t* __restrict cameraCalibration,
                            const float32_t* __restrict xyDevice,
                                   uint32_t             srcStride,
                                   uint32_t             xySize,
                                  float32_t* __restrict xyCamera,
                                   uint32_t             dstStride);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
FASTCV_API int32_t
fcvGeomProjectPoint3x1f32(const float32_t* __restrict pose,
                          const float32_t* __restrict cameraCalibration,
                          const float32_t* __restrict xyz,
                                float32_t* __restrict xyCamera,
                                float32_t* __restrict xyDevice);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvGeomProjectPoint3xNf32(const float32_t* __restrict pose,
                          const float32_t* __restrict cameraCalibration,
                          const float32_t* __restrict xyz,
                                 uint32_t             srcStride,
                                 uint32_t             xyzSize,
                                float32_t* __restrict xyCamera,
                                float32_t* __restrict xyDevice,
                                 uint32_t             dstStride,
                                 uint32_t*            inFront);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvInvertAffineTransformf32( const float32_t *__restrict    M,
                             float32_t *__restrict   invAffineMat );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvGeomHomographyRobustFitf32( const fcvCorrespondences* __restrict corr,
                               float32_t  *__restrict   homography,
                               float32_t   reprojThreshold);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvICPJacobianErrorSE3f32(const uint16_t* __restrict  depthData,
                          uint32_t                    depthWidth,
                          uint32_t                    depthHeight,
                          uint32_t                    depthStride,
                          const float32_t* __restrict refPointsNormals,
                          uint32_t                    numPoints,
                          const float32_t* __restrict refPose,
                          const float32_t*__restrict  camera,
                          float32_t                   sqDistThreshold,
                          float32_t* __restrict       sqDists,
                          float32_t* __restrict       errors,
                          float32_t* __restrict       jacobian);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvGeomHomographyFitf32_v2( const fcvCorrespondences* __restrict corrs,
                            float32_t* __restrict                    homography,
                            uint32_t    mode);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvGeomHomographyRobustFitf32_v2( const fcvCorrespondences* __restrict corr,
                               float32_t  *__restrict   homography,
                               float32_t   reprojThreshold,
                               uint32_t    mode);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvDepthFusion8x8x8xNs16(const fcvDepthFusionBlockConfig* __restrict configs,
                         int16_t* __restrict volumes,
                         uint32_t numBlocks,
                         uint32_t volumeStride,
                         const float32_t* __restrict depthData,
                         uint32_t  depthWidth,
                         uint32_t  depthHeight,
                         uint32_t  depthStride,
                         const float32_t* __restrict cameraCalibration,
                         int16_t maxWeight);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvRemapRGBA8888NNu8( const uint8_t*  __restrict src,
                            uint32_t             srcWidth,
                            uint32_t             srcHeight,
                            uint32_t             srcStride,
                             uint8_t* __restrict dst,
                            uint32_t             dstWidth,
                            uint32_t             dstHeight,
                            uint32_t             dstStride,
                     const float32_t* __restrict mapX,
                     const float32_t* __restrict mapY,
                            uint32_t             mapStride
                      );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvRemapRGBA8888BLu8(  const uint8_t* __restrict src,
                            uint32_t             srcWidth,
                            uint32_t             srcHeight,
                            uint32_t             srcStride,
                             uint8_t* __restrict dst,
                            uint32_t             dstWidth,
                            uint32_t             dstHeight,
                            uint32_t             dstStride,
                     const float32_t* __restrict mapX,
                     const float32_t* __restrict mapY,
                            uint32_t             mapStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvJacobianSE2f32(const uint8_t* __restrict warpedImage,
                 const uint16_t* __restrict warpedBorder,
                  const uint8_t* __restrict targetImage,
                  const int16_t* __restrict targetDX,
                  const int16_t* __restrict targetDY,
                       uint32_t             width,
                       uint32_t             height,
                       uint32_t             stride,
                      float32_t* __restrict sumJTJ,
                      float32_t* __restrict sumJTE,
                      float32_t* __restrict sumError,
                       uint32_t* __restrict numPixels);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------/

FASTCV_API void
fcvTransformAffineClippedu8(const uint8_t* __restrict src,
                                 uint32_t             srcWidth,
                                 uint32_t             srcHeight,
                                 uint32_t             srcStride,
                          const float32_t* __restrict affineMatrix,
                                  uint8_t* __restrict dst,
                                 uint32_t             dstWidth,
                                 uint32_t             dstHeight,
                                 uint32_t             dstStride,
                                 uint32_t* __restrict dstBorder);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------/

FASTCV_API fcvStatus
fcvTransformAffineClippedu8_v2( const uint8_t* __restrict src,
                                uint32_t             srcWidth,
                                uint32_t             srcHeight,
                                uint32_t             srcStride,
                                const float32_t* __restrict affineMatrix,
                                uint8_t* __restrict dst,
                                uint32_t             dstWidth,
                                uint32_t             dstHeight,
                                uint32_t             dstStride,
                                uint32_t* __restrict dstBorder,
                                fcvInterpolationType  interpolation );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------/

FASTCV_API fcvStatus
fcvTransformAffineClippedu8_v3( const uint8_t* __restrict src,
                                uint32_t             srcWidth,
                                uint32_t             srcHeight,
                                uint32_t             srcStride,
                                const float32_t* __restrict affineMatrix,
                                uint8_t* __restrict dst,
                                uint32_t             dstWidth,
                                uint32_t             dstHeight,
                                uint32_t             dstStride,
                                uint32_t* __restrict dstBorder,
                                fcvInterpolationType  interpolation,
                                fcvBorderType borderType,
                                uint8_t  borderValue );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvBGCodeWord**
fcvCreateBGCodeBookModel( uint32_t            srcWidth,
                          uint32_t            srcHeight,
                          void**   __restrict cbmodel );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvReleaseBGCodeBookModel( void** cbmodel );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvConfigBGCodeBookModel(   void* cbmodel,
                            uint8_t cbBound[3],
                            uint8_t minMod[3],
                            uint8_t maxMod[3]);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvBGCodeBookUpdateu8( void*  __restrict cbmodel,
              const uint8_t*  __restrict src,
                   uint32_t              srcWidth,
                   uint32_t              srcHeight,
                   uint32_t              srcStride,
              const uint8_t*  __restrict fgMask,
                   uint32_t              fgMaskStride,
              fcvBGCodeWord** __restrict cbMap,
                    int32_t*  __restrict updateTime );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvBGCodeBookDiffu8( void*  __restrict cbmodel,
            const uint8_t*  __restrict src,
                 uint32_t              srcWidth,
                 uint32_t              srcHeight,
                 uint32_t              srcStride,
                  uint8_t*  __restrict fgMask,
                 uint32_t              fgMaskStride,
            fcvBGCodeWord** __restrict cbMap,
                  int32_t*  __restrict numFgMask );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvBGCodeBookClearStaleu8( void*  __restrict cbmodel,
                        int32_t              staleThresh,
                  const uint8_t*  __restrict fgMask,
                       uint32_t              fgMaskWidth,
                       uint32_t              fgMaskHeight,
                       uint32_t              fgMaskStride,
                  fcvBGCodeWord** __restrict cbMap );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvHoughCircleu8( const uint8_t* __restrict src,
                       uint32_t             srcWidth,
                       uint32_t             srcHeight,
                       uint32_t             srcStride,
                      fcvCircle* __restrict circles,
                       uint32_t* __restrict numCircle,
                       uint32_t             maxCircle,
                       uint32_t             minDist,
                       uint32_t             cannyThreshold,
                       uint32_t             accThreshold,
                       uint32_t             minRadius,
                       uint32_t             maxRadius,
                           void* __restrict data);




//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
FASTCV_API void
fcvHoughLineu8(const uint8_t*  __restrict src,
                    uint32_t              srcWidth,
                    uint32_t              srcHeight,
                    uint32_t              srcStride,
                    float32_t             threshold,
                    uint32_t              maxLines,
                    uint32_t*  __restrict pNumLines,
                    fcvLine*   __restrict pLines);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvDrawContouru8(uint8_t*  __restrict src,
                uint32_t              srcWidth,
                uint32_t              srcHeight,
                uint32_t              srcStride,
                uint32_t              nContours,
          const uint32_t*  __restrict holeFlag,
          const uint32_t*  __restrict numContourPoints,
          const uint32_t** __restrict contourStartPoints,
                uint32_t              pointBufferSize,
          const uint32_t*  __restrict pointBuffer,
                 int32_t              hierarchy[][4],
                uint32_t              max_level,
                 int32_t              thickness,
                 uint8_t              color,
                 uint8_t              hole_color);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvDrawContourInterleavedu8( uint8_t*  __restrict src,
                            uint32_t              srcWidth,
                            uint32_t              srcHeight,
                            uint32_t              srcStride,
                            uint32_t              nContours,
                      const uint32_t*  __restrict holeFlag,
                      const uint32_t*  __restrict numContourPoints,
                      const uint32_t** __restrict contourStartPoints,
                            uint32_t              pointBufferSize,
                      const uint32_t*  __restrict pointBuffer,
                             int32_t              hierarchy[][4],
                            uint32_t              max_level,
                             int32_t              thickness,
                             uint8_t              colorR,
                             uint8_t              colorG,
                             uint8_t              colorB,
                             uint8_t              hole_colorR,
                             uint8_t              hole_colorG,
                             uint8_t              hole_colorB);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvDrawContourPlanaru8( uint8_t*  __restrict src,
                       uint32_t              srcWidth,
                       uint32_t              srcHeight,
                       uint32_t              srcStride,
                       uint32_t              nContours,
                 const uint32_t*  __restrict holeFlag,
                 const uint32_t*  __restrict numContourPoints,
                 const uint32_t** __restrict contourStartPoints,
                       uint32_t              pointBufferSize,
                 const uint32_t*  __restrict pointBuffer,
                        int32_t              hierarchy[][4],
                       uint32_t              max_level,
                        int32_t              thickness,
                        uint8_t              colorR,
                        uint8_t              colorG,
                        uint8_t              colorB,
                        uint8_t              hole_colorR,
                        uint8_t              hole_colorG,
                        uint8_t              hole_colorB);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageMomentsu8( const uint8_t* __restrict   src,
                   uint32_t                    srcWidth,
                   uint32_t                    srcHeight,
                   uint32_t                    srcStride,
                   fcvMoments*                 moments,
                   uint8_t                     binary);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageMomentss32( const int32_t* __restrict src,
                    uint32_t                  srcWidth,
                    uint32_t                  srcHeight,
                    uint32_t                  srcStride,
                    fcvMoments*               moments,
                    uint8_t                   binary);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageMomentsf32(const float32_t* __restrict src,
                  uint32_t              srcWidth,
                  uint32_t              srcHeight,
                  uint32_t              srcStride,
                  fcvMoments*           moments,
                  uint8_t               binary);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API int32_t
fcvSolveLDLf32( float32_t *__restrict         A,
                const float32_t *__restrict   b,
                float32_t *__restrict         diag,
                uint32_t                      N,
                float32_t *__restrict         x );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API float32_t
fcvDotProductf32( const float32_t *__restrict a,
                  const float32_t *__restrict b,
                  uint32_t                    N);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------/

FASTCV_API void
fcv3ChannelTransformAffineClippedBCu8(  const uint8_t *__restrict      src,
                                        uint32_t                       srcWidth,
                                        uint32_t                       srcHeight,
                                        uint32_t                       srcStride,
                                        const float32_t *__restrict    affineMatrix,
                                        uint8_t *__restrict            dst,
                                        uint32_t                       dstWidth,
                                        uint32_t                       dstHeight,
                                        uint32_t                       dstStride,
                                        uint32_t *__restrict           dstBorder);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterThresholdOtsuu8( const uint8_t *__restrict   src,
                          uint32_t                    srcWidth,
                          uint32_t                    srcHeight,
                          uint32_t                    srcStride,
                          uint8_t *__restrict         dst,
                          uint32_t                    dstStride,
                          fcvThreshType               thresholdType);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageDetectEdgePixelsu8( const int16_t* __restrict  gxgy,
                            const uint32_t*__restrict  mag,
                            uint32_t                   gradStride,
                            uint32_t                   topLeftX,
                            uint32_t                   topLeftY,
                            uint32_t                   width,
                            uint32_t                   height,
                            uint32_t                   gridSize,
                            float32_t                  threshold,
                            uint32_t                   nEdgePixelsMax,
                            uint32_t* __restrict       nEdgePixels,
                            uint32_t* __restrict       coordEdgePixels );


//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvGLBPu8(const uint8_t *__restrict src,
           uint32_t srcWidth,
           uint32_t srcHeight,
           uint32_t srcStride,
           uint32_t radius,
           uint32_t neighbors,
           uint8_t *__restrict dst,
           uint32_t dstStride);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvCornerRefineSubPixu8( const uint8_t * __restrict src,
                         uint32_t        srcWidth,
                         uint32_t        srcHeight,
                         uint32_t        srcStride,
                         uint32_t        blockWidth,
                         uint32_t        blockHeight,
                         uint32_t        maxIterations,
                         float32_t       stopCriteria,
                         const uint32_t*__restrict  xyInitial,
                         uint32_t        nCorners,
                         float32_t * __restrict     xyOut);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvGoodFeatureToTracku8( const uint8_t * __restrict src,
                         uint32_t         srcWidth,
                         uint32_t         srcHeight,
                         uint32_t         srcStride,
                         float32_t        distanceMin,
                         uint32_t         border,
                         float32_t        barrier,
                         uint32_t * __restrict   xy,
                         uint32_t         maxnumcorners,
                         uint32_t * __restrict  numcorners);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFindMultipleMaximau8(const uint8_t *__restrict src,
                        uint32_t srcWidth,
                        uint32_t srcHeight,
                        uint32_t srcStride,
                        const float32_t* __restrict pos,
                        const float32_t* __restrict normal,
                        uint32_t maxDistance,
                        uint32_t maxNumMaxima,
                        int32_t minGradient,
                        float32_t maxAngleDiff,
                        float32_t* __restrict maxima,
                        uint32_t* __restrict numMaxima);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvImageDetectLineSegmentsu8(const fcvPyramidLevel_v2* __restrict srcPyr,
                             uint32_t pyrLevel,
                             uint32_t doBlurImage,
                             float32_t maxLineAngle,
                             uint32_t minLineLength,
                             uint32_t minMagnitude,
                             uint32_t maxLineNum,
                             uint32_t* __restrict indexBuffer,
                             fcvLineSegment* __restrict lineSegments,
                             uint32_t* __restrict numLineSegments);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfSquaredDiffsu8( const uint8_t* __restrict                 a,
                        float32_t                                 invLenA,
                        uint32_t                                  dim,
                        const uint8_t* const * __restrict         bList,
                        const float32_t* __restrict               invLenB,
                        uint32_t                                  numB,
                        float32_t* __restrict                     distances );


//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfSquaredDiffsf32( const float32_t* __restrict             a,
                         float32_t                               invLenA,
                         uint32_t                                dim,
                         const float32_t* const * __restrict     bList,
                         const float32_t* __restrict             invLenB,
                         uint32_t                                numB,
                         float32_t* __restrict                   distances );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API int
fcvClusterEuclideanu8( const uint8_t* __restrict     points,
                       int32_t                      numPoints,
                       int32_t                      dim,
                       int32_t                      pointStride,
                       int32_t                      numPointsUsed,
                       int32_t                      numClusters,
                       float32_t* __restrict        clusterCenters,
                       int32_t                      clusterCenterStride,
                       float32_t* __restrict        newClusterCenters,
                       uint32_t* __restrict         clusterMemberCounts,
                       uint32_t* __restrict         clusterBindings,
                       float32_t*                   sumOfClusterDistances );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvTransposeu8( const uint8_t * __restrict  src,
                uint32_t                    srcWidth,
                uint32_t                    srcHeight,
                uint32_t                    srcStride,
                uint8_t * __restrict        dst,
                uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvTransposeu16(  const uint16_t * __restrict  src,
                  uint32_t                     srcWidth,
                  uint32_t                     srcHeight,
                  uint32_t                     srcStride,
                  uint16_t * __restrict        dst,
                  uint32_t                     dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvTransposef32(  const float32_t * __restrict  src,
                  uint32_t                      srcWidth,
                  uint32_t                      srcHeight,
                  uint32_t                      srcStride,
                  float32_t * __restrict        dst,
                  uint32_t                      dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvFlipu8( const uint8_t * src,
           uint32_t        srcWidth,
           uint32_t        srcHeight,
           uint32_t        srcStride,
           uint8_t *       dst,
           uint32_t        dstStride,
           fcvFlipDir      dir );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvFlipu16( const uint16_t * src,
            uint32_t         srcWidth,
            uint32_t         srcHeight,
            uint32_t         srcStride,
            uint16_t *       dst,
            uint32_t         dstStride,
            fcvFlipDir       dir );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvFlipRGB888u8(const uint8_t * src,
                uint32_t  srcWidth,
                uint32_t srcHeight,
                uint32_t srcStride,
                uint8_t * dst,
                uint32_t dstStride,
                fcvFlipDir dir);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvRotateImageu8( const uint8_t *      src,
                  uint32_t             srcWidth,
                  uint32_t             srcHeight,
                  uint32_t             srcStride,
                  uint8_t *            dst,
                  uint32_t             dstStride,
                  fcvRotateDegree      degree );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvRotateImageInterleavedu8( const uint8_t *      src,
                             uint32_t             srcWidth,
                             uint32_t             srcHeight,
                             uint32_t             srcStride,
                             uint8_t *            dst,
                             uint32_t             dstStride,
                             fcvRotateDegree      degree );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvElementMultiplyu8u16( const uint8_t *             src1,
                         uint32_t                    width,
                         uint32_t                    height,
                         uint32_t                    src1Stride,
                         const uint8_t *             src2,
                         uint32_t                    src2Stride,
                         uint16_t * __restrict       dst,
                         uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvElementMultiplyf32(  const float32_t *            src1,
                        uint32_t                     width,
                        uint32_t                     height,
                        uint32_t                     src1Stride,
                        const float32_t *            src2,
                        uint32_t                     src2Stride,
                        float32_t * __restrict       dst,
                        uint32_t                     dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvMatrixMultiplys8s32( const int8_t * __restrict  src1,
                        uint32_t                   src1Width,
                        uint32_t                   src1Height,
                        uint32_t                   src1Stride,
                        const int8_t * __restrict  src2,
                        uint32_t                   src2Width,
                        uint32_t                   src2Stride,
                        int32_t * __restrict       dst,
                        uint32_t                   dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvMatrixMultiplyf32( const float32_t * __restrict   src1,
                      uint32_t                       src1Width,
                      uint32_t                       src1Height,
                      uint32_t                       src1Stride,
                      const float32_t * __restrict   src2,
                      uint32_t                       src2Width,
                      uint32_t                       src2Stride,
                      float32_t * __restrict         dst,
                      uint32_t                       dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API uint32_t
fcvBlockDotProductu8( const uint8_t * __restrict  src1,
                      uint32_t                    blockWidth,
                      uint32_t                    blockHeight,
                      uint32_t                    src1Stride,
                      const uint8_t * __restrict  src2,
                      uint32_t                    src2Stride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API float32_t
fcvBlockDotProductf32( const float32_t * __restrict  src1,
                       uint32_t                      blockWidth,
                       uint32_t                      blockHeight,
                       uint32_t                      src1Stride,
                       const float32_t * __restrict  src2,
                       uint32_t                      src2Stride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvAddu8u16( const uint8_t * __restrict  src1,
             uint32_t                    width,
             uint32_t                    height,
             uint32_t                    src1Stride,
             const uint8_t * __restrict  src2,
             uint32_t                    src2Stride,
             uint16_t * __restrict       dst,
             uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvAdds16( const int16_t * __restrict  src1,
           uint32_t                    width,
           uint32_t                    height,
           uint32_t                    src1Stride,
           const int16_t * __restrict  src2,
           uint32_t                    src2Stride,
           int16_t * __restrict        dst,
           uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API void
fcvAddf32( const float32_t * __restrict  src1,
           uint32_t                      width,
           uint32_t                      height,
           uint32_t                      src1Stride,
           const float32_t * __restrict  src2,
           uint32_t                      src2Stride,
           float32_t * __restrict        dst,
           uint32_t                      dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegrateImageu8u64( const uint8_t* __restrict src,
                        uint32_t                  srcWidth,
                        uint32_t                  srcHeight,
                        uint32_t                  srcStride,
                        uint32_t* __restrict      dstIntgrl,
                        uint64_t* __restrict      dstIntgrlSqrd,
                        uint32_t                  dstIntgrlStride,
                        uint32_t                  dstIntgrlSqrdStride);




//------------------------------------------------------------------------------
//------------------------------------------------------------------------------


FASTCV_API void
fcvImageHistogramEqualizeu8(  const uint8_t * __restrict src,
                              uint32_t                   srcWidth,
                              uint32_t                   srcHeight,
                              uint32_t                   srcStride,
                              uint8_t * __restrict       dst,
                              uint32_t                   dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageSpatialHistogramu8(const uint8_t *__restrict src,
                           uint32_t                  srcWidth,
                           uint32_t                  srcHeight,
                           uint32_t                  srcStride,
                           uint32_t                  numPatterns,
                           uint32_t                  grid_x,
                           uint32_t                  grid_y,
                           float32_t*__restrict      histogram);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterSobel3x3u8s16( const uint8_t* __restrict  src,
                        uint32_t              srcWidth,
                        uint32_t              srcHeight,
                        uint32_t              srcStride,
                        int16_t* __restrict  dx,
                        int16_t* __restrict  dy,
                        uint32_t              dxyStride,
                        fcvBorderType borderType,
                        uint8_t  borderValue);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterSobel5x5u8s16( const uint8_t* __restrict  src,
                     uint32_t              srcWidth,
                     uint32_t              srcHeight,
                     uint32_t              srcStride,
                     int16_t* __restrict  dx,
                     int16_t* __restrict  dy,
                     uint32_t              dxyStride,
                     fcvBorderType borderType,
                     uint8_t  borderValue);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterSobel7x7u8s16( const uint8_t* __restrict  src,
                     uint32_t              srcWidth,
                     uint32_t              srcHeight,
                     uint32_t              srcStride,
                     int16_t* __restrict  dx,
                     int16_t* __restrict  dy,
                     uint32_t              dxyStride,
                     fcvBorderType borderType,
                     uint8_t  borderValue);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterCannyu8( const uint8_t* __restrict src,
                  uint32_t srcWidth,
                  uint32_t srcHeight,
                  uint32_t srcStride,
                  uint8_t  kernelSize,
                  int32_t lowThresh,
                  int32_t highThresh,
                  fcvNormType normType,
                  uint8_t* __restrict dst,
                  uint32_t dstStride,
                  int16_t* __restrict gx,
                  int16_t* __restrict gy,
                  uint32_t gradStride);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvConvertDepthu8s16(const uint8_t *__restrict  src,
                     uint32_t                   srcWidth,
                     uint32_t                   srcHeight,
                     uint32_t                   srcStride,
                     uint8_t                    shift,
                     int16_t *__restrict        dst,
                     uint32_t                   dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvConvertDepths16u8(const int16_t *__restrict      src,
                     uint32_t                   srcWidth,
                     uint32_t                   srcHeight,
                     uint32_t                   srcStride,
                     uint8_t                        shift,
                     fcvConvertPolicy           policy,
                     uint8_t *__restrict            dst,
                     uint32_t                   dstStride );


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvBilateralFilterRecursiveu8(const uint8_t* __restrict src,
                              uint32_t                  srcWidth,
                              uint32_t                  srcHeight,
                              uint32_t                  srcStride,
                              uint8_t* __restrict       dst,
                              uint32_t                  dstStride,
                              float32_t                 sigmaColor,
                              float32_t                 sigmaSpace );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvImageSegmentationSeedRegionGrows16( const int16_t* __restrict src,
                                       uint32_t srcWidth,
                                       uint32_t srcHeight,
                                       uint32_t srcStride,
                                       uint32_t numChannel,
                                       uint32_t threshGrow,
                                       const uint32_t* __restrict pointVector,
                                       uint32_t numSeed,
                                       uint8_t mode,
                                       uint32_t* __restrict segLabel,
                                       uint32_t segLabelStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvNormalizeLocalBoxu8( const uint8_t * __restrict src,
                        uint32_t                   srcWidth,
                        uint32_t                   srcHeight,
                        uint32_t                   srcStride,
                        uint32_t                   patchWidth,
                        uint32_t                   patchHeight,
                        uint32_t                   useStdDev,
                        int8_t * __restrict        dst,
                        uint32_t                   dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvNormalizeLocalBoxf32( const float32_t * __restrict src,
                         uint32_t                     srcWidth,
                         uint32_t                     srcHeight,
                         uint32_t                     srcStride,
                         uint32_t                     patchWidth,
                         uint32_t                     patchHeight,
                         uint32_t                     useStdDev,
                         float32_t * __restrict       dst,
                         uint32_t                     dstStride);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvChannelCombine2Planesu8( const uint8_t *__restrict  src1,
                            uint32_t                   width,
                            uint32_t                   height,
                            uint32_t                   src1Stride,
                            const uint8_t *__restrict  src2,
                            uint32_t                   src2Stride,
                            uint8_t *__restrict        dst,
                            uint32_t                   dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvChannelCombine3Planesu8(const uint8_t *__restrict  src1,
                           uint32_t                   width,
                           uint32_t                   height,
                           uint32_t                   src1Stride,
                           const uint8_t *__restrict  src2,
                           uint32_t                   src2Stride,
                           const uint8_t *__restrict  src3,
                           uint32_t                   src3Stride,
                           uint8_t *__restrict        dst,
                           uint32_t                   dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvChannelCombine4Planesu8(const uint8_t *__restrict    src1,
                            uint32_t                      width,
                            uint32_t                   height,
                            uint32_t                   src1Stride,
                            const uint8_t *__restrict  src2,
                            uint32_t                   src2Stride,
                            const uint8_t *__restrict  src3,
                            uint32_t                   src3Stride,
                            const uint8_t *__restrict  src4,
                            uint32_t                   src4Stride,
                            uint8_t *__restrict        dst,
                            uint32_t                   dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvChannelExtractu8( const uint8_t *__restrict  src1,
                     uint32_t                   srcWidth,
                     uint32_t                     srcHeight,
                     uint32_t                     src1Stride,
                     const uint8_t *__restrict  src2,
                     uint32_t                     src2Stride,
                     const uint8_t *__restrict  src3,
                     uint32_t                     src3Stride,
                     fcvChannelType             srcChannel,
                     fcvImageFormat             srcFormat,
                     uint8_t *__restrict        dst,
                     uint32_t                   dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterMedianMxNs16( const int16_t * __restrict src,
                       uint32_t srcWidth,
                       uint32_t srcHeight,
                       uint32_t srcStride,
                       uint32_t M,
                       uint32_t N,
                       int16_t* __restrict dst,
                       uint32_t dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterConvolveMxNu8s16( const int16_t* __restrict  kernel,
                           uint32_t M,
                           uint32_t N,
                           int8_t shift,
                           const uint8_t* __restrict src,
                           uint32_t srcWidth,
                           uint32_t srcHeight,
                           uint32_t srcStride,
                           int16_t* __restrict dst,
                           uint32_t dstStride,
                           fcvBorderType borderType,
                           uint8_t borderValue );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterConvolveMxNu8(const int16_t* __restrict kernel,
                       uint32_t M,
                       uint32_t N,
                       int8_t shift,
                       const uint8_t* __restrict src,
                       uint32_t srcWidth,
                       uint32_t srcHeight,
                       uint32_t srcStride,
                       uint8_t* __restrict dst,
                       uint32_t dstStride,
                       fcvBorderType borderType,
                       uint8_t  borderValue);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvBoxFilter3x3u8_v2( const uint8_t* __restrict src,
                        uint32_t                srcWidth,
                        uint32_t                srcHeight,
                        uint32_t                srcStride,
                        uint8_t* __restrict     dst,
                        uint32_t                dstStride,
                        fcvBorderType           borderType,
                        uint8_t                 borderValue);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterErode3x3u8_v3( const uint8_t* __restrict src,
                        uint32_t                srcWidth,
                        uint32_t                srcHeight,
                        uint32_t                srcStride,
                        uint8_t* __restrict     dst,
                        uint32_t                dstStride,
                        fcvBorderType           borderType,
                        uint8_t                 borderValue);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterErodeNxNu8(  const uint8_t* __restrict src,
                      uint32_t                  srcWidth,
                      uint32_t                  srcHeight,
                      uint32_t                  srcStride,
                      uint32_t                  N,
                      uint8_t* __restrict       dst,
                      uint32_t                  dstStride);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterDilate3x3u8_v3( const uint8_t* __restrict src,
                        uint32_t                srcWidth,
                        uint32_t                srcHeight,
                        uint32_t                srcStride,
                        uint8_t* __restrict     dst,
                        uint32_t                dstStride,
                        fcvBorderType           borderType,
                        uint8_t                 borderValue);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterDilateNxNu8(  const uint8_t* __restrict src,
                       uint32_t                  srcWidth,
                       uint32_t                  srcHeight,
                       uint32_t                  srcStride,
                       uint32_t                  N,
                       uint8_t* __restrict       dst,
                       uint32_t                  dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterGaussian3x3u8_v3( const uint8_t* __restrict src,
                        uint32_t                srcWidth,
                        uint32_t                srcHeight,
                        uint32_t                srcStride,
                        uint8_t* __restrict     dst,
                        uint32_t                dstStride,
                        fcvBorderType           borderType,
                        uint8_t                 borderValue);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFilterMedian3x3u8_v3( const uint8_t* __restrict src,
                        uint32_t                srcWidth,
                        uint32_t                srcHeight,
                        uint32_t                srcStride,
                        uint8_t* __restrict     dst,
                        uint32_t                dstStride,
                        fcvBorderType           borderType,
                        uint8_t                 borderValue);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvImageSpatialHistogramu8_v2( const uint8_t *__restrict src,
                        uint32_t                  srcWidth,
                        uint32_t                  srcHeight,
                        uint32_t                  srcStride,
                        uint32_t                  numPatterns,
                        uint32_t                  grid_x,
                        uint32_t                  grid_y,
                        float32_t*__restrict      histogram,
                        float32_t                 normalize_factor);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownBLu8( const uint8_t* __restrict  src,
                  uint32_t                   srcWidth,
                  uint32_t                   srcHeight,
                  uint32_t                   srcStride,
                  uint8_t* __restrict        dst,
                  uint32_t                   dstWidth,
                  uint32_t                   dstHeight,
                  uint32_t                   dstStride);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvTableLookupu8( const uint8_t *__restrict src,
                  uint32_t                  srcWidth,
                  uint32_t                  srcHeight,
                  uint32_t                  srcStride,
                  const uint8_t *__restrict lut,
                  uint8_t *__restrict           dst,
                  uint32_t                  dstStride );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvWarpPerspectiveu8_v3( const uint8_t *__restrict  src,
                         uint32_t  srcWidth,
                         uint32_t  srcHeight,
                         uint32_t  srcStride,
                         uint8_t *__restrict  dst,
                         uint32_t  dstWidth,
                         uint32_t  dstHeight,
                         uint32_t  dstStride,
                         float *__restrict  projectionMatrix,
                         fcvInterpolationType  interpolation );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvWarpPerspectiveu8_v4( const uint8_t *__restrict  src,
                         uint32_t  srcWidth,
                         uint32_t  srcHeight,
                         uint32_t  srcStride,
                         uint8_t *__restrict  dst,
                         uint32_t  dstWidth,
                         uint32_t  dstHeight,
                         uint32_t  dstStride,
                         float *__restrict  projectionMatrix,
                         fcvInterpolationType  interpolation,
                         fcvBorderType borderType,
                         uint8_t borderValue
);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvRemapu8(const uint8_t*  __restrict src,
           uint32_t             srcWidth,
           uint32_t             srcHeight,
           uint32_t             srcStride,
           uint8_t* __restrict dst,
           uint32_t             dstWidth,
           uint32_t             dstHeight,
           uint32_t             dstStride,
           const float32_t* __restrict mapX,
           const float32_t* __restrict mapY,
           uint32_t             mapStride,
           fcvInterpolationType  interpolation
);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvRemapu8_v2(const uint8_t*  __restrict src,
           uint32_t             srcWidth,
           uint32_t             srcHeight,
           uint32_t             srcStride,
           uint8_t* __restrict dst,
           uint32_t             dstWidth,
           uint32_t             dstHeight,
           uint32_t             dstStride,
           const float32_t* __restrict mapX,
           const float32_t* __restrict mapY,
           uint32_t             mapStride,
           fcvInterpolationType  interpolation,
           fcvBorderType borderType,
           uint8_t borderValue
);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvMagnitudes16( const int16_t *__restrict  src1,
                  uint32_t                  width,
                  uint32_t                  height,
                  uint32_t                  src1Stride,
                  const int16_t *__restrict src2,
                  uint32_t                  src2Stride,
                  int16_t *__restrict       dst,
                  uint32_t                  dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvPhases16 (  const int16_t *__restrict  src1,
                uint32_t                   width,
                uint32_t                   height,
                uint32_t                   src1Stride,
                const int16_t *__restrict  src2,
                uint32_t                   src2Stride,
                uint8_t *__restrict        dst,
                uint32_t                   dstStride );


//--------------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvFFTu8(const uint8_t* __restrict src,
         uint32_t                  srcWidth,
         uint32_t                  srcHeight,
         uint32_t                  srcStride,
         float32_t* __restrict     dst,
         uint32_t                  dstStride);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvIFFTf32( const float32_t* __restrict src,
            uint32_t                     srcWidth,
            uint32_t                     srcHeight,
            uint32_t                     srcStride,
            uint8_t* __restrict          dst,
            uint32_t                     dstStride );

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvScaleu8( const uint8_t* __restrict src,
            uint32_t srcWidth,
            uint32_t srcHeight,
            uint32_t srcStride,
            uint8_t* __restrict dst,
            uint32_t dstWidth,
            uint32_t dstHeight,
            uint32_t dstStride,
            fcvInterpolationType interpolation);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvScaleu8_v2( const uint8_t* __restrict src,
               uint32_t srcWidth,
               uint32_t srcHeight,
               uint32_t srcStride,
               uint8_t* __restrict dst,
               uint32_t dstWidth,
               uint32_t dstHeight,
               uint32_t dstStride,
               fcvInterpolationType interpolation,
               fcvBorderType borderType,
               uint8_t borderValue);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcv2PlaneWarpPerspectiveu8( const uint8_t* __restrict src1,
                            const uint8_t* __restrict src2,
                            uint32_t srcWidth,
                            uint32_t srcHeight,
                            uint32_t src1Stride,
                            uint32_t src2Stride,
                            uint8_t* __restrict dst1,
                            uint8_t* __restrict dst2,
                            uint32_t dstWidth,
                            uint32_t dstHeight,
                            uint32_t dst1Stride,
                            uint32_t dst2Stride,
                            float32_t* __restrict warpmatrix );

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvScaleDownBy2Gaussian3x3u8(const uint8_t* __restrict src,
                             uint32_t   srcWidth,
                             uint32_t   srcHeight,
                             uint32_t   srcStride,
                             uint8_t* __restrict       dst,
                             uint32_t   dstStride);

//---------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfSquaredDiffss8( const int8_t* __restrict                a,
                        float32_t                               invLenA,
                        uint32_t                                dim,
                        const int8_t* const * __restrict        bList,
                        const float32_t* __restrict             invLenB,
                        uint32_t                                numB,
                        float32_t* __restrict                   distances );



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvAddScalarf32(const float32_t * __restrict  src,
                uint32_t srcWidth,
                uint32_t srcHeight,
                uint32_t srcStride,
                float32_t  scalar,
                float32_t * __restrict dst,
                uint32_t  dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvAddScalars16(const int16_t * __restrict  src,
                uint32_t srcWidth,
                uint32_t srcHeight,
                uint32_t srcStride,
                int16_t  scalar,
                int16_t * __restrict dst,
                uint32_t  dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvMultiplyScalarf32(const float32_t * __restrict  src,
                     uint32_t srcWidth,
                     uint32_t srcHeight,
                     uint32_t srcStride,
                     float32_t  scalar,
                     float32_t * __restrict dst,
                     uint32_t  dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvMultiplyScalars16( const int16_t * __restrict  src,
                      uint32_t srcWidth,
                      uint32_t srcHeight,
                      uint32_t srcStride,
                      int8_t  scalar,
                      int8_t shift,
                      int16_t * __restrict dst,
                      uint32_t  dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvMinMaxLocu8(const uint8_t *__restrict src,
               uint32_t srcWidth,
               uint32_t srcHeight,
               uint32_t srcStride,
               uint8_t *__restrict minVal,
               uint8_t *__restrict maxVal,
               uint32_t *__restrict minLocX,
               uint32_t *__restrict minLocY,
               uint32_t *__restrict maxLocX,
               uint32_t *__restrict maxLocY);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvMinMaxLocu16(const uint16_t *__restrict src,
                uint32_t srcWidth,
                uint32_t srcHeight,
                uint32_t srcStride,
                uint16_t *__restrict minVal,
                uint16_t *__restrict maxVal,
                uint32_t *__restrict minLocX,
                uint32_t *__restrict minLocY,
                uint32_t *__restrict maxLocX,
                uint32_t *__restrict maxLocY);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvMinMaxLocs16(const int16_t *__restrict src,
                uint32_t srcWidth,
                uint32_t srcHeight,
                uint32_t srcStride,
                int16_t *__restrict minVal,
                int16_t *__restrict maxVal,
                uint32_t *__restrict minLocX,
                uint32_t *__restrict minLocY,
                uint32_t *__restrict maxLocX,
                uint32_t *__restrict maxLocY);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvMinMaxLocu32(const uint32_t *__restrict src,
                uint32_t srcWidth,
                uint32_t srcHeight,
                uint32_t srcStride,
                uint32_t *__restrict minVal,
                uint32_t *__restrict maxVal,
                uint32_t *__restrict minLocX,
                uint32_t *__restrict minLocY,
                uint32_t *__restrict maxLocX,
                uint32_t *__restrict maxLocY);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvMinMaxLocs32(const int32_t *__restrict src,
                uint32_t srcWidth,
                uint32_t srcHeight,
                uint32_t srcStride,
                int32_t *__restrict minVal,
                int32_t *__restrict maxVal,
                uint32_t *__restrict minLocX,
                uint32_t *__restrict minLocY,
                uint32_t *__restrict maxLocX,
                uint32_t *__restrict maxLocY);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvMinMaxLocf32(const float32_t *__restrict src,
                uint32_t srcWidth,
                uint32_t srcHeight,
                uint32_t srcStride,
                float32_t *__restrict minVal,
                float32_t *__restrict maxVal,
                uint32_t *__restrict minLocX,
                uint32_t *__restrict minLocY,
                uint32_t *__restrict maxLocX,
                uint32_t *__restrict maxLocY);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvMinMaxLocf32_v2(const float32_t *__restrict src,
                   uint32_t                    srcWidth,
                   uint32_t                    srcHeight,
                   uint32_t                    srcStride,
                   float32_t *__restrict       minVal,
                   float32_t *__restrict       maxVal,
                   uint32_t *__restrict        minLocX,
                   uint32_t *__restrict        minLocY,
                   uint32_t *__restrict        maxLocX,
                   uint32_t *__restrict        maxLocY,
                   uint32_t *__restrict        minCount,
                   uint32_t *__restrict        maxCount,
                   uint32_t                    nMinLocSize,
                   uint32_t                    nMaxLocSize);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvMinMaxLocu8_v2(const uint8_t *__restrict src,
                  uint32_t                  srcWidth,
                  uint32_t                  srcHeight,
                  uint32_t                  srcStride,
                  uint8_t *__restrict       minVal,
                  uint8_t *__restrict       maxVal,
                  uint32_t *__restrict      minLocX,
                  uint32_t *__restrict      minLocY,
                  uint32_t *__restrict      maxLocX,
                  uint32_t *__restrict      maxLocY,
                  uint32_t *__restrict      minCount,
                  uint32_t *__restrict      maxCount,
                  uint32_t                  nMinLocSize,
                  uint32_t                  nMaxLocSize);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvMinMaxLocu16_v2(const uint16_t *__restrict src,
                   uint32_t                   srcWidth,
                   uint32_t                   srcHeight,
                   uint32_t                   srcStride,
                   uint16_t *__restrict       minVal,
                   uint16_t *__restrict       maxVal,
                   uint32_t *__restrict       minLocX,
                   uint32_t *__restrict       minLocY,
                   uint32_t *__restrict       maxLocX,
                   uint32_t *__restrict       maxLocY,
                   uint32_t *__restrict       minCount,
                   uint32_t *__restrict       maxCount,
                   uint32_t                   nMinLocSize,
                   uint32_t                   nMaxLocSize);
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvMinMaxLocs16_v2(const int16_t *__restrict src,
                   uint32_t                  srcWidth,
                   uint32_t                  srcHeight,
                   uint32_t                  srcStride,
                   int16_t *__restrict       minVal,
                   int16_t *__restrict       maxVal,
                   uint32_t *__restrict      minLocX,
                   uint32_t *__restrict      minLocY,
                   uint32_t *__restrict      maxLocX,
                   uint32_t *__restrict      maxLocY,
                   uint32_t *__restrict      minCount,
                   uint32_t *__restrict      maxCount,
                   uint32_t                  nMinLocSize,
                   uint32_t                  nMaxLocSize);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvMinMaxLocu32_v2(const uint32_t *__restrict src,
                   uint32_t                   srcWidth,
                   uint32_t                   srcHeight,
                   uint32_t                   srcStride,
                   uint32_t *__restrict       minVal,
                   uint32_t *__restrict       maxVal,
                   uint32_t *__restrict       minLocX,
                   uint32_t *__restrict       minLocY,
                   uint32_t *__restrict       maxLocX,
                   uint32_t *__restrict       maxLocY,
                   uint32_t *__restrict       minCount,
                   uint32_t *__restrict       maxCount,
                   uint32_t                   nMinLocSize,
                   uint32_t                   nMaxLocSize);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvMinMaxLocs32_v2(const int32_t *__restrict src,
                   uint32_t                  srcWidth,
                   uint32_t                  srcHeight,
                   uint32_t                  srcStride,
                   int32_t *__restrict       minVal,
                   int32_t *__restrict       maxVal,
                   uint32_t *__restrict      minLocX,
                   uint32_t *__restrict      minLocY,
                   uint32_t *__restrict      maxLocX,
                   uint32_t *__restrict      maxLocY,
                   uint32_t *__restrict      minCount,
                   uint32_t *__restrict      maxCount,
                   uint32_t                  nMinLocSize,
                   uint32_t                  nMaxLocSize);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvTransposeRGB888u8(const uint8_t * __restrict src,
                     uint32_t srcWidth,
                     uint32_t srcHeight,
                     uint32_t srcStride,
                     uint8_t * __restrict dst,
                     uint32_t dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvCrossProduct3x1f32(const float32_t * __restrict  a,
                      const float32_t * __restrict  b,
                      float32_t * __restrict  c,
                      uint32_t N);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvSolveLUf32(float32_t* __restrict A,
              float32_t* __restrict b,
              uint32_t             N,
              uint8_t* __restrict pivot,
              float32_t* __restrict x);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvBitwiseAndu8(const uint8_t *             src1,
                uint32_t                    width,
                uint32_t                    height,
                uint32_t                    src1Stride,
                const uint8_t *__restrict   src2,
                uint32_t                    src2Stride,
                uint8_t *                   dst,
                uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvBitwiseXoru8( const uint8_t *                src1,
                    uint32_t                    width,
                    uint32_t                    height,
                    uint32_t                    src1Stride,
                    const uint8_t *__restrict   src2,
                    uint32_t                    src2Stride,
                    uint8_t *                   dst,
                    uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvBitwiseNotu8(const uint8_t *                 src,
                    uint32_t                    width,
                    uint32_t                    height,
                    uint32_t                    srcStride,
                    uint8_t *                   dst,
                    uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvAddu8(const uint8_t *                src1,
         uint32_t                    width,
         uint32_t                    height,
         uint32_t                    src1Stride,
         const uint8_t * __restrict  src2,
         uint32_t                    src2Stride,
         fcvConvertPolicy            policy,
         uint8_t *                   dst,
         uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvAdds16_v2( const int16_t *              src1,
                uint32_t                    width,
                uint32_t                    height,
                uint32_t                    src1Stride,
                const int16_t * __restrict  src2,
                uint32_t                    src2Stride,
                fcvConvertPolicy            policy,
                int16_t *                   dst,
                uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvAddu16u8u16 (const uint16_t *            src1,
                uint32_t                    width,
                uint32_t                    height,
                uint32_t                    src1Stride,
                const uint8_t * __restrict  src2,
                uint32_t                    src2Stride,
                fcvConvertPolicy            policy,
                uint16_t *                  dst,
                uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvSubtractu8 ( const uint8_t *             src1,
                uint32_t                    width,
                uint32_t                    height,
                uint32_t                    src1Stride,
                const uint8_t * __restrict  src2,
                uint32_t                    src2Stride,
                fcvConvertPolicy            policy,
                uint8_t *                   dst,
                uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvSubtracts16 (const int16_t *             src1,
                uint32_t                    width,
                uint32_t                    height,
                uint32_t                    src1Stride,
                const int16_t * __restrict  src2,
                uint32_t                    src2Stride,
                fcvConvertPolicy            policy,
                int16_t *                   dst,
                uint32_t                    dstStride );

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvSubtractu8s16 ( const uint8_t * __restrict   src1,
                    uint32_t                    width,
                    uint32_t                    height,
                    uint32_t                    src1Stride,
                    const uint8_t * __restrict  src2,
                    uint32_t                    src2Stride,
                    int16_t *   __restrict      dst,
                    uint32_t                    dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvElementMultiplys16( const int16_t *       src1,
                       uint32_t              width,
                       uint32_t              height,
                       uint32_t              src1Stride,
                       const int16_t *       src2,
                       uint32_t              src2Stride,
                       int8_t                scaleFactor,
                       fcvConvertPolicy      policy,
                       int16_t *__restrict   dst,
                       uint32_t              dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvElementMultiplyu8s16( const uint8_t *        src1,
                         uint32_t               width,
                         uint32_t               height,
                         uint32_t               src1Stride,
                         const uint8_t *        src2,
                         uint32_t               src2Stride,
                         int8_t                 scaleFactor,
                         fcvConvertPolicy       policy,
                         int16_t *__restrict    dst,
                         uint32_t               dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvElementMultiplyu8( const uint8_t *        src1,
                      uint32_t               width,
                      uint32_t               height,
                      uint32_t               src1Stride,
                      const uint8_t *        src2,
                      uint32_t               src2Stride,
                      int8_t                 scaleFactor,
                      fcvConvertPolicy       policy,
                      uint8_t *__restrict    dst,
                      uint32_t               dstStride );

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvAddWeightedu8(const uint8_t *           src1,
                 uint32_t                  srcWidth,
                 uint32_t                  srcHeight,
                 uint32_t                  src1Stride,
                 const uint8_t *__restrict src2,
                 uint32_t                  src2Stride,
                 float32_t                 alpha,
                 float32_t                 beta,
                 uint8_t *                 dst,
                 uint32_t                  dstStride);


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvAddSquaredu8u16(const uint8_t *__restrict    src1,
                    uint32_t                    width,
                    uint32_t                    height,
                    uint32_t                    src1Stride,
                    const uint16_t *            src2,
                    uint32_t                    src2Stride,
                    int8_t                      scaleFactor,
                    uint16_t *                  dst,
                    uint32_t                    dstStride);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorCbCrSwapu8( const uint8_t* __restrict src,
                    uint32_t                  srcWidth,
                    uint32_t                  srcHeight,
                    uint32_t                  srcStride,
                    uint8_t* __restrict       dst,
                    uint32_t                  dstStride );


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvColorRGB888ToHSV888u8( const uint8_t* __restrict src,
                          uint32_t                  srcWidth,
                          uint32_t                  srcHeight,
                          uint32_t                  srcStride,
                          uint8_t* __restrict       dst,
                          uint32_t                  dstStride);


//------------------------------------------------------------------------------/
//------------------------------------------------------------------------------/

FASTCV_API fcvStatus
fcvSVMPredict2Classf32( fcvSVMKernelType    kernelType,
                        uint32_t  degree,
                        float32_t gamma,
                        float32_t coef0,
                        const float32_t* __restrict sv,
                        uint32_t  svLen,
                        uint32_t  svNum,
                        uint32_t  svStride,
                        const float32_t* __restrict svCoef,
                        float32_t  rho,
                        const float32_t*  __restrict vec,
                        uint32_t  vecNum,
                        uint32_t  vecStride,
                        float32_t* __restrict confidence );


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvTrackLKOpticalFlowu8_v3 ( const uint8_t *__restrict              src1,
                            const uint8_t *__restrict               src2,
                            uint32_t                                width,
                            uint32_t                                height,
                            uint32_t                                stride,
                            const fcvPyramidLevel_v2 * __restrict   src1Pyr,
                            const fcvPyramidLevel_v2 * __restrict   src2Pyr,
                            const float32_t *  __restrict           featureXY,
                            const float32_t *  __restrict           featureXY_estimate,
                            float32_t * __restrict                  featureXY_out,
                            int32_t * __restrict                    featureStatus,
                            int32_t                                 featureLen,
                            int32_t                                 windowWidth,
                            int32_t                                 windowHeight,
                            int32_t                                 nPyramidLevels,
                            fcvTerminationCriteria                  termCriteria,
                            int32_t                                 maxIterations,
                            float32_t                               maxEpsilon,
                            int32_t                                 use_initial_estimate);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvUndistortDisparityConvertDepthf32(const float32_t* __restrict  src,
                                     uint32_t                     srcWidth,
                                     uint32_t                     srcHeight,
                                     uint32_t                     srcStride,
                                     const uint8_t* __restrict    mask,
                                     uint32_t                     maskStride,
                                     const float32_t* __restrict  pixelDistortion,
                                     uint32_t                     pixelDistortionStride,
                                     uint32_t                     convertDepth,
                                     const float32_t* __restrict  imageDistortion,
                                     const float32_t* __restrict  depthParam,
                                     float32_t* __restrict        dst,
                                     uint32_t                     dstStride);


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvRegisterDepthImagef32(const float32_t* __restrict src,
                         uint32_t                    srcWidth,
                         uint32_t                    srcHeight,
                         uint32_t                    srcStride,
                         const float32_t* __restrict Kdinv,
                         const float32_t* __restrict Kc,
                         const float32_t* __restrict Rd2c,
                         const float32_t* __restrict Td2c,
                         float32_t* __restrict       dst,
                         uint32_t                    dstStride);

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
FASTCV_API fcvStatus
fcvConvertDepthImageToPointCloudf32(const float32_t* __restrict src,
                                    uint32_t                    srcWidth,
                                    uint32_t                    srcHeight,
                                    uint32_t                    srcStride,
                                    const float32_t* __restrict Kdinv,
                                    float32_t* __restrict       dst,
                                    uint32_t                    dstStride);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvExtractHOGu16(const uint16_t* __restrict strength,
                 uint32_t                   width,
                 uint32_t                   height,
                 uint32_t                   strengthStride,
                 const uint16_t* __restrict orientation,
                 uint32_t                   orientationStride,
                 uint32_t                   cellSize,
                 uint32_t                   blockSize,
                 uint32_t                   blockStep,
                 uint32_t                   binSize,
                 fcvHOGNormMethod           normMethod,
                 uint16_t* __restrict       hogVector,
                 uint32_t                   flen,
                 void*                      handle);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvHOGInit(uint32_t width,
                         uint32_t         height,
                         uint32_t         cellSize,
                         uint32_t         blockSize,
                         uint32_t         blockStep,
                         uint32_t         binSize,
                         fcvHOGNormMethod normMethod,
                        uint32_t *vecLength,
                        void     **hogHandle
                        );
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

FASTCV_API void fcvHOGDeInit(void* hogHandle);



#ifdef __cplusplus
}//extern "C"
#endif

#endif
