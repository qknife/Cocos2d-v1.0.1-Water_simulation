#import "WorldSceneProtocol.h"
#import "Common.h"
#import "FluidHashList.h"
#import <Foundation/Foundation.h>

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "vector_inline.h"

#define DWORD int

#define hashWidth	   	      (30)
#define hashHeight		      (30)
#define eps                   0.001

typedef float tScalar;

template<class T>
class tArray2D
{
public:
    tArray2D(int mNx = 1, int mNy = 1) : mNx(mNx), mNy(mNy), mData(new T[mNx*mNy]), mWrap(false)
    {};
    tArray2D(int mNx, int mNy, const T & val);
    tArray2D(const tArray2D & orig);
    
    ~tArray2D() {delete [] mData; mData = 0;}
    
    tArray2D<T> & operator=(const tArray2D & rhs);
    
    /// allows resizing. Data will be lost if resizing occurred
    void Resize(unsigned nx, unsigned ny) {
        if (nx == mNx && ny == mNy) return;
        //delete [] mData;
        mData = new T[nx*ny]; mNx = nx; mNy = ny;}
    
    /// enables/disables wrapping
    void SetWrap(bool wrap) {mWrap = wrap;}
    
    //! Unchecked access - no wrapping
    T & operator()(unsigned int i, unsigned int j) {
        return mData[i + j*mNx];}
    //! Unchecked const access
    const T & operator()(unsigned int i, unsigned int j) const {
        return mData[i + j*mNx];}
    //! checked access - unwraps if wrapping set
    T & At(int i, int j);
    //! checked const access
    const T & At(int i, int j) const;
    
    /// Set to a constant value
    void SetTo(T val);
    
    // interpolate
    T Interpolate(float fi, float fj);
    
    //! Calculate gradient using centred differences in the x dir
    void CalcGradientX();
    //! Calculate gradient using centred differences in the y dir
    void CalcGradientY();
    
    // sets each value to its absolute value by comparison with T(0)
    void Abs();
    
    // requires T::operator<(...)
    T GetMin() const;
    T GetMax() const;
    
    // scale to fit within range...
    void SetRange(T val_min, T val_max);
    
    //! add
    tArray2D<T> & Add(const tArray2D<T> & rhs);
    //! add scalar
    template <class T1>
    tArray2D<T> & operator+=(const T1 & rhs)
    {
        for (unsigned int i = 0 ; i < mNx*mNy ; ++i) mData[i] += rhs;
        return *this;
    }
    
    //! subtract
    tArray2D<T> & Subtract(const tArray2D<T> & rhs);
    //! subtract scalar
    template <class T1>
    tArray2D<T> & operator-=(const T1 & rhs)
    {
        for (unsigned int i = 0 ; i < mNx*mNy ; ++i) mData[i] -= rhs;
        return *this;
    }
    
    //! multiply
    tArray2D<T> & Multiply(const tArray2D<T> & rhs);
    //! multiply scalar
    template <class T1>
    tArray2D<T> & operator*=(const T1 & rhs)
    {
        for (unsigned int i = 0 ; i < mNx*mNy ; ++i) mData[i] *= rhs;
        return *this;
    }
    template <class T1>
    tArray2D<T> operator*(const T1 & rhs)
    {
        tArray2D<T> result(*this);
        result *= rhs;
        return result;
    }
    
    //! divide
    tArray2D<T> & Divide(const tArray2D<T> & rhs);
    //! divide scalar
    template <class T1>
    tArray2D<T> & operator/=(const T1 & rhs)
    {
        for (unsigned int i = 0 ; i < mNx*mNy ; ++i) mData[i] /= rhs;
        return *this;
    }
    
    //! raise to a power
    template <class T1>
    tArray2D<T> & Pow(const T1 & rhs)
    {
        for (unsigned int i = 0 ; i < mNx*mNy ; ++i) mData[i] = ::pow(mData[i], rhs);
        return *this;
    }
    
    //! Apply a Gaussian filter with length scale r, extending over a
    //! square of half-width n (so n=1 uses a square of 9 points, n = 2
    //! uses 25 etc). Suggest using n at least 2*r.
    void GaussianFilter(float r, int n);
    
    //! return the 'x' size of the array
    unsigned int GetNx() const {return mNx;}
    //! return the 'y' size of the array
    unsigned int GetNy() const {return mNy;}
    
    /// shifts all the elements...
    void Shift(int offsetX, int offsetY);
    
private:
    inline void UnwrapIndices(int & i, int & y) const;
    
    unsigned int mNx, mNy;
    T * mData;
    bool mWrap;
};

template<class T>
void tArray2D<T>::UnwrapIndices(int & i, int & j) const
{
    if (mWrap == false)
        return;
    
    while (i < 0)
        i += mNx;
    while (j < 0)
        j += mNy;
    
    i = i % mNx;
    j = j % mNy;
}


template<class T>
tArray2D<T>::tArray2D(int mNx, int mNy, const T & val)
:
mNx(mNx), mNy(mNy), mData(new T[mNx*mNy]), mWrap(false)
{
    unsigned int num = mNx*mNy;
    for (unsigned int i = 0 ; i < num ; ++i)
        mData[i] = val;
}

template<class T>
tArray2D<T>::tArray2D(const tArray2D & orig)
: mNx(orig.mNx), mNy(orig.mNy), mData(new T[mNx*mNy]), mWrap(orig.mWrap)
{
    memcpy(mData, orig.mData, mNx*mNy*sizeof(T));
}

template<class T>
tArray2D<T> & tArray2D<T>::operator=(const tArray2D & rhs)
{
    if (&rhs == this)
        return *this;
    
    if (mData)
        delete [] mData;
    
    mNx = rhs.mNx;
    mNy = rhs.mNy;
    mData = new T[mNx*mNy];
    
    mWrap = rhs.mWrap;
    
    memcpy(mData, rhs.mData, mNx*mNy*sizeof(T));
    return *this;
}

template<class T>
T & tArray2D<T>::At(int i, int j)
{
    UnwrapIndices(i, j);
    return operator()(i, j);
}

template<class T>
const T & tArray2D<T>::At(int i, int j) const
{
    UnwrapIndices(i, j);
    return operator()(i, j);
}

template<class T>
void tArray2D<T>::CalcGradientX()
{
    unsigned int i, j;
    
    // could do this much more efficiently by using temporaries on the
    // stack in the loop?
    const tArray2D<T> orig(*this);
    
    if (mWrap == false)
    {
        for (j = 0 ; j < mNy ; ++j)
        {
            operator()(0, j) = orig(1, j) - orig(0, j);
            operator()(mNx-1, j) = orig(mNx-1, j) - orig(mNx-2, j);
            
            for (i = 1 ; i < (mNx-1) ; ++i)
            {
                operator()(i, j) = (orig(i+1, j) - orig(i-1, j))/2;
            }
        }
    }
    else
    {
        for (j = 0 ; j < mNy ; ++j)
        {
            for (i = 0 ; i < mNx ; ++i)
            {
                operator()(i, j) = (orig.at(i+1, j) - orig.at(i-1, j))/2;
            }
        }
    }
}

template<class T>
void tArray2D<T>::CalcGradientY()
{
    unsigned int i, j;
    const tArray2D<T> orig(*this);
    
    if (mWrap == false)
    {
        for (i = 0 ; i < mNx ; ++i)
        {
            operator()(i, 0) = (orig(i, 1) - orig(i, 0))/2;
            operator()(i, mNy-1) = (orig(i, mNy-1) - orig(i, mNy-2))/2;
            for (j = 1 ; j < (mNy-1) ; ++j)
            {
                operator()(i, j) = (orig(i, j+1) - orig(i, j-1))/2;
            }
        }
    }
    else
    {
        for (j = 0 ; j < mNy ; ++j)
        {
            for (i = 0 ; i < mNx ; ++i)
            {
                operator()(i, j) = (orig.At(i, j+1) - orig.At(i, j-1))/2;
            }
        }
    }
}

template<class T>
void tArray2D<T>::Shift(int offsetX, int offsetY)
{
    tArray2D orig(*this);
    for (unsigned i = 0 ; i < mNx ; ++i)
    {
        for (unsigned j = 0 ; j < mNy ; ++j)
        {
            unsigned i0 = (i + offsetX) % mNx;
            unsigned j0 = (j + offsetY) % mNy;
            this->At(i0, j0) = orig.At(i, j);
        }
    }
}

template<class T>
void tArray2D<T>::GaussianFilter(float r, int n)
{
    int i, j, ii, jj, iii, jjj;
    
    int size = (n*2 + 1);
    float * filter = new float[size * size];    
    for (i = 0 ; i < size ; ++i)
    {
        for (j = 0 ; j < size ; ++j)
        {
            filter[i + j * size] = exp ( -( (i-n) * (i-n) + (j-n) * (j-n) ) /
                                        ((float) r * r) );
        }
    }    
    for (i = 0 ; i < (int) mNx ; ++i)
    {
        for (j = 0 ; j < (int) mNy ; ++j)
        {
            T total(0);
            float weight_total = 0;
            for (ii = -n ; ii < (int) n ; ++ii)
            {
                if ( ( ( (iii = i + ii) >= 0 ) &&
                      (iii < (int) mNx) ) ||
                    ( mWrap ) )
                {
                    for (jj = -n ; jj < (int) n ; ++jj)
                    {
                        if ( ( ( (jjj = j + jj) >= 0 ) &&
                              (jjj < (int) mNy) ) ||
                            ( mWrap ) )
                        {
                            // in a valid location
                            int index = (n+ii) + (n+jj)*size;
                            weight_total += filter[index];
                            total += filter[index] * At(iii, jjj);
                        }
                    }
                }
            }
            operator()(i, j) = total / weight_total;
        }
    }
    delete [] filter;
}

template<class T>
void tArray2D<T>::Abs()
{
    unsigned int i;
    for (i = 0 ; i < mNx*mNy ; ++i)
    {
        if (mData[i] < T(0))
            mData[i] = -mData[i];
    }
}

template<class T>
tArray2D<T> & tArray2D<T>::Add(const tArray2D<T> & rhs)
{
    Assert(rhs.mNx == mNx);
    Assert(rhs.mNy == mNy);    
    unsigned int i;
    for (i = 0 ; i < mNx*mNy ; ++i)
        mData[i] += rhs.mData[i];    
    return *this;
}

template<class T>
tArray2D<T> & tArray2D<T>::Subtract(const tArray2D<T> & rhs)
{
    Assert(rhs.mNx == mNx);
    Assert(rhs.mNy == mNy);    
    unsigned int i;
    for (i = 0 ; i < mNx*mNy ; ++i)
        mData[i] -= rhs.mData[i];    
    return *this;
}

template<class T>
tArray2D<T> & tArray2D<T>::Multiply(const tArray2D<T> & rhs)
{
    Assert(rhs.mNx == mNx);
    Assert(rhs.mNy == mNy);    
    unsigned int i;
    for (i = 0 ; i < mNx*mNy ; ++i)
        mData[i] *= rhs.mData[i];    
    return *this;
}

template<class T>
tArray2D<T> & tArray2D<T>::Divide(const tArray2D<T> & rhs)
{
    Assert(rhs.mNx == mNx);
    Assert(rhs.mNy == mNy);    
    unsigned int i;
    for (i = 0 ; i < mNx*mNy ; ++i)
        mData[i] /= rhs.mData[i];    
    return *this;
}

template<class T>
T tArray2D<T>::GetMin() const
{
    T min = mData[0];
    for (unsigned i = 0 ; i < mNx*mNy ; ++i)
    {
        if (mData[i] < min)
            min = mData[i];
    }
    return min;
}

template<class T>
T tArray2D<T>::GetMax() const
{
    
    T max = mData[0];
    for (unsigned i = 0 ; i < mNx*mNy ; ++i)
    {
        if (max < mData[i])
            max = mData[i];
    }
    return max;
}

template<class T>
void tArray2D<T>::SetRange(T valMin, T valMax)
{
    unsigned i;
    T origMin = GetMin();
    T origMax = GetMax();
    // set min to 0 and scale...
    float scale = (valMax - valMin) / (origMax - origMin);
    float offset = valMin - scale * origMin;
    
    for (i = 0 ; i < mNx*mNy ; ++i)
    {
        mData[i] = scale * mData[i] + offset;
    }
}

template<class T>
void tArray2D<T>::SetTo(T val)
{
    for (unsigned i = 0 ; i < mNx*mNy ; ++i)
    {
        mData[i] = val;
    }
}

template<class T>
T tArray2D<T>::Interpolate(float fi, float fj)
{
    Limit(fi, 0.0f, (mNx - 1.0f));
    Limit(fj, 0.0f, (mNy - 1.0f));
    unsigned i0 = (int) (fi);
    unsigned j0 = (int) (fj);
    unsigned i1 = i0 + 1;
    unsigned j1 = j0 + 1;
    if (i1 >= mNx) i1 = mNx - 1;
    if (j1 >= mNy) j1 = mNy - 1;
    float iFrac = fi - i0;
    float jFrac = fj - j0;
    T result =          jFrac * ( iFrac * (*this)(i1, j1) + (1.0f - iFrac) * (*this)(i0, j1) ) +
    (1.0f - jFrac) * ( iFrac * (*this)(i1, j0) + (1.0f - iFrac) * (*this)(i0, j0) );
    return result;
}


class tVector2
{
public:
    /// public access
    tScalar x, y;
    
    tVector2(tScalar x, tScalar y = 0.0f) : x(x), y(y) {}
    tVector2() {}
    
    /// returns pointer to the first element
    tScalar * GetData() {return &x;}
    /// returns pointer to the first element
    const tScalar * GetData() const {return &y;}
    
    void Set(tScalar _x, tScalar _y) {x = _x; y = _y;}
    
    tScalar GetLength() const {return sqrt(x*x+y*y);}
    tScalar GetLengthSq() const {return    x*x+y*y;}
    
    /// Normalise, and return the result
    tVector2 & Normalise() {
        (*this) *= (1.0f / this->GetLength()); return *this;}
    /// Get a normalised copy
    tVector2 GetNormalised() const {  return tVector2(*this).Normalise();}    
    tVector2 & operator+=(const tVector2 & v) {x += v.x, y += v.y; return *this;}
    tVector2 & operator-=(const tVector2 & v) {x -= v.x, y -= v.y; return *this;}    
    tVector2 & operator*=(tScalar f) {x *= f; y *= f; return *this;}
    tVector2 & operator/=(tScalar f) {x /= f; y /= f; return *this;}    
    tVector2 operator-() const {
        return tVector2(-x, -y);}    
    void Show(const char * str) const;    
    friend tVector2 operator+(const tVector2 & v1, const tVector2 & v2);
    friend tVector2 operator-(const tVector2 & v1, const tVector2 & v2);
    friend tVector2 operator*(const tVector2 & v1, tScalar f);
    friend tVector2 operator*(tScalar f, const tVector2 & v1);
    friend tVector2 operator/(const tVector2 & v1, tScalar f);
    friend tScalar Dot(const tVector2 & v1, const tVector2 & v2);    
    // c-style fns avoiding copies
    // out can also be vec1, vec2 or vec3
    friend void AddVector2(tVector2 & out, const tVector2 & vec1, const tVector2 & vec2);
    friend void AddVector2(tVector2 & out, const tVector2 & vec1, const tVector2 & vec2, const tVector2 & vec3);
    friend void SubVector2(tVector2 & out, const tVector2 & vec1, const tVector2 & vec2);
    /// out = scale * vec1
    friend void ScaleVector2(tVector2 & out, const tVector2 & vec1, tScalar scale);
    /// out = vec1 + scale * vec2
    /// out can also be vec1/vec2
    friend void AddScaleVector2(tVector2 & out, const tVector2 & vec1, tScalar scale, const tVector2 & vec2);    
};

inline tVector2 operator+(const tVector2 & v1, const tVector2 & v2)
{
    return tVector2(v1.x + v2.x, v1.y + v2.y);
}

inline tVector2 operator-(const tVector2 & v1, const tVector2 & v2)
{
    return tVector2(v1.x - v2.x, v1.y - v2.y);
}

inline tVector2 operator*(const tVector2 & v1, tScalar f)
{
    return tVector2(v1.x * f, v1.y * f);
}

inline tVector2 operator*(tScalar f, const tVector2 & v1)
{
    return tVector2(v1.x * f, v1.y * f);
}

inline tVector2 operator/(const tVector2 & v1, tScalar f)
{
    return tVector2(v1.x / f, v1.y / f);
}

inline tScalar Dot(const tVector2 & v1, const tVector2 & v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

inline void AddVector2(tVector2 & out, const tVector2 & vec1, const tVector2 & vec2)
{
    out.x = vec1.x + vec2.x;
    out.y = vec1.y + vec2.y;
}

inline void AddVector2(tVector2 & out, const tVector2 & vec1, const tVector2 & vec2, const tVector2 & vec3)
{
    out.x = vec1.x + vec2.x + vec3.x;
    out.y = vec1.y + vec2.y + vec3.y;
}

inline void SubVector2(tVector2 & out, const tVector2 & vec1, const tVector2 & vec2)
{
    out.x = vec1.x - vec2.x;
    out.y = vec1.y - vec2.y;
}

inline void ScaleVector2(tVector2 & out, const tVector2 & vec1, tScalar scale)
{
    out.x = vec1.x * scale;
    out.y = vec1.y * scale;
}

inline void AddScaleVector2(tVector2 & out, const tVector2 & vec1, tScalar scale, const tVector2 & vec2)
{
    out.x = vec1.x + scale * vec2.x;
    out.y = vec1.y + scale * vec2.y;
}

inline tScalar Min(const tVector2& vec) { if (vec.x<vec.y) return vec.x;else return vec.y;}
inline tScalar Max(const tVector2& vec) { if (vec.x>vec.y) return vec.x;else return vec.y;}

class tParticle
{
public:
                // The next particle when were stored in a linked list (the spatial grid)
    tParticle     * mNext;
    tVector2       mR;
    tVector2       mOldR;
    tScalar        mDensity;
    tScalar        mP;
    tVector2       mV;
    tVector2       mPressureForce;
    tVector2       mViscosityForce;
    tVector2       mBodyForce;// body force - gravity + some other forces later?
    tScalar        mCs;// the "color field" for the normal
    tVector2       mN;
    CCSprite *              sp;

};

class tSpatialGridIterator
{
public:
    /// constructor sets the internal state to be sensible,
    tSpatialGridIterator();
    /// This does two things - it does a lookup based on pos and
    /// calculates all the info it will need for subsequent traversals.
    /// Then it returns the first particle. Subsequent particles are
    /// returned by calls to GetNext. May return 0
    tParticle * FindFirst(const tVector2 & pos,
                          const class tSpatialGrid & grid);
    /// updates the internal iterator state and returns the next
    /// particle. Returns 0 if there are no more particles.
    inline tParticle * GetNext(const class tSpatialGrid & grid);
private:
    /// an array of linked lists - each entry points to the first
    /// element of the list. Since this is a 2D grid there are 9
    /// adjacent grid cells that might have a particle
    tParticle * mParticleLists[9];
    /// The number of lists actually in use.
    int mNParticleLists;
    /// current index into mParticleLists
    int mCurrentListIndex;
    /// current particle that we've iterated to and already returned
    /// using FindFirst or GetNext
    tParticle * mCurrentParticle;
};

class tSpatialGrid
{
public:
    tSpatialGrid(const tVector2 & domainMin,
                 const tVector2 & domainMax,
                 tScalar delta);
    void PopulateGrid(tParticle * particles, int nParticles);
private:
    /// let the iterator look at our privates
    friend class tSpatialGridIterator;    
    /// for internal use by tSpatialGrid
    class tSpatialGridEntry
    {
    public:
        /// Store the first element in a linked list - tParticle itself
        /// contains the "next" pointer
        tParticle * mFirstParticle;
    };    
    tArray2D<tSpatialGridEntry>      mGridEntries;    
    /// location of the bottom left corner of the domain
    tVector2                         mDomainMin;
    /// location of the top right corner of the domain
    tVector2                         mDomainMax;
    /// size of each grid cell
    tVector2                         mDelta;
};


inline tParticle * tSpatialGridIterator::GetNext(const tSpatialGrid & grid)
{
    if (0 == mCurrentParticle)
        return 0;
    
    mCurrentParticle = mCurrentParticle->mNext;
    
    if (mCurrentParticle)
        return mCurrentParticle;
    
    // may be some more lists to traverse
    if (++mCurrentListIndex >= mNParticleLists)
        return 0;
    
    return (mCurrentParticle = mParticleLists[mCurrentListIndex]);
}



class QueWorldInteractions : public b2QueryCallback
{
public:
    QueWorldInteractions(cFluidHashList (*grid)[hashHeight], tParticle *particles,float _knorm, float _ktang,float particleradius,float _gTimeStep)
	{
        hashGridList = grid;
        liquid       = particles;
        knorm        = _knorm;
        ktang        = _ktang;
        PartRadius   = particleradius;
        gTimeStep    = _gTimeStep;
    };
    
    bool ReportFixture(b2Fixture* fixture);
    int x, y;
    float deltaT;
    
protected:
    cFluidHashList (*hashGridList)[hashHeight];
    tParticle               *liquid;
    float                   knorm;          //  нормальный коэффициент восстановления
    float                   ktang;          //  тангенциальный коэффициент восстановления
    float                   mju;            //  вязкость
    float                   mro;            //  плотность
    float                   PartRadius;
    float                   gTimeStep ;
};
//*/


@interface World3 : CCLayer <WorldSceneProtocol>
{
	std::string				mSceneName;
   QueWorldInteractions *intersectQueryCallback;    
	int						mTex[1];
	GLuint					instancingShader;
    float                   SmoothRad;
    float                   knorm;          //  нормальный коэффициент восстановления
    float                   ktang;          //  тангенциальный коэффициент восстановления
    float                   mju;            //  вязкость
    float                   mro;            //  плотность
    float                   PressPerDensCoef;
    float                   lastTime;
    float                   thisTime;
    float                   RIntoSpr_Factor;
    int gWindowW;
    int gWindowH;
    
    // render individual particles or a filled in grid
    bool gRenderParticles;
    int gNRenderSamplesX;
    int gNRenderSamplesY;
    tScalar gRenderDensityMin;
    tScalar gRenderDensityMax;
    tScalar gKernelScale;
    // x ranges from 0 to domainX
    tScalar gDomainX;
    // y ranges from 0 to domainY
    tScalar gDomainY;
    
    // width of the container - leave some space for movement
    tScalar gContainerWidth;
    tScalar gContainerHeight;
    
    // position of the bottom left hand corner of the container.
    tScalar gContainerX;
    tScalar gContainerY;
    
    
    // initial height of the fluid surface
    tScalar gInitialFluidHeight;
    
    // gravity - acceleration
    tVector2 gGravity;
    
    // fluid density - this is kg/m2 (since 2D)
    tScalar gDensity0;
    
    // number of moving particles
    int gNParticles;
    
    // gViscosity
    tScalar gViscosity;
    
    // relationship of pressure to density when using eq of state
    // P = gGasK((density/gDensity0)^2 - 1)
    tScalar gGasK;
    
    // integration gTimeStep - for simplicity (especially when using
    // verlet etc) this will be constant so doesn't need to be passed
    // around, at least within this file
    tScalar gTimeStep;
    
    // scale physics time
    tScalar gTimeScale;
    
    // radius of influence of each particle
    tScalar gParticleRadius;
    
    // mass of each particle - gets initialised at startup
    tScalar gParticleMass;
    
    tScalar gKernelH;
    tScalar gKernelH9;
    tScalar gKernelH6;
    tScalar gKernelH4;
    tScalar gKernelH3;
    tScalar gKernelH2;
    
    // normalising factors for the kernels
    tScalar gWPoly6Scale ;
    tScalar gWSpikyScale ;
    tScalar gWViscosityScale ;
    tScalar gWLucyScale ;
    tScalar gBoundaryForceScale;
    tScalar gBoundaryForceScaleDist;
    tSpatialGrid * gSpatialGrid;
    tParticle * gParticles;
    tScalar gObjectDensityFrac;
    // bit hacky - extra force stopping objects getting "welded"
    tScalar gObjectBoundaryScale;
    // Extra buoyancy because of non-realistic pressure
    tScalar gObjectBuoyancyScale;
    cFluidHashList          hashGridList[hashWidth][hashHeight];
    int*                    InHashCellIndexes;
    tScalar                 SCALAR_TINY;
    bool                    gCreateObject;
}

// Particle Utilities
-(void)AllocateParticles:(int)cnt;
-(int)AddParticle;
void AddEmit ( float spacing );

int NumPoints ();	//	{ return mNumPoints; }

// Setup
-(void)Setup:(bool)bStart;

void SetupKernels ();
void SetupDefaultParams ();
void SetupExampleParams ( bool bStart );
void SetupSpacing ();

// Neighbor Search
void Search ();
-(void)InsertParticles;
void BasicSortParticles ();
void BinSortParticles ();
void FindNbrsSlow ();
void FindNbrsGrid ();

// Simulation
-(void)Run:(int)w :(int)h;
-(void)RunSimulateCPUSlow;
-(void)RunSimulateCPUGrid;

-(void)Advance;
void EmitParticles ();
-(void)Exit;
double GetDT(); //		{ return m_DT; }


@end