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

#define MAX_PARAM			50
#define GRID_UCHAR			0xFF
#define GRID_UNDEF			4294967295

#define RUN_PAUSE			0
#define RUN_SEARCH			1
#define RUN_VALIDATE		2
#define RUN_CPU_SLOW		3
#define RUN_CPU_GRID		4
#define RUN_CUDA_RADIX		5
#define RUN_CUDA_INDEX		6
#define RUN_CUDA_FULL		7
#define RUN_CUDA_CLUSTER	8
#define RUN_PLAYBACK		9

// Scalar params
#define PMODE				0
#define PNUM				1
#define PEXAMPLE			2
#define PSIMSIZE			3
#define PSIMSCALE			4
#define PGRID_DENSITY		5
#define PGRIDSIZE			6
#define PVISC				7
#define PRESTDENSITY		8
#define PMASS				9
#define PRADIUS				10
#define PDIST				11
#define PSMOOTHRADIUS		12
#define PINTSTIFF			13
#define PEXTSTIFF			14
#define PEXTDAMP			15
#define PACCEL_LIMIT		16
#define PVEL_LIMIT			17
#define PSPACING			18
#define PGROUND_SLOPE		19
#define PFORCE_MIN			20
#define PFORCE_MAX			21
#define PMAX_FRAC			22
#define PDRAWMODE			23
#define PDRAWSIZE			24
#define PDRAWGRID			25
#define PDRAWTEXT			26
#define PCLR_MODE			27
#define PPOINT_GRAV_AMT		28
#define PSTAT_OCCUPY		29
#define PSTAT_GRIDCNT		30
#define PSTAT_NBR			31
#define PSTAT_NBRMAX		32
#define PSTAT_SRCH			33
#define PSTAT_SRCHMAX		34
#define PSTAT_PMEM			35
#define PSTAT_GMEM			36
#define PTIME_INSERT		37
#define PTIME_SORT			38
#define PTIME_COUNT			39
#define PTIME_PRESS			40
#define PTIME_FORCE			41
#define PTIME_ADVANCE		42
#define PTIME_RECORD		43
#define PTIME_RENDER		44
#define PTIME_TOGPU			45
#define PTIME_FROMGPU		46
#define PFORCE_FREQ			47


// Vector params
#define PVOLMIN				0
#define PVOLMAX				1
#define PBOUNDMIN			2
#define PBOUNDMAX			3
#define PINITMIN			4
#define PINITMAX			5
#define PEMIT_POS			6
#define PEMIT_ANG			7
#define PEMIT_DANG			8
#define PEMIT_SPREAD		9
#define PEMIT_RATE			10
#define PPOINT_GRAV_POS		11
#define PPLANE_GRAV_DIR		12

// Booleans
#define PRUN				0
#define PDEBUG				1
#define PUSE_CUDA			2
#define	PUSE_GRID			3
#define PWRAP_X				4
#define PWALL_BARRIER		5
#define PLEVY_BARRIER		6
#define PDRAIN_BARRIER		7
#define PPLANE_GRAV_ON		11
#define PPROFILE			12
#define PCAPTURE			13

#define BFLUID				2
#define DWORD int

#define hashWidth	   	    (30)
#define hashHeight		    (30)
#define eps                 0.001

typedef  b2Vec2 tVector2 ;
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
        delete [] mData; mData = new T[nx*ny]; mNx = nx; mNy = ny;}
    
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

class tParticle
{
public:
                // The next particle when were stored in a linked list (the spatial grid)
    tParticle     * mNext;
                // position
    tVector2       mR;
                // previous position
    tVector2       mOldR;
                 // density calculated at this particle
    tScalar        mDensity;
                // pressure - diagnosed from density
    tScalar        mP;
                // velocity - diagnosed from position since we use verlet/particle
                    // integration
    tVector2       mV;
             // pressure force
    tVector2       mPressureForce;
               // gViscosity force
    tVector2       mViscosityForce;
              // body force - gravity + some other forces later?
    tVector2       mBodyForce;
               // the "color field" for the normal
    tScalar        mCs;
               // the normal field (grad(mCs) so not normalised)
    tVector2 mN;

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
    /// sets up the internal grid so that it ranges over the domain
    /// specified, and each grid cell is AT LEAST of size delta. This
    /// means that for every point in a cell, all other objects within a
    /// distance less than delta can be found by traversing the
    /// neighbouring cells
    tSpatialGrid(const tVector2 & domainMin,
                 const tVector2 & domainMax,
                 tScalar delta);
    
    ///
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


struct sPart
{
    CCSprite *              sp;
    b2Vec2				    mPos;           //
    b2Vec2				    mOldPos;        //
    float                   mDensity;       //
    float                   mPress;         //  давление
    b2Vec2				    mVel;           //  скорость
    b2Vec2                  mPressureForce; //
    b2Vec2                  mViscosityForce;//
    b2Vec2                  mN;
    b2Vec2                  mBodyForce;
    float                   mMas;
    
    BOOL                    isVisible;  //  признак отображения
    
};


class QueWorldInteractions : public b2QueryCallback
{
public:
    QueWorldInteractions(cFluidHashList (*grid)[hashHeight], sPart *particles,float _knorm, float _ktang,float particleradius)
	{
        hashGridList = grid;
        liquid = particles;
        knorm = _knorm;
        ktang = _ktang;
        PartRadius = particleradius;
    };
    
    bool ReportFixture(b2Fixture* fixture);
    int x, y;
    float deltaT;
    
protected:
    cFluidHashList (*hashGridList)[hashHeight];
    sPart                   *liquid;
    float                   knorm;          //  нормальный коэффициент восстановления
    float                   ktang;          //  тангенциальный коэффициент восстановления
    float                   mju;            //  вязкость
    float                   mro;            //  плотность
    float                   PartRadius;
};
//*/


@interface World3 : CCLayer <WorldSceneProtocol>
{
	std::string				mSceneName;
   QueWorldInteractions *intersectQueryCallback;

	int						mNumPoints;
	int						mTex[1];
	GLuint					instancingShader;
    float                   SmoothRad;
    sPart*                  liquid;
    float                   knorm;          //  нормальный коэффициент восстановления
    float                   ktang;          //  тангенциальный коэффициент восстановления
    float                   mju;            //  вязкость
    float                   mro;            //  плотность
    float                   mDensity0;
    float                   PressPerDensCoef;
    float                   ParticleRadius;
    float                   gTimeStep;
    //  параметры жидкости
    float                   gDensity0;
    float                   gParticleRadius;
    float                   gGasK;
    float                   gViscosity;
    float                   volume;
    float                   gParticleMass;
    // константы ядер
    float                   kernelScale;
    float                   gKernelH     ;
    float                   gKernelH9    ;
    float                   gKernelH6    ;
    float                   gKernelH4    ;
    float                   gKernelH3    ;
    float                   gKernelH2    ;
    float                   gWPoly6Scale ;
    float                   gWSpikyScale ;
    float                   gWViscosityScale;
    float                   gWLucyScale;
    cFluidHashList          hashGridList[hashWidth][hashHeight];
    int*                    InHashCellIndexes;
    
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
void SetupAddVolume ( b2Vec3 min, b2Vec3 max, float spacing, float offs );
void SetupGridAllocate ( b2Vec3 min, b2Vec3 max, float sim_scale, float cell_size, float border );
void ParseXML ( std::string name, int id, bool bStart );

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

// Acceleration Grid
int getGridCell ( int p, b2Vec3& gc );
int getGridCell ( b2Vec3& p, b2Vec3& gc );
int getGridTotal ();//		{ return m_GridTotal; }
int getSearchCnt ();//		{ return m_GridAdjCnt; }
b2Vec3 getCell ( int gc );
b2Vec3 GetGridRes ();//		{ return m_GridRes; }
b2Vec3 GetGridMin ();//		{ return m_GridMin; }
b2Vec3 GetGridMax ();//		{ return m_GridMax; }
b2Vec3 GetGridDelta ();//	{ return m_GridDelta; }

// Acceleration Neighbor Tables
void AllocateNeighborTable ();
void ClearNeighborTable ();
void ResetNeighbors ();
int GetNeighborTableSize ();//	{ return m_NeighborNum; }
void ClearNeighbors ( int i );
int AddNeighbor();
int AddNeighbor( int i, int j, float d );

// Parameters
void SetParam (int p, float v );//		{ m_Param[p] = v; }
void SetParam (int p, int v );//		{ m_Param[p] = (float) v; }
float GetParam ( int p );//			{ return (float) m_Param[p]; }
float SetParam ( int p, float v, float mn, float mx );//	{ m_Param[p] = v ; if ( m_Param[p] > mx ) m_Param[p] = mn; return m_Param[p];}
float IncParam ( int p, float v, float mn, float mx );//	{
//	m_Param[p] += v;
//	if ( m_Param[p] < mn ) m_Param[p] = mn;
//	if ( m_Param[p] > mx ) m_Param[p] = mn;
//	return m_Param[p];
//}
b2Vec3 GetVec ( int p );//			{ return m_Vec[p]; }
void SetVec ( int p, b2Vec3 v );//	{ m_Vec[p] = v; }
void Toggle ( int p );//				{ m_Toggle[p] = !m_Toggle[p]; }
bool GetToggle ( int p );//			{ return m_Toggle[p]; }

@end