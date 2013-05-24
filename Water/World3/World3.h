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


struct NList
{
	int num;
	int first;
};

struct Fluid
{						// offset - TOTAL: 72 (must be multiple of 12)
	b2Vec3 pos;			        // 0
	b2Vec3		vel;			// 12
	b2Vec3		veleval;		// 24
	b2Vec3		force;			// 36
	float		pressure;		// 48
	float		density;		// 52
	int			grid_cell;		// 56
	int			grid_next;		// 60
	int			clr;			// 64
	int			padding;		// 68
};

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