#import "WorldSceneProtocol.h"
#import "Common.h"

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
struct NList
{
	int num;
	int first;
};

struct Fluid
{						// offset - TOTAL: 72 (must be multiple of 12)
	b2Vec3 pos;			// 0
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

@interface World3 : CCLayer <WorldSceneProtocol>
{
	std::string				mSceneName;
	
	// Time
	int							m_Frame;
	double						m_DT;
	double						m_Time;
	
	// Simulation Parameters
	double						m_Param [ MAX_PARAM ];			// see defines above
	b2Vec3					m_Vec [ MAX_PARAM ];
	bool						m_Toggle [ MAX_PARAM ];
	
	// SPH Kernel functions
	double					m_R2, m_Poly6Kern, m_LapKern, m_SpikyKern;
	
	// Particle Buffers
	int						mNumPoints;
	int						mMaxPoints;
	int						mGoodPoints;
	b2Vec3*				mPos;
	int*					mClr;
	b2Vec3*				mVel;
	b2Vec3*				mVelEval;
	unsigned short*			mAge;
	float*					mPressure;
	float*					mDensity;
	b2Vec3*				mForce;
	uint*					mGridCell;
	uint*					mClusterCell;
	uint*					mGridNext;
	uint*					mNbrNdx;
	uint*					mNbrCnt;
	
	// Acceleration Grid
	uint*					m_Grid;
	uint*					m_GridCnt;
	int						m_GridTotal;			// total # cells
	b2Vec3				m_GridRes;				// resolution in each axis
	b2Vec3				m_GridMin;				// volume of grid (may not match domain volume exactly)
	b2Vec3				m_GridMax;
	b2Vec3				m_GridSize;				// physical size in each axis
	b2Vec3				m_GridDelta;
	int						m_GridSrch;
	int						m_GridAdjCnt;
	int						m_GridAdj[216];
	
	// Acceleration Neighbor Table
	int						m_NeighborNum;
	int						m_NeighborMax;
	int*					m_NeighborTable;
	float*					m_NeighborDist;
	
	char*					mPackBuf;
	int*					mPackGrid;
	
	int						mVBO[3];
	
	// Record/Playback
	int						mFileNum;
	std::string				mFileName;
	float					mFileSize;
	FILE*					mFP;
	int						mLastPoints;
	
	int						mSpherePnts;
	int						mTex[1];
	GLuint					instancingShader;
	
	// Selected particle
	int						mSelected;
	// Saved results (for algorithm validation)
	uint*					mSaveNdx;
	uint*					mSaveCnt;
	uint*					mSaveNeighbors;
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

// Smoothed Particle Hydrodynamics
void ComputePressureGrid ();			// O(kn) - spatial grid
void ComputeForceGrid ();				// O(kn) - spatial grid
void ComputeForceGridNC ();				// O(cn) - neighbor table

// GPU Support functions
void AllocatePackBuf ();
void PackParticles ();
void UnpackParticles ();

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