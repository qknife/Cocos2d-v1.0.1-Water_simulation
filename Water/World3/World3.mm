// Import the interfaces
#import "World3.h"
#import "Common.h"

#define EPSILON			0.00001f			//for collision detection

// HelloWorldLayer implementation
@implementation World3

// on "init" you need to initialize your instance
-(id) init
{
	if( (self=[super init]))
	{
		// enable accelerometer
		self.isAccelerometerEnabled = YES;
		
        mNumPoints = 0;
		mMaxPoints = 0;
		mPackBuf = 0x0;
		mPackGrid = 0x0;
		mFP = 0x0;
		
		mPos = 0x0;
		mClr = 0x0;
		mVel = 0x0;
		mVelEval = 0x0;
		mAge = 0x0;
		mPressure = 0x0;
		mDensity = 0x0;
		mForce = 0x0;
		mClusterCell = 0x0;
		mGridNext = 0x0;
		mNbrNdx = 0x0;
		mNbrCnt = 0x0;
		mSelected = -1;
		m_Grid = 0x0;
		m_GridCnt = 0x0;
		
		m_Frame = 0;
		
		m_NeighborTable = 0x0;
		m_NeighborDist = 0x0;
		
		m_Param [ PMODE ]		= RUN_CPU_SLOW;
		m_Param [ PEXAMPLE ]	= 1;
		m_Param [ PGRID_DENSITY ] = 2.0;
		m_Param [ PNUM ]		= 8192; //65536 * 128;
		
		
		m_Toggle [ PDEBUG ]		=	false;
		m_Toggle [ PUSE_GRID ]	=	false;
		m_Toggle [ PPROFILE ]	=	false;
		m_Toggle [ PCAPTURE ]   =	false;
		
		[self Setup:true];
		
		// param
//		obj_from.x = 0;		obj_from.y = 0;		obj_from.z = 20;		// emitter
//		obj_angs.x = 118.7;	obj_angs.y = 200;	obj_angs.z = 1.0;
//		obj_dang.x = 1;	obj_dang.y = 1;		obj_dang.z = 0;
//		
//		psys.Setup (true);
//		psys.SetVec ( PEMIT_ANG, Vector3DF ( obj_angs.x, obj_angs.y, obj_angs.z ) );
//		psys.SetVec ( PEMIT_POS, Vector3DF ( obj_from.x, obj_from.y, obj_from.z ) );
//		
//		psys.SetParam ( PCLR_MODE, iClrMode );
		
		[self Run:0 :0];
		
		[self schedule: @selector(tick:) interval:1.0f / 30.0f];
	}
	
	return self;
}

-(void)draw
{
	[Common draw];
}


-(void)particlesCountUp:(NSInteger)diff_
{
}

-(void)particlesCountDown:(NSInteger)diff_
{
}

-(void) tick: (ccTime) dt
{

}

- (void)accelerometer:(UIAccelerometer*)accelerometer didAccelerate:(UIAcceleration*)acceleration
{
	[Common processAccelometry:acceleration];
}

-(void)Setup:(bool)bStart;
{
	m_Frame = 0;
	m_Time = 0;
	
//	ClearNeighborTable ();
	mNumPoints = 0;
	
//	SetupDefaultParams ();
//	
//	SetupExampleParams ( bStart );
	
	m_Param [PGRIDSIZE] = 2*m_Param[PSMOOTHRADIUS] / m_Param[PGRID_DENSITY];
	
	[self AllocateParticles:m_Param[PNUM]];
//	AllocatePackBuf ();
//	
//	SetupKernels ();
//	
//	SetupSpacing ();
//	
//	SetupAddVolume ( m_Vec[PINITMIN], m_Vec[PINITMAX], m_Param[PSPACING], 0.1 );													// Create the particles
//	
//	SetupGridAllocate ( m_Vec[PVOLMIN], m_Vec[PVOLMAX], m_Param[PSIMSCALE], m_Param[PGRIDSIZE], 1.0 );	// Setup grid
}

-(void)Exit
{
	free ( mPos );
	free ( mClr );
	free ( mVel );
	free ( mVelEval );
	free ( mAge );
	free ( mPressure );
	free ( mDensity );
	free ( mForce );
	free ( mClusterCell );
	free ( mGridCell );
	free ( mGridNext );
	free ( mNbrNdx );
	free ( mNbrCnt );
}

// Allocate particle memory
-(void)AllocateParticles:(int)cnt
{
	int nump = 0;		// number to copy from previous data
	
	b2Vec3* srcPos = mPos;
	mPos = (b2Vec3*)		malloc ( cnt*sizeof(b2Vec3) );
	if ( srcPos != 0x0 )	{ memcpy ( mPos, srcPos, nump *sizeof(b2Vec3)); free ( srcPos ); }
	
	DWORD* srcClr = mClr;
	mClr = (DWORD*)			malloc ( cnt*sizeof(DWORD) );
	if ( srcClr != 0x0 )	{ memcpy ( mClr, srcClr, nump *sizeof(DWORD)); free ( srcClr ); }
	
	b2Vec3* srcVel = mVel;
	mVel = (b2Vec3*)		malloc ( cnt*sizeof(b2Vec3) );
	if ( srcVel != 0x0 )	{ memcpy ( mVel, srcVel, nump *sizeof(b2Vec3)); free ( srcVel ); }
	
	b2Vec3* srcVelEval = mVelEval;
	mVelEval = (b2Vec3*)	malloc ( cnt*sizeof(b2Vec3) );
	if ( srcVelEval != 0x0 ) { memcpy ( mVelEval, srcVelEval, nump *sizeof(b2Vec3)); free ( srcVelEval ); }
	
	unsigned short* srcAge = mAge;
	mAge = (unsigned short*) malloc ( cnt*sizeof(unsigned short) );
	if ( srcAge != 0x0 )	{ memcpy ( mAge, srcAge, nump *sizeof(unsigned short)); free ( srcAge ); }
	
	float* srcPress = mPressure;
	mPressure = (float*) malloc ( cnt*sizeof(float) );
	if ( srcPress != 0x0 ) { memcpy ( mPressure, srcPress, nump *sizeof(float)); free ( srcPress ); }
	
	float* srcDensity = mDensity;
	mDensity = (float*) malloc ( cnt*sizeof(float) );
	if ( srcDensity != 0x0 ) { memcpy ( mDensity, srcDensity, nump *sizeof(float)); free ( srcDensity ); }
	
	b2Vec3* srcForce = mForce;
	mForce = (b2Vec3*)	malloc ( cnt*sizeof(b2Vec3) );
	if ( srcForce != 0x0 )	{ memcpy ( mForce, srcForce, nump *sizeof(b2Vec3)); free ( srcForce ); }
	
	uint* srcCell = mClusterCell;
	mClusterCell = (uint*)	malloc ( cnt*sizeof(uint) );
	if ( srcCell != 0x0 )	{ memcpy ( mClusterCell, srcCell, nump *sizeof(uint)); free ( srcCell ); }
	
	uint* srcGCell = mGridCell;
	mGridCell = (uint*)	malloc ( cnt*sizeof(uint) );
	if ( srcGCell != 0x0 )	{ memcpy ( mGridCell, srcGCell, nump *sizeof(uint)); free ( srcGCell ); }
	
	uint* srcNext = mGridNext;
	mGridNext = (uint*)	malloc ( cnt*sizeof(uint) );
	if ( srcNext != 0x0 )	{ memcpy ( mGridNext, srcNext, nump *sizeof(uint)); free ( srcNext ); }
	
	uint* srcNbrNdx = mNbrNdx;
	mNbrNdx = (uint*)		malloc ( cnt*sizeof(uint) );
	if ( srcNbrNdx != 0x0 )	{ memcpy ( mNbrNdx, srcNbrNdx, nump *sizeof(uint)); free ( srcNbrNdx ); }
	
	uint* srcNbrCnt = mNbrCnt;
	mNbrCnt = (uint*)		malloc ( cnt*sizeof(uint) );
	if ( srcNbrCnt != 0x0 )	{ memcpy ( mNbrCnt, srcNbrCnt, nump *sizeof(uint)); free ( srcNbrCnt ); }
	
	m_Param[PSTAT_PMEM] = 68 * 2 * cnt;
	
	mMaxPoints = cnt;
}

-(int)AddParticle
{
	if ( mNumPoints >= mMaxPoints ) return -1;
	int n = mNumPoints;
	(mPos + n)->Set ( 0,0,0 );
	(mVel + n)->Set ( 0,0,0 );
	(mVelEval + n)->Set ( 0,0,0 );
	(mForce + n)->Set ( 0,0,0 );
	*(mPressure + n) = 0;
	*(mDensity + n) = 0;
	*(mGridNext + n) = -1;
	*(mClusterCell + n) = -1;
	
	mNumPoints++;
	return n;
}
//
//void SetupAddVolume ( b2Vec3 min, b2Vec3 max, float spacing, float offs )
//{
//	b2Vec3 pos;
//	int n, p;
//	float dx, dy, dz, x, y, z;
//	int cntx, cnty, cntz;
//	cntx = ceil( (max.x-min.x-offs) / spacing );
//	cntz = ceil( (max.z-min.z-offs) / spacing );
//	int cnt = cntx * cntz;
//	int xp, yp, zp, c2;
//	float odd;
//	
//	dx = max.x-min.x;
//	dy = max.y-min.y;
//	dz = max.z-min.z;
//	
//	c2 = cnt/2;
//	for (float y = min.y+offs; y <= max.y; y += spacing ) {
//		for (int xz=0; xz < cnt; xz++ ) {
//			
//			x = min.x+offs + (xz % int(cntx))*spacing;
//			z = min.z+offs + (xz / int(cntx))*spacing;
//			/*if ( xy < c2 ) {
//			 zp = xy / int(dx);
//			 x = min.x+offs + (xz % int(cntx/2) )*spacing*2 + (zp % 2)*spacing;
//			 z = min.z+offs + zp * spacing;
//			 } else {
//			 zp = (xy-c2) / int(dx);
//			 x = min.x+offs + ( (xz-c2) % int(cntx/2) )*spacing*2 + (zp % 2)*spacing;
//			 z = min.z+offs + (cntz-1-zp) * spacing;
//			 }*/
//			p = [self AddParticle];
//			if ( p != -1 ) {
//				(mPos+p)->Set ( x,y,z);
//				*(mClr+p) = COLORA( (x-min.x)/dx, (y-min.y)/dy, (z-min.z)/dz, 1);
//				//*(mClr+p) = COLORA( 0.25, +0.25 + (y-min.y)*.75/dy, 0.25 + (z-min.z)*.75/dz, 1);  // (x-min.x)/dx
//			}
//		}
//	}
//}
//
//void AddEmit ( float spacing )
//{
//	int p;
//	b2Vec3 dir;
//	b2Vec3 pos;
//	float ang_rand, tilt_rand;
//	float rnd = m_Vec[PEMIT_RATE].y * 0.15;
//	int x = (int) sqrt(m_Vec[PEMIT_RATE].y);
//	
//	for ( int n = 0; n < m_Vec[PEMIT_RATE].y; n++ ) {
//		ang_rand = (float(rand()*2.0/RAND_MAX) - 1.0) * m_Vec[PEMIT_SPREAD].x;
//		tilt_rand = (float(rand()*2.0/RAND_MAX) - 1.0) * m_Vec[PEMIT_SPREAD].y;
//		dir.x = cos ( ( m_Vec[PEMIT_ANG].x + ang_rand) * DEGtoRAD ) * sin( ( m_Vec[PEMIT_ANG].y + tilt_rand) * DEGtoRAD ) * m_Vec[PEMIT_ANG].z;
//		dir.y = sin ( ( m_Vec[PEMIT_ANG].x + ang_rand) * DEGtoRAD ) * sin( ( m_Vec[PEMIT_ANG].y + tilt_rand) * DEGtoRAD ) * m_Vec[PEMIT_ANG].z;
//		dir.z = cos ( ( m_Vec[PEMIT_ANG].y + tilt_rand) * DEGtoRAD ) * m_Vec[PEMIT_ANG].z;
//		pos = m_Vec[PEMIT_POS];
//		pos.x += spacing * (n/x);
//		pos.y += spacing * (n%x);
//		
//		p = AddParticle ();
//		*(mPos+n) = pos;
//		*(mVel+n) = dir;
//		*(mVelEval+n) = dir;
//		*(mAge+n) = 0;
//		*(mClr+n) = COLORA ( m_Time/10.0, m_Time/5.0, m_Time /4.0, 1 );
//	}
//}

-(void)RunSimulateCPUSlow
{
	[self InsertParticles];
	[self Advance];
}

-(void)RunSimulateCPUGrid
{
	[self InsertParticles];
	[self Advance];
}

//void EmitParticles ()
//{
//	if ( m_Vec[PEMIT_RATE].x > 0 && (++m_Frame) % (int) m_Vec[PEMIT_RATE].x == 0 ) {
//		float ss = m_Param [ PDIST ] / m_Param[ PSIMSCALE ];		// simulation scale (not Schutzstaffel)
//		AddEmit ( ss );
//	}
//}


-(void)Run:(int)w :(int)h
{
	// Clear sim timers
	m_Param[ PTIME_INSERT ] = 0.0;
	m_Param[ PTIME_SORT ] = 0.0;
	m_Param[ PTIME_COUNT ] = 0.0;
	m_Param[ PTIME_PRESS ] = 0.0;
	m_Param[ PTIME_FORCE ] = 0.0;
	m_Param[ PTIME_ADVANCE ] = 0.0;
	
	// Run
	switch ( (int) m_Param[PMODE] )
	{
		case RUN_CPU_SLOW:		[self RunSimulateCPUSlow];	break;
		case RUN_CPU_GRID:		[self RunSimulateCPUGrid];	break;
	};
	
	m_Time += m_DT;
	m_Frame++;
}

//void AllocatePackBuf ()
//{
//	if ( mPackBuf != 0x0 ) free ( mPackBuf );
//	mPackBuf = (char*) malloc ( sizeof(Fluid) * mMaxPoints );
//}
//
////------- NOT CURRENTLY USED
//void PackParticles ()
//{
//	// Bin particles in memory according to grid cells.
//	// This is equivalent to a partial bucket sort, as a GPU radix sort is not necessary.
//	
//	int j;
//	char* dat = mPackBuf;
//	int cnt = 0;
//	
//	for (int c=0; c < m_GridTotal; c++) {
//		j = m_Grid[c];
//		mPackGrid[c] = cnt;
//		while ( j != -1 ) {
//			*(b2Vec3*) dat = *(mPos+j);			dat += sizeof(b2Vec3);
//			*(b2Vec3*) dat = *(mVel+j);			dat += sizeof(b2Vec3);
//			*(b2Vec3*) dat = *(mVelEval+j);		dat += sizeof(b2Vec3);
//			*(b2Vec3*) dat = *(mForce+j);		dat += sizeof(b2Vec3);
//			*(float*) dat =		*(mPressure+j);		dat += sizeof(float);
//			*(float*) dat =		*(mDensity+j);		dat += sizeof(float);
//			*(int*) dat =		*(mClusterCell+j);	dat += sizeof(int);					// search cell
//			*(int*) dat =		c;					dat += sizeof(int);					// container cell
//			*(DWORD*) dat =		*(mClr+j);			dat += sizeof(DWORD);
//			dat += sizeof(int);
//			j = *(mGridNext+j);
//			cnt++;
//		}
//	}
//	mGoodPoints = cnt;
//	
//	//--- Debugging - Print packed particles
//	/*printf ( "\nPACKED\n" );
//	 for (int n=cnt-30; n < cnt; n++ ) {
//	 dat = mPackBuf + n*sizeof(Fluid);
//	 printf ( " %d: %d, %d\n", n, *((int*) (dat+56)), *((int*) (dat+60)) );
//	 }*/
//}
//
////------- NOT CURRENTLY USED
//void UnpackParticles ()
//{
//	char* dat = mPackBuf;
//	
//	b2Vec3*  ppos =		mPos;
//	b2Vec3*  pforce =	mForce;
//	b2Vec3*  pvel =		mVel;
//	b2Vec3*  pveleval =	mVelEval;
//	float*		ppress =	mPressure;
//	float*		pdens =		mDensity;
//	DWORD*		pclr =		mClr;
//	
//	for (int n=0; n < mGoodPoints; n++ ) {
//		*ppos++ =		*(b2Vec3*) dat;		dat += sizeof(b2Vec3);
//		*pvel++ =		*(b2Vec3*) dat;		dat += sizeof(b2Vec3);
//		*pveleval++ =	*(b2Vec3*) dat;		dat += sizeof(b2Vec3);
//		*pforce++ =		*(b2Vec3*) dat;		dat += sizeof(b2Vec3);
//		*ppress++ =		*(float*) dat;			dat += sizeof(float);
//		*pdens++ =		*(float*) dat;			dat += sizeof(float);
//		dat += sizeof(int);
//		dat += sizeof(int);
//		*pclr++ =		*(DWORD*) dat;			dat += sizeof(DWORD);
//		dat += sizeof(int);
//	}
//}
//

-(void)Advance
{
	b2Vec3 norm, z;
	b2Vec3 dir, accel;
	b2Vec3 vnext;
	b2Vec3 bmin, bmax;
	b2Vec4 clr;
	double adj;
	float AL, AL2, SL, SL2, ss, radius;
	float stiff, damp, speed, diff;
	
	AL = m_Param[PACCEL_LIMIT];	AL2 = AL*AL;
	SL = m_Param[PVEL_LIMIT];	SL2 = SL*SL;
	
	stiff = m_Param[PEXTSTIFF];
	damp = m_Param[PEXTDAMP];
	radius = m_Param[PRADIUS];
	bmin = m_Vec[PBOUNDMIN];
	bmax = m_Vec[PBOUNDMAX];
	ss = m_Param[PSIMSCALE];
	
	// Get particle buffers
	b2Vec3*	ppos = mPos;
	b2Vec3*	pvel = mVel;
	b2Vec3*	pveleval = mVelEval;
	b2Vec3*	pforce = mForce;
	DWORD*		pclr = mClr;
	float*		ppress = mPressure;
	float*		pdensity = mDensity;
	
	// Advance each particle
	for ( int n=0; n < NumPoints(); n++ ) {
		
		if ( mGridCell[n] == GRID_UNDEF) continue;
		
		// Compute Acceleration
		accel = *pforce;
		accel *= m_Param[PMASS];
		
		// Boundary Conditions
		// Y-axis walls
		diff = radius - ( ppos->y - (bmin.y+ (ppos->x-bmin.x)*m_Param[PGROUND_SLOPE] ) )*ss;
		if (diff > EPSILON ) {
			norm.Set ( -m_Param[PGROUND_SLOPE], 1.0 - m_Param[PGROUND_SLOPE], 0 );
			adj = stiff * diff - damp * b2Dot(norm,*pveleval );
			accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
		}
		diff = radius - ( bmax.y - ppos->y )*ss;
		if (diff > EPSILON) {
			norm.Set ( 0, -1, 0 );
			adj = stiff * diff - damp * b2Dot(norm,*pveleval );
			accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
		}
		
		// X-axis walls
		if ( !m_Toggle[PWRAP_X] ) {
			diff = radius - ( ppos->x - (bmin.x + (sin(m_Time*m_Param[PFORCE_FREQ])+1)*0.5 * m_Param[PFORCE_MIN]) )*ss;
			//diff = 2 * radius - ( p->pos.x - min.x + (sin(m_Time*10.0)-1) * m_Param[FORCE_XMIN_SIN] )*ss;
			if (diff > EPSILON ) {
				norm.Set ( 1.0, 0, 0 );
				adj = (m_Param[ PFORCE_MIN ]+1) * stiff * diff - damp * b2Dot(norm,*pveleval ) ;
				accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
			}
			
			diff = radius - ( (bmax.x - (sin(m_Time*m_Param[PFORCE_FREQ])+1)*0.5* m_Param[PFORCE_MAX]) - ppos->x )*ss;
			if (diff > EPSILON) {
				norm.Set ( -1, 0, 0 );
				adj = (m_Param[ PFORCE_MAX ]+1) * stiff * diff - damp * b2Dot(norm,*pveleval );
				accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
			}
		}
		
		// Z-axis walls
		diff = radius - ( ppos->z - bmin.z )*ss;
		if (diff > EPSILON) {
			norm.Set ( 0, 0, 1 );
			adj = stiff * diff - damp * b2Dot(norm,*pveleval );
			accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
		}
		diff = radius - ( bmax.z - ppos->z )*ss;
		if (diff > EPSILON) {
			norm.Set ( 0, 0, -1 );
			adj = stiff * diff - damp * b2Dot(norm,*pveleval );
			accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
		}
		
		
		// Wall barrier
		if ( m_Toggle[PWALL_BARRIER] ) {
			diff = 2 * radius - ( ppos->x - 0 )*ss;
			if (diff < 2*radius && diff > EPSILON && fabs(ppos->y) < 3 && ppos->z < 10) {
				norm.Set ( 1.0, 0, 0 );
				adj = 2*stiff * diff - damp * b2Dot(norm,*pveleval ) ;
				accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
			}
		}
		
		// Levy barrier
		if ( m_Toggle[PLEVY_BARRIER] ) {
			diff = 2 * radius - ( ppos->x - 0 )*ss;
			if (diff < 2*radius && diff > EPSILON && fabs(ppos->y) > 5 && ppos->z < 10) {
				norm.Set ( 1.0, 0, 0 );
				adj = 2*stiff * diff - damp * b2Dot(norm,*pveleval ) ;
				accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
			}
		}
		// Drain barrier
		if ( m_Toggle[PDRAIN_BARRIER] ) {
			diff = 2 * radius - ( ppos->z - bmin.z-15 )*ss;
			if (diff < 2*radius && diff > EPSILON && (fabs(ppos->x)>3 || fabs(ppos->y)>3) ) {
				norm.Set ( 0, 0, 1);
				adj = stiff * diff - damp * b2Dot(norm,*pveleval );
				accel.x += adj * norm.x; accel.y += adj * norm.y; accel.z += adj * norm.z;
			}
		}
		
		// Plane gravity
		accel += m_Vec[PPLANE_GRAV_DIR];
		
		// Point gravity
		if ( m_Param[PPOINT_GRAV_AMT] > 0 ) {
			norm.x = ( ppos->x - m_Vec[PPOINT_GRAV_POS].x );
			norm.y = ( ppos->y - m_Vec[PPOINT_GRAV_POS].y );
			norm.z = ( ppos->z - m_Vec[PPOINT_GRAV_POS].z );
			norm.Normalize ();
			norm *= m_Param[PPOINT_GRAV_AMT];
			accel -= norm;
		}
		
		// Acceleration limiting
		speed = accel.x*accel.x + accel.y*accel.y + accel.z*accel.z;
		if ( speed > AL2 ) {
			accel *= AL / sqrt(speed);
		}
		
		// Velocity limiting
		speed = pvel->x*pvel->x + pvel->y*pvel->y + pvel->z*pvel->z;
		if ( speed > SL2 ) {
			speed = SL2;
			(*pvel) *= SL / sqrt(speed);
		}
		
		// Leapfrog Integration ----------------------------
		vnext = accel;
		vnext *= m_DT;
		vnext += *pvel;						// v(t+1/2) = v(t-1/2) + a(t) dt
		
		*pveleval = *pvel;
		*pveleval += vnext;
		*pveleval *= 0.5;					// v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5		used to compute forces later
		*pvel = vnext;
		vnext *= m_DT/ss;
		*ppos += vnext;						// p(t+1) = p(t) + v(t+1/2) dt
		
		/*if ( m_Param[PCLR_MODE]==1.0 ) {
		 adj = fabs(vnext.x)+fabs(vnext.y)+fabs(vnext.z) / 7000.0;
		 adj = (adj > 1.0) ? 1.0 : adj;
		 *pclr = COLORA( 0, adj, adj, 1 );
		 }
		 if ( m_Param[PCLR_MODE]==2.0 ) {
		 float v = 0.5 + ( *ppress / 1500.0);
		 if ( v < 0.1 ) v = 0.1;
		 if ( v > 1.0 ) v = 1.0;
		 *pclr = COLORA ( v, 1-v, 0, 1 );
		 }*/
		if ( speed > SL2*0.1) {
			adj = SL2*0.1;
			clr.fromClr ( *pclr );
			clr += float(2/255.0);
			clr.Clamp ( 1, 1, 1, 1);
			*pclr = clr.toClr();
		}
		if ( speed < 0.01 ) {
			clr.fromClr ( *pclr);
			clr.x -= float(1/255.0);		if ( clr.x < 0.2 ) clr.x = 0.2;
			clr.y -= float(1/255.0);		if ( clr.y < 0.2 ) clr.y = 0.2;
			*pclr = clr.toClr();
		}
		
		// Euler integration -------------------------------
		/* accel += m_Gravity;
		 accel *= m_DT;
		 p->vel += accel;				// v(t+1) = v(t) + a(t) dt
		 p->vel_eval += accel;
		 p->vel_eval *= m_DT/d;
		 p->pos += p->vel_eval;
		 p->vel_eval = p->vel;  */
		
		
		if ( m_Toggle[PWRAP_X] ) {
			diff = ppos->x - (m_Vec[PBOUNDMIN].x + 2);			// -- Simulates object in center of flow
			if ( diff <= 0 ) {
				ppos->x = (m_Vec[PBOUNDMAX].x - 2) + diff*2;
				ppos->z = 10;
			}
		}
		
		ppos++;
		pvel++;
		pveleval++;
		pforce++;
		pclr++;
		ppress++;
		pdensity++;
	}
	
}

//void ClearNeighborTable ()
//{
//	if ( m_NeighborTable != 0x0 )	free (m_NeighborTable);
//	if ( m_NeighborDist != 0x0)		free (m_NeighborDist );
//	m_NeighborTable = 0x0;
//	m_NeighborDist = 0x0;
//	m_NeighborNum = 0;
//	m_NeighborMax = 0;
//}
//
//void ResetNeighbors ()
//{
//	m_NeighborNum = 0;
//}
//
//// Allocate new neighbor tables, saving previous data
//int AddNeighbor ()
//{
//	if ( m_NeighborNum >= m_NeighborMax ) {
//		m_NeighborMax = 2*m_NeighborMax + 1;
//		int* saveTable = m_NeighborTable;
//		m_NeighborTable = (int*) malloc ( m_NeighborMax * sizeof(int) );
//		if ( saveTable != 0x0 ) {
//			memcpy ( m_NeighborTable, saveTable, m_NeighborNum*sizeof(int) );
//			free ( saveTable );
//		}
//		float* saveDist = m_NeighborDist;
//		m_NeighborDist = (float*) malloc ( m_NeighborMax * sizeof(float) );
//		if ( saveDist != 0x0 ) {
//			memcpy ( m_NeighborDist, saveDist, m_NeighborNum*sizeof(int) );
//			free ( saveDist );
//		}
//	};
//	m_NeighborNum++;
//	return m_NeighborNum-1;
//}
//
//void ClearNeighbors ( int i )
//{
//	*(mNbrCnt+i) = 0;
//}
//
//int AddNeighbor( int i, int j, float d )
//{
//	int k = AddNeighbor();
//	m_NeighborTable[k] = j;
//	m_NeighborDist[k] = d;
//	if (*(mNbrCnt+i) == 0 ) *(mNbrNdx+i) = k;
//	(*(mNbrCnt+i))++;
//	return k;
//}
//
//// Ideal grid cell size (gs) = 2 * smoothing radius = 0.02*2 = 0.04
//// Ideal domain size = k*gs/d = k*0.02*2/0.005 = k*8 = {8, 16, 24, 32, 40, 48, ..}
////    (k = number of cells, gs = cell size, d = simulation scale)
//void SetupGridAllocate ( b2Vec3 min, b2Vec3 max, float sim_scale, float cell_size, float border )
//{
//	float world_cellsize = cell_size / sim_scale;
//	
//	m_GridMin = min;
//	m_GridMax = max;
//	m_GridSize = m_GridMax;
//	m_GridSize -= m_GridMin;
//	m_GridRes.x = ceil ( m_GridSize.x / world_cellsize );		// Determine grid resolution
//	m_GridRes.y = ceil ( m_GridSize.y / world_cellsize );
//	m_GridRes.z = ceil ( m_GridSize.z / world_cellsize );
//	m_GridSize.x = m_GridRes.x * cell_size / sim_scale;				// Adjust grid size to multiple of cell size
//	m_GridSize.y = m_GridRes.y * cell_size / sim_scale;
//	m_GridSize.z = m_GridRes.z * cell_size / sim_scale;
//	m_GridDelta = m_GridRes;		// delta = translate from world space to cell #
//	m_GridDelta /= m_GridSize;
//	
//	m_GridTotal = (int)(m_GridRes.x * m_GridRes.y * m_GridRes.z);
//	
//	// Allocate grid
//	if ( m_Grid == 0x0 ) free (m_Grid);
//	if ( m_GridCnt == 0x0 ) free (m_GridCnt);
//	m_Grid = (uint*) malloc ( sizeof(uint*) * m_GridTotal );
//	m_GridCnt = (uint*) malloc ( sizeof(uint*) * m_GridTotal );
//	memset ( m_Grid, GRID_UCHAR, m_GridTotal*sizeof(uint) );
//	memset ( m_GridCnt, GRID_UCHAR, m_GridTotal*sizeof(uint) );
//	
//	m_Param[PSTAT_GMEM] = 12 * m_GridTotal;		// Grid memory used
//	
//	// Number of cells to search:
//	// n = (2r / w) +1,  where n = 1D cell search count, r = search radius, w = world cell width
//	//
//	m_GridSrch =  floor(2*(m_Param[PSMOOTHRADIUS]/sim_scale) / world_cellsize) + 1;
//	if ( m_GridSrch < 2 ) m_GridSrch = 2;
//	m_GridAdjCnt = m_GridSrch * m_GridSrch * m_GridSrch ;			// 3D search count = n^3, e.g. 2x2x2=8, 3x3x3=27, 4x4x4=64
//	
//	if ( m_GridSrch > 6 ) {
//		printf ( "ERROR: Neighbor search is n > 6. \n " );
//		exit(-1);
//	}
//	
//	int cell = 0;
//	for (int y=0; y < m_GridSrch; y++ )
//		for (int z=0; z < m_GridSrch; z++ )
//			for (int x=0; x < m_GridSrch; x++ )
//				m_GridAdj[cell++] = ( y*m_GridRes.z + z )*m_GridRes.x +  x ;			// -1 compensates for ndx 0=empty
//	
//	
//	printf ( "Adjacency table (CPU) \n");
//	for (int n=0; n < m_GridAdjCnt; n++ ) {
//		printf ( "  ADJ: %d, %d\n", n, m_GridAdj[n] );
//	}
//	
//	if ( mPackGrid != 0x0 ) free ( mPackGrid );
//	mPackGrid = (int*) malloc ( sizeof(int) * m_GridTotal );
//	
//	
//}
//
//int getGridCell ( int p, b2Vec3& gc )
//{
//	return getGridCell ( *(mPos+p), gc );
//}
//int getGridCell ( b2Vec3& pos, b2Vec3& gc )
//{
//	gc.x = (int)( (pos.x - m_GridMin.x) * m_GridDelta.x);			// Cell in which particle is located
//	gc.y = (int)( (pos.y - m_GridMin.y) * m_GridDelta.y);
//	gc.z = (int)( (pos.z - m_GridMin.z) * m_GridDelta.z);
//	return (int)( (gc.y*m_GridRes.z + gc.z)*m_GridRes.x + gc.x);
//}
//b2Vec3 getCell ( int c )
//{
//	b2Vec3 gc;
//	int xz = m_GridRes.x*m_GridRes.z;
//	gc.y = c / xz;				c -= gc.y*xz;
//	gc.z = c / m_GridRes.x;		c -= gc.z*m_GridRes.x;
//	gc.x = c;
//	return gc;
//}
//
//void InsertParticles ()
//{
//	int gs;
//	int gx, gy, gz;
//	
//	// Reset all grid pointers and neighbor tables to empty
//	memset ( mGridNext,		GRID_UCHAR, NumPoints()*sizeof(uint) );
//	memset ( mGridCell,		GRID_UCHAR, NumPoints()*sizeof(uint) );
//	memset ( mClusterCell,	GRID_UCHAR, NumPoints()*sizeof(uint) );
//	
//	// Reset all grid cells to empty
//	memset ( m_Grid,		GRID_UCHAR, m_GridTotal*sizeof(uint) );
//	memset ( m_GridCnt,				 0, m_GridTotal*sizeof(uint) );
//	
//	// Insert each particle into spatial grid
//	b2Vec3 gc;
//	b2Vec3* ppos =	mPos;
//	uint* pgrid =		mGridCell;
//	uint* pcell =		mClusterCell;
//	uint* pnext =		mGridNext;
//	
//	float poff = m_Param[PSMOOTHRADIUS] / m_Param[PSIMSCALE];
//	
//	int ns = pow ( m_GridAdjCnt, 1/3.0 );
//	register int xns, yns, zns;
//	xns = m_GridRes.x - m_GridSrch;
//	yns = m_GridRes.y - m_GridSrch;
//	zns = m_GridRes.z - m_GridSrch;
//	
//	m_Param[ PSTAT_OCCUPY ] = 0.0;
//	m_Param [ PSTAT_GRIDCNT ] = 0.0;
//	
//	for ( int n=0; n < NumPoints(); n++ ) {
//		gs = getGridCell ( *ppos, gc );
//		if ( gc.x >= 1 && gc.x <= xns && gc.y >= 1 && gc.y <= yns && gc.z >= 1 && gc.z <= zns ) {
//			// put current particle at head of grid cell, pointing to next in list (previous head of cell)
//			*pgrid = gs;
//			*pnext = m_Grid[gs];
//			if ( *pnext == GRID_UNDEF ) m_Param[ PSTAT_OCCUPY ] += 1.0;
//			m_Grid[gs] = n;
//			m_GridCnt[gs]++;
//			m_Param [ PSTAT_GRIDCNT ] += 1.0;
//			/* -- 1/2 cell offset search method
//			 gx = (int)( (-poff + ppos->x - m_GridMin.x) * m_GridDelta.x);
//			 if ( gx < 0 ) gx = 0;
//			 if ( gx > m_GridRes.x-2 ) gx = m_GridRes.x-2;
//			 gy = (int)( (-poff + ppos->y - m_GridMin.y) * m_GridDelta.y);
//			 if ( gy < 0 ) gy = 0;
//			 if ( gy > m_GridRes.y-2 ) gx = m_GridRes.y-2;
//			 gz = (int)( (-poff + ppos->z - m_GridMin.z) * m_GridDelta.z);
//			 if ( gz < 0 ) gz = 0;
//			 if ( gz > m_GridRes.z-2 ) gz = m_GridRes.z-2;
//			 *pcell = (int)( (gy*m_GridRes.z + gz)*m_GridRes.x + gx) ;	// Cell in which to start 2x2x2 search*/
//		} else {
//			b2Vec3 vel, ve;
//			vel = *(mVel + n);
//			ve = *(mVelEval + n);
//			float pr, dn;
//			pr = *(mPressure + n);
//			dn = *(mDensity + n);
//			//printf ( "WARNING: Out of Bounds: %d, P<%f %f %f>, V<%f %f %f>, prs:%f, dns:%f\n", n, ppos->x, ppos->y, ppos->z, vel.x, vel.y, vel.z, pr, dn );
//			//ppos->x = -1; ppos->y = -1; ppos->z = -1;
//		}
//		pgrid++;
//		ppos++;
//		pnext++;
//		pcell++;
//	}
//	
//	// STATS
//	/*m_Param[ PSTAT_OCCUPY ] = 0;
//	 m_Param[ PSTAT_GRIDCNT ] = 0;
//	 for (int n=0; n < m_GridTotal; n++) {
//	 if ( m_GridCnt[n] > 0 )  m_Param[ PSTAT_OCCUPY ] += 1.0;
//	 m_Param [ PSTAT_GRIDCNT ] += m_GridCnt[n];
//	 }*/
//}

//void FindNbrsSlow ()
//{
//	// O(n^2)
//	// Does not require grid
//	
//	b2Vec3 dst;
//	float dsq;
//	float d2 = m_Param[PSIMSCALE]*m_Param[PSIMSCALE];
//	
//	ResetNeighbors ();
//	
//	b2Vec3 *ipos, *jpos;
//	ipos = mPos;
//	for (int i=0; i < NumPoints(); i++ ) {
//		jpos = mPos;
//		ClearNeighbors ( i );
//		for (int j=0; j < NumPoints(); j++ ) {
//			dst = *ipos;
//			dst -= *jpos;
//			dsq = d2*(dst.x*dst.x + dst.y*dst.y + dst.z*dst.z);
//			if ( i != j && dsq <= m_R2 ) {
//				AddNeighbor( i, j, sqrt(dsq) );
//			}
//			jpos++;
//		}
//		ipos++;
//	}
//}
//
//void FindNbrsGrid ()
//{
//	// O(n^2)
//	// Does not require grid
//	
//	b2Vec3 dst;
//	float dsq;
//	int j;
//	int nadj = (m_GridRes.z + 1)*m_GridRes.x + 1;
//	float d2 = m_Param[PSIMSCALE]*m_Param[PSIMSCALE];
//	
//	ResetNeighbors ();
//	
//	b2Vec3 *ipos, *jpos;
//	ipos = mPos;
//	for (int i=0; i < NumPoints(); i++ ) {
//		ClearNeighbors ( i );
//		
//		if ( *(mGridCell+i) != GRID_UNDEF ) {
//			for (int cell=0; cell < m_GridAdjCnt; cell++) {
//				j = m_Grid [ *(mGridCell+i) - nadj + m_GridAdj[cell] ] ;
//				while ( j != GRID_UNDEF ) {
//					if ( i==j ) { j = *(mGridNext+j); continue; }
//					dst = *ipos;
//					dst -= *(mPos+j);
//					dsq = d2*(dst.x*dst.x + dst.y*dst.y + dst.z*dst.z);
//					if ( dsq <= m_R2 ) {
//						AddNeighbor( i, j, sqrt(dsq) );
//					}
//					j = *(mGridNext+j);
//				}
//			}
//		}
//		ipos++;
//	}
//}
//
//
//// Compute Pressures - Using spatial grid, and also create neighbor table
//void ComputePressureGrid ()
//{
//	int i, j, cnt = 0;
//	int nbr;
//	float dx, dy, dz, sum, dsq, c;
//	float d = m_Param[PSIMSCALE];
//	float d2 = d*d;
//	float radius = m_Param[PSMOOTHRADIUS] / m_Param[PSIMSCALE];
//	
//	b2Vec3*	ipos	= mPos;
//	float*		ipress	= mPressure;
//	float*		idensity = mDensity;
//	uint*		inbr	= mNbrNdx;
//	uint*		inbrcnt = mNbrCnt;
//	
//	b2Vec3	dst;
//	int			nadj = (m_GridRes.z + 1)*m_GridRes.x + 1;
//	int*		jnext;
//	
//	int nbrcnt = 0;
//	int srch = 0;
//	
//	for ( i=0; i < NumPoints(); i++ ) {
//		
//		sum = 0.0;
//		
//		if ( *(mGridCell+i) != GRID_UNDEF ) {
//			for (int cell=0; cell < m_GridAdjCnt; cell++) {
//				j = m_Grid [  *(mGridCell+i) - nadj + m_GridAdj[cell] ] ;
//				while ( j != GRID_UNDEF ) {
//					if ( i==j ) { j = *(mGridNext+j) ; continue; }
//					dst = *(mPos + j);
//					dst -= *ipos;
//					dsq = d2*(dst.x*dst.x + dst.y*dst.y + dst.z*dst.z);
//					if ( dsq <= m_R2 ) {
//						c =  m_R2 - dsq;
//						sum += c * c * c;
//						nbrcnt++;
//						/*nbr = AddNeighbor();			// get memory for new neighbor
//						 *(m_NeighborTable + nbr) = j;
//						 *(m_NeighborDist + nbr) = sqrt(dsq);
//						 inbr->num++;*/
//					}
//					srch++;
//					j = *(mGridNext+j) ;
//				}
//			}
//		}
//		*idensity = sum * m_Param[PMASS] * m_Poly6Kern ;
//		*ipress = ( *idensity - m_Param[PRESTDENSITY] ) * m_Param[PINTSTIFF];
//		*idensity = 1.0f / *idensity;
//		
//		ipos++;
//		idensity++;
//		ipress++;
//	}
//	// Stats:
//	m_Param [ PSTAT_NBR ] = float(nbrcnt);
//	m_Param [ PSTAT_SRCH ] = float(srch);
//	if ( m_Param[PSTAT_NBR] > m_Param [ PSTAT_NBRMAX ] ) m_Param [ PSTAT_NBRMAX ] = m_Param[PSTAT_NBR];
//	if ( m_Param[PSTAT_SRCH] > m_Param [ PSTAT_SRCHMAX ] ) m_Param [ PSTAT_SRCHMAX ] = m_Param[PSTAT_SRCH];
//}
//
//// Compute Forces - Using spatial grid with saved neighbor table. Fastest.
//void ComputeForceGrid ()
//{
//	b2Vec3 force;
//	register float pterm, vterm, dterm;
//	int i, j, nbr;
//	float c, d;
//	float dx, dy, dz;
//	float mR, mR2, visc;
//	
//	d = m_Param[PSIMSCALE];
//	mR = m_Param[PSMOOTHRADIUS];
//	visc = m_Param[PVISC];
//	
//	b2Vec3*	ipos = mPos;
//	b2Vec3*	iveleval = mVelEval;
//	b2Vec3*	iforce = mForce;
//	float*		ipress = mPressure;
//	float*		idensity = mDensity;
//	
//	int			jndx;
//	b2Vec3	jpos;
//	float		jdist;
//	float		jpress;
//	float		jdensity;
//	b2Vec3	jveleval;
//	float		dsq;
//	float		d2 = d*d;
//	int			nadj = (m_GridRes.z + 1)*m_GridRes.x + 1;
//	
//	for ( i=0; i < NumPoints(); i++ ) {
//		
//		iforce->Set ( 0, 0, 0 );
//		
//		if ( *(mGridCell+i) != GRID_UNDEF ) {
//			for (int cell=0; cell < m_GridAdjCnt; cell++) {
//				j = m_Grid [  *(mGridCell+i) - nadj + m_GridAdj[cell] ];
//				while ( j != GRID_UNDEF ) {
//					if ( i==j ) { j = *(mGridNext+j); continue; }
//					jpos = *(mPos + j);
//					dx = ( ipos->x - jpos.x);		// dist in cm
//					dy = ( ipos->y - jpos.y);
//					dz = ( ipos->z - jpos.z);
//					dsq = d2*(dx*dx + dy*dy + dz*dz);
//					if ( dsq <= m_R2 ) {
//						
//						jdist = sqrt(dsq);
//						
//						jpress = *(mPressure + j);
//						jdensity = *(mDensity + j);
//						jveleval = *(mVelEval + j);
//						dx = ( ipos->x - jpos.x);		// dist in cm
//						dy = ( ipos->y - jpos.y);
//						dz = ( ipos->z - jpos.z);
//						c = (mR-jdist);
//						pterm = d * -0.5f * c * m_SpikyKern * ( *ipress + jpress ) / jdist;
//						dterm = c * (*idensity) * jdensity;
//						vterm = m_LapKern * visc;
//						iforce->x += ( pterm * dx + vterm * ( jveleval.x - iveleval->x) ) * dterm;
//						iforce->y += ( pterm * dy + vterm * ( jveleval.y - iveleval->y) ) * dterm;
//						iforce->z += ( pterm * dz + vterm * ( jveleval.z - iveleval->z) ) * dterm;
//					}
//					j = *(mGridNext+j);
//				}
//			}
//		}
//		ipos++;
//		iveleval++;
//		iforce++;
//		ipress++;
//		idensity++;
//	}
//}
//
//
//// Compute Forces - Using spatial grid with saved neighbor table. Fastest.
//void ComputeForceGridNC ()
//{
//	b2Vec3 force;
//	register float pterm, vterm, dterm;
//	int i, j, nbr;
//	float c, d;
//	float dx, dy, dz;
//	float mR, mR2, visc;
//	
//	d = m_Param[PSIMSCALE];
//	mR = m_Param[PSMOOTHRADIUS];
//	visc = m_Param[PVISC];
//	
//	b2Vec3*	ipos = mPos;
//	b2Vec3*	iveleval = mVelEval;
//	b2Vec3*	iforce = mForce;
//	float*		ipress = mPressure;
//	float*		idensity = mDensity;
//	uint*		inbr =	mNbrNdx;
//	uint*		inbrcnt = mNbrCnt;
//	
//	int			jndx;
//	b2Vec3	jpos;
//	float		jdist;
//	float		jpress;
//	float		jdensity;
//	b2Vec3	jveleval;
//	
//	for ( i=0; i < NumPoints(); i++ ) {
//		
//		iforce->Set ( 0, 0, 0 );
//		
//		jndx = *inbr;
//		for (int nbr=0; nbr < *inbrcnt; nbr++ ) {
//			j = *(m_NeighborTable+jndx);
//			jpos = *(mPos + j);
//			jpress = *(mPressure + j);
//			jdensity = *(mDensity + j);
//			jveleval = *(mVelEval + j);
//			jdist = *(m_NeighborDist + jndx);
//			dx = ( ipos->x - jpos.x);		// dist in cm
//			dy = ( ipos->y - jpos.y);
//			dz = ( ipos->z - jpos.z);
//			c = ( mR - jdist );
//			pterm = d * -0.5f * c * m_SpikyKern * ( *ipress + jpress ) / jdist;
//			dterm = c * (*idensity) * jdensity;
//			vterm = m_LapKern * visc;
//			iforce->x += ( pterm * dx + vterm * ( jveleval.x - iveleval->x) ) * dterm;
//			iforce->y += ( pterm * dy + vterm * ( jveleval.y - iveleval->y) ) * dterm;
//			iforce->z += ( pterm * dz + vterm * ( jveleval.z - iveleval->z) ) * dterm;
//			jndx++;
//		}
//		ipos++;
//		iveleval++;
//		iforce++;
//		ipress++;
//		idensity++;
//		inbr++;
//	}
//}
//
//int getLastRecording ()
//{
//	FILE* fp;
//	int num = 0;
//	fp = fopen ( getFilename(num).c_str(), "rb" );
//	while ( fp != 0x0 ) {			// skip existing recordings
//		fclose ( fp );
//		num++;
//		fp = fopen ( getFilename(num).c_str(), "rb" );
//	}
//	return num-1;
//}
//void StartPlayback ( int p )
//{
//	if ( p < 0 ) return;
//	
//	m_Param[PMODE] = RUN_PLAYBACK;
//	mFileNum = p;
//	mFileName = getFilename ( mFileNum );
//	if ( mFP != 0x0 ) { fclose ( mFP ); mFP = 0x0; }
//	char name[100];
//	strcpy ( name, mFileName.c_str() );
//	mFP = fopen ( name, "rb" );
//	if ( mFP==0x0 || ferror(mFP) ) {
//		printf ( "ERROR: Cannot read file %s\n", mFileName.c_str() );
//		perror ( "  " );
//		exit ( -1 );
//	}
//	m_Frame = 0;
//}
//
//void RunPlayback ()
//{
//	if ( feof (mFP) ) StartPlayback ( mFileNum );
//	
//	// Read number of points and channels
//	int result = fread ( &mNumPoints, sizeof(int), 1, mFP );
//	if ( ferror (mFP) || result != 1 )		{ StartPlayback ( mFileNum ); return; }
//	if ( feof(mFP) || mNumPoints <= 0 )		{ StartPlayback ( mFileNum ); return; }
//	
//	int channels, dsize;
//	fread ( &channels, sizeof(int), 1, mFP );
//	
//	// Allocate extra memory if needed
//	if ( mNumPoints > mMaxPoints ) {
//		AllocateParticles ( mNumPoints );
//		AllocatePackBuf ();
//	}
//	
//	char*	dat = mPackBuf;
//	b2Vec3*  ppos =		mPos;
//	b2Vec3*  pvel =		mVel;
//	float*		pdens =		mDensity;
//	DWORD*		pclr =		mClr;
//	
//	// Read data
//	if ( channels == 2 ) {
//		dsize = sizeof(b2Vec3)+sizeof(DWORD);
//		result = fread ( dat, dsize, mNumPoints, mFP );
//		if ( ferror (mFP) || result != mNumPoints ) { StartPlayback ( mFileNum ); return; }
//		for (int n=0; n < mNumPoints; n++ ) {
//			*ppos++ =  *(b2Vec3*) dat;		dat += sizeof(b2Vec3);
//			*pclr++ =  *(DWORD*) dat;			dat += sizeof(DWORD);
//		}
//	} else {
//		dsize = sizeof(b2Vec3)+sizeof(b2Vec3)+sizeof(float)+sizeof(DWORD);
//		result = fread ( dat, dsize, mNumPoints, mFP );
//		if ( ferror (mFP) || result != mNumPoints ) { StartPlayback ( mFileNum ); return; }
//		for (int n=0; n < mNumPoints; n++ ) {
//			*ppos++ =  *(b2Vec3*) dat;		dat += sizeof(b2Vec3);
//			*pvel++ =  *(b2Vec3*) dat;		dat += sizeof(b2Vec3);
//			*pdens++ = *(float*) dat;			dat += sizeof(float);
//			*pclr++ =  *(DWORD*) dat;			dat += sizeof(DWORD);
//		}
//	}
//}
//
//
//
//std::string getModeStr ()
//{
//	char buf[100];
//	
//	switch ( (int) m_Param[PMODE] ) {
//		case RUN_SEARCH:		sprintf ( buf, "SEARCH ONLY (CPU)" );		break;
//		case RUN_VALIDATE:		sprintf ( buf, "VALIDATE GPU to CPU");		break;
//		case RUN_CPU_SLOW:		sprintf ( buf, "SIMULATE CPU Slow");		break;
//		case RUN_CPU_GRID:		sprintf ( buf, "SIMULATE CPU Grid");		break;
//		case RUN_CUDA_RADIX:	sprintf ( buf, "SIMULATE CUDA Radix Sort");	break;
//		case RUN_CUDA_INDEX:	sprintf ( buf, "SIMULATE CUDA Index Sort" ); break;
//		case RUN_CUDA_FULL:	sprintf ( buf, "SIMULATE CUDA Full Sort" );	break;
//		case RUN_CUDA_CLUSTER:	sprintf ( buf, "SIMULATE CUDA Clustering" );	break;
//		case RUN_PLAYBACK:		sprintf ( buf, "PLAYBACK (%s)", mFileName.c_str() ); break;
//	};
//	//sprintf ( buf, "RECORDING (%s, %.4f MB)", mFileName.c_str(), mFileSize ); break;
//	return buf;
//};
//
//
//void getModeClr ()
//{
//	glColor4f ( 1, 1, 0, 1 );
//	/*break;
//	 switch ( mMode ) {
//	 case RUN_PLAYBACK:		glColor4f ( 0, 1, 0, 1 ); break;
//	 case RUN_RECORD:		glColor4f ( 1, 0, 0, 1 ); break;
//	 case RUN_SIM:			glColor4f ( 1, 1, 0, 1 ); break;
//	 }*/
//}
//
//int SelectParticle ( int x, int y, int wx, int wy, Camera3D& cam )
//{
//	Vector4DF pnt;
//	b2Vec3* ppos = mPos;
//	
//	for (int n = 0; n < NumPoints(); n++ ) {
//		pnt = cam.project ( *ppos );
//		pnt.x = (pnt.x+1.0)*0.5 * wx;
//		pnt.y = (pnt.y+1.0)*0.5 * wy;
//		
//		if ( x > pnt.x-8 && x < pnt.x+8 && y > pnt.y-8 && y < pnt.y+8 ) {
//			mSelected = n;
//			return n;
//		}
//		ppos++;
//	}
//	mSelected = -1;
//	return -1;
//}
//
//
//void DrawParticleInfo ( int p )
//{
//	char disp[256];
//	
//	glColor4f ( 1.0, 1.0, 1.0, 1.0 );
//	sprintf ( disp, "Particle: %d", p );		drawText ( 10, 20, disp );
//	
//	b2Vec3 gc;
//	int gs = getGridCell ( p, gc );
//	sprintf ( disp, "Grid Cell:    <%d, %d, %d> id: %d", gc.x, gc.y, gc.z, gs );		drawText ( 10, 40, disp );
//	
//	int cc = *(mClusterCell + p);
//	gc = getCell ( cc );
//	sprintf ( disp, "Cluster Cell: <%d, %d, %d> id: %d", gc.x, gc.y, gc.z, cc );		drawText ( 10, 50, disp );
//	
//	sprintf ( disp, "Neighbors:    " );
//	int cnt = *(mNbrCnt + p);
//	int ndx = *(mNbrNdx + p);
//	for ( int n=0; n < cnt; n++ ) {
//		sprintf ( disp, "%s%d, ", disp, m_NeighborTable[ ndx ] );
//		ndx++;
//	}
//	drawText ( 10, 70, disp );
//	
//	if ( cc != -1 ) {
//		sprintf ( disp, "Cluster Group: ");		drawText ( 10, 90, disp);
//		int cadj;
//		int stotal = 0;
//		for (int n=0; n < m_GridAdjCnt; n++ ) {		// Cluster group
//			cadj = cc+m_GridAdj[n];
//			gc = getCell ( cadj );
//			sprintf ( disp, "<%d, %d, %d> id: %d, cnt: %d ", gc.x, gc.y, gc.z, cc+m_GridAdj[n], m_GridCnt[ cadj ] );	drawText ( 20, 100+n*10, disp );
//			stotal += m_GridCnt[cadj];
//		}
//		
//		sprintf ( disp, "Search Overhead: %f (%d of %d), %.2f%% occupancy", float(stotal)/ cnt, cnt, stotal, float(cnt)*100.0/stotal );
//		drawText ( 10, 380, disp );
//	}
//}
//
//
//
//void SetupKernels ()
//{
//	m_Param [ PDIST ] = pow ( m_Param[PMASS] / m_Param[PRESTDENSITY], 1/3.0 );
//	m_R2 = m_Param [PSMOOTHRADIUS] * m_Param[PSMOOTHRADIUS];
//	m_Poly6Kern = 315.0f / (64.0f * 3.141592 * pow( m_Param[PSMOOTHRADIUS], 9) );	// Wpoly6 kernel (denominator part) - 2003 Muller, p.4
//	m_SpikyKern = -45.0f / (3.141592 * pow( m_Param[PSMOOTHRADIUS], 6) );			// Laplacian of viscocity (denominator): PI h^6
//	m_LapKern = 45.0f / (3.141592 * pow( m_Param[PSMOOTHRADIUS], 6) );
//}
//
//void SetupDefaultParams ()
//{
//	//  Range = +/- 10.0 * 0.006 (r) =	   0.12			m (= 120 mm = 4.7 inch)
//	//  Container Volume (Vc) =			   0.001728		m^3
//	//  Rest Density (D) =				1000.0			kg / m^3
//	//  Particle Mass (Pm) =			   0.00020543	kg						(mass = vol * density)
//	//  Number of Particles (N) =		4000.0
//	//  Water Mass (M) =				   0.821		kg (= 821 grams)
//	//  Water Volume (V) =				   0.000821     m^3 (= 3.4 cups, .21 gals)
//	//  Smoothing Radius (R) =             0.02			m (= 20 mm = ~3/4 inch)
//	//  Particle Radius (Pr) =			   0.00366		m (= 4 mm  = ~1/8 inch)
//	//  Particle Volume (Pv) =			   2.054e-7		m^3	(= .268 milliliters)
//	//  Rest Distance (Pd) =			   0.0059		m
//	//
//	//  Given: D, Pm, N
//	//    Pv = Pm / D			0.00020543 kg / 1000 kg/m^3 = 2.054e-7 m^3
//	//    Pv = 4/3*pi*Pr^3    cuberoot( 2.054e-7 m^3 * 3/(4pi) ) = 0.00366 m
//	//     M = Pm * N			0.00020543 kg * 4000.0 = 0.821 kg
//	//     V =  M / D              0.821 kg / 1000 kg/m^3 = 0.000821 m^3
//	//     V = Pv * N			 2.054e-7 m^3 * 4000 = 0.000821 m^3
//	//    Pd = cuberoot(Pm/D)    cuberoot(0.00020543/1000) = 0.0059 m
//	//
//	// Ideal grid cell size (gs) = 2 * smoothing radius = 0.02*2 = 0.04
//	// Ideal domain size = k*gs/d = k*0.02*2/0.005 = k*8 = {8, 16, 24, 32, 40, 48, ..}
//	//    (k = number of cells, gs = cell size, d = simulation scale)
//	
//	// "The viscosity coefficient is the dynamic viscosity, visc > 0 (units Pa.s),
//	// and to include a reasonable damping contribution, it should be chosen
//	// to be approximately a factor larger than any physical correct viscosity
//	// coefficient that can be looked up in the literature. However, care should
//	// be taken not to exaggerate the viscosity coefficient for fluid materials.
//	// If the contribution of the viscosity force density is too large, the net effect
//	// of the viscosity term will introduce energy into the system, rather than
//	// draining the system from energy as intended."
//	//    Actual visocity of water = 0.001 Pa.s    // viscosity of water at 20 deg C.
//	
//	m_Time = 0;							// Start at T=0
//	m_DT = 0.003;
//	
//	m_Param [ PSIMSCALE ] =		0.005;			// unit size
//	m_Param [ PVISC ] =			0.35;			// pascal-second (Pa.s) = 1 kg m^-1 s^-1  (see wikipedia page on viscosity)
//	m_Param [ PRESTDENSITY ] =	600.0;			// kg / m^3
//	m_Param [ PSPACING ]	=	0.0;			// spacing will be computed automatically from density in most examples (set to 0 for autocompute)
//	m_Param [ PMASS ] =			0.00020543;		// kg
//	m_Param [ PRADIUS ] =		0.02;			// m
//	m_Param [ PDIST ] =			0.0059;			// m
//	m_Param [ PSMOOTHRADIUS ] =	0.01;			// m
//	m_Param [ PINTSTIFF ] =		1.5;
//	m_Param [ PEXTSTIFF ] =		50000.0;
//	m_Param [ PEXTDAMP ] =		100.0;
//	m_Param [ PACCEL_LIMIT ] =	150.0;			// m / s^2
//	m_Param [ PVEL_LIMIT ] =	3.0;			// m / s
//	m_Param [ PMAX_FRAC ] = 1.0;
//	m_Param [ PPOINT_GRAV_AMT ] = 0.0;
//	
//	m_Param [ PGROUND_SLOPE ] = 0.0;
//	m_Param [ PFORCE_MIN ] = 0.0;
//	m_Param [ PFORCE_MAX ] = 0.0;
//	m_Param [ PFORCE_FREQ ] = 8.0;
//	m_Toggle [ PWRAP_X ] = false;
//	m_Toggle [ PWALL_BARRIER ] = false;
//	m_Toggle [ PLEVY_BARRIER ] = false;
//	m_Toggle [ PDRAIN_BARRIER ] = false;
//	
//	m_Param [ PSTAT_NBRMAX ] = 0 ;
//	m_Param [ PSTAT_SRCHMAX ] = 0 ;
//	
//	m_Vec [ PPOINT_GRAV_POS ].Set ( 0, 50, 0 );
//	m_Vec [ PPLANE_GRAV_DIR ].Set ( 0, -9.8, 0 );
//	m_Vec [ PEMIT_POS ].Set ( 0, 0, 0 );
//	m_Vec [ PEMIT_RATE ].Set ( 0, 0, 0 );
//	m_Vec [ PEMIT_ANG ].Set ( 0, 90, 1.0 );
//	m_Vec [ PEMIT_DANG ].Set ( 0, 0, 0 );
//	
//	// Default sim config
//	m_Toggle [ PRUN ] = true;				// Run integrator
//	m_Param [PGRIDSIZE] = m_Param[PSMOOTHRADIUS] * 2;
//	m_Param [PDRAWMODE] = 1;				// Sprite drawing
//	m_Param [PDRAWGRID] = 0;				// No grid
//	m_Param [PDRAWTEXT] = 0;				// No text

//void SetupExampleParams ( bool bStart )
//{
//	b2Vec3 pos;
//	b2Vec3 min, max;
//	
//	switch ( (int) m_Param[PEXAMPLE] ) {
//			
//		case 0:	{	// Regression test. N x N x N static grid
//			
//			int k = ceil ( pow ( (float) m_Param[PNUM], (float) 1.0/3.0f ) );
//			m_Vec [ PVOLMIN ].Set ( 0, 0, 0 );
//			m_Vec [ PVOLMAX ].Set ( 2+(k/2), 2+(k/2), 2+(k/2) );
//			m_Vec [ PINITMIN ].Set ( 1, 1, 1 );
//			m_Vec [ PINITMAX ].Set ( 1+(k/2), 1+(k/2), 1+(k/2) );
//			
//			m_Param [ PPOINT_GRAV_AMT ] = 0.0;		// No gravity
//			m_Vec [ PPLANE_GRAV_DIR ].Set ( 0, 0, 0 );
//			m_Param [ PSPACING ] = 0.5;				// Fixed spacing		Dx = x-axis density
//			m_Param [ PSMOOTHRADIUS ] =	m_Param [PSPACING];		// Search radius
//			m_Toggle [ PRUN ] = false;				// Do NOT run sim. Neighbors only.
//			m_Param [PDRAWMODE] = 0;				// Point drawing
//			m_Param [PDRAWGRID] = 1;				// Grid drawing
//			m_Param [PDRAWTEXT] = 1;				// Text drawing
//			m_Param [PSIMSCALE ] = 1.0;
//			
//		} break;
//		case 1:		// Wave pool
//			m_Vec [ PVOLMIN ].Set ( -100, 0, -100 );
//			m_Vec [ PVOLMAX ].Set (  100, 100, 100 );
//			m_Vec [ PINITMIN ].Set ( -50, 20, -90 );
//			m_Vec [ PINITMAX ].Set (  90, 90,  90 );
//			m_Param [ PFORCE_MIN ] = 10.0;
//			m_Param [ PGROUND_SLOPE ] = 0.04;
//			break;
//		case 2:		// Large coast
//			m_Vec [ PVOLMIN ].Set ( -200, 0, -40 );
//			m_Vec [ PVOLMAX ].Set (  200, 200, 40 );
//			m_Vec [ PINITMIN ].Set ( -120, 40, -30 );
//			m_Vec [ PINITMAX ].Set (  190, 190,  30 );
//			m_Param [ PFORCE_MIN ] = 20.0;
//			m_Param [ PGROUND_SLOPE ] = 0.10;
//			break;
//		case 3:		// Small dam break
//			m_Vec [ PVOLMIN ].Set ( -40, 0, -40  );
//			m_Vec [ PVOLMAX ].Set ( 40, 60, 40 );
//			m_Vec [ PINITMIN ].Set ( 0, 8, -35 );
//			m_Vec [ PINITMAX ].Set ( 35, 55, 35 );
//			m_Param [ PFORCE_MIN ] = 0.0;
//			m_Param [ PFORCE_MAX ] = 0.0;
//			m_Vec [ PPLANE_GRAV_DIR ].Set ( 0.0, -9.8, 0.0 );
//			break;
//		case 4:		// Dual-Wave pool
//			m_Vec [ PVOLMIN ].Set ( -100, 0, -15 );
//			m_Vec [ PVOLMAX ].Set ( 100, 100, 15 );
//			m_Vec [ PINITMIN ].Set ( -80, 8, -10 );
//			m_Vec [ PINITMAX ].Set ( 80, 90, 10 );
//			m_Param [ PFORCE_MIN ] = 20.0;
//			m_Param [ PFORCE_MAX ] = 20.0;
//			m_Vec [ PPLANE_GRAV_DIR ].Set ( 0.0, -9.8, 0 );
//			break;
//		case 5:		// Microgravity
//			m_Vec [ PVOLMIN ].Set ( -80, 0, -80 );
//			m_Vec [ PVOLMAX ].Set ( 80, 100, 80 );
//			m_Vec [ PINITMIN ].Set ( -60, 40, -60 );
//			m_Vec [ PINITMAX ].Set ( 60, 80, 60 );
//			m_Vec [ PPLANE_GRAV_DIR ].Set ( 0, -1, 0 );
//			m_Param [ PGROUND_SLOPE ] = 0.1;
//			break;
//	}
//	
//}
//
//void SetupSpacing ()
//{
//	m_Param [ PSIMSIZE ] = m_Param [ PSIMSCALE ] * (m_Vec[PVOLMAX].z - m_Vec[PVOLMIN].z);
//	
//	if ( m_Param[PSPACING] == 0 ) {
//		// Determine spacing from density
//		m_Param [PDIST] = pow ( m_Param[PMASS] / m_Param[PRESTDENSITY], 1/3.0 );
//		m_Param [PSPACING] = m_Param [ PDIST ]*0.87 / m_Param[ PSIMSCALE ];
//	} else {
//		// Determine density from spacing
//		m_Param [PDIST] = m_Param[PSPACING] * m_Param[PSIMSCALE] / 0.87;
//		m_Param [PRESTDENSITY] = m_Param[PMASS] / pow ( m_Param[PDIST], 3.0 );
//	}
//	printf ( "Add Particles. Density: %f, Spacing: %f, PDist: %f\n", m_Param[PRESTDENSITY], m_Param [ PSPACING ], m_Param[ PDIST ] );
//	
//	// Particle Boundaries
//	m_Vec[PBOUNDMIN] = m_Vec[PVOLMIN];		m_Vec[PBOUNDMIN] += 2.0*(m_Param[PGRIDSIZE] / m_Param[PSIMSCALE]);
//	m_Vec[PBOUNDMAX] = m_Vec[PVOLMAX];		m_Vec[PBOUNDMAX] -= 2.0*(m_Param[PGRIDSIZE] / m_Param[PSIMSCALE]);
//}
//
//
//void TestPrefixSum ( int num )
//{
//	printf ( "------------------\n");
//	printf ( "TESTING PREFIX SUM\n");
//	printf ( "Num: %d\n", num );
//	
//	srand ( 2564 );		// deterministic test
//	
//	// Allocate input and output lists
//	int* listIn = (int*) malloc( num * sizeof(int) );
//	int* listOutCPU = (int*) malloc( num * sizeof(int) );
//	int* listOutGPU = (int*) malloc( num * sizeof(int) );
//	
//	// Build list of pseudo-random numbers
//	for (int n=0; n < num; n++)
//		listIn[n] = int ((rand()*4.0f) / RAND_MAX);
//	printf ( "Input: "); for (int n=num-10; n < num; n++)	printf ( "%d ", listIn[n] ); printf (" (last 10 values)\n");		// print first 10
//	
//	// Prefix Sum on CPU
//	int sum = 0;
//	mint::Time start, cpu_stop, gpu_stop;
//	start.SetSystemTime ( ACC_NSEC );
//	for (int n=0; n < num; n++) {
//		listOutCPU[n] = sum;
//		sum += listIn[n];
//	}
//	cpu_stop.SetSystemTime ( ACC_NSEC ); cpu_stop = cpu_stop - start;
//	printf ( "CPU:   "); for (int n=num-10; n < num; n++)	printf ( "%d ", listOutCPU[n] ); printf (" (last 10 values)\n");		// print first 10
//	
//	// Prefix Sum on GPU
//	prefixSumToGPU ( (char*) listIn, num, sizeof(int) );
//	start.SetSystemTime ( ACC_NSEC );
//	prefixSumInt ( num );
//	gpu_stop.SetSystemTime ( ACC_NSEC ); gpu_stop = gpu_stop - start;
//	prefixSumFromGPU ( (char*) listOutGPU, num, sizeof(int) );
//	
//	printf ( "GPU:   "); for (int n=num-10; n < num; n++)	printf ( "%d ", listOutGPU[n] ); printf (" (last 10 values)\n");		// print first 10
//	
//	printf ( "Time CPU: %s\n", cpu_stop.GetReadableTime().c_str() );
//	printf ( "Time GPU: %s\n", gpu_stop.GetReadableTime().c_str() );
//	
//	// Validate results
//	int ok = 0;
//	for (int n=0; n < num; n++) {
//		if ( listOutCPU[n] == listOutGPU[n] ) ok++;
//	}
//	printf ( "Validate: %d OK. (Bad: %d)\n", ok, num-ok );
//	printf ( "Press any key to continue..\n");
//	_getch();
//}

@end


