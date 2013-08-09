// Import the interfaces
#import "World3.h"
#import "Common.h"
#import "DLRenderTexture.h"

#define MH(_p_) (SIZE.height / 100.0f * _p_) / PTM_RATIO

float InterPriclScaleFactor;

bool ParticleSolidCollision3(b2Fixture* fixture, b2Vec2& particlePos, b2Vec2& nearestPos, b2Vec2& impactNormal, float particleRadius);
void SeparateParticleFromBody3(int particleIdx, b2Vec2& nearestPos, b2Vec2& normal, tParticle *liquid,float knorm, float ktang);

float sq2 (float x) {return x * x;}

bool QueWorldInteractions::ReportFixture(b2Fixture* fixture)
{
    int numParticles = hashGridList[x][y].GetSize();
    hashGridList[x][y].ResetIterator();
    for(int i = 0; i < numParticles; i++)
    {
        int particleIdx = hashGridList[x][y].GetNext();
        b2Vec2 particlePos=b2Vec2( CC_CONTENT_SCALE_FACTOR() * liquid[particleIdx].mR.x * InterPriclScaleFactor,
                                   CC_CONTENT_SCALE_FACTOR() * liquid[particleIdx].mR.y * InterPriclScaleFactor);
        if(fixture->GetBody()->GetType() == b2_staticBody)
        {
            b2Vec2 nearestPos(0,0);
            b2Vec2 normal(0,0);
            bool inside = ParticleSolidCollision3(fixture, particlePos, nearestPos, normal, PartRadius);
            if (inside)
            {
                SeparateParticleFromBody3(particleIdx, nearestPos, normal, liquid, knorm, ktang);
            }
        }
        else
        {
            b2Vec2 nearestPos(0,0);
            b2Vec2 normal(0,0);
            bool inside = ParticleSolidCollision3(fixture, particlePos, nearestPos, normal, PartRadius);
            if (inside)
            {
                float dotnv = normal.x * liquid[particleIdx].mV.x + normal.y * liquid[particleIdx].mV.y;
                if (dotnv<0)
                  {                                           
                   b2Vec2 pointVelocity = fixture->GetBody()->GetLinearVelocityFromWorldPoint(particlePos);                   
                   b2Vec2 particleVelocity = b2Vec2(liquid[particleIdx].mV.x,liquid[particleIdx].mV.y);                    
                   particleVelocity *= deltaT;            
                   pointVelocity *= deltaT;                      
                   b2Vec2 relativeVelocity = particleVelocity - pointVelocity;                      
                   b2Vec2 pointVelNormal = normal;
                   pointVelNormal *= b2Dot(relativeVelocity, normal);
                   b2Vec2 pointVelTangent = relativeVelocity - pointVelNormal;                      
                   const float slipFriction = 0.1f;
                   pointVelTangent*= slipFriction;
					  
//                   liquid[particleIdx].mV.x -= 0.9f * 2 * dotnv * normal.x +  pointVelocity.x + pointVelTangent.x;
//                   liquid[particleIdx].mV.y -= 0.9f * 2 * dotnv * normal.y +  pointVelocity.y  + pointVelTangent.y;
//                   liquid[particleIdx].mR = liquid[particleIdx].mOldR + gTimeStep * liquid[particleIdx].mV;
					liquid[particleIdx].mR = tVector2(nearestPos.x, nearestPos.y)/(InterPriclScaleFactor * CC_CONTENT_SCALE_FACTOR());
			//		  liquid[particleIdx].mV = tVector2(pointVelocity.x,pointVelocity.y);
					  
                  }
                  float CurPres = liquid[particleIdx].mP;
                  if (CurPres>0)
                  {
                   fixture->GetBody()->ApplyLinearImpulse(-0.9f*sq2(InterPriclScaleFactor/12)*CurPres * PartRadius * normal, particlePos);
                  }
            }
        }
    }
    return true;
}

#define EPSILON			0.00001f			//for collision detection

static float fluidMinX;
static float fluidMaxX;
static float fluidMinY;
static float fluidMaxY;
float sizeh, sizew;

//NSArray * WaterCell;
const int CouVert =5;
float WaterCell[CouVert][2]={{20,50},{39,42},{70,40.f},{85, 50.f},{32, 60}};


@implementation World3

tSpatialGrid::tSpatialGrid(const tVector2 & domainMin,
                           const tVector2 & domainMax,
                           tScalar delta)
{
    mDomainMin = domainMin;
    mDomainMax = domainMax;
    tVector2 range = domainMax - domainMin;
    int nx = (int) (range.x / delta);
    int ny = (int) (range.y / delta);
    assert(nx >= 1);
    assert(ny >= 1);
    mDelta.x = range.x / nx;
    mDelta.y = range.y / ny;
    tSpatialGridEntry emptyEntry = {0};
    mGridEntries.Resize(nx, ny);
    mGridEntries.SetTo(emptyEntry);
}

void tSpatialGrid::PopulateGrid(tParticle * particles, int nParticles)
{
    int i, j, iParticle;
    const int nx = mGridEntries.GetNx();
    const int ny = mGridEntries.GetNy();
    for (i = 0 ; i < nx ; ++i)
    {
        for (j = 0 ; j < ny ; ++j)
        {
            mGridEntries(i, j).mFirstParticle = 0;
        }
    }
    for (iParticle = 0 ; iParticle < nParticles ; ++iParticle)
    {
        tParticle & p = particles[iParticle];
        i = (int) ((p.mR.x - mDomainMin.x) / mDelta.x);
        j = (int) ((p.mR.y - mDomainMin.y) / mDelta.y);
        tSpatialGridEntry & grid = mGridEntries(i, j);
        
        if (0 == grid.mFirstParticle)
        {
            grid.mFirstParticle = &p;
            p.mNext = 0;
        }
        else
        {
            // insert at the beginning
            tParticle * next = grid.mFirstParticle;
            p.mNext = next;
            grid.mFirstParticle = &p;
        }
    }
}


tSpatialGridIterator::tSpatialGridIterator()
{
    mNParticleLists = 0;
    mCurrentListIndex = 0;
    mCurrentParticle = 0;
}

tParticle * tSpatialGridIterator::FindFirst(const tVector2 & pos,
                                            const tSpatialGrid & grid)
{
    const int nx = grid.mGridEntries.GetNx();
    const int ny = grid.mGridEntries.GetNy();
    int i = (int) ((pos.x - grid.mDomainMin.x) / grid.mDelta.x);
    int j = (int) ((pos.y - grid.mDomainMin.y) / grid.mDelta.y);
    mNParticleLists = 0;
    mCurrentListIndex = 0;
    mCurrentParticle = 0;
    // set up iteration through adjacent cells
    const int di[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
    const int dj[9] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
    
    for (unsigned iCell = 9 ; iCell-- != 0 ; )
    {
        const int ii = i + di[iCell];
        const int jj = j + dj[iCell];
        if (ii >= 0 && ii < nx && jj >= 0 && jj < ny)
        {
            if (grid.mGridEntries(ii, jj).mFirstParticle != 0)
            {
                mParticleLists[mNParticleLists] = grid.mGridEntries(ii, jj).mFirstParticle;
                ++mNParticleLists;
                assert(mNParticleLists <= 9);
            }
        }
    }
    if (mNParticleLists > 0)
    {
        mCurrentParticle = mParticleLists[mCurrentListIndex];
        return mCurrentParticle;
    }
    else
    {
        return 0;
    }
}

inline float myMap(float val, float minInput, float maxInput, float minOutput, float maxOutput)
{
    float result = (val - minInput) / (maxInput - minInput);
    result *= (maxOutput - minOutput);
    result += minOutput;
    return result;
}

inline int hashX(float x)
{
    float f = myMap(x, fluidMinX, fluidMaxX, 0, hashWidth-.001f);
    return (int)f;
}

inline int hashY(float y)
{
    float f = myMap(y, fluidMinY, fluidMaxY, 0, hashHeight-.001f);
    return (int)f;
}

-(id) init
{
	if( (self=[super init]))
	{
		// enable accelerometer
		self.isAccelerometerEnabled = YES;
		CGSize t=[Common size];
		fluidMinX = MW(0);
		fluidMaxX = MW(100);
		fluidMinY = MH(0);
		fluidMaxY = MH(100);
		[self schedule: @selector(tick:) interval:1.0f / 60.0f];
        lastTime=0;thisTime=0;
	}
	return self;
}
////==============================================================
//// WPoly6
////==============================================================
- (tScalar) WPoly6 : (const tVector2) r
{
    tScalar r2 = r.GetLengthSq();
    if (r2 > gKernelH2) return 0.0f;
    tScalar a = gKernelH2 - r2;
    return gWPoly6Scale * a * a * a;
}
//==============================================================
// WPoly6Grad
//==============================================================
- (tVector2) WPoly6Grad : (const tVector2) r
{
    tScalar r2 = r.GetLengthSq();
    if (r2 > gKernelH2) return tVector2(0.0f, 0.0f);
    const tScalar f = -6.0f * gWPoly6Scale;
    tScalar a = gKernelH2 - r2;
    tScalar b = f * a * a;
    return tVector2(b * r.x, b * r.y);
}
//==============================================================
// WSpiky
//==============================================================
- (tScalar) WSpiky : (const tVector2) R
{
    tScalar r2 = R.GetLengthSq();
    if (r2 > gKernelH2) return 0.0f;
    tScalar a = gKernelH - sqrt(r2);
    return gWSpikyScale * a * a * a;
}
//==============================================================
// WSpikyGrad
//==============================================================
- (tVector2) WSpikyGrad : (const tVector2) R
{
    tScalar r2 = R.GetLengthSq();
    if (r2 > gKernelH2) return tVector2(0.0f, 0.0f);
    static const tScalar minR2 = 1.0E-12;
    if (r2 < minR2) r2 = minR2;
    tScalar r = sqrt(r2);
    tScalar a = -3.0f * gWSpikyScale * (gKernelH - r) * (gKernelH - r) / r;
    return tVector2(a * R.x, a * R.y);
}
//==============================================================
// WViscosity
//==============================================================
- (tScalar) WViscosity : (const tVector2) R
{
    tScalar r2 = R.GetLengthSq();
    if (r2 > gKernelH2) return 0.0f;
    static const tScalar minR2 = 1.0E-12;
    if (r2 < minR2) r2 = minR2;
    tScalar r = sqrt(r2);
    tScalar r3 = r * r * r;
    return gWViscosityScale * ( ( (-r3 / (2.0f * gKernelH3)) +
                                 (r2 / gKernelH2) +
                                 gKernelH / (2.0f * r)) - 1.0f);
}
//========================================================
// WViscosityLap
//========================================================
- (tScalar) WViscosityLap : (const tVector2) R
{
    tScalar r2 = R.GetLengthSq();
    if (r2 > gKernelH2) return 0.0f;
    tScalar r = sqrt(r2);
    return gWViscosityScale * (6.0f / gKernelH3) * (gKernelH - r);
}
//==============================================================
// WLucy
// My cat's called Lucy
//==============================================================
- (tScalar) WLucy : (const tVector2) R
{
    tScalar r2 = R.GetLengthSq();
    if (r2 > gKernelH2) return 0.0f;
    tScalar r = sqrt(r2);
    tScalar a = (1.0f + 3.0f * r / gKernelH) * cube(1 - r / gKernelH);
    return gWLucyScale * a;
}
//==============================================================
// WLucyGrad
//==============================================================
- (tVector2) WLucyGrad : (const tVector2) R
{
    tScalar r2 = R.GetLengthSq();
    if (r2 > gKernelH2) return tVector2(0.0f, 0.0f);
    tScalar r = sqrt(r2);
    tScalar a = -12.0f * (gKernelH2 - 2.0f * r * gKernelH + r2) / gKernelH4;
    return tVector2(a * R.x, a * R.y);
}

-(void)draw
{
	[Common draw];
}

int PriorArrayIndex (int idx)
{
    if (idx > 0)
        return idx-1;
    else return CouVert-1;
}

int NextArrayIndex (int idx)
{
    return ((idx + 1) % CouVert);
}

-(void)createbody
{
	for (int i=0;i<2;i++)
	{
	// Define the dynamic body.
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
    float UNITw=MW(1.f),UNITh=MH(1.f);
	bodyDef.position.Set(0,0);
	b2Body *body = WORLD->CreateBody(&bodyDef);
	// Define another box shape for our dynamic body.
	//b2CircleShape blob;    blob.m_radius = 8/PTM_RATIO;
    b2Vec2 *vrts;
    //float reg=7.f;
    
    vrts=new b2Vec2[5];
    vrts[0]=b2Vec2((30.f + 33.f * i) * UNITw, (80.f + 0.f * i) * UNITh);
    vrts[1]=b2Vec2((35.f + 33.f * i) * UNITw, (71.f + 0.f * i) * UNITh);
    vrts[2]=b2Vec2((50.f + 33.f * i) * UNITw, (71.f + 0.f * i) * UNITh);
    vrts[3]=b2Vec2((55.f + 33.f * i) * UNITw, (80.f + 0.f * i) * UNITh);
    vrts[4]=b2Vec2((40.f + 33.f * i) * UNITw, (85.f + 0.f * i) * UNITh);
    
    b2PolygonShape blob;
    blob.Set(vrts, 5);
    
	// Define the dynamic body fixture.
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &blob;
	fixtureDef.density = 0.3f;
	fixtureDef.friction = 0.1f;
    fixtureDef.restitution = 0.4f;
	body->CreateFixture(&fixtureDef);
	}

    //free(vrts);
}


-(void)particlesCountUp:(NSInteger)diff_
{
    CGSize size = [[CCDirector sharedDirector] winSize];
    sizeh = size.height;sizew = size.width;    
    [self createbody];
    tScalar gContainerWidthFrac = 1.f;
    tScalar gContainerHeightFrac = 1.f;
    tScalar gInitialFluidHeightFrac = 0.25f;
    tScalar kernelScale = 4.5f;
    gBoundaryForceScale = 0.0f;
    gBoundaryForceScaleDist = 0.1f;
    gWindowW       =  size.width ;
    gWindowH       =  size.height ;
    gRenderParticles  =   false;
    gNRenderSamplesX   =     96;
    gNRenderSamplesY   =     64;
    gRenderDensityMin  =    500;
    gRenderDensityMax  =    800;
    gDomainX           =      2;
    gDensity0        =     4500.0;
    gViscosity       =        0.28;    //  0.25
    gGasK            =         5;     //  3.2
    gTimeStep       =         0.01;
    gTimeScale       =        1.0;
    gBoundaryForceScale   = 400.0;
    gBoundaryForceScaleDist = 0.03;
    gCreateObject       =  true;
    gObjectDensityFrac   =    0.5;
    gObjectBoundaryScale  =  20.0;
    gObjectBuoyancyScale  =  12.0;
    gParticleRadius = 0.026f;
    float gPositionRadius  = 0.048f;
    gDomainY = gDomainX * gWindowH / gWindowW;
    gContainerWidth = gContainerWidthFrac * gDomainX;
    gContainerHeight = gContainerHeightFrac * gDomainY;
    gContainerX = 0.5f * gDomainX - 0.5f * gContainerWidth;
    gContainerY = 0;//0.1f * gDomainY/CC_CONTENT_SCALE_FACTOR();
    gInitialFluidHeight = gInitialFluidHeightFrac * gContainerHeight;
    gKernelH = kernelScale * gParticleRadius;
    gKernelH9 = pow(gKernelH, 9.0f);
    gKernelH6 = pow(gKernelH, 6.0f);
    gKernelH4 = pow(gKernelH, 4.0f);
    gKernelH3 = pow(gKernelH, 3.0f);
    gKernelH2 = pow(gKernelH, 2.0f);
    RIntoSpr_Factor = CC_CONTENT_SCALE_FACTOR() / PTM_RATIO;
    [self NormaliseKernels];
    InterPriclScaleFactor =12 *size.height /1024.f;
    int i;
    gNParticles=0;
    int MinHeiIndex,  RightMax;
    float minhval = 10000.f, maxhval = -1.f;
    for(int i = 0; i < CouVert; i++)
    {
        if (minhval>WaterCell[i][1])
        {
            minhval = WaterCell[i][1];
            MinHeiIndex = i;
        }
        if (maxhval < WaterCell[i][1])
        {
            maxhval = WaterCell[i][1];
            RightMax = i;
        }
    }
    float unith =  gContainerHeight/100,//*sizeh/1024.f,
          unitw =  gContainerWidth/100;// ; * sizew/768.f;
   
    if ( WaterCell[PriorArrayIndex(MinHeiIndex)][0] < WaterCell[MinHeiIndex][0])
    {
        i = MinHeiIndex;
        int k = MinHeiIndex, nextk = NextArrayIndex(MinHeiIndex);
        while (i!= RightMax)
        {
            int j = PriorArrayIndex(i);
            for(float cY = WaterCell[i][1] * unith; cY < WaterCell[j][1]*unith-eps; cY +=gPositionRadius)
            {
                if (WaterCell[nextk][1]*unith < cY-eps)
                {
                    k = nextk;
                    nextk = NextArrayIndex(k);
                }
                float MinBound = WaterCell[i][0]*unitw +
                (cY - WaterCell[i][1]*unith)*
                (WaterCell[j][0]*unitw-WaterCell[i][0]*unitw)/
                (WaterCell[j][1]*unith-WaterCell[i][1]*unith) ,
                MaxBound    = WaterCell[k][0]*unitw +
                (cY - WaterCell[k][1]*unith)*
                (WaterCell[nextk][0]*unitw-WaterCell[k][0]*unitw)/
                (WaterCell[nextk][1]*unith-WaterCell[k][1]*unith);
                for(float cX=  MinBound ;  cX<  MaxBound; cX += gPositionRadius    )     gNParticles++;
            }
            i = j;
        }
    }
    else
    {
        i = MinHeiIndex;
        int k = MinHeiIndex, nextk = PriorArrayIndex(MinHeiIndex);
        while (i!= RightMax)
        {
            int j = NextArrayIndex(i);
            for(float cY = WaterCell[i][1]*unith; cY < WaterCell[j][1]*unith-eps; cY +=gPositionRadius)
            {
                if (WaterCell[nextk][1]*unith < cY-eps)
                {
                    k = nextk;
                    nextk = PriorArrayIndex(k);
                }
                float MinBound = WaterCell[i][0]*unitw +
                (cY - WaterCell[i][1]*unith)*
                (WaterCell[j][0]*unitw-WaterCell[i][0]*unitw)/
                (WaterCell[j][1]*unith-WaterCell[i][1]*unith) ,
                MaxBound    = WaterCell[k][0]*unitw +
                (cY - WaterCell[k][1]*unith)*
                (WaterCell[nextk][0]*unitw-WaterCell[k][0]*unitw)/
                (WaterCell[nextk][1]*unith-WaterCell[k][1]*unith);
                for(float cX=  MinBound ;    cX<  MaxBound;  cX += gPositionRadius    )   gNParticles++;
            }
            i = j;
        }
    }
    
    tScalar volume = 4 * gNParticles * gParticleRadius * gParticleRadius;
    gParticleMass = volume * gDensity0 / gNParticles;
    gParticles = new tParticle[gNParticles];
    gSpatialGrid = new tSpatialGrid(
                                    tVector2(-gContainerWidth, -gContainerHeight),
                                    tVector2(gDomainX + gContainerWidth, gDomainY + gContainerHeight), gKernelH);
    if (gParticles)
	{
		intersectQueryCallback = new QueWorldInteractions(hashGridList, gParticles,knorm,ktang,gParticleRadius,gTimeStep);
	}
    int Indx=0;
    if ( WaterCell[PriorArrayIndex(MinHeiIndex)][0] < WaterCell[MinHeiIndex][0])
    {
        i = MinHeiIndex;
        int k = MinHeiIndex, nextk = NextArrayIndex(MinHeiIndex);
        while (i!= RightMax)
        {
            int j = PriorArrayIndex(i);
            for(float cY = WaterCell[i][1]*unith; cY < WaterCell[j][1]*unith-eps; cY +=gPositionRadius)
            {
                if (WaterCell[nextk][1]*unith < cY-eps)
                {
                    k = nextk;
                    nextk = NextArrayIndex(k);
                }
                float MinBound = WaterCell[i][0]*unitw +
                (cY - WaterCell[i][1]*unith)*
                (WaterCell[j][0]*unitw-WaterCell[i][0]*unitw)/
                (WaterCell[j][1]*unith-WaterCell[i][1]*unith) ,
                MaxBound    = WaterCell[k][0]*unitw +
                (cY - WaterCell[k][1]*unith)*
                (WaterCell[nextk][0]*unitw-WaterCell[k][0]*unitw)/
                (WaterCell[nextk][1]*unith-WaterCell[k][1]*unith);
                for(float cX=  MinBound ;  cX<  MaxBound; cX += gPositionRadius    )
                {
                    gParticles[Indx].mR.Set(cX , cY);
                    Indx++;
                }
            }
            i = j;
        }
    }
    else
    {
        i = MinHeiIndex;
        int k = MinHeiIndex, nextk = PriorArrayIndex(MinHeiIndex);
        while (i!= RightMax)
        {
            int j = NextArrayIndex(i);
            for(float cY = WaterCell[i][1]*unith; cY < WaterCell[j][1]*unith-eps; cY +=gPositionRadius)
            {
                if (WaterCell[nextk][1] * unith < cY - eps)
                {
                    k = nextk;
                    nextk = PriorArrayIndex(k);
                }
                float MinBound = WaterCell[i][0]*unitw +
                (cY - WaterCell[i][1]*unith)*
                (WaterCell[j][0]*unitw-WaterCell[i][0]*unitw)/
                (WaterCell[j][1]*unith-WaterCell[i][1]*unith) ,
                MaxBound    = WaterCell[k][0]*unitw +
                (cY - WaterCell[k][1]*unith)*
                (WaterCell[nextk][0]*unitw-WaterCell[k][0]*unitw)/
                (WaterCell[nextk][1]*unith-WaterCell[k][1]*unith);
                for(float cX=  MinBound ;  cX<  MaxBound;   cX += gPositionRadius    )
                {
                    gParticles[Indx].mR.Set(cX, cY);
                    Indx++;
                }
            }
            i = j;
        }
    }
    for (int i = 0 ; i < gNParticles ; ++i)
    {
        CCSprite *sprite = [CCSprite spriteWithFile:@"drop.png"]; // создать указатель на спрайт
        sprite.position = ccp(InterPriclScaleFactor * gParticles[i].mR.x * PTM_RATIO,
							  InterPriclScaleFactor * gParticles[i].mR.y * PTM_RATIO) ;
		sprite.visible = false;
        gParticles[i].mOldR = gParticles[i].mR;
        gParticles[i].mV.Set(0.0f, 0.0f);
        gParticles[i].mDensity = gDensity0;
        gParticles[i].mP = [self CalculatePressure : gDensity0];
        gParticles[i].mPressureForce.Set(0.0f, 0.0f);
        gParticles[i].mViscosityForce.Set(0.0f, 0.0f);
        gParticles[i].mBodyForce.Set(0.0f, 0.0f);
        gParticles[i].mCs = 0.0f;
        gParticles[i].mN.Set(0.0f, 0.0f);
        gParticles[i].mNext = 0;
        gParticles[i].sp=sprite;
        gParticles[i].sp.scaleX = 0.12f * CC_CONTENT_SCALE_FACTOR() * InterPriclScaleFactor;
        gParticles[i].sp.scaleY = 0.12f * CC_CONTENT_SCALE_FACTOR() * InterPriclScaleFactor;
        [BATCH addChild:gParticles[i].sp];
    }
}

//==============================================================
// ImposeBoundaryConditions
//==============================================================
-(void) ImposeBoundaryConditions
{
    // velocity scaling when colliding with walls
    tVector2 normal;
    for (int i = gNParticles ; i-- != 0 ; )
    {
        if (gParticles[i].mR.x < gContainerX)
        {
            gParticles[i].mR.x = gContainerX;
        }
        else if (gParticles[i].mR.x > gContainerX + gContainerWidth)
        {
            gParticles[i].mR.x = gContainerX + gContainerWidth;
        }
        if (gParticles[i].mR.y < gContainerY)
        {
            gParticles[i].mR.y = gContainerY;
        }
        else if (gParticles[i].mR.y > gContainerY + gContainerHeight)
        {
            gParticles[i].mR.y = gContainerY + gContainerHeight;
        }
    }
}

-(int) NeighBoordsFind: (int) a_ : (int) b_ : (int) startindex
{
    int i = startindex;
    hashGridList[a_][b_].ResetIterator();
    InHashCellIndexes[i] = hashGridList[a_][b_].GetNext();
    while (InHashCellIndexes[i]>-1)
    {
        i++;
        InHashCellIndexes[i] = hashGridList[a_][b_].GetNext();
    }
    return i;
}

-(void) spritevisibles
{
    float porog =1.f;
    int count=0;
    float visibleradius = porog * gParticleRadius;
    for (int x = 0; x < hashWidth; ++x)  //// процедура определения необходимости отображения
	{
		for (int y = 0; y < hashHeight; ++y)    // соответствующего спрайта
		{
			if(!hashGridList[x][y].IsEmpty())
			{
				int a, b;
                hashGridList[x][y].ResetIterator();
                a = hashGridList[x][y].GetNext();
                while (a != -1)
                {
                    if (gParticles[a].sp.visible)
                    {
                        BOOL isLeft = NO, isRightBottom = NO, isRightTop = NO,
                        isxminer=NO, isxmaxer=NO, isyminer= NO, isymaxer=NO;
                        hashGridList[x][y].ResetIterator();
                        b = hashGridList[x][y].GetNext();
                        while ( b!=-1)
                        {
                            if ((b!=a)  &&  gParticles[b].sp.visible)
                            {
                                float dx = gParticles[a].mR.x-gParticles[b].mR.x ,
                                dy = gParticles[a].mR.y-gParticles[b].mR.y;
                                if (!isLeft)
                                {
                                    isLeft = ((dx -  visibleradius) * (dx -  visibleradius ) + dy * dy < visibleradius *  visibleradius) &&
                                    (dx * dx + dy * dy < visibleradius *  visibleradius);
                                }
                                if (!isRightBottom )
                                {
                                    isRightBottom= ((dx +  0.5f * visibleradius) * (dx +   0.5f * visibleradius ) +
                                                    (dy - 0.866f * visibleradius) * (dy - 0.866f * visibleradius) <
                                                    visibleradius *  visibleradius) &&
                                    (dx * dx + dy * dy < visibleradius *  visibleradius) ;
                                }
                                if (!isRightTop )
                                {
                                    isRightTop  = ((dx +  0.5f * visibleradius) * (dx +   0.5f * visibleradius ) +
                                                   (dy + 0.866f * visibleradius) * (dy + 0.866f * visibleradius) <
                                                   visibleradius *  visibleradius) &&
                                    (dx * dx + dy * dy < visibleradius *  visibleradius);
                                }
                                isxminer = isxminer || (dx<-0.1 * gParticleRadius) ;
                                isxmaxer = isxmaxer || (dx>0.1 * gParticleRadius)  ;
                                isyminer = isyminer || (dy < - 0.1 * gParticleRadius) ;
                                isymaxer = isymaxer || (dy > 0.1 * gParticleRadius) ;
                            }
                            b = hashGridList[x][y].GetNext();
                        }
                        hashGridList[x][y].ResetIterator();
                        b =-1;
                        while (b!=a)
                        {
                            b = hashGridList[x][y].GetNext();
                        }
                        gParticles[a].sp.visible = !(isLeft && isRightBottom && isRightTop  && (( isxminer && isxmaxer) || (isyminer && isymaxer )) );
                        if (!gParticles[a].sp.visible)
                        {
                            count++;
                        }
                    }
                    a = hashGridList[x][y].GetNext();
                }
			}
		}
	}
    count++;
}

-(void) bodytouchtest :(float) deltaT
{
    for (int x = 0; x < hashWidth; ++x)
	{
		for (int y = 0; y < hashHeight; ++y)
		{
			if(!hashGridList[x][y].IsEmpty())
			{
				float minX = myMap((float)x, 0, hashWidth, fluidMinX, fluidMaxX);
				float maxX = myMap((float)x+1, 0, hashWidth, fluidMinX, fluidMaxX);
				float minY = myMap((float)y, 0, hashHeight, fluidMinY, fluidMaxY);
				float maxY = myMap((float)y+1, 0, hashHeight, fluidMinY, fluidMaxY);
				b2AABB aabb;
				aabb.lowerBound.Set(minX, minY);
				aabb.upperBound.Set(maxX, maxY);
				if (intersectQueryCallback)
				{
					intersectQueryCallback->x = x;
					intersectQueryCallback->y = y;
					intersectQueryCallback->deltaT = deltaT;
					WORLD->QueryAABB(intersectQueryCallback, aabb);
				}
			}
		}
	}
}

-(void) tick: (ccTime) dt
{
    b2Vec2 gw=WORLD->GetGravity();
    int32 velocityIterations = 1;
    int32  positionIterations = 1;
    gGravity =tVector2( 0.7* gw.x,0.7 * gw.y);
   
    WORLD->Step(1.0f / 30.0f, velocityIterations, positionIterations);
    
    [self Display :  1.f/60];
    
}

- (void)accelerometer:(UIAccelerometer*)accelerometer didAccelerate:(UIAcceleration*)acceleration
{
	[Common processAccelometry:acceleration];
}

bool ParticleSolidCollision3(b2Fixture* fixture, b2Vec2& particlePos, b2Vec2& nearestPos, b2Vec2& impactNormal,float particleRadius)
{
	if (fixture->GetShape()->GetType() == b2Shape::e_circle)
	{
		b2CircleShape* pCircleShape = static_cast<b2CircleShape*>(fixture->GetShape());
		const b2Transform& xf = fixture->GetBody()->GetTransform();
		float radius = pCircleShape->m_radius + 0.48f;//2 * particleRadius;
		b2Vec2 circlePos = xf.p + pCircleShape->m_p;
		b2Vec2 delta = particlePos - circlePos;
		if (delta.LengthSquared() > radius * radius)
        {
			return false;
        }
		delta.Normalize();
		delta *= radius;
		nearestPos = delta + pCircleShape->m_p;
        
		impactNormal = (nearestPos - circlePos);
		impactNormal.Normalize();
		return true;
	}
	else if (fixture->GetShape()->GetType() == b2Shape::e_polygon)
	{
		b2PolygonShape* pPolyShape = static_cast<b2PolygonShape*>(fixture->GetShape());
		const b2Transform& xf = fixture->GetBody()->GetTransform();
		int numVerts = pPolyShape->GetVertexCount();
		b2Vec2 vertices[b2_maxPolygonVertices];
		b2Vec2 normals[b2_maxPolygonVertices];
		for (int32 i = 0; i < numVerts; ++i)
		{
			vertices[i] = b2Mul(xf, pPolyShape->m_vertices[i]);
			normals[i] = b2Mul(xf.q, pPolyShape->m_normals[i]);
		}
		float shortestDistance = 99999.0f;
		for (int i = 0; i < numVerts ; ++i)
		{
            b2Vec2 vertex = vertices[i] + 0.2f * (InterPriclScaleFactor/12) * CC_CONTENT_SCALE_FACTOR() * normals[i] - particlePos;
			float distance = b2Dot(normals[i], vertex);
			if (distance < 0)
			{
				return false;
			}
			if (distance < shortestDistance)
			{
				shortestDistance = distance;
				nearestPos = b2Vec2(
                                    normals[i].x * distance + particlePos.x,
                                    normals[i].y * distance + particlePos.y);
				impactNormal = normals[i];
			}
		}
		return true;
	}
	else
	{
		// Unrecognised shape type
		assert(false);
		return false;
	}
}

void SeparateParticleFromBody3(int particleIdx, b2Vec2& nearestPos, b2Vec2& normal, tParticle *liquid,float knorm, float ktang)
{
	liquid[particleIdx].mR = tVector2(nearestPos.x, nearestPos.y)/(InterPriclScaleFactor * CC_CONTENT_SCALE_FACTOR());
//	liquid[particleIdx].mOldR = liquid[particleIdx].mR + 0.2f * (liquid[particleIdx].mOldR - liquid[particleIdx].mR);
//	liquid[particleIdx].mV = tVector2(0, 0);
}

-(void)Exit
{
    free(gParticles);
}

float cube(float a)
{
    return a * a * a;
}
//==============================================================
// CalculatePressure
//==============================================================
-(tScalar) CalculatePressure : (tScalar) density
{
    return gGasK * (cube(density/gDensity0) - 1.0f);
}
//==============================================================
// CalculateNewParticles
//==============================================================
-(void) CalculateNewParticles
{
    tVector2 force;
    for (int i = 0 ; i < gNParticles ; ++i)
    {
        AddVector2(force,
                   gParticles[i].mBodyForce,
                   gParticles[i].mPressureForce,
                   gParticles[i].mViscosityForce);
        // use verlet to integrate
        tScalar damping = 0.01f;
        tVector2 tmp = gParticles[i].mR;
        gParticles[i].mR += (1.0f - damping) *
        (gParticles[i].mR - gParticles[i].mOldR) +
        (gTimeStep * gTimeStep / gParticleMass) * force;
        gParticles[i].mOldR = tmp;
        // cache velocity as it's useful
        gParticles[i].mV = (gParticles[i].mR - gParticles[i].mOldR) / gTimeStep;
    }
}
//==============================================================
// CalculateNormalField
//==============================================================
-(void) CalculateNormalField
{
    int i;
    tSpatialGridIterator gridIter;
    for (i = gNParticles ; i-- != 0 ; )
    {
        gParticles[i].mCs = 0.0f;
        for (tParticle * particle = gridIter.FindFirst(gParticles[i].mR, *gSpatialGrid) ;
             particle != 0 ;
             particle = gridIter.GetNext(*gSpatialGrid))
        {
            if (particle->mDensity > 0.0f)
            {
                gParticles[i].mCs += gParticleMass *
                [self WPoly6 : (gParticles[i].mR - particle->mR)/particle->mDensity ];
            }
        }
    }
    for (i = gNParticles ; i-- != 0 ; )
    {
        gParticles[i].mN.Set(0.0f, 0.0f);
        for (tParticle * particle = gridIter.FindFirst(gParticles[i].mR, *gSpatialGrid) ;
             particle != 0 ;
             particle = gridIter.GetNext(*gSpatialGrid))
        {
            if (particle->mDensity > 0.0f)
            {
                gParticles[i].mN +=
                (gParticleMass * particle->mCs / particle->mDensity) *
                [self  WPoly6Grad : (gParticles[i].mR - particle->mR)];
            }
        }
    }
}
//==============================================================
// CheckKernel
//==============================================================
- (void) NormaliseKernels
{
    int n = 1000;
    tScalar xmin = -1.0f * gKernelH;
    tScalar ymin = xmin;
    tScalar xmax = -xmin;
    tScalar ymax = xmax;
    tScalar dx = (xmax - xmin) / (n - 1);
    tScalar dy = (ymax - ymin) / (n - 1);
    tScalar dA = dx * dy;
    tScalar totalPoly6 = 0.0f;
    tScalar totalSpiky = 0.0f;
    tScalar totalViscosity = 0.0f;
    tScalar totalLucy = 0.0f;
    gWPoly6Scale = 1.0f;
    gWSpikyScale = 1.0f;
    gWViscosityScale = 1.0f;
    gWLucyScale = 1.0f;
    for (int i = 0 ; i < n ; ++i)
    {
        for (int j = 0 ; j < n ; ++j)
        {
            tVector2 r(xmin + i * dx, ymin + j * dy);
            totalPoly6 +=[self WPoly6 : r] * dA;
            totalSpiky += [self WSpiky : r] * dA;
            totalViscosity += [self WViscosity : r] * dA;
            totalLucy += [self WLucy : r] * dA;
        }
    }
    gWPoly6Scale = 1.0f / totalPoly6;
    gWSpikyScale = 1.0f / totalSpiky;
    gWViscosityScale = 1.0f / totalViscosity;
    gWLucyScale = 1.0f / totalLucy;
}
//==============================================================
// CalculatePressureAndDensities
//==============================================================
-(void) CalculatePressureAndDensities
{
    tVector2 r;
    tSpatialGridIterator gridIter;
    for (int i = gNParticles ; i-- != 0 ; )
    {
        gParticles[i].mDensity = 0.0f;
        for (tParticle * particle = gridIter.FindFirst(gParticles[i].mR, *gSpatialGrid);
             particle != 0 ;
             particle = gridIter.GetNext(*gSpatialGrid))
        {
            SubVector2(r, gParticles[i].mR, particle->mR);
            gParticles[i].mDensity += gParticleMass * [self WPoly6 : r];
        }
        gParticles[i].mP = [self CalculatePressure : gParticles[i].mDensity];
    }
}
//==============================================================
// CalculateForces
// The forces between particles are symmetric so only calculate
// them once per pair
//==============================================================
-(void) CalculateForces
{
    tVector2 tmp;
    tVector2 r;
    int i;
    
    for (i = gNParticles ; i-- != 0 ; )
    {
        ScaleVector2(gParticles[i].mBodyForce, gGravity, gParticleMass);
        gParticles[i].mPressureForce.Set(0.0f, 0.0f);
        gParticles[i].mViscosityForce.Set(0.0f, 0.0f);
    }
    
    tSpatialGridIterator gridIter;
    for (i = gNParticles ; i-- != 0 ; )
    {
        for (tParticle * particle = gridIter.FindFirst(gParticles[i].mR, *gSpatialGrid) ;
             particle != 0 ;
             particle = gridIter.GetNext(*gSpatialGrid))
        {
            // only do each pair once
            if (particle > &gParticles[i])
                continue;
            if (particle->mDensity > SCALAR_TINY)
            {
                // pressure
                SubVector2(r, gParticles[i].mR, particle->mR);
                if (r.GetLengthSq() < gKernelH2)
                {
                    ScaleVector2(tmp,
                                 [self WSpikyGrad:r],
                                 gParticleMass * (gParticles[i].mP + particle->mP) /
                                 (2.0f * particle->mDensity));
                    gParticles[i].mPressureForce -= tmp;
                    particle->mPressureForce += tmp;
                    
                    // viscosity
                    ScaleVector2(tmp,
                                 particle->mV - gParticles[i].mV,
                                 gViscosity * gParticleMass * [self WViscosityLap : r] / particle->mDensity);
                    gParticles[i].mViscosityForce += tmp;
                    particle->mViscosityForce -= tmp;
                }
            }
        }
    }
    //AddBoundaryForces();
}
//==============================================================
// PreventParticleCohabitation
//==============================================================
-(void) PreventParticleCohabitation
{
    tScalar minDist = 0.5f * gParticleRadius;
    tScalar minDist2 = minDist * minDist;
    tScalar resolveFrac = 0.1f;
    tVector2 delta;
    tSpatialGridIterator gridIter;
    for (int i = gNParticles ; i-- != 0 ; )
    {
        for (tParticle * particle = gridIter.FindFirst(gParticles[i].mR, *gSpatialGrid) ;
             particle != 0 ;
             particle = gridIter.GetNext(*gSpatialGrid))
        {
            // only need to check each pair once
            if (particle > &gParticles[i])
                continue;
            SubVector2(delta, particle->mR, gParticles[i].mR);
            tScalar deltaLenSq = delta.GetLengthSq();
            if (deltaLenSq > minDist2)
                continue;
            if (deltaLenSq > SCALAR_TINY)
            {
                tScalar deltaLen = sqrt(deltaLenSq);
                tScalar diff = resolveFrac * 0.5f * (deltaLen - minDist) / deltaLen;
                delta *= diff;
                gParticles[i].mR += delta;
                gParticles[i].mOldR += delta;
                particle->mR -= delta;
                particle->mOldR -= delta;
            }
            else
            {
                gParticles[i].mR.x += 0.5f * resolveFrac * minDist;
                gParticles[i].mOldR.x += 0.5f * resolveFrac * minDist;
                particle->mR.x -= 0.5f * resolveFrac * minDist;
                particle->mOldR.x -= 0.5f * resolveFrac * minDist;
            }
        }
//		float Vele = sqrt(gParticles[i].mV.y * gParticles[i].mV.y + gParticles[i].mV.y * gParticles[i].mV.y);
//		if (Vele > 0.1f)
//		{
//			gParticles[i].mV *=0.1f/Vele;
//		}
	}
}

-(void) Display : (float) dt
{
    lastTime = thisTime;
    thisTime+= dt;
    tScalar ddt = (thisTime - lastTime); //* 0.001f;
    ddt *= gTimeScale;
    static tScalar residual = 0.0f;
    if (ddt > 0.2f)
    {
        ddt = 0.2f;
        residual = 0.0f;
    }
    int nLoops = (int) ((ddt + residual)/ gTimeStep);
    if (nLoops > 0)
    {
        residual = ddt - (nLoops * gTimeStep);
        for (int iLoop = 0 ; iLoop < nLoops ; ++iLoop)
        {
            [self Integrate];
        }
        lastTime = thisTime;
    }
    for(int a = 0; a < hashWidth; a++)
    {
		for(int b = 0; b < hashHeight; b++)
		{
			hashGridList[a][b].Clear();
	 	}
    }
    for(int a = 0; a < gNParticles; a++)
    {
		gParticles[a].sp.visible =  YES;
		int hcell = myMap(//InterPriclScaleFactor * CC_CONTENT_SCALE_FACTOR() * gParticles[a].mR.x
						     gParticles[a].sp.position.x * CC_CONTENT_SCALE_FACTOR()/PTM_RATIO
						  , fluidMinX, fluidMaxX, 0, hashWidth-.001f);
		int vcell = myMap(//InterPriclScaleFactor * CC_CONTENT_SCALE_FACTOR() * gParticles[a].mR.y
						  gParticles[a].sp.position.y * CC_CONTENT_SCALE_FACTOR()/PTM_RATIO
						  , fluidMinY, fluidMaxY, 0, hashHeight-.001f);
		if(hcell > -1 && hcell < hashWidth && vcell > -1 && vcell < hashHeight)
		{
			hashGridList[hcell][vcell].PushBack(a);
		}
    }
    [self bodytouchtest : dt];
   // [self spritevisibles];
    for (int i=0;i<gNParticles;i++)
    {
        
        if (InterPriclScaleFactor * gParticles[i].mR.y< sizeh /10/PTM_RATIO)
        {
            gParticles[i].mR.y = sizeh/10/InterPriclScaleFactor/PTM_RATIO;
        }
        
        gParticles[i].sp.position = ccp(InterPriclScaleFactor*gParticles[i].mR.x*PTM_RATIO,
                                        InterPriclScaleFactor*gParticles[i].mR.y *PTM_RATIO   ) ;
    }
}
-(void) VellocityConstrainter
{
for (int i = 0; i < gNParticles; i++)
	{float vel = sqrt(sq2(gParticles[i].mOldR.x - gParticles[i].mR.x)  +sq2(gParticles[i].mOldR.y - gParticles[i].mR.y)) ;
	if ( vel/gTimeStep > 5.f)
		gParticles[i].mOldR = gParticles[i].mR + 1.f * (gTimeStep/vel) * (gParticles[i].mOldR - gParticles[i].mR);
	}
}
//==============================================================
// Integrate
//==============================================================
-(void) Integrate
{    
    [self CalculatePressureAndDensities];
    [self CalculateForces];
	

    
	[self CalculateNewParticles];
	

	
    [self ImposeBoundaryConditions];
	
	[self PreventParticleCohabitation];
	
	
	
    gSpatialGrid->PopulateGrid(gParticles, gNParticles);
}

@end




