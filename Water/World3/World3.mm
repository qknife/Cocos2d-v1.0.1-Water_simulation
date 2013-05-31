// Import the interfaces
#import "World3.h"
#import "Common.h"

bool ParticleSolidCollision3(b2Fixture* fixture, b2Vec2& particlePos, b2Vec2& nearestPos, b2Vec2& impactNormal, float particleRadius);
void SeparateParticleFromBody3(int particleIdx, b2Vec2& nearestPos, b2Vec2& normal, tParticle *liquid,float knorm, float ktang);
//*

bool QueWorldInteractions::ReportFixture(b2Fixture* fixture)
{
    int numParticles = hashGridList[x][y].GetSize();
    hashGridList[x][y].ResetIterator();
    for(int i = 0; i < numParticles; i++)
    {
        int particleIdx = hashGridList[x][y].GetNext();         
        b2Vec2 particlePos=b2Vec2(InterPriclScaleFactor * CC_CONTENT_SCALE_FACTOR() * liquid[particleIdx].mR.x,
                                  InterPriclScaleFactor * CC_CONTENT_SCALE_FACTOR() * liquid[particleIdx].mR.y);
          
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
                b2Vec2 particleVelocity = b2Vec2(liquid[particleIdx].mV.x,liquid[particleIdx].mV.y);
                particleVelocity *= deltaT;
                b2Vec2 impulsePos = particlePos;                
                b2Vec2 pointVelocity = fixture->GetBody()->GetLinearVelocityFromWorldPoint(impulsePos);
                b2Vec2 pointVelocityAbsolute = pointVelocity;
                pointVelocity *= deltaT;                
                b2Vec2 relativeVelocity = particleVelocity - pointVelocity;                
                b2Vec2 pointVelNormal = normal;
                pointVelNormal *= b2Dot(relativeVelocity, normal);
                b2Vec2 pointVelTangent = relativeVelocity - pointVelNormal;
                const float slipFriction = 0.3f;                
                pointVelTangent *= slipFriction;
                b2Vec2 impulse = pointVelNormal - pointVelTangent;
                fixture->GetBody()->ApplyLinearImpulse(impulse, impulsePos);
                b2Vec2 buoyancy = b2Vec2(0, 10.f);
                const float buoyancyAdjuster = 0.f;
                buoyancy *= buoyancyAdjuster;
                fixture->GetBody()->ApplyForce(buoyancy, fixture->GetBody()->GetPosition());
                liquid[particleIdx].mV -= tVector2(impulse.x,impulse.y );
                liquid[particleIdx].mV += tVector2(pointVelocityAbsolute.x,pointVelocityAbsolute.y);
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
float sizeh;

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
   // Limit(i, 0, nx-1);
  //  Limit(j, 0, ny-1);
    
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

// on "init" you need to initialize your instance
-(id) init
{//*

	if( (self=[super init]))
	{
		// enable accelerometer
		self.isAccelerometerEnabled = YES;
		CGSize t=[Common size];
		fluidMinX = MW(0);
		fluidMaxX = MW(100);
		fluidMinY = MH(0);
		fluidMaxY = MH(100);
        knorm = 0.00f;  // 0.6f
        ktang = 0.00f;  //0.8f
      //  [self ReadConfig];
                  
		[self schedule: @selector(tick:) interval:1.0f / 60.0f];
       // SCALAR_TINY = 1E-3;//??????????????????????????????????????????????????????????????
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
//*/
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

-(void)particlesCountUp:(NSInteger)diff_
{
    
    CGSize size = [[CCDirector sharedDirector] winSize];
   
    tScalar gContainerWidthFrac = 1.;//0.8f;
    tScalar gContainerHeightFrac = 1.2f;//0.5f;
    tScalar gInitialFluidHeightFrac = 0.25f; //0.9f;
    tScalar kernelScale = 5.0f;
    gBoundaryForceScale = 0.0f;
    gBoundaryForceScaleDist = 0.1f;
    gWindowW       =  size.width ;  //   640;
    gWindowH       =  size.height ;  //   480;
    gRenderParticles  =   false;
    gNRenderSamplesX   =     96;
    gNRenderSamplesY   =     64;
    gRenderDensityMin  =    500;
    gRenderDensityMax  =    800;
    gDomainX           =      2;
    gDensity0        =     5000.0;//1000.0
    gNParticles      =      400;
    gViscosity       =        0.25;  //  0.05
    gGasK            =       3.0;//10.0
    gTimeStep       =         0.01;
    gTimeScale       =        1.0;
    gBoundaryForceScale   = 400.0;
    gBoundaryForceScaleDist = 0.03;
    gCreateObject       =  true;
    gObjectDensityFrac   =    0.5;
    gObjectBoundaryScale  =  20.0;
    gObjectBuoyancyScale  =  12.0;
    gDomainY = gDomainX * gWindowH / gWindowW;
    gContainerWidth = gContainerWidthFrac * gDomainX;
    gContainerHeight = gContainerHeightFrac * gDomainX;
    gContainerX = 0.5f * gDomainX - 0.5f * gContainerWidth;
    gContainerY = 0.1f * gDomainY/CC_CONTENT_SCALE_FACTOR();
    gInitialFluidHeight = gInitialFluidHeightFrac * gContainerHeight;
    
    tScalar volume = 0.7f*gContainerWidth * gInitialFluidHeight;//0.5
    gParticleRadius = sqrt(volume/(4.0f * gNParticles));
    
    gParticleMass = volume * gDensity0 / gNParticles;
    gKernelH = kernelScale * gParticleRadius;
    gKernelH9 = pow(gKernelH, 9.0f);
    gKernelH6 = pow(gKernelH, 6.0f);
    gKernelH4 = pow(gKernelH, 4.0f);
    gKernelH3 = pow(gKernelH, 3.0f);
    gKernelH2 = pow(gKernelH, 2.0f);
    RIntoSpr_Factor = CC_CONTENT_SCALE_FACTOR() / PTM_RATIO;
    [self NormaliseKernels];
    gParticles = new tParticle[gNParticles];
    gSpatialGrid = new tSpatialGrid(
                                    tVector2(-gContainerWidth, -gContainerHeight),
                                    tVector2(gDomainX + gContainerWidth, gDomainY + gContainerHeight), gKernelH);
    
    if (gParticles)
	{
		intersectQueryCallback = new QueWorldInteractions(hashGridList, gParticles,knorm,ktang,gParticleRadius);
        //      eulerIntersectQueryCallback = new QueryWorldPostIntersect(hashGridList, liquid);
	}
    sizeh = size.height;
    //InterPriclScaleFactor = 12.0f;
    int i, k=(int) sqrt(gNParticles);
    for (i = 0 ; i < gNParticles ; ++i)
    {
        gParticles[i].mR.Set(
                             gContainerX +  0.25f + 0.6f * gContainerWidth  * (i / k ) / k  ,
                             gContainerY + 2.0f + 0.5f *  gInitialFluidHeight * (i % k) / k);
        
        CCSprite *sprite = [CCSprite spriteWithFile:@"drop.png"]; // создать указатель на спрайт
        sprite.position = ccp(InterPriclScaleFactor*gParticles[i].mR.x/PTM_RATIO, InterPriclScaleFactor*gParticles[i].mR.y/PTM_RATIO) ;
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
        gParticles[i].sp.scaleX = 1.2f*CC_CONTENT_SCALE_FACTOR();
        gParticles[i].sp.scaleY = 1.2f*CC_CONTENT_SCALE_FACTOR();
        [BATCH addChild:gParticles[i].sp];
    }
}

-(void)particlesCountDown:(NSInteger)diff_
{
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
    float porog =10.f;
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
                           isxminer=NO, isxmaxer=NO, isyminer=NO, isymaxer=NO;
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
                        isxminer = isxminer || ((dx<-0.2 * gParticleRadius) && (dy < -0.2 * gParticleRadius) );
                        isxmaxer = isxmaxer || ((dx>0.2 * gParticleRadius) && (dy < -0.2 * gParticleRadius) );
                        isyminer = isyminer || ((dx<-0.2 * gParticleRadius) && (dy > 0.2 * gParticleRadius) );
                        isymaxer = isymaxer || ((dx>0.2 * gParticleRadius) && (dy > 0.2 * gParticleRadius) );
                      }
                     b = hashGridList[x][y].GetNext();
                    }
                    hashGridList[x][y].ResetIterator();
                    b =-1;
                    while (b!=a)
                    {
                        b = hashGridList[x][y].GetNext();
                    }
                    gParticles[a].sp.visible = !(isLeft && isRightBottom && isRightTop && isxminer && isxmaxer && isyminer && isymaxer);
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
    gGravity =tVector2( 0.7* gw.x,0.7 * gw.y);/// -1.1* gw.y,1.1 * gw.y       ///////////////////////////////////////////////////////
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
		float radius = pCircleShape->m_radius + 16.f*particleRadius* CC_CONTENT_SCALE_FACTOR();
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
            b2Vec2 vertex = vertices[i] + 0.4f*CC_CONTENT_SCALE_FACTOR() * normals[i] - particlePos;
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

-(void)Setup:(bool)bStart;
{
	//mNumPoints = 0;
}

void SeparateParticleFromBody3(int particleIdx, b2Vec2& nearestPos, b2Vec2& normal, tParticle *liquid,float knorm, float ktang)
{
	liquid[particleIdx].mR = tVector2(nearestPos.x,nearestPos.y)/(InterPriclScaleFactor*CC_CONTENT_SCALE_FACTOR());
//    tVector2 V = liquid[particleIdx].mV;
//    float VNModule = V.x * normal.x + V.y * normal.y;
//    tVector2 Vn;
//    Vn.x = VNModule * normal.x;
//    Vn.y = VNModule * normal.y;
//  	liquid[particleIdx].mV += (1 + knorm) * Vn  + 2 * (ktang - 1) * (Vn + V);
}

-(void)Exit
{
    //free(liquid);
    //free(InHashCellIndexes);
    free(gParticles);
}

float cube(float a)
{
    return a*a*a;
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
 tScalar damping = 0.01f; // 0-1
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
    // now n = grad(Cs)
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
    }
}

-(void) Display : (float) dt
{
    //static int lastTime = glutGet(GLUT_ELAPSED_TIME);
    //int thisTime = glutGet(GLUT_ELAPSED_TIME);
   //*
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
        gParticles[a].sp.visible=YES;
        
		int hcell = myMap(InterPriclScaleFactor * CC_CONTENT_SCALE_FACTOR() * gParticles[a].mR.x, fluidMinX, fluidMaxX, 0, hashWidth-.001f);
		int vcell = myMap(InterPriclScaleFactor * CC_CONTENT_SCALE_FACTOR() * gParticles[a].mR.y, fluidMinY, fluidMaxY, 0, hashHeight-.001f);
        
		if(hcell > -1 && hcell < hashWidth && vcell > -1 && vcell < hashHeight)
		{
			hashGridList[hcell][vcell].PushBack(a);
		}
    }
    [self bodytouchtest : dt];

    
    [self spritevisibles];
    for (int i=0;i<gNParticles;i++)
    {
        
        if (InterPriclScaleFactor * gParticles[i].mR.y< sizeh /10/PTM_RATIO)
         {
             gParticles[i].mR.y = //CC_CONTENT_SCALE_FACTOR()*
                                  sizeh/10/InterPriclScaleFactor/PTM_RATIO;
         }
        
        gParticles[i].sp.position = ccp(InterPriclScaleFactor*gParticles[i].mR.x*PTM_RATIO,
                                      InterPriclScaleFactor*gParticles[i].mR.y *PTM_RATIO   ) ;
    }

}

//==============================================================
// Integrate
//==============================================================
-(void) Integrate
{
   // if (gInteractionMode == INTERACTION_FOUNTAIN)
   //     DoFountain();
    
    [self CalculatePressureAndDensities];
    [self CalculateForces];
    [self CalculateNewParticles];
    [self ImposeBoundaryConditions];
    [self PreventParticleCohabitation];
    gSpatialGrid->PopulateGrid(gParticles, gNParticles);
    

   // [self CalculateNormalField];

    
    // and the objects
    //IntegrateObjects();
}



/*// 
 
  //==============================================================  |__________________________
 // AddBoundaryForce                                                |
 // adds to the force var passed in                                 |  по идее это нахрен
 //==============================================================   |  у нас свои 
-(void) AddBoundaryForce :(const tVector2) pos : (tVector2) force:  |  ограничения для жидкости
(tScalar) dist: (tScalar) scale                                     |
 {                                                                  |
 tScalar delta = pos.y - gContainerY;
 if (delta < 0.0f) delta = 0.0f;
 if (delta < dist)
 force.y += scale * (1.0f - (delta/dist)*(delta/dist));
 }
  
 //==============================================================
 // AddBoundaryForce
 // slightly hacky, but just do this on the lower boundary to stop
 // particles bunching up there. Could easily generalise this
 //==============================================================
 void AddBoundaryForces()
 {
 for (int i = 0 ; i < gNParticles ; ++i)
 {
 AddBoundaryForce(gParticles[i].mR,
 gParticles[i].mBodyForce,                                          |
 gBoundaryForceScaleDist,                                           |
 gBoundaryForceScale);                                              |
 }                                                                  |
 }                                                                  |_______________________________
 //========================================================----------------------------------------------------------
 // tResolver
 //========================================================
 class tResolver : public tPenetrationResolver
 {
 bool ResolvePenetration(tShapeParticle & particle);
 };
 
 //========================================================
 // ResolvePenetration
 //========================================================
 bool tResolver::ResolvePenetration(tShapeParticle & particle)
 {
 bool moved = false;
 if (particle.mPos.x < gContainerX)                                     Тоже ограничения (?)
 {
 particle.mPos.x = gContainerX;
 moved = true;
 }
 else if (particle.mPos.x > gContainerX + gContainerWidth)
 {
 particle.mPos.x = gContainerX + gContainerWidth;
 moved = true;
 }
 if (particle.mPos.y < gContainerY)
 {
 particle.mPos.y = gContainerY;
 moved = true;
 }
 else if (particle.mPos.y > gContainerY + gContainerHeight)
 {
 particle.mPos.y = gContainerY + gContainerHeight;
 moved = true;
 }
 return moved;
 }                                                              ___________________________________________________________
 
 //========================================================
 // SetObjectForces                                             ???????????????????????????????????????????
 //========================================================
 void SetObjectForces()
 {
 if (!gRectangle)
 return;
 int nObjectParticles;
 int iObjectParticle;
 tShapeParticle * objectParticles =
 gRectangle->GetParticles(nObjectParticles);
 for (iObjectParticle = nObjectParticles ;
 iObjectParticle-- != 0 ; )
 {
 // do gravity etc
 tVector2 bodyForce =
 gGravity / objectParticles[iObjectParticle].mInvMass;
 objectParticles[iObjectParticle].mForce = bodyForce;
 }
 
 for (iObjectParticle = nObjectParticles ;
 iObjectParticle-- != 0 ; )
 {
 // assume that the points are in order for traversing the
 // object. We sample each edge at intervals of gParticleRadius,
 // get the force at each sample (pressure is force/distance) which
 // must be perpendicular to the object edge, and distribute it
 // between the particles at the end of the line, according to the
 // position.
 int iStart = iObjectParticle;
 int iEnd = (iStart + 1) % nObjectParticles;
 tShapeParticle & particleStart = objectParticles[iStart];
 tShapeParticle & particleEnd = objectParticles[iEnd];
 tVector2 edgeDelta = particleEnd.mPos - particleStart.mPos;
 tScalar edgeLen = (edgeDelta).GetLength();
 tVector2 edgeDir = edgeDelta / edgeLen;
 tVector2 edgeNormal(edgeDir.y, -edgeDir.x); // points out of the object
 int nSamples = 1 + (int) (edgeLen / gParticleRadius);
 tScalar sampleLen = edgeLen / nSamples;
 tSpatialGridIterator gridIter;
 for (int iSample = nSamples ; iSample-- != 0 ; )
 {
 // samples are at the mid positions of the edge element
 tScalar fracStart = 1.0f - (0.5f + iSample)/nSamples;
 tScalar fracEnd = 1.0f - fracStart;
 tVector2 samplePos = fracStart * particleStart.mPos +
 fracEnd * particleEnd.mPos;
 
 tVector2 localPressureForce(0.0f, 0.0f);
 // evaluate the fluid pressure here - derived from the local density
 tScalar localDensity = 0.0f;
 tVector2 pressureGradient(0.0f, 0.0f);
 for (tParticle * particle = gridIter.FindFirst(samplePos, *gSpatialGrid) ;
 particle != 0 ;
 particle = gridIter.GetNext(*gSpatialGrid))
 {
 tVector2 r = samplePos - particle->mR;
 if (r.GetLengthSq() < gKernelH2)
 {
 localDensity += gParticleMass * WSpiky(r);
 if (particle->mDensity > 0.0f)
 pressureGradient -= WPoly6Grad(r) *
 (gParticleMass * particle->mP /
 particle->mDensity);
 }
 }
 // don't bother with viscosity
 
 // now we've calculated the density use it to decide whether or
 // not to add the hacky boundary force
 if ( (0 == iSample) &&
 (localDensity > 0.5f * gDensity0) )
 {
 AddBoundaryForce(objectParticles[iStart].mPos,
 objectParticles[iStart].mForce,
 2 * gBoundaryForceScaleDist,
 -gObjectBoundaryScale * gGravity.y /
 objectParticles[iStart].mInvMass);
 }
 
 // the factor in here is a horrible hack because the ideal-gas
 // approx doesn't really give a good pressure gradient... etc
 localPressureForce = gObjectBuoyancyScale * sampleLen *
 edgeNormal * (Dot(pressureGradient, edgeNormal));
 objectParticles[iStart].mForce += fracStart * localPressureForce;
 objectParticles[iEnd].mForce += fracEnd * localPressureForce;
 }
 }
 }
 
 //========================================================
 // IntegrateObjects                                          ?????????????????????????????????????????
 //========================================================
 void IntegrateObjects()
 {
 if (!gRectangle)
 return;
 
 if (INTERACTION_BOX != gInteractionMode)
 {
 SetObjectForces();
 gRectangle->Integrate(gTimeStep);
 }
 tResolver resolver;
 gRectangle->ResolveConstraints(4, resolver);
 }
 

 

 
 //==============================================================
 // DoFountain                                                      ?????????????????????????????????????????????????
 //==============================================================
 void DoFountain()
 {
 static int fountainParticle = 0;
 tVector2 vel(2.0f, 2.0f);
 
 int w = glutGet(GLUT_WINDOW_WIDTH);
 int h = glutGet(GLUT_WINDOW_HEIGHT);
 
 tVector2 pos = (gDomainX / gWindowW) * gOldMousePos;
 tScalar frac = 0.05f;
 pos += frac * tVector2(RangedRandom(0.0f, gDomainX), RangedRandom(0.0f, gDomainY));
 
 gParticles[fountainParticle].mR = pos;
 gParticles[fountainParticle].mOldR = pos - gTimeStep * vel;
 gParticles[fountainParticle].mV = vel;
 fountainParticle = (fountainParticle + 1) % gNParticles;
 }
 
 

// ________________________________________________________________________________________________________________________________________
// //==============================================================                                                                     |
// // DrawContainer                                                                                                                     |
// //==============================================================                                                                     |
// void DrawContainer()                                                                                                                 |
// {                                                                                                                                    |
// GLCOLOR3(1.0f,1.0f, 1.0f);
// glBegin(GL_QUADS);
// GLVERTEX2(gContainerX, gContainerY + gContainerHeight);
// GLVERTEX2(gContainerX, gContainerY);
// GLVERTEX2(gContainerX + gContainerWidth, gContainerY);
// GLVERTEX2(gContainerX + gContainerWidth, gContainerY + gContainerHeight);
// GLVERTEX2(gContainerX, gContainerY + gContainerHeight);
// glEnd();
// }
// 
// //========================================================
// // DrawQuad
// //========================================================
// inline void DrawQuad(const tVector2 & min, const tVector2 & max)
// {
// GLVERTEX2(min.x, min.y);
// GLVERTEX2(max.x, min.y);
// GLVERTEX2(max.x, max.y);
// GLVERTEX2(min.x, max.y);
// }
// 
// //==============================================================
// // DrawVoid
// //==============================================================
// void DrawVoid()
// {
// GLCOLOR3(0.5f,0.5f, 0.5f);
// glBegin(GL_QUADS);
// DrawQuad(tVector2(0.0f, 0.0f), tVector2(gDomainX, gContainerY));
// DrawQuad(tVector2(0.0f, gContainerY + gContainerHeight),
// tVector2(gDomainX, gDomainY));
// DrawQuad(tVector2(0.0f, 0.0f), tVector2(gContainerX, gDomainY));
// DrawQuad(tVector2(gContainerX + gContainerWidth, 0.0f),
// tVector2(gDomainX, gDomainY));
// glEnd();
// }
// 
// //==============================================================
// // DrawGridWater
// // sets up a grid to render between the water particles...
// //==============================================================
// void DrawGridWater()
// {
// glColor3f(0.0f,0.0f, 1.0f);
// 
// // number points
// int nx = gNRenderSamplesX;
// int ny = gNRenderSamplesY;
// tScalar dx = gContainerWidth / (nx - 1);
// tScalar dy = gContainerHeight / (ny - 1);
// 
// static tArray2D<tScalar> densities(nx, ny);
// densities.Resize(nx, ny);
// 
// tSpatialGridIterator gridIter;
// tVector2 r;
// for (int ix = nx ; ix-- != 0 ; )
// {
// for (int iy = ny ; iy-- != 0 ; )
// {
// r.Set(gContainerX + ix * dx, gContainerY + iy * dy);
// // now evaluate the density at this point
// tScalar d = 0.0f;
// for (tParticle * particle = gridIter.FindFirst(r, *gSpatialGrid) ;
// particle != 0 ;
// particle = gridIter.GetNext(*gSpatialGrid))
// {
// d += gParticleMass * WPoly6(r - particle->mR);
// }
// densities(ix, iy) = d;
// }
// }
// 
// tScalar min = gRenderDensityMin;
// tScalar max = gRenderDensityMax;
// glBegin(GL_QUADS);
// int i, j;
// tScalar grey;
// for (i = nx-1 ; i-- != 0 ; )
// {
// for (j = ny-1 ; j-- != 0 ; )
// {
// tScalar x = gContainerX + i * dx;
// tScalar y = gContainerY + j * dy;
// grey = (densities(i, j) - min) / (max - min);
// GLCOLOR3(0, 0, grey);
// GLVERTEX2(x, y);
// grey = (densities(i+1, j) - min) / (max - min);
// GLCOLOR3(0, 0, grey);
// GLVERTEX2(x + dx, y);
// grey = (densities(i+1, j+1) - min) / (max - min);
// GLCOLOR3(0, 0, grey);
// GLVERTEX2(x + dx, y + dy);
// grey = (densities(i, j+1) - min) / (max - min);
// GLCOLOR3(0, 0, grey);
// GLVERTEX2(x, y + dy);
// }
// }
// glEnd();
// }
// 
// 
// //==============================================================
// // DrawWater
// //==============================================================
// void DrawWater()
// {
// if (gRenderParticles)
// {
// int i;
// GLCOLOR3(0.0f,0.0f, 1.0f);
// glPointSize(6);
// glEnable(GL_POINT_SMOOTH);
// glBegin(GL_POINTS);
// for (i = gNParticles ; i-- != 0 ; )
// {
// GLVERTEX2(gParticles[i].mR.x, gParticles[i].mR.y);
// }
// glEnd();
// }
// else
// {
// DrawGridWater();
// }
// }
// 
// //========================================================
// // DrawObjects
// //========================================================
// void DrawObjects()
// {                                                                                                                            |
// if (gRectangle)                                                                                                              |
// gRectangle->Draw();                                                                                                          |
// }                                                                                                                            |
// _______________________________________________________________________________________________________________________________
 
// //========================================================____________________________________________________________________________________
// // ApplyImpulse                                                                                                                              |
// //========================================================
// void ApplyImpulse(const tVector2 & deltaVel)
// {
// for (int i = gNParticles ; i-- != 0 ; )
// {
// gParticles[i].mV += deltaVel;
// gParticles[i].mOldR -= deltaVel * gTimeStep;
// }
// }
// 
// //==============================================================
// // MoveContainer
// //==============================================================
// void MoveContainer(const tVector2 & dR)
// {
// gContainerX += dR.x;
// gContainerY += dR.y;
// }
// 
// //==============================================================
// // MoveBox
// //==============================================================
// void MoveBox(const tVector2 & dR)
// {
// if (gRectangle)
// gRectangle->Move(dR);
// }
// 
// //==============================================================
// // Keyboard
// //==============================================================
// void Keyboard( unsigned char key, int x, int y )
// {
// switch (key)
// {
// case 'q':
// case 27:
// exit(0);
// break;
// case 'a':
// ApplyImpulse(tVector2(-1.0f, 0.0f));
// break;
// case 'd':
// ApplyImpulse(tVector2(1.0f, 0.0f));
// break;
// case 's':
// ApplyImpulse(tVector2(0.0f, -1.0f));
// break;
// case 'w':
// ApplyImpulse(tVector2(0.0f, 1.0f));
// break;
// default:
// break;
// }
// }
// 
// //==============================================================
// // Mouse
// //==============================================================
// void Mouse(int button, int state, int x, int y)
// {
// if (GLUT_UP == state)
// {
// gInteractionMode = INTERACTION_NONE;
// return;
// }
// 
// int h = glutGet(GLUT_WINDOW_HEIGHT);
// gOldMousePos.Set(x, h-y);
// if (GLUT_LEFT_BUTTON == button)
// gInteractionMode = INTERACTION_CONTAINER;
// else if (GLUT_RIGHT_BUTTON == button)
// gInteractionMode = INTERACTION_BOX;
// else if (GLUT_MIDDLE_BUTTON == button)
// gInteractionMode = INTERACTION_FOUNTAIN;
// }
// 
// //==============================================================
// // MouseMotion
// //==============================================================
// void MouseMotion(int x, int y)
// {
// int h = glutGet(GLUT_WINDOW_HEIGHT);
// tVector2 newPos(x, h-y);
// tVector2 delta = (gDomainX / gWindowW) * (newPos - gOldMousePos);
// if (INTERACTION_CONTAINER == gInteractionMode)
// {
// MoveContainer(delta);
// }
// else if (INTERACTION_BOX == gInteractionMode)
// {
// MoveBox(delta);
// }
// gOldMousePos = newPos;                                                                                                                   |
// }____________________________________________________________________________________________________________________________________________
 
 
 
 //==============================================================
 // main
 //==============================================================
 int main(int argc, char* argv[])
 {
 // GLUT routines
 glutInit(&argc, argv);
 
 TRACE("Water simulation by Danny Chapman Dec 2004\n");
 
 string configFileName("water.cfg");
 bool configFileOk;
 if (argc > 1)
 configFileName = string(argv[1]);
 gConfigFile = new tConfigFile(configFileName, configFileOk);
 
 if (!configFileOk)
 TRACE("Warning: Unable to open main config file: %s\n",
 configFileName.c_str());
 
 ReadConfig();
 
 glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
 glutInitWindowSize( gWindowW, gWindowH );
 
 // Open a window
 glutCreateWindow( "Water - Danny Chapman Dec 2004" );
 glClearColor( 0.0,0.0,0.0,1 );
 // set the projection so that we draw things in world space, not
 // screen space
 gluOrtho2D(0.0f, gDomainX, 0.0f, gDomainY);
 
 glutDisplayFunc(&Display);
 glutIdleFunc(&Idle);
 glutKeyboardFunc(&Keyboard);
 glutMouseFunc(&Mouse);
 glutMotionFunc(&MouseMotion);
 
 // my code
 Initialise();
 
 glutMainLoop();
 
 delete gConfigFile;
 
 return 1;
 
 };
 
 //*/

@end




