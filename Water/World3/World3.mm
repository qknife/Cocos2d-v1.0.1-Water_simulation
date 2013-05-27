// Import the interfaces
#import "World3.h"
#import "Common.h"

bool ParticleSolidCollision3(b2Fixture* fixture, b2Vec2& particlePos, b2Vec2& nearestPos, b2Vec2& impactNormal, float particleRadius);
void SeparateParticleFromBody3(int particleIdx, b2Vec2& nearestPos, b2Vec2& normal, sPart *liquid,float knorm,float ktang);
//*

bool QueWorldInteractions::ReportFixture(b2Fixture* fixture)
{
    int numParticles = hashGridList[x][y].GetSize();
    hashGridList[x][y].ResetIterator();
   
    // Iterate through all the particles in this cell
    for(int i = 0; i < numParticles; i++)
    {
        int particleIdx = hashGridList[x][y].GetNext();
         
        b2Vec2 particlePos = liquid[particleIdx].mPos;
        if(fixture->GetBody()->GetType() == b2_staticBody)
        {
            b2Vec2 nearestPos(0,0);
            b2Vec2 normal(0,0);
            
            // electrodruid TODO: moving particles out to the nearest edge in this way
            // can cause leaking and tunnelling, particularly for high-velocity particles.
            // Perhaps some kind of approach involving raycasting between the old particle
            // position and the current one would work better?
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
                b2Vec2 particleVelocity = liquid[particleIdx].mVel;
                
                particleVelocity *= deltaT;

                
                // electrodruid: Still not sure exactly what the paper meant by
                // "intersection position", but taking the current particle position
                // seems to give the least-bad results
                b2Vec2 impulsePos = particlePos;
                //									b2Vec2 impulsePos = nearestPos;
                
                b2Vec2 pointVelocity = fixture->GetBody()->GetLinearVelocityFromWorldPoint(impulsePos);
                b2Vec2 pointVelocityAbsolute = pointVelocity;
                // electrodruid: I think this does need to be here
                pointVelocity *= deltaT;
                
                b2Vec2 relativeVelocity = particleVelocity - pointVelocity;
                
                b2Vec2 pointVelNormal = normal;
                pointVelNormal *= b2Dot(relativeVelocity, normal);
                b2Vec2 pointVelTangent = relativeVelocity - pointVelNormal;
                
                // Should be a value between 0.0f and 1.0f
                const float slipFriction = 0.3f;
                
                pointVelTangent *= slipFriction;
                b2Vec2 impulse = pointVelNormal - pointVelTangent;
                
                // electrodruid: not sure if this should be here
                //									impulse *= deltaT;
                
                // electrodruid: Don't know if this should be a force or an impulse...
                fixture->GetBody()->ApplyLinearImpulse(impulse, impulsePos);
                //									 pShape->GetBody()->ApplyForce(impulse, impulsePos);
                
                // electrodruid: Bodies with low mass don't float, they just spin too
                // fast because of low rotational inertia. As well as fudging for the
                // spinning, try to add buoyancy by adding a force to negate (some of)
                // the gravity affecting the body. This needs to be tuned properly
                // for different bodies
                b2Vec2 buoyancy = b2Vec2(0, 10.f);
                const float buoyancyAdjuster = 0.f;
                buoyancy *= buoyancyAdjuster;
                
                fixture->GetBody()->ApplyForce(buoyancy, fixture->GetBody()->GetPosition());
                
                // move the particles away from the body
                liquid[particleIdx].mVel -= impulse;
                liquid[particleIdx].mVel += pointVelocityAbsolute;
            }
        }
    }
    return true;
}
//*/
#define EPSILON			0.00001f			//for collision detection

static float fluidMinX;
static float fluidMaxX;
static float fluidMinY;
static float fluidMaxY;


// HelloWorldLayer implementation
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
     //   Limit(i, 0, nx-1);
     //   Limit(j, 0, ny-1);
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
        
        mNumPoints = 200;
    
		CGSize t=[Common size];
		fluidMinX = MW(0);
		fluidMaxX = MW(100);
		fluidMinY = MH(0);
		fluidMaxY = MH(100);
        knorm = 0.6f;
        ktang = 0.8f;
        gTimeStep = 0.01f;
        gDensity0 = 1000.0f;
        gViscosity = 0.005f;
        gGasK = 1.0f;
        kernelScale = 5.0f;
        
//        gDomainY = gDomainX * gWindowH / gWindowW;
//        gContainerWidth = gContainerWidthFrac * gDomainX;
//        gContainerHeight = gContainerHeightFrac * gDomainX;
//        gContainerX = 0.5f * gDomainX - 0.5f * gContainerWidth;
//        gContainerY = 0.1f * gDomainY;
        //gInitialFluidHeight = gInitialFluidHeightFrac * gContainerHeight;
        volume = 50;
        gParticleRadius = sqrt(volume /(4.0f * mNumPoints));
        gParticleMass = volume * gDensity0 / mNumPoints;
        gKernelH = kernelScale * gParticleRadius;
        gKernelH9 = pow(gKernelH, 9.0f);
        gKernelH6 = pow(gKernelH, 6.0f);
        gKernelH4 = pow(gKernelH, 4.0f);
        gKernelH3 = pow(gKernelH, 3.0f);
        gKernelH2 = pow(gKernelH, 2.0f);
        gParticleMass = volume * gDensity0 / mNumPoints;
        [self NormaliseKernels];
        //[self Setup:true];
		//[self Run:0 :0];		
		[self schedule: @selector(tick:) interval:1.0f / 60.0f];
	}
	
	return self;
}

-(void) NormaliseKernels
{
    int n = 1000;
    float xmin = -1.0f * gKernelH;
    float ymin = xmin;
    float xmax = -xmin;
    float ymax = xmax;
    float dx = (xmax - xmin) / (n - 1);
    float dy = (ymax - ymin) / (n - 1);
    float dA = dx * dy;
    float totalPoly6 = 0.0f;
    float totalSpiky = 0.0f;
    float totalViscosity = 0.0f;
    float totalLucy = 0.0f;
    gWPoly6Scale = 1.0f;
    gWSpikyScale = 1.0f;
    gWViscosityScale = 1.0f;
    gWLucyScale = 1.0f;
    for (int i = 0 ; i < n ; ++i)
    {
        for (int j = 0 ; j < n ; ++j)
        {
            b2Vec2 r = b2Vec2(xmin + i * dx, ymin + j * dy);
            totalPoly6 += [self WPoly6: r] * dA;
            totalSpiky += [self WSpiky : r] * dA;
            totalViscosity += [self WViscosity : r] * dA;
            totalLucy += [self WLucy:r] * dA;
        }
    }
    gWPoly6Scale = 1.0f / totalPoly6;
    gWSpikyScale = 1.0f / totalSpiky;
    gWViscosityScale = 1.0f / totalViscosity;
    gWLucyScale = 1.0f / totalLucy;
}


float LenSq(b2Vec2 r)
{
    return r.x * r.x + r.y * r.y;
}

////==============================================================
//// WPoly6
////==============================================================
//*
-(float) WPoly6 : (b2Vec2) r
{
    float r2 = r.x * r.x + r.y * r.y;
    if (r2 > gKernelH2) return 0.0f;
    float a = gKernelH2 - r2;
    return gWPoly6Scale * a * a * a;
}
//*/
////==============================================================
//// WPoly6Grad
////==============================================================
-(b2Vec2) WPoly6Grad : (b2Vec2) r
{
    b2Vec2 Result;
    float r2 = LenSq(r);
    if (r2 > gKernelH2)
         Result = b2Vec2(0.0f, 0.0f);
    float f = -6.0f * gWPoly6Scale;
    float a = gKernelH2 - r2;
    float b = f * a * a;
    Result = b2Vec2(b * r.x, b * r.y);
    return Result;
}

////==============================================================
//// WSpiky
////==============================================================
-(float) WSpiky : (b2Vec2) R
{
    float r2 = LenSq(R);
    if (r2 > gKernelH2) return 0.0f;
    float a = gKernelH - sqrt(r2);
    return gWSpikyScale * a * a * a;
}

////==============================================================
//// WSpikyGrad
////==============================================================
-(b2Vec2) WSpikyGrad : (b2Vec2) R
{
    b2Vec2 Result;
    float r2 = LenSq(R);
    if (r2 > gKernelH2) Result=b2Vec2(0.0f, 0.0f);
    float minR2 = 1.0E-12;
    if (r2 < minR2) r2 = minR2;
    float r = sqrt(r2);
    float a = -3.0f * gWSpikyScale * (gKernelH - r) * (gKernelH - r) / r;
    Result = b2Vec2(a * R.x, a * R.y);
    return Result;
}

////==============================================================
//// WViscosity
////==============================================================
-(float) WViscosity : (b2Vec2) R
{
    float r2 = LenSq(R);
    if (r2 > gKernelH2) return 0.0f;
    static const float minR2 = 1.0E-12;
    if (r2 < minR2) r2 = minR2;
    float r = sqrt(r2);
    float r3 = r * r * r;
    return gWViscosityScale * ( ( (-r3 / (2.0f * gKernelH3)) +
                                 (r2 / gKernelH2) +
                                 gKernelH / (2.0f * r)) - 1.0f);
}

////========================================================
//// WViscosityLap
////========================================================
-(float) WViscosityLap : (b2Vec2) R
{
    float r2 = LenSq(R);
    if (r2 > gKernelH2) return 0.0f;
    float r = sqrt(r2);
    return gWViscosityScale * (6.0f / gKernelH3) * (gKernelH - r);
}

////==============================================================
//// WLucy
//// My cat's called Lucy
////==============================================================
-(float) WLucy : (b2Vec2) R
{
    float r2 = LenSq(R);
    if (r2 > gKernelH2) return 0.0f;
    float r = sqrt(r2);
    float b = 1 - r / gKernelH;
    float a = (1.0f + 3.0f * r / gKernelH) * cube(b);
    return gWLucyScale * a;
}

////==============================================================
//// WLucyGrad
-(b2Vec2) WLucyGrad : (b2Vec2) R
{
    b2Vec2 Result;
    float r2 = LenSq(R);
    if (r2 > gKernelH2) Result = b2Vec2(0.0f, 0.0f);
    float r = sqrt(r2);
    float a = -12.0f * (gKernelH2 - 2.0f * r * gKernelH + r2) / gKernelH4;
    Result = b2Vec2(a * R.x, a * R.y);
    return Result;
}

-(void)draw
{
	[Common draw];
}

-(void)particlesCountUp:(NSInteger)diff_
{
    CGSize size = [[CCDirector sharedDirector] winSize];
	void *tmp = liquid;
    intersectQueryCallback = NULL;
	
	liquid = (sPart *)calloc(sizeof(sPart), PARTICLES_COUNT + diff_);
	InHashCellIndexes = (int *) calloc(sizeof(sPart), PARTICLES_COUNT + diff_);
	// recreate delegates
	if (liquid)
	{
		intersectQueryCallback = new QueWorldInteractions(hashGridList, liquid, knorm, ktang, ParticleRadius);
	}
	if (tmp)
	{
		memcpy(liquid, tmp, sizeof(sPart) * PARTICLES_COUNT);
		free(tmp);
	}
    //mNumPoints = PARTICLES_COUNT + diff_;
    int Rowscount = (int) sqrt(mNumPoints),
        Colcount = mNumPoints/ Rowscount+1;
    float  Factor = CC_CONTENT_SCALE_FACTOR() / PTM_RATIO;
	for(NSInteger i = 0; i < Rowscount; i++)//создать diff_ элементов
	{
     for(NSInteger j = 0; j < Colcount; j++)
       {
        if (i*Colcount+j<mNumPoints)
        {
         CGPoint p = ccp(    (int) size.width * 0.43f + 2*i * gParticleRadius/Factor,
                        (int) size.height * 0.95f -  j * 2 * gParticleRadius/Factor);
		 CCSprite *sprite = [CCSprite spriteWithFile:@"drop.png"]; // создать указатель на спрайт
         [BATCH addChild:sprite];                                  // показать спрайт
         sprite.position = p;                     // задать координаты спрайту
         liquid[i*Colcount+j].sp = sprite;
      
         liquid[i*Colcount+j].mPos = b2Vec2(p.x * Factor, p.y * Factor);
         liquid[i*Colcount+j].mOldPos = liquid[i*Colcount+j].mPos;
         liquid[i*Colcount+j].mVel = b2Vec2(0, 0);
         liquid[i*Colcount+j].mDensity = gDensity0;
         liquid[i*Colcount+j].mPress = [self CalculatePressure : gDensity0];
         liquid[i*Colcount+j].mPressureForce  = b2Vec2(0.0f, 0.0f);
         liquid[i*Colcount+j].mViscosityForce = b2Vec2(0.0f, 0.0f);
         liquid[i*Colcount+j].mBodyForce = b2Vec2(0.0f, 0.0f);
            
         liquid[i*Colcount+j].isVisible = YES;
       //  liquid[i*Colcount+j].mMas  = (mDensity0+liquid[i*Colcount+j].mPress/PressPerDensCoef)*3.14159*ParticleRadius*ParticleRadius/4;
          
            

//            gParticles[i].mPressureForce.Set(0.0f, 0.0f);
//            gParticles[i].mViscosityForce.Set(0.0f, 0.0f);
//            gParticles[i].mBodyForce.Set(0.0f, 0.0f);
            
        }
       }
        
     }
}

-(void)particlesCountDown:(NSInteger)diff_
{
}

-(void) Check
{
    for (int i = 0; i < mNumPoints; ++i)
    {
        if (liquid[i].mPos.y < (SIZE.height / 10) / PTM_RATIO)
        {
            liquid[i].mPos = b2Vec2(liquid[i].mPos.x, (SIZE.height / 10) / PTM_RATIO);
            liquid[i].mVel.y *= -0.8f;
        }
        else if (liquid[i].mPos.y > SIZE.height / PTM_RATIO)
        {
            liquid[i].mPos = b2Vec2(liquid[i].mPos.x, SIZE.height / PTM_RATIO);
           liquid[i].mVel.y *= -0.8f;
        }  
        if (liquid[i].mPos.x < 0)
        {
            liquid[i].mPos= b2Vec2(0, liquid[i].mPos.y);
            liquid[i].mVel.x *= -0.8f;
        }
        else if (liquid[i].mPos.x > SIZE.width / PTM_RATIO)
        {
            liquid[i].mPos = b2Vec2(SIZE.width / PTM_RATIO, liquid[i].mPos.y);
            liquid[i].mVel.x *= -0.8f;
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

-(void) hashCreate
{
for(int a = 0; a < hashWidth; a++)
	{
		for(int b = 0; b < hashHeight; b++)
		{
			hashGridList[a][b].Clear();
		}
	}
  for(int a = 0; a < mNumPoints; a++)
	{
        liquid[a].isVisible = YES;
		int hcell = myMap(liquid[a].mPos.x, fluidMinX, fluidMaxX, 0, hashWidth-.001f);
		int vcell = myMap(liquid[a].mPos.y, fluidMinY, fluidMaxY, 0, hashHeight-.001f);
        
		if(hcell > -1 && hcell < hashWidth && vcell > -1 && vcell < hashHeight)
		{
			hashGridList[hcell][vcell].PushBack(a);
            if (hcell>0)
             {
              if(vcell>0)
                {
                  hashGridList[hcell-1][vcell-1].PushBack(a);
                }
              hashGridList[hcell-1][vcell].PushBack(a);
              if(vcell<hashHeight-1)
                {
                    hashGridList[hcell-1][vcell+1].PushBack(a);
                }
             if (hcell<hashWidth-1)
               {
                   if(vcell>0)
                   {
                       hashGridList[hcell+1][vcell-1].PushBack(a);
                   }
                   hashGridList[hcell+1][vcell].PushBack(a);
                   if(vcell<hashHeight-1)
                   {
                       hashGridList[hcell+1][vcell+1].PushBack(a);
                   }
               }
             }
            if (vcell>0)
             {
               hashGridList[hcell][vcell-1].PushBack(a);
               if (vcell<hashHeight-1)
                 {
                  hashGridList[hcell][vcell+1].PushBack(a);
                 }
             }
		}
    }
}

-(void) spritevisibles
{
  float porog = 1.0f;
  for(int x = 0; x < hashWidth; x++)   // процедура определения необходимости отображения
 	{
        for(int y = 0; y < hashHeight; y++)         // соответствующего спрайта
		{
         if(!hashGridList[x][y].IsEmpty())
           {
               int a, b;
               hashGridList[x][y].ResetIterator();
               for(int i=1;i<=4;i++) a = hashGridList[x][y].GetNext();
               while (a>-1)
                 {
                    bool isXN=false, isXV=false, isYN=false, isYV=false;
                    hashGridList[x][y].ResetIterator();
                    while (b!=a)
                     {
                        b = hashGridList[x][y].GetNext();
                        if (!isXN)
                          {
                              float dx = liquid[a].mPos.x-liquid[b].mPos.x ,
                                    dy = liquid[a].mPos.y-liquid[b].mPos.y;
                              isXN = ((dx -  ParticleRadius) * (dx -  ParticleRadius ) + dy * dy <  ParticleRadius *  ParticleRadius) &&
                                   (dx * dx + dy * dy <porog*  ParticleRadius *  ParticleRadius);
                          }
                        if (!isXV)
                         {
                             float dx = liquid[a].mPos.x-liquid[b].mPos.x ,
                                   dy = liquid[a].mPos.y-liquid[b].mPos.y;
                             isXV = ((dx +  ParticleRadius) * (dx +  ParticleRadius ) + dy * dy <  ParticleRadius *  ParticleRadius) &&
                                    (dx * dx + dy * dy < porog* ParticleRadius *  ParticleRadius);
                         }
                         if (!isYN)
                         {
                             float dx = liquid[a].mPos.x-liquid[b].mPos.x ,
                                   dy = liquid[a].mPos.y-liquid[b].mPos.y;
                             isYN = ((dy -  ParticleRadius) * (dy -  ParticleRadius ) + dx * dx <   ParticleRadius *  ParticleRadius) &&
                                   (dx * dx + dy * dy < porog *  ParticleRadius *   ParticleRadius);
                         }
                         if (!isYV)
                         {
                             float dx = liquid[a].mPos.x-liquid[b].mPos.x ,
                                   dy = liquid[a].mPos.y-liquid[b].mPos.y;
                             isYV = ((dy +  ParticleRadius) * (dy +  ParticleRadius ) + dy * dy <  ParticleRadius *  ParticleRadius) &&
                                   (dx * dx + dy * dy < porog* ParticleRadius *  ParticleRadius);
                         }
                     }
                     liquid[a].isVisible=!(isXN && isXV && isYN && isYV);
                     a = hashGridList[x][y].GetNext();
                 }
           }
		}
    }

}

-(void) bodytouchtest :(float) deltaT
{
 // Iterate through the grid, and do an AABB test for every grid containing particles
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


-(void) PreventParticleCohabilities
{
    tScalar minDist = 0.5f * gParticleRadius;
    tScalar minDist2 = minDist * minDist;
    tScalar resolveFrac = 0.4f;
    tVector2 delta;
    
    
    for(int a = 0; a < hashWidth; a++)
	{
		for(int b = 0; b < hashHeight; b++)
		{
			int couNeighBoords = [self NeighBoordsFind: a : b  : 0];
            
            for (int i=0; i < couNeighBoords; i++)
            {
                int a1 = InHashCellIndexes[i];
                for (int j = i+1; j < couNeighBoords; j++)
                {
                    int a2 = InHashCellIndexes[j];
                    delta = liquid[a2].mPos - liquid[a1].mPos;
                    tScalar deltaLenSq = LenSq(delta);
                    
                if (deltaLenSq > eps)
                 {
                   tScalar deltaLen = sqrt(deltaLenSq);
                   tScalar diff = resolveFrac * 0.5f * (deltaLen - minDist) / deltaLen;
                   delta = diff * delta;
                   liquid[a1].mPos += delta;
                   liquid[a1].mOldPos += delta;
                   liquid[a2].mPos -= delta;
                   liquid[a2].mOldPos -= delta;
                 }
                else
                 {
                   liquid[a1].mPos.x += 0.5f * resolveFrac * minDist;
                   liquid[a1].mOldPos.x += 0.5f * resolveFrac * minDist;
                   liquid[a2].mPos.x -= 0.5f * resolveFrac * minDist;
                   liquid[a2].mOldPos.x -= 0.5f * resolveFrac * minDist;
                 }
                }
            }
		}
    }
}


-(void)Integrate:(float)deltaT
{
    [self hashCreate];
  //*/////////////////////////////////// ФИЗИКА!!! /////////////////////////////////
    b2Vec2 r;
    b2Vec2 g = WORLD->GetGravity();
    for(int a = 0; a < hashWidth; a++)
	{
		for(int b = 0; b < hashHeight; b++)
		{
			int couNeighBoords = [self NeighBoordsFind: a : b  : 0];

            for (int i=0; i < couNeighBoords; i++)
             {
               int a1 = InHashCellIndexes[i];
               liquid[a1].mDensity = 0.f;
                 
               for (int j = 0; j < couNeighBoords; j++)
                    if  (i != j)                                     // расчет плотностей и давлений
                    {
                     int a2 = InHashCellIndexes[j];
                     r = liquid[a1].mPos - liquid[a2].mPos;
                        liquid[a1].mDensity += gParticleMass * [self WPoly6:r];
                    }
               liquid[a1].mPress =[self CalculatePressure: liquid[i].mDensity];
            }
		}
    }    
    tVector2 tmp;

    int i;
//
    for (i = mNumPoints ; i-- != 0 ; )
    {
        liquid[i].mBodyForce=gParticleMass * g;
        liquid[i].mPressureForce  =  b2Vec2(0.0f, 0.0f);
        liquid[i].mViscosityForce =  b2Vec2(0.0f, 0.0f);             // инициализация плотностей сил
    }
   
    for(int a = 0; a < hashWidth; a++)                  // расчет сил
	{
		for(int b = 0; b < hashHeight; b++)
		{
			int couNeighBoords = [self NeighBoordsFind: a : b  : 0];
            
            for (int i=0; i < couNeighBoords; i++)
            {
                int a1 = InHashCellIndexes[i];
                for (int j = i+1; j < couNeighBoords; j++)
                    {
                      int a2 = InHashCellIndexes[j];
                      if (liquid[a2].mDensity > eps)
                         {
                            r = liquid[a1].mPos - liquid[a2].mPos;
                            if (LenSq(r) < gKernelH2)
                            {
                              tmp =(gParticleMass * (liquid[a1].mPress + liquid[a2].mPress) /(2.0f * liquid[a2].mDensity) )*
                                     [self WSpikyGrad:r];
                              liquid[a1].mPressureForce -= tmp;
                              liquid[a2].mPressureForce += tmp;                                
                              tmp = (gViscosity * gParticleMass * [self WViscosityLap:r] / liquid[a2].mDensity)
                                 * (liquid[a2].mVel - liquid[a1].mVel);
                              liquid[a1].mViscosityForce += tmp;
                              liquid[a2].mViscosityForce -= tmp;
                            }
                         }
                    }
            }
		}
    }
    tVector2 force;
    for (int i = 0 ; i < mNumPoints ; ++i)                // пересечет новых положений
    {
        force =liquid[i].mBodyForce+liquid[i].mPressureForce+liquid[i].mViscosityForce; 
        // use verlet to integrate
        tScalar damping = 0.01f; // 0-1             
        tVector2 tmp = liquid[i].mPos;
        liquid[i].mPos += (1.0f - damping) *
           (liquid[i].mPos - liquid[i].mOldPos) +
           (gTimeStep * gTimeStep / gParticleMass) * force;     
        liquid[i].mOldPos = tmp;
        // cache velocity as it's useful
        liquid[i].mVel = (1.f / gTimeStep)*
                    (liquid[i].mPos - liquid[i].mOldPos) ;
    }
    
    ///////////////////////
    
}

-(void) tick: (ccTime) dt
{//*
    dt = 1.f / 60;
    [self Integrate:dt];
    [self Check];
    [self PreventParticleCohabilities ];
   // [self spritevisibles];
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    [self bodytouchtest:dt];
    for(NSInteger i=0;i<mNumPoints;i++)
      {
       liquid[i].sp.position = ccp(PTM_RATIO * liquid[i].mPos.x / CC_CONTENT_SCALE_FACTOR(),
                                   PTM_RATIO * liquid[i].mPos.y / CC_CONTENT_SCALE_FACTOR());
    
      }
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
		float radius = pCircleShape->m_radius + particleRadius;
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
            b2Vec2 vertex = vertices[i] + 0.4f * normals[i] - particlePos;
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

void SeparateParticleFromBody3(int particleIdx, b2Vec2& nearestPos, b2Vec2& normal, sPart *liquid,float knorm, float ktang)
{
	liquid[particleIdx].mPos = nearestPos;
    b2Vec2 V = liquid[particleIdx].mVel;
    b2Vec2 Vn = - b2Dot(V, normal) * normal  ;
  	liquid[particleIdx].mVel += (1 + knorm) * Vn  + 2 * (ktang - 1) * (Vn + V);
}

-(void)Exit
{
    free(liquid);
    free(InHashCellIndexes);
}

float cube(float a)
{
    return a*a*a;
}

//*

//==============================================================
// CalculatePressure
//==============================================================
-(float) CalculatePressure : (float) density
{
    return gGasK * (cube(density/gDensity0) - 1.0f);
}


/*//
 
 int gWindowW;
 int gWindowH;
 
 // render individual particles or a filled in grid
 bool gRenderParticles;
 int gNRenderSamplesX;
 int gNRenderSamplesY;
 tScalar gRenderDensityMin;
 tScalar gRenderDensityMax;
 
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
 
 // slight hack for the boundary - apply force proportional to this
 // when within gBoundaryForceScaleDist of the (lower?) boundary
 tScalar gBoundaryForceScale;
 tScalar gBoundaryForceScaleDist;
 
 // for the kernels
 tScalar gKernelH;
 tScalar gKernelH9;
 tScalar gKernelH6;
 tScalar gKernelH4;
 tScalar gKernelH3;
 tScalar gKernelH2;
 
 // normalising factors for the kernels
 tScalar gWPoly6Scale = 0.0f;
 tScalar gWSpikyScale = 0.0f;
 tScalar gWViscosityScale = 0.0f;
 tScalar gWLucyScale = 0.0f;
 
 // The array of gNParticles particles
 tParticle * gParticles = 0;
 
 // use a grid to speed up the integrals since we only need to use
 // nearby particles
 tSpatialGrid * gSpatialGrid = 0;
 
 // create an object on startup?
 bool gCreateObject;
 ::tRectangle * gRectangle = 0;
 // ratio of object density to the liquid
 tScalar gObjectDensityFrac;
 // bit hacky - extra force stopping objects getting "welded"
 tScalar gObjectBoundaryScale;
 // Extra buoyancy because of non-realistic pressure
 tScalar gObjectBuoyancyScale;
 
 // for user interaction - when the user clicks, record the mouse
 // position so we can track it afterwards
 enum tInteractionMode
 {
 INTERACTION_NONE,
 INTERACTION_BOX,
 INTERACTION_CONTAINER,
 INTERACTION_FOUNTAIN
 };
 tInteractionMode gInteractionMode = INTERACTION_NONE;
 tVector2 gOldMousePos(0.0f, 0.0f);
 
 //==============================================================
 // A bunch of Kernels follow - not all of them get used but it's nice
 // to have a menu to choose from :)
 //==============================================================
 
 //==============================================================
 // WPoly6
 //==============================================================
 inline tScalar WPoly6(const tVector2 & r)
 {
 tScalar r2 = r.GetLengthSq();
 if (r2 > gKernelH2) return 0.0f;
 tScalar a = gKernelH2 - r2;
 return gWPoly6Scale * a * a * a;
 }
 
 //==============================================================
 // WPoly6Grad
 //==============================================================
 inline tVector2 WPoly6Grad(const tVector2 & r)
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
 inline tScalar WSpiky(const tVector2 & R)
 {
 tScalar r2 = R.GetLengthSq();
 if (r2 > gKernelH2) return 0.0f;
 tScalar a = gKernelH - Sqrt(r2);
 return gWSpikyScale * a * a * a;
 }
 
 //==============================================================
 // WSpikyGrad
 //==============================================================
 inline tVector2 WSpikyGrad(const tVector2 & R)
 {
 tScalar r2 = R.GetLengthSq();
 if (r2 > gKernelH2) return tVector2(0.0f, 0.0f);
 static const tScalar minR2 = 1.0E-12;
 if (r2 < minR2) r2 = minR2;
 tScalar r = Sqrt(r2);
 tScalar a = -3.0f * gWSpikyScale * (gKernelH - r) * (gKernelH - r) / r;
 return tVector2(a * R.x, a * R.y);
 }
 
 //==============================================================
 // WViscosity
 //==============================================================
 inline tScalar WViscosity(const tVector2 & R)
 {
 tScalar r2 = R.GetLengthSq();
 if (r2 > gKernelH2) return 0.0f;
 static const tScalar minR2 = 1.0E-12;
 if (r2 < minR2) r2 = minR2;
 tScalar r = Sqrt(r2);
 tScalar r3 = r * r * r;
 return gWViscosityScale * ( ( (-r3 / (2.0f * gKernelH3)) +
 (r2 / gKernelH2) +
 gKernelH / (2.0f * r)) - 1.0f);
 }
 
 //========================================================
 // WViscosityLap
 //========================================================
 inline tScalar WViscosityLap(const tVector2 & R)
 {
 tScalar r2 = R.GetLengthSq();
 if (r2 > gKernelH2) return 0.0f;
 tScalar r = Sqrt(r2);
 return gWViscosityScale * (6.0f / gKernelH3) * (gKernelH - r);
 }
 
 //==============================================================
 // WLucy
 // My cat's called Lucy
 //==============================================================
 inline tScalar WLucy(const tVector2 & R)
 {
 tScalar r2 = R.GetLengthSq();
 if (r2 > gKernelH2) return 0.0f;
 tScalar r = Sqrt(r2);
 tScalar a = (1.0f + 3.0f * r / gKernelH) * Cube(1 - r / gKernelH);
 return gWLucyScale * a;
 }
 
 //==============================================================
 // WLucyGrad
 // If you're feeling lazy:
 // http://www.calc101.com/webMathematica/derivatives.jsp
 // with
 // (1 + 3 * ((x^2 + y^2)^0.5)/H) * ((1 - ((x^2 + y^2)^0.5)/H)^3)
 //==============================================================
 inline tVector2 WLucyGrad(const tVector2 & R)
 {
 tScalar r2 = R.GetLengthSq();
 if (r2 > gKernelH2) return tVector2(0.0f, 0.0f);
 tScalar r = Sqrt(r2);
 tScalar a = -12.0f * (gKernelH2 - 2.0f * r * gKernelH + r2) / gKernelH4;
 return tVector2(a * R.x, a * R.y);
 }
 
 //==============================================================
 // CalculatePressure
 //==============================================================
 inline tScalar CalculatePressure(tScalar density)
 {
 return gGasK * (Cube(density/gDensity0) - 1.0f);
 }
 
 //==============================================================
 // CalculateNewParticles
 //==============================================================
 void CalculateNewParticles()
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
 gParticles[i].mV =
 (gParticles[i].mR - gParticles[i].mOldR) / gTimeStep;
 }
 }
 
 //==============================================================
 // AddBoundaryForce
 // adds to the force var passed in
 //==============================================================
 inline void AddBoundaryForce(const tVector2 & pos, tVector2 & force,
 tScalar dist, tScalar scale)
 {
 tScalar delta = pos.y - gContainerY;
 if (delta < 0.0f) delta = 0.0f;
 if (delta < dist)
 force.y += scale * (1.0f - Sq(delta/dist));
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
 gParticles[i].mBodyForce,
 gBoundaryForceScaleDist,
 gBoundaryForceScale);
 }
 }
 
 //==============================================================
 // ImposeBoundaryConditions
 //==============================================================
 void ImposeBoundaryConditions()
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
 
 if (gRectangle)
 {
 if (gRectangle->MovePointOutOfObject(gParticles[i].mR, normal))
 {
 // do nothing extra
 }
 }
 }
 }
 
 #ifdef CALC_NORMAL_FIELD
 
 //==============================================================
 // CalculateNormalField
 //==============================================================
 void CalculateNormalField()
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
 gParticles[i].mCs += gParticleMass * WPoly6(gParticles[i].mR - particle->mR) /
 particle->mDensity;
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
 WPoly6Grad(gParticles[i].mR - particle->mR);
 }
 }
 }
 }
 #endif
 
 //==============================================================
 // CheckKernel
 //==============================================================
 void NormaliseKernels()
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
 totalPoly6 += WPoly6(r) * dA;
 totalSpiky += WSpiky(r) * dA;
 totalViscosity += WViscosity(r) * dA;
 totalLucy += WLucy(r) * dA;
 }
 }
 gWPoly6Scale = 1.0f / totalPoly6;
 gWSpikyScale = 1.0f / totalSpiky;
 gWViscosityScale = 1.0f / totalViscosity;
 gWLucyScale = 1.0f / totalLucy;
 }
 
 //========================================================
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
 if (particle.mPos.x < gContainerX)
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
 }
 
 //========================================================
 // SetObjectForces
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
 // IntegrateObjects
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
 // CalculatePressureAndDensities
 //==============================================================
 void CalculatePressureAndDensities()
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
 gParticles[i].mDensity += gParticleMass * WPoly6(r);
 }
 gParticles[i].mP = CalculatePressure(gParticles[i].mDensity);
 }
 }
 
 //==============================================================
 // CalculateForces
 // The forces between particles are symmetric so only calculate
 // them once per pair
 //==============================================================
 void CalculateForces()
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
 WSpikyGrad(r),
 gParticleMass * (gParticles[i].mP + particle->mP) /
 (2.0f * particle->mDensity));
 gParticles[i].mPressureForce -= tmp;
 particle->mPressureForce += tmp;
 
 // viscosity
 ScaleVector2(tmp,
 particle->mV - gParticles[i].mV,
 gViscosity * gParticleMass * WViscosityLap(r) / particle->mDensity);
 gParticles[i].mViscosityForce += tmp;
 particle->mViscosityForce -= tmp;
 }
 }
 }
 }
 
 AddBoundaryForces();
 }
 
 //==============================================================
 // PreventParticleCohabitation
 //==============================================================
 void PreventParticleCohabitation()
 {
 tScalar minDist = 0.5f * gParticleRadius;
 tScalar minDist2 = Sq(minDist);
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
 tScalar deltaLen = Sqrt(deltaLenSq);
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
 
 //==============================================================
 // DoFountain
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
 
 //==============================================================
 // Integrate
 //==============================================================
 void Integrate()
 {
 if (gInteractionMode == INTERACTION_FOUNTAIN)
 DoFountain();
 
 CalculatePressureAndDensities();
 CalculateForces();
 CalculateNewParticles();
 ImposeBoundaryConditions();
 PreventParticleCohabitation();
 gSpatialGrid->PopulateGrid(gParticles, gNParticles);
 
 #ifdef CALC_NORMAL_FIELD
 CalculateNormalField();
 #endif
 
 // and the objects
 IntegrateObjects();
 }
 
 //==============================================================
 // Initialise
 //==============================================================
 void Initialise()
 {
 NormaliseKernels();
 
 gParticles = new tParticle[gNParticles];
 gSpatialGrid = new tSpatialGrid(
 tVector2(-gContainerWidth, -gContainerHeight),
 tVector2(gDomainX + gContainerWidth, gDomainY + gContainerHeight), gKernelH);
 
 int i;
 for (i = 0 ; i < gNParticles ; ++i)
 {
 gParticles[i].mR.Set(
 gContainerX + RangedRandom(0.0f, gContainerWidth),
 gContainerY + RangedRandom(0.0f, gInitialFluidHeight));
 gParticles[i].mOldR = gParticles[i].mR;
 gParticles[i].mV.Set(0.0f, 0.0f);
 gParticles[i].mDensity = gDensity0;
 gParticles[i].mP = CalculatePressure(gDensity0);
 gParticles[i].mPressureForce.Set(0.0f, 0.0f);
 gParticles[i].mViscosityForce.Set(0.0f, 0.0f);
 gParticles[i].mBodyForce.Set(0.0f, 0.0f);
 #ifdef CALC_NORMAL_FIELD
 gParticles[i].mCs = 0.0f;
 gParticles[i].mN.Set(0.0f, 0.0f);
 #endif
 gParticles[i].mNext = 0;
 }
 
 gSpatialGrid->PopulateGrid(gParticles, gNParticles);
 
 if (gCreateObject)
 {
 tScalar widthFrac = 0.4f;
 tScalar heightFrac = 0.2f;
 gRectangle = new ::tRectangle(widthFrac * gContainerWidth,
 heightFrac * gContainerHeight,
 gObjectDensityFrac * gDensity0);
 gRectangle->SetCoGPosition(
 tVector2(gContainerX + 0.5f * gContainerWidth,
 gContainerY + gContainerHeight -
 heightFrac * gContainerHeight));
 gRectangle->SetParticleForces(gGravity);
 }
 }
 
 //==============================================================
 // DrawContainer
 //==============================================================
 void DrawContainer()
 {
 GLCOLOR3(1.0f,1.0f, 1.0f);
 glBegin(GL_QUADS);
 GLVERTEX2(gContainerX, gContainerY + gContainerHeight);
 GLVERTEX2(gContainerX, gContainerY);
 GLVERTEX2(gContainerX + gContainerWidth, gContainerY);
 GLVERTEX2(gContainerX + gContainerWidth, gContainerY + gContainerHeight);
 GLVERTEX2(gContainerX, gContainerY + gContainerHeight);
 glEnd();
 }
 
 //========================================================
 // DrawQuad
 //========================================================
 inline void DrawQuad(const tVector2 & min, const tVector2 & max)
 {
 GLVERTEX2(min.x, min.y);
 GLVERTEX2(max.x, min.y);
 GLVERTEX2(max.x, max.y);
 GLVERTEX2(min.x, max.y);
 }
 
 //==============================================================
 // DrawVoid
 //==============================================================
 void DrawVoid()
 {
 GLCOLOR3(0.5f,0.5f, 0.5f);
 glBegin(GL_QUADS);
 DrawQuad(tVector2(0.0f, 0.0f), tVector2(gDomainX, gContainerY));
 DrawQuad(tVector2(0.0f, gContainerY + gContainerHeight),
 tVector2(gDomainX, gDomainY));
 DrawQuad(tVector2(0.0f, 0.0f), tVector2(gContainerX, gDomainY));
 DrawQuad(tVector2(gContainerX + gContainerWidth, 0.0f),
 tVector2(gDomainX, gDomainY));
 glEnd();
 }
 
 //==============================================================
 // DrawGridWater
 // sets up a grid to render between the water particles...
 //==============================================================
 void DrawGridWater()
 {
 glColor3f(0.0f,0.0f, 1.0f);
 
 // number points
 int nx = gNRenderSamplesX;
 int ny = gNRenderSamplesY;
 tScalar dx = gContainerWidth / (nx - 1);
 tScalar dy = gContainerHeight / (ny - 1);
 
 static tArray2D<tScalar> densities(nx, ny);
 densities.Resize(nx, ny);
 
 tSpatialGridIterator gridIter;
 tVector2 r;
 for (int ix = nx ; ix-- != 0 ; )
 {
 for (int iy = ny ; iy-- != 0 ; )
 {
 r.Set(gContainerX + ix * dx, gContainerY + iy * dy);
 // now evaluate the density at this point
 tScalar d = 0.0f;
 for (tParticle * particle = gridIter.FindFirst(r, *gSpatialGrid) ;
 particle != 0 ;
 particle = gridIter.GetNext(*gSpatialGrid))
 {
 d += gParticleMass * WPoly6(r - particle->mR);
 }
 densities(ix, iy) = d;
 }
 }
 
 tScalar min = gRenderDensityMin;
 tScalar max = gRenderDensityMax;
 glBegin(GL_QUADS);
 int i, j;
 tScalar grey;
 for (i = nx-1 ; i-- != 0 ; )
 {
 for (j = ny-1 ; j-- != 0 ; )
 {
 tScalar x = gContainerX + i * dx;
 tScalar y = gContainerY + j * dy;
 grey = (densities(i, j) - min) / (max - min);
 GLCOLOR3(0, 0, grey);
 GLVERTEX2(x, y);
 grey = (densities(i+1, j) - min) / (max - min);
 GLCOLOR3(0, 0, grey);
 GLVERTEX2(x + dx, y);
 grey = (densities(i+1, j+1) - min) / (max - min);
 GLCOLOR3(0, 0, grey);
 GLVERTEX2(x + dx, y + dy);
 grey = (densities(i, j+1) - min) / (max - min);
 GLCOLOR3(0, 0, grey);
 GLVERTEX2(x, y + dy);
 }
 }
 glEnd();
 }
 
 
 //==============================================================
 // DrawWater
 //==============================================================
 void DrawWater()
 {
 if (gRenderParticles)
 {
 int i;
 GLCOLOR3(0.0f,0.0f, 1.0f);
 glPointSize(6);
 glEnable(GL_POINT_SMOOTH);
 glBegin(GL_POINTS);
 for (i = gNParticles ; i-- != 0 ; )
 {
 GLVERTEX2(gParticles[i].mR.x, gParticles[i].mR.y);
 }
 glEnd();
 }
 else
 {
 DrawGridWater();
 }
 }
 
 //========================================================
 // DrawObjects
 //========================================================
 void DrawObjects()
 {
 if (gRectangle)
 gRectangle->Draw();
 }
 
 //==============================================================
 // Display
 //==============================================================
 void Display()
 {
 static int lastTime = glutGet(GLUT_ELAPSED_TIME);
 int thisTime = glutGet(GLUT_ELAPSED_TIME);
 tScalar dt = (thisTime - lastTime) * 0.001f;
 dt *= gTimeScale;
 static tScalar residual = 0.0f;
 if (dt > 0.2f)
 {
 dt = 0.2f;
 residual = 0.0f;
 TRACE("can't keep up \n");
 }
 int nLoops = (int) ((dt + residual)/ gTimeStep);
 if (nLoops > 0)
 {
 residual = dt - (nLoops * gTimeStep);
 
 for (int iLoop = 0 ; iLoop < nLoops ; ++iLoop)
 {
 Integrate();
 }
 lastTime = thisTime;
 }
 
 // count FPS
 {
 static int lastTimeForFPS = thisTime;
 static int counter = 0;
 if (thisTime - lastTimeForFPS > 5000)
 {
 TRACE("FPS = %5.2f\n", counter / 5.0f);
 counter = 0;
 lastTimeForFPS = thisTime;
 }
 else
 {
 ++counter;
 }
 }
 
 glClear(GL_COLOR_BUFFER_BIT );
 DrawContainer();
 DrawWater();
 DrawObjects();
 DrawVoid();
 glutSwapBuffers();
 }
 
 //==============================================================
 // Idle
 //==============================================================
 void Idle()
 {
 Display();
 }
 
 //========================================================
 // ApplyImpulse
 //========================================================
 void ApplyImpulse(const tVector2 & deltaVel)
 {
 for (int i = gNParticles ; i-- != 0 ; )
 {
 gParticles[i].mV += deltaVel;
 gParticles[i].mOldR -= deltaVel * gTimeStep;
 }
 }
 
 //==============================================================
 // MoveContainer
 //==============================================================
 void MoveContainer(const tVector2 & dR)
 {
 gContainerX += dR.x;
 gContainerY += dR.y;
 }
 
 //==============================================================
 // MoveBox
 //==============================================================
 void MoveBox(const tVector2 & dR)
 {
 if (gRectangle)
 gRectangle->Move(dR);
 }
 
 //==============================================================
 // Keyboard
 //==============================================================
 void Keyboard( unsigned char key, int x, int y )
 {
 switch (key)
 {
 case 'q':
 case 27:
 exit(0);
 break;
 case 'a':
 ApplyImpulse(tVector2(-1.0f, 0.0f));
 break;
 case 'd':
 ApplyImpulse(tVector2(1.0f, 0.0f));
 break;
 case 's':
 ApplyImpulse(tVector2(0.0f, -1.0f));
 break;
 case 'w':
 ApplyImpulse(tVector2(0.0f, 1.0f));
 break;
 default:
 break;
 }
 }
 
 //==============================================================
 // Mouse
 //==============================================================
 void Mouse(int button, int state, int x, int y)
 {
 if (GLUT_UP == state)
 {
 gInteractionMode = INTERACTION_NONE;
 return;
 }
 
 int h = glutGet(GLUT_WINDOW_HEIGHT);
 gOldMousePos.Set(x, h-y);
 if (GLUT_LEFT_BUTTON == button)
 gInteractionMode = INTERACTION_CONTAINER;
 else if (GLUT_RIGHT_BUTTON == button)
 gInteractionMode = INTERACTION_BOX;
 else if (GLUT_MIDDLE_BUTTON == button)
 gInteractionMode = INTERACTION_FOUNTAIN;
 }
 
 //==============================================================
 // MouseMotion
 //==============================================================
 void MouseMotion(int x, int y)
 {
 int h = glutGet(GLUT_WINDOW_HEIGHT);
 tVector2 newPos(x, h-y);
 tVector2 delta = (gDomainX / gWindowW) * (newPos - gOldMousePos);
 if (INTERACTION_CONTAINER == gInteractionMode)
 {
 MoveContainer(delta);
 }
 else if (INTERACTION_BOX == gInteractionMode)
 {
 MoveBox(delta);
 }
 gOldMousePos = newPos;
 }
 
 //==============================================================
 // ReadConfig
 //==============================================================
 void ReadConfig()
 {
 #define GET(val) gConfigFile->GetValue(#val, val)
 Assert(gConfigFile);
 
 // defaults
 gWindowW = 640;
 gWindowH = 480;
 gRenderParticles = false;
 gNRenderSamplesX = 64;
 gNRenderSamplesY = 48;
 gRenderDensityMin = 500.0f;
 gRenderDensityMax = 800.0f;
 
 gDomainX = 2.0f;
 
 tScalar gContainerWidthFrac = 0.8f;
 tScalar gContainerHeightFrac = 0.5f;
 tScalar gInitialFluidHeightFrac = 0.9f;
 gGravity.Set(0.0f, -10.0f);
 gDensity0 = 1000.0f;
 gNParticles = 200;
 gViscosity = 0.05f;
 gGasK = 10.0f;
 gTimeStep = 1.0f / 100.0f;
 gTimeScale = 1.0f;
 tScalar kernelScale = 5.0f;
 gBoundaryForceScale = 0.0f;
 gBoundaryForceScaleDist = 0.1f;
 gCreateObject = true;
 gObjectDensityFrac = 0.5f;
 gObjectBoundaryScale = 20.0f;
 gObjectBuoyancyScale = 6.0f;
 
 GET(gWindowW);
 GET(gWindowH);
 GET(gRenderParticles);
 GET(gNRenderSamplesX);
 GET(gNRenderSamplesY);
 GET(gRenderDensityMin);
 GET(gRenderDensityMax);
 GET(gDomainX);
 GET(gContainerWidthFrac);
 GET(gContainerHeightFrac);
 GET(gInitialFluidHeightFrac);
 gConfigFile->GetValue("gGravityX", gGravity.x);
 gConfigFile->GetValue("gGravityY", gGravity.y);
 GET(gDensity0);
 GET(gNParticles);
 GET(gViscosity);
 GET(gGasK);
 GET(gTimeStep);
 GET(gTimeScale);
 GET(kernelScale);
 GET(gBoundaryForceScale);
 GET(gBoundaryForceScaleDist);
 GET(gCreateObject);
 GET(gObjectDensityFrac);
 GET(gObjectBoundaryScale);
 GET(gObjectBuoyancyScale);
 
 gDomainY = gDomainX * gWindowH / gWindowW;
 gContainerWidth = gContainerWidthFrac * gDomainX;
 gContainerHeight = gContainerHeightFrac * gDomainX;
 gContainerX = 0.5f * gDomainX - 0.5f * gContainerWidth;
 gContainerY = 0.1f * gDomainY;
 gInitialFluidHeight = gInitialFluidHeightFrac * gContainerHeight;
 gParticleRadius = Sqrt(gContainerWidth * gInitialFluidHeight /
 (4.0f * gNParticles));
 tScalar volume = gContainerWidth * gInitialFluidHeight;
 gParticleMass = volume * gDensity0 / gNParticles;
 
 gKernelH = kernelScale * gParticleRadius;
 gKernelH9 = pow(gKernelH, 9.0f);
 gKernelH6 = pow(gKernelH, 6.0f);
 gKernelH4 = pow(gKernelH, 4.0f);
 gKernelH3 = pow(gKernelH, 3.0f);
 gKernelH2 = pow(gKernelH, 2.0f);
 }
 
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




