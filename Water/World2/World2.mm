
#import "World2.h"
#import "cmath"

bool ParticleSolidCollision(b2Fixture* fixture, b2Vec2& particlePos, b2Vec2& nearestPos, b2Vec2& impactNormal);
void SeparateParticleFromBody(int particleIdx, b2Vec2& nearestPos, b2Vec2& normal, sParticle *liquid);

bool QueryWorldInteractions::ReportFixture(b2Fixture* fixture)
{
    int numParticles = hashGridList[x][y].GetSize();
    hashGridList[x][y].ResetIterator();
    
    // Iterate through all the particles in this cell
    for(int i = 0; i < numParticles; i++)
    {
        int particleIdx = hashGridList[x][y].GetNext();
        
        b2Vec2 particlePos = liquid[particleIdx].mPosition;
        if(fixture->GetBody()->GetType() == b2_staticBody)
        {
            b2Vec2 nearestPos(0,0);
            b2Vec2 normal(0,0);
            
            // electrodruid TODO: moving particles out to the nearest edge in this way
            // can cause leaking and tunnelling, particularly for high-velocity particles.
            // Perhaps some kind of approach involving raycasting between the old particle
            // position and the current one would work better?
            bool inside = ParticleSolidCollision(fixture, particlePos, nearestPos, normal);
            
            if (inside)
            {
                SeparateParticleFromBody(particleIdx, nearestPos, normal, liquid);
            }
        }
        else
        {
            b2Vec2 nearestPos(0,0);
            b2Vec2 normal(0,0);
            bool inside = ParticleSolidCollision(fixture, particlePos, nearestPos, normal);
            
            if (inside)
            {
                b2Vec2 particleVelocity = liquid[particleIdx].mVelocity;
                
                // electrodruid: I think this does need to be here
                particleVelocity *= deltaT;
                // electrodruid: not sure if this should be here
                //									particleVelocity *= liquid[particleIdx].mMass;
                
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
                liquid[particleIdx].mVelocity -= impulse;
                liquid[particleIdx].mVelocity += pointVelocityAbsolute;
            }
        }
    } //*/
    return true;
}
/*
 bool QueryWorldPostIntersect::ReportFixture(b2Fixture *fixture)
 {
 int numParticles = hashGridList[x][y].GetSize();
 hashGridList[x][y].ResetIterator();
 
 for(int i = 0; i < numParticles; i++)
 {
 int particleIdx = hashGridList[x][y].GetNext();
 
 b2Vec2 particlePos = liquid[particleIdx].mPosition;
 if(fixture->GetBody()->GetType() == b2_dynamicBody)
 {
 b2Vec2 nearestPos(0,0);
 b2Vec2 normal(0,0);
 bool inside = ParticleSolidCollision(fixture, particlePos, nearestPos, normal);
 
 if (inside)
 {
 SeparateParticleFromBody(particleIdx, nearestPos, normal, liquid);
 }
 }
 }
 return true;
 }//*/

static float fluidMinX;
static float fluidMaxX;
static float fluidMinY;
static float fluidMaxY;

@implementation World2

inline float b2Random(float lo, float hi) {
    return ((hi - lo) * CCRANDOM_0_1() + lo);;
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

-(id)init
{
    if (self = [super init])
	{
        srandom(time(NULL));
        
        rad = 0.6f;
        visc = 0.002f;
        idealRad = 50.0f;
        totalMass = 30.f;
		
        boxWidth = 1.f;
        boxHeight = 1.f;
        
        self.isAccelerometerEnabled = YES;
		
		fluidMinX = MW(0);
		fluidMaxX = MW(100);
		fluidMinY = MH(0);
		fluidMaxY = MH(100);
        
        [self schedule:@selector(update:) interval:1.0f/30.0f];
    }
	
    return self;
}

-(void)particlesCountUp:(NSInteger)diff_
{
	CGSize size = [[CCDirector sharedDirector] winSize];
	
	// recreate data
	void *tmp = liquid;
	
	intersectQueryCallback = NULL;
    //	eulerIntersectQueryCallback = NULL;
	
	liquid = (sParticle *)calloc(sizeof(sParticle), PARTICLES_COUNT + diff_);
	
	// recreate delegates
	if (liquid)
	{
		intersectQueryCallback = new QueryWorldInteractions(hashGridList, liquid);
        //      eulerIntersectQueryCallback = new QueryWorldPostIntersect(hashGridList, liquid);
	}
	
	// if was old alloc - replace data to new alloc
	if (tmp)
	{
		memcpy(liquid, tmp, sizeof(sParticle) * PARTICLES_COUNT);
		free(tmp);
	}
	
	tmp = vlenBuffer;
	vlenBuffer = (float *)calloc(sizeof(float), PARTICLES_COUNT + diff_);
	
	if (tmp)
	{
		memcpy(vlenBuffer, tmp, sizeof(float) * PARTICLES_COUNT);
		free(tmp);
	}
	
	// Particles
	float massPerParticle = totalMass / (PARTICLES_COUNT + diff_);
    
	for(NSInteger i = PARTICLES_COUNT; i < PARTICLES_COUNT + diff_; i++)
	{
		CGPoint p = ccp(
                        (int)size.width / 100 * (24 + 3*sin(6.28f*i/(PARTICLES_COUNT + diff_)) ),
                        // (int)size.width / 100 * 23 + random()%(int)size.width / 20,
                        (int)size.height / 100 * (90 - 10*cos(6.28f*i/(PARTICLES_COUNT + diff_)) )
                        //   size.height-(random()%(int)size.height / 5)
                        
                        
                        );;
		
		liquid[i].mPosition = b2Vec2(p.x * CC_CONTENT_SCALE_FACTOR() / PTM_RATIO, p.y * CC_CONTENT_SCALE_FACTOR() / PTM_RATIO);
		liquid[i].mVelocity = b2Vec2(0.0f, 0.0f);
		liquid[i].mMass = massPerParticle;
		liquid[i].mRestitution = 0.4f;
		liquid[i].mFriction = 0.0f;
		
		CCSprite *sp = [CCSprite spriteWithFile:@"drop.png"];
		[BATCH addChild:sp];
		
		liquid[i].sp = sp;
		liquid[i].sp.position = ccp(p.x, p.y);
	}
}

-(void)particlesCountDown:(NSInteger)diff_
{
	for(NSInteger i = PARTICLES_COUNT - 1; i >= PARTICLES_COUNT - diff_; i--)
	{
		[liquid[i].sp removeFromParentAndCleanup:YES];
	}
}

-(void)draw
{
	[Common draw];
}

-(void)accelerometer:(UIAccelerometer *)accelerometer didAccelerate:(UIAcceleration *)acceleration
{
    [Common processAccelometry:acceleration];
}

-(void)update:(ccTime)dt
{
	if (SCENE != (CCScene<WorldSceneProtocol> *)self)
	{
		return;
	}
    
	// Update positions, and hash them
    for (NSInteger i = 0; i < PARTICLES_COUNT; ++i)
	{
        b2Vec2 g = WORLD->GetGravity();
		
        liquid[i].mVelocity.x += g.x * dt;
		liquid[i].mVelocity.y += g.y * dt;
		liquid[i].mPosition.x += liquid[i].mVelocity.x * dt;
		liquid[i].mPosition.y += liquid[i].mVelocity.y * dt;
	}
	
	[self checkBounds];
	
	for (NSInteger i = 0; i < PARTICLES_COUNT; ++i)
	{
		liquid[i].sp.position = ccp(PTM_RATIO * liquid[i].mPosition.x / CC_CONTENT_SCALE_FACTOR(), PTM_RATIO * liquid[i].mPosition.y / CC_CONTENT_SCALE_FACTOR());
	}
    [self hashLocations];
    [self applyLiquidConstraint:dt];
    [self processWorldInteractions:dt];// реакция на барьеры
    //   [self dampenLiquid];
	//[self resolveIntersections:dt];
}

-(void)clearHashGrid
{
	for(int a = 0; a < hashWidth; a++)
	{
		for(int b = 0; b < hashHeight; b++)
		{
			hashGridList[a][b].Clear();
		}
	}
}

-(void)hashLocations
{
	[self clearHashGrid];
    
	for(int a = 0; a < PARTICLES_COUNT; a++)
	{
		int hcell = hashX(liquid[a].mPosition.x);
		int vcell = hashY(liquid[a].mPosition.y);
        
		if(hcell > -1 && hcell < hashWidth && vcell > -1 && vcell < hashHeight)
		{
			hashGridList[hcell][vcell].PushBack(a);
		}
	}
}

// Fix up the tail pointers for the hashGrid cells after we've monkeyed with them to
// make the neighbours list for a particle
-(void)resetGridTailPointers:(int)particleIdx
{
	int hcell = hashX(liquid[particleIdx].mPosition.x);
	int vcell = hashY(liquid[particleIdx].mPosition.y);
    
	for(int nx = -1; nx < 2; nx++)
	{
		for(int ny = -1; ny < 2; ny++)
		{
			int xc = hcell + nx;
			int yc = vcell + ny;
            
			if(xc > -1 && xc < hashWidth && yc > -1 && yc < hashHeight)
			{
				if(!hashGridList[xc][yc].IsEmpty())
				{
					hashGridList[xc][yc].UnSplice();
				}
			}
		}
	}
}

-(void)applyLiquidConstraint:(float)deltaT
{
	// * Unfortunately, this simulation method is not actually scale
    // * invariant, and it breaks down for rad < ~3 or so.  So we need
    // * to scale everything to an ideal rad and then scale it back after.
    
	float multiplier = idealRad / rad;
    
	float static xchange[3000] = { 0.0f };
	float static ychange[3000] = { 0.0f };
    
	float static xs[3000];
	float static ys[3000];
	float static vxs[3000];
	float static vys[3000];
    
	for (int i = 0; i < PARTICLES_COUNT; ++i)
	{
		xs[i] = multiplier*liquid[i].mPosition.x;
		ys[i] = multiplier*liquid[i].mPosition.y;
		vxs[i] = multiplier*liquid[i].mVelocity.x;
		vys[i] = multiplier*liquid[i].mVelocity.y;
		xchange[i] = 0;
		ychange[i] = 0;
	}
    
	cFluidHashList neighbours;
    
	float *vlen = vlenBuffer;
    
	for(int i = 0; i < PARTICLES_COUNT; i++)
	{
		// Populate the neighbor list from the 9 proximate cells
		int hcell = hashX(liquid[i].mPosition.x);
		int vcell = hashY(liquid[i].mPosition.y);
        
		bool bFoundFirstCell = false;
		for(int nx = -1; nx < 2; nx++)
		{
			for(int ny = -1; ny < 2; ny++)
			{
				int xc = hcell + nx;
				int yc = vcell + ny;
				if(xc > -1 && xc < hashWidth && yc > -1 && yc < hashHeight)
				{
					if(!hashGridList[xc][yc].IsEmpty())
					{
						if(!bFoundFirstCell)
						{
							// Set the head and tail of the beginning of our neighbours list
							neighbours.SetHead(hashGridList[xc][yc].pHead());
							neighbours.SetTail(hashGridList[xc][yc].pTail());
							bFoundFirstCell = true;
						}
						else
						{
							// We already have a neighbours list, so just add this cell's particles onto
							// the end of it.
							neighbours.Splice(hashGridList[xc][yc].pHead(), hashGridList[xc][yc].pTail());
						}
					}
				}
			}
		}
        
		int neighboursListSize = neighbours.GetSize();
		neighbours.ResetIterator();
        
		// Particle pressure calculated by particle proximity
		// Pressures = 0 if all particles within range are idealRad distance away
		float p = 0.0f;
		float pnear = 0.0f;
		for(int a = 0; a < neighboursListSize; a++)
		{
			int n = neighbours.GetNext();
            
			int j = n;
            
			float vx = xs[j]-xs[i];//liquid[j]->GetWorldCenter().x - liquid[i]->GetWorldCenter().x;
			float vy = ys[j]-ys[i];//liquid[j]->GetWorldCenter().y - liquid[i]->GetWorldCenter().y;
            
			//early exit check
			if(vx > -idealRad && vx < idealRad && vy > -idealRad && vy < idealRad)
			{
				float vlensqr = (vx * vx + vy * vy);
				//within idealRad check
				if(vlensqr < idealRad*idealRad)
				{
					vlen[a] = b2Sqrt(vlensqr);
					if (vlen[a] < b2_linearSlop)
					{
                        //						vlen[a] = idealRad-.01f;
                        vlen[a] = b2_linearSlop;
					}
					float oneminusq = 1.0f-(vlen[a] / idealRad);
					p = (p + oneminusq*oneminusq);
					pnear = (pnear + oneminusq*oneminusq*oneminusq);
				}
				else
				{
					vlen[a] = MAXFLOAT;
				}
			}
		}
        
		// Now actually apply the forces
		float pressure = (p - 5.0f) / 2.0f; //normal pressure term
		float presnear = pnear / 2.0f; //near particles term
		float changex = 0.0f;
		float changey = 0.0f;
        
		neighbours.ResetIterator();
        
		for(int a = 0; a < neighboursListSize; a++)
		{
			int n = neighbours.GetNext();
            
			int j = n;
            
			float vx = xs[j]-xs[i];//liquid[j]->GetWorldCenter().x - liquid[i]->GetWorldCenter().x;
			float vy = ys[j]-ys[i];//liquid[j]->GetWorldCenter().y - liquid[i]->GetWorldCenter().y;
			if(vx > -idealRad && vx < idealRad && vy > -idealRad && vy < idealRad)
			{
				if(vlen[a] < idealRad)
				{
					float q = vlen[a] / idealRad;
					float oneminusq = 1.0f-q;
					float factor = oneminusq * (pressure + presnear * oneminusq) / (2.0f*vlen[a]);
					float dx = vx * factor;
					float dy = vy * factor;
					float relvx = vxs[j] - vxs[i];
					float relvy = vys[j] - vys[i];
					factor = visc * oneminusq * deltaT;
					dx -= relvx * factor;
					dy -= relvy * factor;
                    
					xchange[j] += dx;
					ychange[j] += dy;
					changex -= dx;
					changey -= dy;
				}
			}
		}
        
		xchange[i] += changex;
		ychange[i] += changey;
        
		// We've finished with this neighbours list, so go back and re-null-terminate all of the
		// grid cells lists ready for the next particle's neighbours list.
		[self resetGridTailPointers:i];
	}
	
	for (int i=0; i < PARTICLES_COUNT; ++i)
	{
		liquid[i].mPosition += b2Vec2(xchange[i] / multiplier, ychange[i] / multiplier);
        
		b2Vec2 delta_velocity = b2Vec2(xchange[i] / (multiplier*deltaT), ychange[i] / (multiplier*deltaT));
		liquid[i].mVelocity += delta_velocity;
	}
}

-(void)checkBounds
{
	for (int i = 0; i < PARTICLES_COUNT; ++i)
	{
		if (liquid[i].mPosition.y < (SIZE.height / 10) / PTM_RATIO)
		{
			liquid[i].mPosition = b2Vec2(liquid[i].mPosition.x, (SIZE.height / 10) / PTM_RATIO);
			liquid[i].mVelocity.y = 0;
		}
		else if (liquid[i].mPosition.y > SIZE.height / PTM_RATIO)
		{
			liquid[i].mPosition = b2Vec2(liquid[i].mPosition.x, SIZE.height / PTM_RATIO);
			liquid[i].mVelocity.y = 0;
		}
		
		if (liquid[i].mPosition.x < 0)
		{
			liquid[i].mPosition = b2Vec2(0, liquid[i].mPosition.y);
			liquid[i].mVelocity.x = 0;
		}
		else if (liquid[i].mPosition.x > SIZE.width / PTM_RATIO)
		{
			liquid[i].mPosition = b2Vec2(SIZE.width / PTM_RATIO, liquid[i].mPosition.y);
			liquid[i].mVelocity.x = 0;
		}
	}
}

-(void)dampenLiquid
{
	for (int i = 0; i < PARTICLES_COUNT; ++i)
	{
		liquid[i].mVelocity.x *= 0.995f;
		liquid[i].mVelocity.y *= 0.995f;
	}
}

// Handle interactions with the world
-(void)processWorldInteractions:(float)deltaT
{
	// Iterate through the grid, and do an AABB test for every grid containing particles
	for (int x = 0; x < hashWidth; ++x)
	{
		for (int y = 0; y < hashWidth; ++y)
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


// Detect an intersection between a particle and a b2Shape, and also try to suggest the nearest
// point on the shape to move the particle to, and the shape normal at that point

bool ParticleSolidCollision(b2Fixture* fixture, b2Vec2& particlePos, b2Vec2& nearestPos, b2Vec2& impactNormal)
{
    const float particleRadius = 0.2f;
    
	if (fixture->GetShape()->GetType() == b2Shape::e_circle)
	{
		b2CircleShape* pCircleShape = static_cast<b2CircleShape*>(fixture->GetShape());
		const b2Transform& xf = fixture->GetBody()->GetTransform();
		float radius = pCircleShape->m_radius + 2 * particleRadius;
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
            b2Vec2 vertex = vertices[i] + 4 * particleRadius * normals[i] - particlePos;
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

// Move the partice from inside a body to the nearest point outside, and (if appropriate), adjust
// the particle's velocity
void SeparateParticleFromBody(int particleIdx, b2Vec2& nearestPos, b2Vec2& normal, sParticle *liquid)
{
	liquid[particleIdx].mPosition = nearestPos;
    
#ifndef VERLET_INTEGRATION
	// input velocities
	b2Vec2 V = liquid[particleIdx].mVelocity;
	float vn = b2Dot(V, normal);	 // impact speed
	//					V -= (2.0f * vn) * normal;
    
	b2Vec2 Vn = vn * normal; // impact velocity vector
	b2Vec2 Vt = V - Vn; // tangencial veloctiy vector (across the surface of collision).
    
	// now the output velocities ('r' for response).
	float restitution = liquid[particleIdx].mRestitution;
	b2Vec2 Vnr = -Vn;
	Vnr.x *= restitution;
	Vnr.y *= restitution;
    
	float invFriction = 1.0f - liquid[particleIdx].mFriction;
	b2Vec2 Vtr = Vt;
	Vtr.x *= invFriction;
	Vtr.y *= invFriction;
    
	// resulting velocity
	V = Vnr + Vtr;
    
	liquid[particleIdx].mVelocity = V;
#endif
}

-(void)stepFluidParticles:(float)deltaT
{
	for (NSInteger i = 0; i < PARTICLES_COUNT; ++i)
	{
        b2Vec2 g = WORLD->GetGravity();
		
        liquid[i].mVelocity.x += g.x * deltaT;
		liquid[i].mVelocity.y += g.y * deltaT;
		liquid[i].mPosition.x += liquid[i].mVelocity.x * deltaT;
		liquid[i].mPosition.y += liquid[i].mVelocity.y * deltaT;
	}
	
	[self checkBounds];
	
	for (NSInteger i = 0; i < PARTICLES_COUNT; ++i)
	{
		liquid[i].sp.position = ccp(PTM_RATIO * liquid[i].mPosition.x / CC_CONTENT_SCALE_FACTOR(), PTM_RATIO * liquid[i].mPosition.y / CC_CONTENT_SCALE_FACTOR());
	}
}
/*
 -(void)resolveIntersections:(float)deltaT
 {
 // Iterate through the grid, and do an AABB test for every grid containing particles
 for (int x = 0; x < hashWidth; ++x)
 {
 for (int y = 0; y < hashWidth; ++y)
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
 
 if (eulerIntersectQueryCallback)
 {
 eulerIntersectQueryCallback->x = x;
 eulerIntersectQueryCallback->y = y;
 eulerIntersectQueryCallback->deltaT = deltaT;
 WORLD->QueryAABB(eulerIntersectQueryCallback, aabb);
 }
 }
 }
 }
 }//*/

@end