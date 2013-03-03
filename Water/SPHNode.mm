//
//  SPHNode.m
//  SPH
//
//  Created by Vasiliy Yanushevich on 11/2/12.
//  Copyright 2012 Vasiliy Yanushevich. All rights reserved.
//

#import "SPHNode.h"
#import "cmath"

bool ParticleSolidCollision(b2Fixture* fixture, b2Vec2& particlePos, b2Vec2& nearestPos, b2Vec2& impactNormal);
void SeparateParticleFromBody(int particleIdx, b2Vec2& nearestPos, b2Vec2& normal, sParticle *liquid);

bool QueryWorldInteractions::ReportFixture(b2Fixture* fixture) {
    // electrodruid: Handy debug code to show which grid cells are being considered.
    // How well this technique will scale depends on a few factors - number (and radius) of the
    // particles, the size of the grid, the number of shapes in the b2broadphase, but in general
    // it seems to be quicker to do things this way than to have particles exist in the broadphase
    // and to test for pairs.
    
    // 					b2Color red(1.0f, 0.0f, 0.0f);
    // 					b2Vec2 v1(minX, maxY);
    // 					b2Vec2 v2(maxX, maxY);
    // 					b2Vec2 v3(maxX, minY);
    // 					b2Vec2 v4(minX, minY);
    // 					m_debugDraw.DrawSegment(v1, v2, red);
    // 					m_debugDraw.DrawSegment(v2, v3, red);
    // 					m_debugDraw.DrawSegment(v3, v4, red);
    // 					m_debugDraw.DrawSegment(v4, v1, red);
    
    
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
//                b2Vec2 buoyancy = -m_world->GetGravity();
                b2Vec2 buoyancy = b2Vec2(0, 10.f);
                const float buoyancyAdjuster = 0.f;
                buoyancy *= buoyancyAdjuster;
                
                fixture->GetBody()->ApplyForce(buoyancy, fixture->GetBody()->GetPosition());
                
                // move the particles away from the body
#ifdef VERLET_INTEGRATION
                SeparateParticleFromBody(particleIdx, nearestPos, normal, liquid);
#else
                liquid[particleIdx].mVelocity -= impulse;
                liquid[particleIdx].mVelocity += pointVelocityAbsolute;
#endif
            }
        }
    }
    return true;
}

bool QueryWorldPostIntersect::ReportFixture(b2Fixture *fixture) {
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
}

@implementation SPHNode

inline float b2Random(float lo, float hi) {
    return ((hi - lo) * CCRANDOM_0_1() + lo);;
}

const float fluidMinX = 5.0f;
const float fluidMaxX = 19.0f;
const float fluidMinY = 2.0f;
const float fluidMaxY = 25.0f;


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
    if (self = [super init]) {
        srandom(time(NULL));
        
        rad = 0.6f;
        visc = 0.002f;
        idealRad = 50.0f;
        totalMass = 30.f;
        boxWidth = 1.f;
        boxHeight = 1.f;
        self.anchorPoint = ccp(0, 0);
        self.position = ccp(0,0);
        
        self.isAccelerometerEnabled = YES;
        
        particle_sprites = [[CCSpriteBatchNode batchNodeWithFile:@"drop.png"] retain];
		        
        [self createStaticGeometry];
        intersectQueryCallback = new QueryWorldInteractions(hashGridList, liquid);
        eulerIntersectQueryCallback = new QueryWorldPostIntersect(hashGridList, liquid);
        
        [self schedule:@selector(update:) interval:1.0f/60.0f];
    }
    return self;
}

-(void)createStaticGeometry
{
    m_world = new b2World(b2Vec2(0.f, -10.f));
    
    m_debugDraw = new GLESDebugDraw( 32.f );
	m_world->SetDebugDraw(m_debugDraw);
	
	uint32 flags = 0;
	flags += b2Draw::e_shapeBit;
	m_debugDraw->SetFlags(flags);
	
	
    // Static geometry
    b2PolygonShape sd;
    sd.SetAsBox(5.0f, 0.5f);
    
    b2BodyDef bd;
    bd.type = b2_staticBody;
    bd.position.Set(12.5f, 7.0f);
    b2Body* ground = m_world->CreateBody(&bd);

    b2FixtureDef gFix;
    gFix.shape = &sd;

    ground->CreateFixture(&gFix);
    sd.SetAsBox(0.5f, 10.0f,b2Vec2(-7.5f,9.5f),0);
    ground->CreateFixture(&gFix);
    sd.SetAsBox(0.5f, 10.0f,b2Vec2(5.5f,9.5f),0);
    ground->CreateFixture(&gFix);
    sd.SetAsBox(4.5f, 0.5f,b2Vec2(0.0f,4.0f),0.57f);
    ground->CreateFixture(&gFix);

	b2CircleShape cd;
    cd.m_radius = 0.5f;
    cd.m_p = b2Vec2(-6.0f,-3.0f);
    gFix.shape = &cd;
    ground->CreateFixture(&gFix);
    
	// Particles
	float massPerParticle = totalMass / nParticles;
    
	float cx = 14.0f;
	float cy = 25.0f;
    
	for (int i=0; i<nParticles; ++i)
	{
		liquid[i].mPosition = b2Vec2(b2Random(cx-boxWidth*.5f, cx+boxWidth*.5f), b2Random(cy-boxHeight*.5f, cy+boxHeight*.5f));
        
		liquid[i].mOldPosition = liquid[i].mPosition;
		liquid[i].mVelocity = b2Vec2(0.0f, 0.0f);
		liquid[i].mAcceleration = b2Vec2(0, -5.0f);
        
		liquid[i].mMass = massPerParticle;
		liquid[i].mRestitution = 0.4f;
		liquid[i].mFriction = 0.0f;
        
        CCSprite *sp = [CCSprite spriteWithFile:@"drop.png"];
        [particle_sprites addChild:sp];
        
        liquid[i].sp = sp;
		liquid[i].sp.position = ccp(32.f * liquid[i].mPosition.x, 32.f * liquid[i].mPosition.y);
	}
    
	// Box
    b2FixtureDef polyDef;
    b2PolygonShape shape;
	shape.SetAsBox(b2Random(0.5f,0.5f), b2Random(0.5f,0.5f));
	   
	polyDef.density = 4.0f;
	b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Vec2(14.0f,8.0f);
    
	// electrodruid: As well as the above note about density, some angular damping seems to help with the
	// crazy spinning, although I'd like to not have to add it - feels like a hack.
	bodyDef.angularDamping = 0.5f;
    
	bod = m_world->CreateBody(&bodyDef);
    polyDef.shape = &shape;
	bod->CreateFixture(&polyDef);
}

-(void) draw
{
	glDisable(GL_ALPHA_TEST);
	glPushMatrix();
	glDisable(GL_TEXTURE_2D);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

	m_world->DrawDebugData();

    glEnable(GL_TEXTURE_2D);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glPopMatrix();
	
	CGSize screenSize = [CCDirector sharedDirector].winSize;
    
    if(renderTextureB==nil){
        renderTextureB = [DLRenderTexture renderTextureWithWidth:screenSize.width height:screenSize.height pixelFormat:kCCTexture2DPixelFormat_RGBA4444];
        renderTextureB.position = ccp(390,520);//(80, 140);//390,520);//screenSize.width/2, screenSize.height/2);
		renderTextureB.anchorPoint = ccp(0,0);
        [self addChild:renderTextureB];
    }
	
    [renderTextureB clear:0.0 g:0.0 b:0.0 a:0.0];
    [renderTextureB begin];
    [particle_sprites visit];
//	[super draw];
    [renderTextureB end];
}

-(void)dealloc
{
    delete m_world;
    delete intersectQueryCallback;
    [super dealloc];
}

-(void)accelerometer:(UIAccelerometer *)accelerometer didAccelerate:(UIAcceleration *)acceleration
{
    m_world->SetGravity(b2Vec2(10.f * acceleration.x, 10.f * acceleration.y));
}

-(void)update:(ccTime)dt
{
	dt = 1.0f / 60.0f;

	// Update positions, and hash them
    [self stepFluidParticles:dt];
    [self hashLocations];
	[self applyLiquidConstraint:dt];
    [self processWorldInteractions:dt];
//    [self dampenLiquid];
    [self checkBounds];
    
	// Update box2d positions
	m_world->Step(dt, 8, 3);
    
	// Resolve any remaining intersections
	[self resolveIntersections:dt];
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
    
	for(int a = 0; a < nParticles; a++)
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
    
	float static xchange[nParticles] = { 0.0f };
	float static ychange[nParticles] = { 0.0f };
    
	float static xs[nParticles];
	float static ys[nParticles];
	float static vxs[nParticles];
	float static vys[nParticles];
    
	for (int i=0; i<nParticles; ++i)
	{
		xs[i] = multiplier*liquid[i].mPosition.x;
		ys[i] = multiplier*liquid[i].mPosition.y;
		vxs[i] = multiplier*liquid[i].mVelocity.x;
		vys[i] = multiplier*liquid[i].mVelocity.y;
		xchange[i] = 0;
		ychange[i] = 0;
	}
    
	cFluidHashList neighbours;
    
	float* vlen = vlenBuffer;
    
	for(int i = 0; i < nParticles; i++)
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
	
	for (int i=0; i<nParticles; ++i)
	{
		liquid[i].mPosition += b2Vec2(xchange[i] / multiplier, ychange[i] / multiplier);
        
#ifndef VERLET_INTEGRATION
		liquid[i].mVelocity += b2Vec2(xchange[i] / (multiplier*deltaT), ychange[i] / (multiplier*deltaT));
#endif
	}
}

-(void)checkBounds
{
	float massPerParticle = totalMass / nParticles;
    
	for (int i=0; i<nParticles; ++i)
	{
		if (liquid[i].mPosition.y < -1.0f)
		{
			float cx = 14.0f + b2Random(-0.6f,0.6f);
			float cy = 20.0f + b2Random(-2.3f,2.0f);
            
			liquid[i].mPosition = b2Vec2(cx, cy);
			liquid[i].mOldPosition = liquid[i].mPosition;
			liquid[i].mVelocity = b2Vec2(0.0f, 0.0f);
			liquid[i].mAcceleration = b2Vec2(0, -5.0f);
            
			liquid[i].mMass = massPerParticle;
			liquid[i].mRestitution = 0.4f;
			liquid[i].mFriction = 0.0f;
		}
	}
}

-(void)dampenLiquid
{
	for (int i=0; i<nParticles; ++i)
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
                
                intersectQueryCallback->x = x;
                intersectQueryCallback->y = y;
                intersectQueryCallback->deltaT = deltaT;
                m_world->QueryAABB(intersectQueryCallback, aabb);
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
            b2Vec2 vertex = vertices[i] + particleRadius * normals[i] - particlePos;
			float distance = b2Dot(normals[i], vertex);
            
			if (distance < 0.0f)
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



// Move the particle from inside a body to the nearest point outside, and (if appropriate), adjust
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
	for (int i = 0; i < nParticles; ++i)
	{
        b2Vec2 g = m_world->GetGravity();
        liquid[i].mVelocity.x += g.x * deltaT;
		liquid[i].mVelocity.y += g.y * deltaT;
		liquid[i].mPosition.x += liquid[i].mVelocity.x * deltaT;
		liquid[i].mPosition.y += liquid[i].mVelocity.y * deltaT;		
        liquid[i].sp.position = ccp(32.f * liquid[i].mPosition.x, 32.f * liquid[i].mPosition.y);
	}
}

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
                
                eulerIntersectQueryCallback->x = x;
                eulerIntersectQueryCallback->y = y;
                eulerIntersectQueryCallback->deltaT = deltaT;
                m_world->QueryAABB(eulerIntersectQueryCallback, aabb);
			}
		}
	}
}




@end
