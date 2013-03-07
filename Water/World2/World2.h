//
//  SPHNode.h
//  SPH
//
//  Created by Vasiliy Yanushevich on 11/2/12.
//  Copyright 2012 Vasiliy Yanushevich. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "Common.h"
#import "FluidHashList.h"
#import "WorldSceneProtocol.h"

// SPH node for simulating Smoothed-Particle hydrodynamics
// Individual particles are stored as struct, to keep things as simple as possible
// Everything is just a copy-paste of the ElectroDruid code from this post: http://www.box2d.org/forum/viewtopic.php?f=3&t=574&sid=0f208bac89ee07a05d5a524ef3b652cc&start=70

#define hashWidth		(40)
#define hashHeight		(40)

const int nominalNeighbourListLength = 256;

struct sParticle
{
	sParticle() : mPosition(0,0), mVelocity(0,0),
    mForce(0,0), mMass(1.0f), mRestitution(1.0f), mFriction(0.0f) {}
	~sParticle() {}
    
	b2Vec2 mPosition;
	b2Vec2 mVelocity;
	b2Vec2 mForce;
    CCSprite *sp;
    
	// electrodruid TODO - these can probably be moved out as global values to save storing them
	float mMass;
	float mRestitution;
	float mFriction;
};

class QueryWorldInteractions : public b2QueryCallback
{
public:
    QueryWorldInteractions(cFluidHashList (*grid)[hashHeight], sParticle *particles)
	{
        hashGridList = grid;
        liquid = particles;
    };
    
    bool ReportFixture(b2Fixture* fixture);
    int x, y;
    float deltaT;
    
protected:
    cFluidHashList (*hashGridList)[hashHeight];
    sParticle *liquid;
};

class QueryWorldPostIntersect : public b2QueryCallback
{
public:
    QueryWorldPostIntersect(cFluidHashList (*grid)[hashHeight], sParticle *particles)
	{
        hashGridList = grid;
        liquid = particles;
    };
    
    bool ReportFixture(b2Fixture* fixture);
    int x, y;
    float deltaT;
    
protected:
    cFluidHashList (*hashGridList)[hashHeight];
    sParticle *liquid;
};

@interface World2 : CCLayer<WorldSceneProtocol>
{
    QueryWorldInteractions *intersectQueryCallback;
    QueryWorldPostIntersect *eulerIntersectQueryCallback;
    
    // MAGIC NUMBERS
	float totalMass;
	float boxWidth;
	float boxHeight;
	float rad;
	float visc;
	float idealRad;
    
	sParticle *liquid;
    
	// This buffer almost certainly doesn't need to be this big, but
	// better safe than sorry for now. Works much quicker as a statically-defined re-usable array
	// than as a std::vector though. It's a small memory hit but a reasonably big performance boost.
	float *vlenBuffer;
    
	// Speedy new custom linked-list thing
	cFluidHashList hashGridList[hashWidth][hashHeight];
    
	// This buffer almost certainly doesn't need to be this big, but
	// better safe than sorry for now
	b2Shape *mNeighboursBuffer[nominalNeighbourListLength];
}

-(void)dampenLiquid;
-(void)checkBounds;
-(void)hashLocations;
-(void)stepFluidParticles:(float)deltaT;
-(void)processWorldInteractions:(float)deltaT;
-(void)resolveIntersections:(float)deltaT;
-(void)applyLiquidConstraint:(float)deltaT;

@end
