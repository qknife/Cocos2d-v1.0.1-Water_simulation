// Import the interfaces
#import "World1.h"
#import "Common.h"

// HelloWorldLayer implementation
@implementation World1

// on "init" you need to initialize your instance
-(id) init
{
	if( (self=[super init]))
	{
		// enable accelerometer
		self.isAccelerometerEnabled = YES;
        
		[self schedule: @selector(tick:) interval:1.0f/60.0f];
	}
	
	return self;
}

-(void)initializeScene
{
	CGSize size = [[CCDirector sharedDirector] winSize];
	
	for(NSInteger i = 0; i < PARTICLES_COUNT; i++)
	{
		[self addNewSpriteWithCoords:ccp(random()%(int)size.width / 5, size.height-(random()%(int)size.height / 5))];
	}
}

-(void) draw
{
	[Common draw];
}

-(void) addNewSpriteWithCoords:(CGPoint)p
{
	CCSprite *sprite = [CCSprite spriteWithFile:@"drop.png"];
	[BATCH addChild:sprite];
	sprite.position = ccp( p.x, p.y);
    
	// Define the dynamic body.
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
    
	bodyDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
	bodyDef.userData = sprite;
	b2Body *body = WORLD->CreateBody(&bodyDef);
    
	// Define another box shape for our dynamic body.
	b2CircleShape blob;
    blob.m_radius = (sprite.contentSize.width / 16)/PTM_RATIO;
    
	// Define the dynamic body fixture.
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &blob;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.0f;
    fixtureDef.restitution = 0.4f;
	body->CreateFixture(&fixtureDef);
}

-(void)particlesCountUp:(NSInteger)diff_
{
	CGSize size = [[CCDirector sharedDirector] winSize];
	
	for(NSInteger i = 0; i < diff_; i++)
	{
		[self addNewSpriteWithCoords:ccp(random()%(int)size.width / 5, size.height-(random()%(int)size.height / 5))];
	}
}

-(void)particlesCountDown:(NSInteger)diff_
{
	for (NSInteger i = 0; i< diff_; i++)
	{
		b2Body *b = WORLD->GetBodyList();
		[((CCSprite *)b->GetUserData()) removeFromParentAndCleanup:YES];
		WORLD->DestroyBody(b);
	}
}

-(void) tick: (ccTime) dt
{
	//It is recommended that a fixed time step is used with Box2D for stability
	//of the simulation, however, we are using a variable time step here.
	//You need to make an informed choice, the following URL is useful
	//http://gafferongames.com/game-physics/fix-your-timestep/
    
	int32 velocityIterations = 3;
	int32 positionIterations = 1;
    
	// Instruct the world to perform a single step of simulation. It is
	// generally best to keep the time step and iterations fixed.
	WORLD->Step(1.0f / 60.0f, velocityIterations, positionIterations);
    
	//Iterate over the bodies in the physics world
	for (b2Body* b = WORLD->GetBodyList(); b; b = b->GetNext())
	{
		if (b->GetUserData() != NULL) {
			//Synchronize the AtlasSprites position and rotation with the corresponding body
			CCSprite *myActor = (CCSprite*)b->GetUserData();
			myActor.position = CGPointMake( b->GetPosition().x * PTM_RATIO, b->GetPosition().y * PTM_RATIO);
		}
	}
	
	WORLD->ClearForces ();
}

- (void)accelerometer:(UIAccelerometer*)accelerometer didAccelerate:(UIAcceleration*)acceleration
{
	[Common processAccelometry:acceleration];
}

@end