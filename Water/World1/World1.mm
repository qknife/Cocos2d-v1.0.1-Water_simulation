// Import the interfaces
#import "World1.h"

// HelloWorldLayer implementation
@implementation World1

// on "init" you need to initialize your instance
-(id) init
{
	if( (self=[super init]))
	{
		// enable accelerometer
		self.isAccelerometerEnabled = YES;
        
		[self schedule: @selector(tick:) interval:1.0f/30.0f];
	}
	
	return self;
}

-(void)draw
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
    
	bodyDef.position.Set(p.x * CC_CONTENT_SCALE_FACTOR() / PTM_RATIO, p.y * CC_CONTENT_SCALE_FACTOR() / PTM_RATIO);
	bodyDef.userData = sprite;
	b2Body *body = WORLD->CreateBody(&bodyDef);
    
	// Define another box shape for our dynamic body.
	b2CircleShape blob;
    blob.m_radius = (sprite.contentSizeInPixels.width / 16)/PTM_RATIO;
    
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
		[self addNewSpriteWithCoords:ccp((int)size.width / 100 * 23
                                         + random()%(int)size.width / 20, size.height-(random()%(int)size.height / 4))];
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
	if (SCENE != (CCScene<WorldSceneProtocol> *)self)
	{
		return;
	}
	
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
			myActor.position = CGPointMake( b->GetPosition().x * PTM_RATIO / CC_CONTENT_SCALE_FACTOR(), b->GetPosition().y * PTM_RATIO / CC_CONTENT_SCALE_FACTOR());
		}
	}
	
	WORLD->ClearForces ();
}

- (void)accelerometer:(UIAccelerometer*)accelerometer didAccelerate:(UIAcceleration*)acceleration
{
	[Common processAccelometry:acceleration];
}

@end