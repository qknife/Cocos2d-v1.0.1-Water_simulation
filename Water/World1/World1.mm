// Import the interfaces
#import "World1.h"
#import "Common.h"
#import "DLRenderTexture.h"
#import "SPHNode.h"

// HelloWorldLayer implementation
@implementation World1

// on "init" you need to initialize your instance
-(id) init
{
	if( (self=[super init]))
	{
		// enable touches
		self.isTouchEnabled = YES;
        
		// enable accelerometer
		self.isAccelerometerEnabled = YES;
        
		CGSize s = [[CCDirector sharedDirector] winSize];
		
		//Set up sprite
		batch = [[CCSpriteBatchNode batchNodeWithFile:@"drop.png" capacity:300] retain];
		
		CGSize screenSize = [CCDirector sharedDirector].winSize;
		renderTextureB = [DLRenderTexture renderTextureWithWidth:screenSize.width height:screenSize.height pixelFormat:kCCTexture2DPixelFormat_RGBA4444];
        renderTextureB.position = ccp(screenSize.width/2, screenSize.height/2);
        [self addChild:renderTextureB];
        
        NSInteger maxBlobs = 200;
		for(NSInteger i = 0; i<maxBlobs; i++){
            [self addNewSpriteWithCoords:ccp(random()%(int)s.width/5, s.height-(random()%(int)s.height/5))];
        }
		
		[self schedule: @selector(tick:) interval:1.0f/60.0f];
	}
	return self;
}

-(void)initializeScene
{
	
}

- (void)drawLiquid{    
  
    glDisable(GL_ALPHA_TEST);

	[renderTextureB beginWithClear:0 g:0 b:0 a:0];	
    [batch visit];
    [renderTextureB end];
}

-(void) draw
{
	glDisable(GL_ALPHA_TEST);
	
	//    [renderTextureB clear:0.0 g:0.0 b:0.0 a:0.0];
    [renderTextureB begin];
	
	// save clear color
	GLfloat	clearColor[4];
	glGetFloatv(GL_COLOR_CLEAR_VALUE,clearColor);
	
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// restore clear color
	glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);
	
    [batch visit];
    [renderTextureB end];
	
	glPushMatrix();
	glDisable(GL_TEXTURE_2D);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	
	WORLD->DrawDebugData();
	
    glEnable(GL_TEXTURE_2D);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glPopMatrix();
}

-(void) addNewSpriteWithCoords:(CGPoint)p
{
    
	CCSprite *sprite = [CCSprite spriteWithFile:@"drop.png"];
   // [sprite setScale:0.7];
	[batch addChild:sprite];
    
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

//- (void)ccTouchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
//{
//	//Add a new body/atlas sprite at the touched location
//	for( UITouch *touch in touches ) {
//		CGPoint location = [touch locationInView: [touch view]];
//        
//		location = [[CCDirector sharedDirector] convertToGL: location];
//        
//		[self addNewSpriteWithCoords: location];
//	}
//}

- (void)accelerometer:(UIAccelerometer*)accelerometer didAccelerate:(UIAcceleration*)acceleration
{
	static float prevX=0, prevY=0;
    
	//#define kFilterFactor 0.05f
#define kFilterFactor 1.0f	// don't use filter. the code is here just as an example
    
	float accelX = (float) acceleration.x * kFilterFactor + (1- kFilterFactor)*prevX;
	float accelY = (float) acceleration.y * kFilterFactor + (1- kFilterFactor)*prevY;
    
	prevX = accelX;
	prevY = accelY;
    
	// accelerometer values are in "Portrait" mode. Change them to Landscape left
	// multiply the gravity by 15
	b2Vec2 gravity(accelX * 15, accelY * 15);
    
	WORLD->SetGravity( gravity );
}

// on "dealloc" you need to release all your retained objects
- (void) dealloc
{
	// in case you have something to dealloc, do it in this method
//	delete WORLD;
//	WORLD = NULL;
//    
//	delete m_debugDraw;
    
	// don't forget to call "super dealloc"
	[super dealloc];
}

-(void) onEnter
{
	[super onEnter];
//    CCScene *next = [CCScene node];
//    SPHNode *sph = [SPHNode node];
////	sph.position = ccp(0,300);
//    [next addChild: sph];
//	[[CCDirector sharedDirector] replaceScene:[CCTransitionFade transitionWithDuration:1.0 scene:next ]];
}

@end