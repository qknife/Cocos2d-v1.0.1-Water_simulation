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
    blob.m_radius = (sprite.contentSizeInPixels.width / 10)/PTM_RATIO;
    
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
	//*
    int i = 0;float ParticleRadius=15.f,porog=1.0f;
    for (b2Body* b = WORLD->GetBodyList(); b; b = b->GetNext())
	{
		if (b->GetUserData() != NULL)
        {
			//Synchronize the AtlasSprites position and rotation with the corresponding body
			CCSprite *myActor = (CCSprite*)b->GetUserData();
			myActor.position = CGPointMake( b->GetPosition().x * PTM_RATIO / CC_CONTENT_SCALE_FACTOR(), b->GetPosition().y * PTM_RATIO / CC_CONTENT_SCALE_FACTOR());
            i++;
            myActor.visible = YES;
            if (i>3)
              {
               BOOL isXN = NO, isXV = NO, isYN = NO, isYV = NO;
               for (b2Body* a = WORLD->GetBodyList(); (a) && (a!=b) ; a = a->GetNext())
                 {
                   CCSprite *myPrevActor = (CCSprite*)a->GetUserData();
             // myPrevActor.position.x myPrevActor.position.y
               //  myActor.position.x myActor.position.y
                     if (!isXN)
                     {
                         float dx = myPrevActor.position.x  -  myActor.position.x ,
                         dy = myPrevActor.position.y-myActor.position.y;
                         isXN = ((dx -  ParticleRadius) * (dx -  ParticleRadius ) + dy * dy <  ParticleRadius *  ParticleRadius) &&
                         (dx * dx + dy * dy <porog*  ParticleRadius *  ParticleRadius);
                     }
                     if (!isXV)
                     {
                         float dx = myPrevActor.position.x-myActor.position.x ,
                         dy = myPrevActor.position.y-myActor.position.y;
                         isXV = ((dx +  ParticleRadius) * (dx +  ParticleRadius ) + dy * dy <  ParticleRadius *  ParticleRadius) &&
                         (dx * dx + dy * dy < porog* ParticleRadius *  ParticleRadius);
                     }
                     if (!isYN)
                     {
                         float dx = myPrevActor.position.x-myActor.position.x ,
                         dy = myPrevActor.position.y-myActor.position.y;
                         isYN = ((dy -  ParticleRadius) * (dy -  ParticleRadius ) + dx * dx <   ParticleRadius *  ParticleRadius) &&
                         (dx * dx + dy * dy < porog *  ParticleRadius *   ParticleRadius);
                     }
                     if (!isYV)
                     {
                         float dx = myPrevActor.position.x-myActor.position.x ,
                         dy = myPrevActor.position.y-myActor.position.y;
                         isYV = ((dy +  ParticleRadius) * (dy +  ParticleRadius ) + dy * dy <  ParticleRadius *  ParticleRadius) &&
                         (dx * dx + dy * dy < porog* ParticleRadius *  ParticleRadius);
                     }
                 }
               myActor.visible = !(isXN && isXV && isYN && isYV);
              }
        }
	}
	//*/
	WORLD->ClearForces ();
}
/*
float porog = 1.0f;

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
    
//*/


- (void)accelerometer:(UIAccelerometer*)accelerometer didAccelerate:(UIAcceleration*)acceleration
{
	[Common processAccelometry:acceleration];
}

@end